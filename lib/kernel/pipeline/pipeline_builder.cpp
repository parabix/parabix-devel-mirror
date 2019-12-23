#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/pipeline/optimizationbranch.h>
#include <kernel/core/kernel_builder.h>
#include <boost/container/flat_map.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Constants.h>
#include <toolchain/toolchain.h>

// TODO: the builders should detect if there is only one kernel in a pipeline / both branches are equivalent and return the single kernel. Modify addOrDeclareMainFunction.

// TODO: make a templated compile method to automatically validate and cast the main function to the correct type

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compile()
 ** ------------------------------------------------------------------------------------------------------------- */
void * ProgramBuilder::compile() {
    Kernel * const kernel = makeKernel();
    if (LLVM_UNLIKELY(kernel == nullptr)) {
        report_fatal_error("Main pipeline contains no kernels nor function calls.");
    }
    mDriver.addKernel(kernel);
    mDriver.generateUncachedKernels();
    return mDriver.finalizeObject(kernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel * PipelineBuilder::initializeKernel(Kernel * const kernel) {
    mDriver.addKernel(kernel);
    mKernels.emplace_back(kernel);
    return kernel;
}

using Kernels = PipelineBuilder::Kernels;

enum class VertexType { Kernel, StreamSet, Scalar };

using AttrId = Attribute::KindId;

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, VertexType, unsigned>;

using Vertex = Graph::vertex_descriptor;
using Map = flat_map<const Relationship *, Vertex>;

inline Relationship * getRelationship(not_null<Relationship *> r) {
    return r.get();
}

inline Relationship * getRelationship(const Binding & b) {
    return getRelationship(b.getRelationship());
}

template <typename Graph>
inline typename graph_traits<Graph>::edge_descriptor out_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (out_degree(u, G) == 1);
    return *out_edges(u, G).first;
}

void enumerateProducerBindings(const VertexType type, const Vertex producerVertex, const Bindings & bindings, Graph & G, Map & M, const Kernels & K) {
    const auto n = bindings.size();


    for (unsigned i = 0; i < n; ++i) {
        Relationship * const rel = getRelationship(bindings[i]);
        if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
        const auto f = M.find(rel);
        if (LLVM_UNLIKELY(f != M.end())) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            const auto existingProducer = target(out_edge(f->second, G), G);
            out << "Both " << K[existingProducer]->getName() <<
                   " and " << K[producerVertex]->getName() <<
                   " produce the same stream.";
            report_fatal_error(out.str());
        }
        const auto bufferVertex = add_vertex(type, G);
        M.emplace(rel, bufferVertex);
        add_edge(bufferVertex, producerVertex, i, G); // buffer -> producer ordering
    }
}

template <typename Array>
void enumerateConsumerBindings(const VertexType type, const Vertex consumerVertex, const Array & array, Graph & G, Map & M, const Kernels & K) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        Relationship * const rel = getRelationship(array[i]);
        assert ("relationship cannot be null!" && rel);
        if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
        const auto f = M.find(rel);
        if (LLVM_UNLIKELY(f == M.end())) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            if (consumerVertex < K.size()) {
                const Kernel * const consumer = K[consumerVertex];
                const Binding & input = ((type == VertexType::Scalar)
                                           ? consumer->getInputScalarBinding(i)
                                           : consumer->getInputStreamSetBinding(i));
                out << "input " << i << " (" << input.getName() << ") of ";
                out << "kernel " << consumer->getName();
            } else { // TODO: function calls should retain name
                out << "argument " << i << " of ";
                out << "function call " << (consumerVertex - K.size() + 1);
            }
            out << " is not a constant, produced by a kernel or an input to the pipeline";
            report_fatal_error(out.str());
        }
        const auto bufferVertex = f->second;
        assert (bufferVertex < num_vertices(G));
        assert (G[bufferVertex] == type);
        add_edge(consumerVertex, bufferVertex, i, G); // consumer -> buffer ordering
    }
}

inline char getRelationshipType(const VertexType type) {
    switch (type) {
        case VertexType::StreamSet:
            return 'S';
        case VertexType::Scalar:
            return 'V';
        default: llvm_unreachable("unknown relationship type");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addAttributesFrom
 *
 * Add any attributes from a set of kernels
 ** ------------------------------------------------------------------------------------------------------------- */
void addKernelProperties(const std::vector<Kernel *> & kernels, Kernel * const output) {
    unsigned mustTerminate = 0;
    bool canTerminate = false;
    bool sideEffecting = false;
    bool fatalTermination = false;
    unsigned stride = 0;
    for (const Kernel * kernel : kernels) {
        for (const Attribute & attr : kernel->getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::MustExplicitlyTerminate:
                    mustTerminate++;
                    break;
                case AttrId::MayFatallyTerminate:
                    fatalTermination = true;
                    break;
                case AttrId::CanTerminateEarly:
                    canTerminate = true;
                    break;
                case AttrId::SideEffecting:
                    sideEffecting = true;
                    break;
                default: continue;
            }
        }
        assert (kernel->getStride());
        if (stride) {
            stride = boost::lcm(stride, kernel->getStride());
        } else {
            stride = kernel->getStride();
        }
    }

    if (fatalTermination) {
        output->addAttribute(MayFatallyTerminate());
    }
    if (LLVM_UNLIKELY(mustTerminate == kernels.size())) {
        output->addAttribute(MustExplicitlyTerminate());
    } else if (canTerminate && !fatalTermination) {
        output->addAttribute(CanTerminateEarly());
    }
    if (sideEffecting) {
        output->addAttribute(SideEffecting());
    }
    output->setStride(stride);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel * PipelineBuilder::makeKernel() {

    mDriver.generateUncachedKernels();
    for (const auto & builder : mNestedBuilders) {
        Kernel * const kernel = builder->makeKernel();
        if (LLVM_UNLIKELY(kernel == nullptr)) continue;
        mDriver.addKernel(kernel);
        mKernels.push_back(kernel);
    }
    mDriver.generateUncachedKernels();

    const auto numOfKernels = mKernels.size();
    const auto numOfCalls = mCallBindings.size();

    // TODO: optimization must be able to synchronize non-InternallySynchronized kernels to
    // allow the following.

//    if (LLVM_UNLIKELY(numOfKernels <= 1 && numOfCalls == 0 && !mRequiresPipeline)) {
//        if (numOfKernels == 0) {
//            return nullptr;
//        } else {
//            return mKernels.back();
//        }
//    }

    constexpr auto pipelineInput = 0U;
    constexpr auto firstKernel = 1U;
    const auto firstCall = firstKernel + numOfKernels;
    const auto pipelineOutput = firstCall + numOfCalls;

    Graph G(pipelineOutput + 1);
    Map M;


    enumerateProducerBindings(VertexType::Scalar, pipelineInput, mInputScalars, G, M, mKernels);
    enumerateProducerBindings(VertexType::StreamSet, pipelineInput, mInputStreamSets, G, M, mKernels);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const Kernel * const k = mKernels[i];
        enumerateProducerBindings(VertexType::Scalar, firstKernel + i, k->getOutputScalarBindings(), G, M, mKernels);
        enumerateProducerBindings(VertexType::StreamSet, firstKernel + i, k->getOutputStreamSetBindings(), G, M, mKernels);
    }
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const Kernel * const k = mKernels[i];
        enumerateConsumerBindings(VertexType::Scalar, firstKernel + i, k->getInputScalarBindings(), G, M, mKernels);
        enumerateConsumerBindings(VertexType::StreamSet, firstKernel + i, k->getInputStreamSetBindings(), G, M, mKernels);
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        enumerateConsumerBindings(VertexType::Scalar, firstCall + i, mCallBindings[i].Args, G, M, mKernels);
    }
    enumerateConsumerBindings(VertexType::Scalar, pipelineOutput, mOutputScalars, G, M, mKernels);
    enumerateConsumerBindings(VertexType::StreamSet, pipelineOutput, mOutputStreamSets, G, M, mKernels);

    std::string signature;
    signature.reserve(1024);
    raw_string_ostream out(signature);

    out << 'P' << mNumOfThreads;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        out << "+CYC";
    }
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {
        out << "+BIC";
    }
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        out << "+DB";
    }
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {
        out << "+SS";
    }

    for (unsigned i = 0; i < numOfKernels; ++i) {
        out << "_K" << mKernels[i]->getFamilyName();
    }

    for (unsigned i = 0; i < numOfCalls; ++i) {
        out << "_C" << mCallBindings[i].Name;
    }

    const auto firstRelationship = pipelineOutput + 1;
    const auto lastRelationship = num_vertices(G);

    for (auto i = firstRelationship; i != lastRelationship; ++i) {
        if (LLVM_UNLIKELY(out_degree(i, G) == 0)) continue;
        out << '@' << getRelationshipType(G[i]);
        const auto e = out_edge(i, G);
        const auto j = target(e, G);
        out << j << '.' << G[e];
        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const auto k = source(e, G);
            out << '_' << k << '.' << G[e];
        }
    }
    out.flush();

    PipelineKernel * const pipeline =
        new PipelineKernel(mDriver, std::move(signature),
                           mNumOfThreads, codegen::BufferSegments,
                           std::move(mKernels), std::move(mCallBindings),
                           std::move(mInputStreamSets), std::move(mOutputStreamSets),
                           std::move(mInputScalars), std::move(mOutputScalars));

    addKernelProperties(pipeline->getKernels(), pipeline);

    return pipeline;
}

using AttributeCombineSet = flat_map<AttrId, unsigned>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief combineAttributes
 ** ------------------------------------------------------------------------------------------------------------- */
void combineAttributes(const Binding & S, AttributeCombineSet & C) {
    for (const Attribute & s : S) {
        auto f = C.find(s.getKind());
        if (LLVM_LIKELY(f == C.end())) {
            C.emplace(s.getKind(), s.amount());
        } else {
            // TODO: we'll need some form of attribute combination function
            f->second = std::max(f->second, s.amount());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeAttributes
 ** ------------------------------------------------------------------------------------------------------------- */
void writeAttributes(const AttributeCombineSet & C, Binding & S) {
    S.clear();
    for (auto i = C.begin(); i != C.end(); ++i) {
        Attribute::KindId k;
        unsigned m;
        std::tie(k, m) = *i;
        S.addAttribute(Attribute(k, m));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
bool combineBindingAttributes(const Bindings & A, const Bindings & B, Bindings & R) {
    const auto n = A.size();
    if (LLVM_UNLIKELY(n != B.size() || n != R.size())) {
        return false;
    }
    AttributeCombineSet C;
    for (unsigned i = 0; i < n; ++i) {
        combineAttributes(A[i], C);
        combineAttributes(B[i], C);
        combineAttributes(R[i], C);
        writeAttributes(C, R[i]);
        C.clear();
    }
    return true;
}




/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel * OptimizationBranchBuilder::makeKernel() {

    // TODO: the rates of the optimization branches should be determined by
    // the actual kernels within the branches.

    mDriver.generateUncachedKernels();
    Kernel * const nonZero = mNonZeroBranch->makeKernel();
    if (nonZero) mDriver.addKernel(nonZero);
    Kernel * const allZero = mAllZeroBranch->makeKernel();
    if (allZero) mDriver.addKernel(allZero);
    mDriver.generateUncachedKernels();

    std::string name;
    raw_string_ostream out(name);

    out << "OB:";
    if (LLVM_LIKELY(isa<StreamSet>(mCondition))) {
        StreamSet * const streamSet = cast<StreamSet>(mCondition);
        out << streamSet->getFieldWidth() << "x" << streamSet->getNumElements();
    } else {
        Scalar * const scalar = cast<Scalar>(mCondition);
        out << scalar->getFieldWidth();
    }
    out << ";Z=\"";
    if (nonZero) out << nonZero->getFamilyName();
    out << "\";N=\"";
    if (allZero) out << allZero->getFamilyName();
    out << "\"";
    out.flush();

    combineBindingAttributes(nonZero->getInputStreamSetBindings(),
                             allZero->getInputStreamSetBindings(),
                             mInputStreamSets);

    combineBindingAttributes(nonZero->getOutputStreamSetBindings(),
                             allZero->getOutputStreamSetBindings(),
                             mOutputStreamSets);



    OptimizationBranch * const br =
            new OptimizationBranch(mDriver.getBuilder(), std::move(name),
                                   mCondition, nonZero, allZero,
                                   std::move(mInputStreamSets), std::move(mOutputStreamSets),
                                   std::move(mInputScalars), std::move(mOutputScalars));

    addKernelProperties({nonZero, allZero}, br);

    return br;
}

Scalar * PipelineBuilder::getInputScalar(const StringRef name) {
    for (Binding & input : mInputScalars) {
        if (name.equals(input.getName())) {
            if (input.getRelationship() == nullptr) {
                input.setRelationship(mDriver.CreateScalar(input.getType()));
            }
            return cast<Scalar>(input.getRelationship());
        }
    }
    report_fatal_error("no scalar named " + name);
}

void PipelineBuilder::setInputScalar(const StringRef name, Scalar * value) {
    for (Binding & input : mInputScalars) {
        if (name.equals(input.getName())) {
            input.setRelationship(value);
            return;
        }
    }
    report_fatal_error("no scalar named " + name);
}

Scalar * PipelineBuilder::getOutputScalar(const StringRef name) {
    for (Binding & output : mOutputScalars) {
        if (name.equals(output.getName())) {
            if (output.getRelationship() == nullptr) {
                output.setRelationship(mDriver.CreateScalar(output.getType()));
            }
            return cast<Scalar>(output.getRelationship());
        }
    }
    report_fatal_error("no scalar named " + name);
}

void PipelineBuilder::setOutputScalar(const StringRef name, Scalar * value) {
    for (Binding & output : mOutputScalars) {
        if (name.equals(output.getName())) {
            output.setRelationship(value);
            return;
        }
    }
    report_fatal_error("no scalar named " + name);
}


PipelineBuilder::PipelineBuilder(BaseDriver & driver,
    Bindings && stream_inputs, Bindings && stream_outputs,
    Bindings && scalar_inputs, Bindings && scalar_outputs,
    const unsigned numOfThreads, const bool requiresPipeline)
: mDriver(driver)
, mNumOfThreads(numOfThreads)
, mNumOfBufferSegments(numOfThreads)
, mRequiresPipeline(requiresPipeline)
, mInputStreamSets(stream_inputs)
, mOutputStreamSets(stream_outputs)
, mInputScalars(scalar_inputs)
, mOutputScalars(scalar_outputs) {

    for (unsigned i = 0; i < mInputScalars.size(); i++) {
        Binding & input = mInputScalars[i];
        if (input.getRelationship() == nullptr) {
            input.setRelationship(driver.CreateScalar(input.getType()));
        }
    }
    for (unsigned i = 0; i < mInputStreamSets.size(); i++) {
        Binding & input = mInputStreamSets[i];
        if (LLVM_UNLIKELY(input.getRelationship() == nullptr)) {
            report_fatal_error(input.getName() + " must be set upon construction");
        }
    }
    for (unsigned i = 0; i < mOutputStreamSets.size(); i++) {
        Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(output.getRelationship() == nullptr)) {
            report_fatal_error(output.getName() + " must be set upon construction");
        }
    }
    for (unsigned i = 0; i < mOutputScalars.size(); i++) {
        Binding & output = mOutputScalars[i];
        if (output.getRelationship() == nullptr) {
            output.setRelationship(driver.CreateScalar(output.getType()));
        }
    }

}

PipelineBuilder::PipelineBuilder(Internal, BaseDriver & driver,
    Bindings stream_inputs, Bindings stream_outputs,
    Bindings scalar_inputs, Bindings scalar_outputs)
: PipelineBuilder(driver,
                  std::move(stream_inputs), std::move(stream_outputs),
                  std::move(scalar_inputs), std::move(scalar_outputs),
                  1, false) {

}

ProgramBuilder::ProgramBuilder(
    BaseDriver & driver,
    Bindings && stream_inputs, Bindings && stream_outputs,
    Bindings && scalar_inputs, Bindings && scalar_outputs)
: PipelineBuilder(
      driver,
      std::move(stream_inputs), std::move(stream_outputs),
      std::move(scalar_inputs), std::move(scalar_outputs),
      codegen::ThreadNum, true) {

}

OptimizationBranchBuilder::OptimizationBranchBuilder(
      BaseDriver & driver,
      Relationship * const condition,
      Bindings && stream_inputs, Bindings && stream_outputs,
      Bindings && scalar_inputs, Bindings && scalar_outputs)
: PipelineBuilder(PipelineBuilder::Internal{},
      driver,
      std::move(stream_inputs), std::move(stream_outputs),
      std::move(scalar_inputs), std::move(scalar_outputs))
, mCondition(condition)
, mNonZeroBranch(std::unique_ptr<PipelineBuilder>(
                    new PipelineBuilder(
                    PipelineBuilder::Internal{}, mDriver,
                    mInputStreamSets, mOutputStreamSets,
                    mInputScalars, mOutputScalars)))
, mAllZeroBranch(std::unique_ptr<PipelineBuilder>(
                    new PipelineBuilder(
                    PipelineBuilder::Internal{}, mDriver,
                    mInputStreamSets, mOutputStreamSets,
                    mInputScalars, mOutputScalars))) {

}

OptimizationBranchBuilder::~OptimizationBranchBuilder() {

}

}
