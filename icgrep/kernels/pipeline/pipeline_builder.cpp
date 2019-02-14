#include <kernels/pipeline_builder.h>
#include <kernels/optimizationbranch.h>
#include <kernels/kernel_builder.h>
#include <boost/container/flat_map.hpp>
#include <boost/function_output_iterator.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/topological_sort.hpp>
#include <llvm/Support/raw_ostream.h>
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
    PipelineKernel * const pk = cast<PipelineKernel>(makeKernel());
    mDriver.addKernel(pk);
    mDriver.generateUncachedKernels();
    return mDriver.finalizeObject(pk);
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

inline const Relationship * getRelationship(not_null<const Relationship *> r) {
    return r.get();
}

inline const Relationship * getRelationship(const Binding & b) {
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
        const Relationship * const rel = getRelationship(bindings[i]);
        if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
        const auto f = M.find(rel);
        if (LLVM_UNLIKELY(f != M.end())) {
            std::string tmp;
            raw_string_ostream out(tmp);
            const auto existingProducer = target(out_edge(f->second, G), G);
            out << "Both " << K[existingProducer]->getName() <<
                   " and " << K[producerVertex]->getName() <<
                   " produce the same stream.";
            throw std::runtime_error(out.str());
        }
        const auto bufferVertex = add_vertex(type, G);
        M.emplace(rel, bufferVertex);
        add_edge(bufferVertex, producerVertex, i, G); // buffer -> producer ordering
    }
}

template <typename Array>
void enumerateConsumerBindings(const VertexType type, const Vertex consumerVertex, const Array & array, Graph & G, Map & M) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        const Relationship * const rel = getRelationship(array[i]);
        if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
        const auto f = M.find(rel); assert (f != M.end());
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
    unsigned stride = 0;
    for (const Kernel * kernel : kernels) {
        if (kernel->hasAttribute(AttrId::MustExplicitlyTerminate)) {
            mustTerminate++;
        } else if (kernel->hasAttribute(AttrId::CanTerminateEarly)) {
            canTerminate = true;
        }
        if (kernel->hasAttribute(AttrId::SideEffecting)) {
            sideEffecting = true;
        }
        assert (kernel->getStride());
        if (stride) {
            stride = boost::lcm(stride, kernel->getStride());
        } else {
            stride = kernel->getStride();
        }
    }
    if (LLVM_UNLIKELY(mustTerminate == kernels.size())) {
        output->addAttribute(MustExplicitlyTerminate());
    } else if (canTerminate || mustTerminate) {
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
        mDriver.addKernel(kernel);
        mKernels.push_back(kernel);
    }
    mDriver.generateUncachedKernels();

    const auto numOfKernels = mKernels.size();
    const auto numOfCalls = mCallBindings.size();
    const auto pipelineInput = 0U;
    const auto firstKernel = 1U;
    const auto firstCall = firstKernel + numOfKernels;
    const auto pipelineOutput = firstCall + numOfCalls;

    Graph G(pipelineOutput + 1);
    Map M;

    enumerateProducerBindings(VertexType::Scalar, pipelineInput, mInputScalars, G, M, mKernels);
    enumerateProducerBindings(VertexType::StreamSet, pipelineInput, mInputStreamSets, G, M, mKernels);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const Kernel * const k = mKernels[i];
        enumerateConsumerBindings(VertexType::Scalar, firstKernel + i, k->getInputScalarBindings(), G, M);
        enumerateConsumerBindings(VertexType::StreamSet, firstKernel + i, k->getInputStreamSetBindings(), G, M);
        enumerateProducerBindings(VertexType::Scalar, firstKernel + i, k->getOutputScalarBindings(), G, M, mKernels);
        enumerateProducerBindings(VertexType::StreamSet, firstKernel + i, k->getOutputStreamSetBindings(), G, M, mKernels);
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        enumerateConsumerBindings(VertexType::Scalar, firstCall + i, mCallBindings[i].Args, G, M);
    }
    enumerateConsumerBindings(VertexType::Scalar, pipelineOutput, mOutputScalars, G, M);
    enumerateConsumerBindings(VertexType::StreamSet, pipelineOutput, mOutputStreamSets, G, M);

    std::string signature;
    signature.reserve(1024);
    raw_string_ostream out(signature);

    out << 'P' << mNumOfThreads;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        out << "+CYC";
    }
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & k = mKernels[i];
        out << "_K";
        if (k->hasFamilyName()) {
            out << k->getFamilyName();
        } else {
            out << k->getName();
        }
    }

    for (unsigned i = 0; i < numOfCalls; ++i) {
        const auto j = i - firstCall;
        assert (j < numOfCalls);
        out << "_C" << mCallBindings[j].Name;
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

    const auto & b = mDriver.getBuilder();
    Type * const voidPtrTy = b->getVoidPtrTy();
    Constant * const voidPtrVal = Constant::getNullValue(voidPtrTy);

    auto addInputScalar = [&](std::string name) {
        mInputScalars.emplace_back(name, CreateConstant(voidPtrVal), FixedRate(1), Family());
    };

    for (unsigned i = 0; i < numOfKernels; ++i) {
        Kernel * const k = mKernels[i];
        if (k->hasFamilyName()) {
            const std::string prefix = "F" + std::to_string(i);
            if (LLVM_LIKELY(k->isStateful())) {
                addInputScalar(prefix);
            }
            addInputScalar(prefix + INITIALIZE_FUNCTION_POINTER_SUFFIX);
            addInputScalar(prefix + DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
            addInputScalar(prefix + FINALIZE_FUNCTION_POINTER_SUFFIX);
        }
    }

    PipelineKernel * const pk =
        new PipelineKernel(b, std::move(signature), mNumOfThreads, mNumOfThreads,
                           std::move(mKernels), std::move(mCallBindings),
                           std::move(mInputStreamSets), std::move(mOutputStreamSets),
                           std::move(mInputScalars), std::move(mOutputScalars));

    addKernelProperties(pk->getKernels(), pk);

    return pk;
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

    mDriver.generateUncachedKernels();

    Kernel * const nonZero = mNonZeroBranch->makeKernel();
    mDriver.addKernel(nonZero);

    Kernel * const allZero = mAllZeroBranch->makeKernel();
    mDriver.addKernel(allZero);

    mDriver.generateUncachedKernels();

    std::string name;
    raw_string_ostream out(name);

    out << "OB:";

    mCondition->getType()->print(out);

    out << ";Z=\"";

    if (nonZero->hasFamilyName()) {
        out << nonZero->getFamilyName();
    } else {
        out << nonZero->getName();
    }

    out << "\";N=\"";

    if (allZero->hasFamilyName()) {
        out << allZero->getFamilyName();
    } else {
        out << allZero->getName();
    }

    out << "\"";
    out.flush();

    // TODO: if the condition is also one of the normal inputs and the rate is compatible,
    // we could avoid sending it through.

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

Scalar * PipelineBuilder::getInputScalar(const std::string & name) {
    for (Binding & input : mInputScalars) {
        if (input.getName() == name) {
            if (input.getRelationship() == nullptr) {
                input.setRelationship(mDriver.CreateScalar(input.getType()));
            }
            return cast<Scalar>(input.getRelationship());
        }
    }
    return nullptr;
}

void PipelineBuilder::setInputScalar(const std::string & name, Scalar * value) {
    for (Binding & input : mInputScalars) {
        if (input.getName() == name) {
            input.setRelationship(value);
            break;
        }
    }
}

Scalar * PipelineBuilder::getOutputScalar(const std::string & name) {
    for (Binding & output : mOutputScalars) {
        if (output.getName() == name) {
            if (output.getRelationship() == nullptr) {
                output.setRelationship(mDriver.CreateScalar(output.getType()));
            }
            return cast<Scalar>(output.getRelationship());
        }
    }
    return nullptr;
}

void PipelineBuilder::setOutputScalar(const std::string & name, Scalar * value) {
    for (Binding & output : mOutputScalars) {
        if (output.getName() == name) {
            output.setRelationship(value);
            break;
        }
    }
}


PipelineBuilder::PipelineBuilder(BaseDriver & driver,
    Bindings && stream_inputs, Bindings && stream_outputs,
    Bindings && scalar_inputs, Bindings && scalar_outputs, const unsigned numOfThreads)
: mDriver(driver)
, mNumOfThreads(numOfThreads)
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
    Bindings scalar_inputs, Bindings scalar_outputs, const unsigned numOfThreads)
: PipelineBuilder(driver,
                  std::move(stream_inputs), std::move(stream_outputs),
                  std::move(scalar_inputs), std::move(scalar_outputs),
                  numOfThreads) {

}

ProgramBuilder::ProgramBuilder(
    BaseDriver & driver,
    Bindings && stream_inputs, Bindings && stream_outputs,
    Bindings && scalar_inputs, Bindings && scalar_outputs)
: PipelineBuilder(
      driver,
      std::move(stream_inputs), std::move(stream_outputs),
      std::move(scalar_inputs), std::move(scalar_outputs),
      codegen::ThreadNum) {

}

OptimizationBranchBuilder::OptimizationBranchBuilder(
      BaseDriver & driver,
      Relationship * const condition,
      Bindings && stream_inputs, Bindings && stream_outputs,
      Bindings && scalar_inputs, Bindings && scalar_outputs)
: PipelineBuilder(
      driver,
      std::move(stream_inputs), std::move(stream_outputs),
      std::move(scalar_inputs), std::move(scalar_outputs))
, mCondition(condition)
, mNonZeroBranch(std::unique_ptr<PipelineBuilder>(
                    new PipelineBuilder(
                    PipelineBuilder::Internal{}, mDriver,
                    mInputStreamSets, mOutputStreamSets,
                    mInputScalars, mOutputScalars, 1)))
, mAllZeroBranch(std::unique_ptr<PipelineBuilder>(
                    new PipelineBuilder(
                    PipelineBuilder::Internal{}, mDriver,
                    mInputStreamSets, mOutputStreamSets,
                    mInputScalars, mOutputScalars, 1))) {

}

OptimizationBranchBuilder::~OptimizationBranchBuilder() {

}

}
