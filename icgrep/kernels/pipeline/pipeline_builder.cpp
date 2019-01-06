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
#include <queue>

#warning if two kernels have an identical signature and take the same inputs, they ought to produce the same outputs unless they are nondeterministic.

#warning the pipeline ordering should be canonicalized to ensure that when multiple kernels could be scheduled the same one will always be chosen.


using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace kernel {

#warning TODO: make a templated compile method to automatically validate and cast the main function to the correct type?

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compile()
 ** ------------------------------------------------------------------------------------------------------------- */
void * ProgramBuilder::compile() {
    PipelineKernel * const pk =
        cast<PipelineKernel>(makeKernel());
    pk->initializeBindings(mDriver);
    mDriver.addKernel(pk);
    mDriver.generateUncachedKernels();
    Function * const main =
        addOrDeclareMainFunction(pk);
    return mDriver.finalizeObject(main);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel * PipelineBuilder::initializeKernel(Kernel * const kernel) {
    kernel->initializeBindings(mDriver);
    mDriver.addKernel(kernel);
    mKernels.emplace_back(kernel);
    return kernel;
}

using Kernels = PipelineBuilder::Kernels;

enum class VertexType { Kernel, StreamSet, Scalar };

using AttrId = Attribute::KindId;

struct PipelineVertexData {
    bool active;
    VertexType type;
    PipelineVertexData() : active(false), type(VertexType::Kernel) { }
};

using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, PipelineVertexData, unsigned>;

using Vertex = Graph::vertex_descriptor;
using Map = flat_map<const Relationship *, Vertex>;
using Queue = std::queue<Vertex>;

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
        const auto bufferVertex = add_vertex(G);
        M.emplace(rel, bufferVertex);
        G[bufferVertex].type = type;
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
        assert (G[bufferVertex].type == type);
        add_edge(consumerVertex, bufferVertex, i, G); // consumer -> buffer ordering
    }
}

void markActiveKernels(const Vertex initial, Queue & Q, Graph & G) {
    assert (Q.empty());
    if (G[initial].active) return;
    G[initial].active = true;
    auto u = initial;
    for (;;) {
        for (const auto e : make_iterator_range(out_edges(u, G))) {
            const auto v = target(e, G);
            if (G[v].active) {
                continue;
            }
            G[v].active = true;
            Q.push(v);
        }
        if (Q.empty()) {
            break;
        }
        u = Q.front();
        Q.pop();
    }
}

inline void markActiveKernels(const unsigned numOfKernels, const unsigned numOfCalls, const Vertex pipelineVertex, const PipelineKernel::Kernels & K, Graph & G) {
    Queue Q;
    for (unsigned i = 0; i < numOfKernels; ++i) {
        if (K[i]->hasAttribute(AttrId::SideEffecting)) {
            markActiveKernels(vertex(i, G), Q, G);
        }
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        markActiveKernels(vertex(numOfKernels + i, G), Q, G);
    }
    markActiveKernels(pipelineVertex, Q, G);
}

inline void clearUnmarkedVertices(const Vertex pipelineVertex, Graph & G) {
    G[pipelineVertex].active = false;
    for (auto i : make_iterator_range(vertices(G))) {
        if (LLVM_UNLIKELY(!G[i].active)) {
            clear_vertex(i, G);
        }
    }
}

inline std::vector<Vertex> getTopologicalOrdering(const unsigned limit, Graph & G) {
    // now take a topological ordering of the graph
    std::vector<Vertex> ordering;
    ordering.reserve(limit);
    auto inserter = make_function_output_iterator([&](const Vertex i) {
        if (i < limit && G[i].active) {
            ordering.push_back(i);
        }
    });
    topological_sort(G, inserter);
    assert (ordering.size() <= limit);
    return ordering;
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
unsigned addKernelProperties(const std::vector<Kernel *> & kernels, Kernel * const output) {
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
    return stride;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel * PipelineBuilder::makeKernel() {

    for (const auto & builder : mNestedBuilders) {
        mKernels.push_back(builder->makeKernel());
    }

    const auto numOfKernels = mKernels.size();
    const auto numOfCalls = mCallBindings.size();
    const auto pipelineVertex = numOfKernels + numOfCalls;

    Graph G(pipelineVertex + 1);
    Map M;

    enumerateProducerBindings(VertexType::Scalar, pipelineVertex, mInputScalars, G, M, mKernels);
    enumerateProducerBindings(VertexType::StreamSet, pipelineVertex, mInputStreamSets, G, M, mKernels);
    for (unsigned i = 0; i < numOfKernels; ++i) {
        enumerateProducerBindings(VertexType::Scalar, i, mKernels[i]->getOutputScalarBindings(), G, M, mKernels);
        enumerateProducerBindings(VertexType::StreamSet, i, mKernels[i]->getOutputStreamSetBindings(), G, M, mKernels);
    }
    for (unsigned i = 0; i < numOfKernels; ++i) {
        enumerateConsumerBindings(VertexType::Scalar, i, mKernels[i]->getInputScalarBindings(), G, M);
        enumerateConsumerBindings(VertexType::StreamSet, i, mKernels[i]->getInputStreamSetBindings(), G, M);
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        enumerateConsumerBindings(VertexType::Scalar, numOfKernels + i, mCallBindings[i].Args, G, M);
    }
    enumerateConsumerBindings(VertexType::Scalar, pipelineVertex, mOutputScalars, G, M);
    enumerateConsumerBindings(VertexType::StreamSet, pipelineVertex, mOutputStreamSets, G, M);

    clear_in_edges(pipelineVertex, G);
    markActiveKernels(numOfKernels, numOfCalls, pipelineVertex, mKernels, G);
    clearUnmarkedVertices(pipelineVertex, G);

    const auto ordering = getTopologicalOrdering(numOfKernels + numOfCalls, G);

    std::string signature;
    raw_string_ostream out(signature);

    out << 'P' << mNumOfThreads;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        out << "+CYC";
    }
    for (auto i : ordering) {
        if (LLVM_LIKELY(i < numOfKernels)) {
            const auto & k = mKernels[i];
            out << "_K";
            if (k->hasFamilyName()) {
                out << k->getFamilyName();
            } else {
                out << k->getName();
            }
        } else {
            const auto j = i - numOfKernels;
            assert (j < numOfCalls);
            out << "_C" << mCallBindings[j].Name;
        }
    }

    std::vector<unsigned> index(numOfKernels + numOfCalls);

    unsigned j = 0;
    for (unsigned i : ordering) {
        index[i] = j++;
    }

    const auto firstRelationship = pipelineVertex + 1;
    const auto lastRelationship = num_vertices(G);

    for (auto i = firstRelationship; i != lastRelationship; ++i) {
        if (LLVM_UNLIKELY(out_degree(i, G) == 0)) continue;
        const PipelineVertexData & vd = G[i];
        assert (vd.active);
        out << '@' << getRelationshipType(vd.type);
        const auto e = out_edge(i, G);
        const auto j = target(e, G);
        assert (j < numOfKernels);
        const auto s = index[j];
        assert (s != std::numeric_limits<unsigned>::max());
        out << s << '.' << G[e];
        for (const auto e : make_iterator_range(in_edges(i, G))) {
            const auto j = source(e, G);
            assert (j < pipelineVertex);
            const auto t = index[j];
            assert (t != std::numeric_limits<unsigned>::max());
            out << '_' << t << '.' << G[e];
        }
    }
    out.flush();

    Kernels pipeline;
    pipeline.reserve(ordering.size());

    const std::unique_ptr<kernel::KernelBuilder> & b = mDriver.getBuilder();
    Type * const addrPtrTy = b->getVoidPtrTy();
    for (auto i : ordering) {
        if (LLVM_LIKELY(i < numOfKernels)) {
            Kernel * const k = mKernels[i];
            if (k->hasFamilyName()) {
                const auto kn = PipelineKernel::makeKernelName(k, index[i] + 1);
                addInputScalar(addrPtrTy, kn);
                addInputScalar(addrPtrTy, kn + INITIALIZE_FUNCTION_POINTER_SUFFIX);
                addInputScalar(addrPtrTy, kn + DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
                addInputScalar(addrPtrTy, kn + FINALIZE_FUNCTION_POINTER_SUFFIX);
            }
            pipeline.emplace_back(k);
        }
    }

    PipelineKernel * const pk =
        new PipelineKernel(b, std::move(signature), mNumOfThreads,
                           std::move(pipeline), std::move(mCallBindings),
                           std::move(mInputStreamSets), std::move(mOutputStreamSets),
                           std::move(mInputScalars), std::move(mOutputScalars));

    pk->setStride(addKernelProperties(pk->getKernels(), pk));

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

    Kernel * const trueBranch = mTrueBranch->makeKernel();

    Kernel * const falseBranch = mFalseBranch->makeKernel();

    std::string name;
    raw_string_ostream out(name);

    out << "OB:";

    mCondition->getType()->print(out);

    out << ";Z=\"";

    if (trueBranch->hasFamilyName()) {
        out << trueBranch->getFamilyName();
    } else {
        out << trueBranch->getName();
    }

    out << "\";N=\"";

    if (falseBranch->hasFamilyName()) {
        out << falseBranch->getFamilyName();
    } else {
        out << falseBranch->getName();
    }

    out << "\"";
    out.flush();

    // TODO: if the condition is also one of the normal inputs and the rate is compatible,
    // we could avoid sending it through.

    combineBindingAttributes(trueBranch->getInputStreamSetBindings(),
                             falseBranch->getInputStreamSetBindings(),
                             mInputStreamSets);

    combineBindingAttributes(trueBranch->getOutputStreamSetBindings(),
                             falseBranch->getOutputStreamSetBindings(),
                             mOutputStreamSets);

    OptimizationBranch * const br =
            new OptimizationBranch(mDriver.getBuilder(), std::move(name),
                                   mCondition, trueBranch, falseBranch,
                                   std::move(mInputStreamSets), std::move(mOutputStreamSets),
                                   std::move(mInputScalars), std::move(mOutputScalars));

    br->setStride(addKernelProperties({trueBranch, falseBranch}, br));

    return br;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineBuilder::addInputScalar(llvm::Type * type, std::string name) {
    mInputScalars.emplace_back(name, CreateConstant(Constant::getNullValue(type)), FixedRate(1), Family());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * PipelineBuilder::addOrDeclareMainFunction(PipelineKernel * const k) {
    auto & b = mDriver.getBuilder();
    b->setModule(mDriver.getMainModule());
    k->addKernelDeclarations(b);
    const auto method = k->hasStaticMain() ? PipelineKernel::DeclareExternal : PipelineKernel::AddInternal;
    return k->addOrDeclareMainFunction(b, method);
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

template <typename IfType>
inline void addCondition(Relationship * const condition, Bindings & bindings) {
    if (isa<IfType>(condition)) {
        bindings.emplace_back(OptimizationBranch::CONDITION_TAG, condition, FixedRate(1));
    }
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
, mTrueBranch(nullptr)
, mFalseBranch(nullptr) {
    addCondition<StreamSet>(condition, mInputStreamSets);
    addCondition<Scalar>(condition, mInputScalars);
}

OptimizationBranchBuilder::~OptimizationBranchBuilder() {

}

}
