#ifndef TERMINATION_LOGIC_HPP
#define TERMINATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>

#define DISABLE_TERMINATION_SIGNAL_COUNTING_VARIABLES

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 *
 * The termination graph is a minimum vertex-disjoint path cover of the transitive reduction of I/O dependencies.
 * It "kernel path" models which kernels to are controlled by a specific termination "counter" and the incoming
 * edges to the P_{out} vertex indicates which counters must be checked by the pipeline to determine whether the
 * work has finished.
 ** ------------------------------------------------------------------------------------------------------------- */
TerminationGraph PipelineCompiler::makeTerminationGraph() {
    using Edge = TerminationGraph::edge_descriptor;

    TerminationGraph G(PipelineOutput + 1);

    // 1) copy and summarize producer -> consumer relations from the buffer graph
    for (unsigned consumer = FirstKernel; consumer <= PipelineOutput; ++consumer) {
        for (const auto & e : make_iterator_range(in_edges(consumer, mBufferGraph))) {
            const auto buffer = source(e, mBufferGraph);
            const auto producer = parent(buffer, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[e];
            const Binding & input = rd.Binding;
            const auto mayConsumeNoInput = (rd.Minimum.numerator() == 0) && (consumer != PipelineOutput);
            if (LLVM_UNLIKELY(mayConsumeNoInput || input.hasAttribute(AttrId::ZeroExtended))) {
                continue;
            }
            add_edge(producer, consumer, G);
        }
    }
    clear_out_edges(PipelineInput, G);

    // 2) copy and summarize any input scalars to internal calls or output scalars of the pipeline
    assert (FirstCall == PipelineOutput + 1);
    for (unsigned consumer = PipelineOutput; consumer <= LastCall; ++consumer) {
        for (const auto & relationship : make_iterator_range(in_edges(consumer, mScalarGraph))) {
            const auto r = source(relationship, mScalarGraph);
            for (const auto & producer : make_iterator_range(in_edges(r, mScalarGraph))) {
                const auto k = source(producer, mScalarGraph);
                assert ("cannot occur" && k != PipelineOutput);
                add_edge(k, PipelineOutput, G);
            }
        }
    }

    // 3) remove any incoming edges to a kernel that must explicitly terminate;
    // 4) create a k_i -> P_out edge for every kernel with a side effect attribute
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::MustExplicitlyTerminate))) {
            clear_in_edges(i, G);
        }
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::SideEffecting))) {
            add_edge(i, PipelineOutput, G);
        }
    }

    transitive_reduction_dag(G);

    // TODO: Reevaulate the "counting" concept. The current system is error prone
    // with some thread interleaving of u32u8 that I have yet to characterize.
    // TODO: Try placing counters on seperate cache lines.
    #ifdef DISABLE_TERMINATION_SIGNAL_COUNTING_VARIABLES
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        remove_out_edge_if(i, [&](Edge e) { return target(e, G) != PipelineOutput; }, G);
    }
    #else

    // 5) remove any incoming edges to a kernel that may terminate or have
    // inputs from multiple (non-transitively dependent) sources.
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i];
        if (kernel->hasAttribute(AttrId::CanTerminateEarly)) {
            clear_in_edges(i, G);
        }
    }
    // Compute the minimum vertex-disjoint path cover through G, where we consider only
    // kernel nodes and any kernel that may terminate has its in-edges removed. The
    // number of paths is the number of counters required. The ∑ ceil(log2(path length)
    // is close to the minimum number of bits required to encode kernel termination in
    // the pipeline. The complication is when an edge could belong to multiple paths but
    // adding it would increase the cost of one but not the other(s). Ignoring this,
    // Kőnig's theorem shows we can solve this in polynomial time.

    using TerminationFlowEdge =
        adjacency_list_traits<vecS, vecS, directedS>::edge_descriptor;

    using TerminationFlowGraph =
        adjacency_list<vecS, vecS, directedS,
            property<vertex_name_t, unsigned>,
            property<edge_capacity_t, unsigned,
            property<edge_residual_capacity_t, unsigned,
            property<edge_reverse_t, TerminationFlowEdge>>>>;

    // Compute the auxillary flow graph {s, V_out, V_in, t}

    const auto numOfKernels = (mLastKernel - mFirstKernel);
    const auto s = 0;
    const auto firstOut = 0;
    const auto firstIn = firstOut + numOfKernels;
    const auto t = firstIn + numOfKernels + 1;

    TerminationFlowGraph F(t + 1);

    auto capacity = boost::get(edge_capacity, F);
    auto residual = boost::get(edge_residual_capacity, F);
    auto rev = boost::get(edge_reverse, F);

    auto add_flow_edge = [&](const unsigned u, const unsigned v) {
        TerminationFlowEdge e1, e2;
        bool in1, in2;
        std::tie(e1, in1) = add_edge(u, v, F);
        std::tie(e2, in2) = add_edge(v, u, F);
        assert (in1 && in2);
        capacity[e1] = 1;
        capacity[e2] = 0;
        rev[e1] = e2;
        rev[e2] = e1;
    };

    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        add_flow_edge(s, firstOut + i);
        for (auto e : make_iterator_range(in_edges(i, G))) {
            const auto j = source(e, G);
            if (LLVM_UNLIKELY(j == mPipelineInput)) continue;
            add_flow_edge(firstOut + j, firstIn + i);
        }
        add_flow_edge(firstIn + i, t);
        clear_in_edges(i, G);
    }

    std::vector<default_color_type> color(t + 1);
    std::vector<TerminationFlowEdge> pred(t + 1);
    auto m = edmonds_karp_max_flow(F, s, t, capacity, residual, rev, color.data(), pred.data());

    // Write the path cover of G
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        for (const auto e : make_iterator_range(out_edges(i, F))) {
            if (capacity[e] > residual[e]) {
                const auto j = target(e, F) - firstIn;
                add_edge(i, j, G);
                --m;
            }
        }
    }

    assert ("did not construct the full path cover?" && m == 0);
    #endif

    // Record the path distance to indicate the value to check
    unsigned pathCount = 0;
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        if (in_degree(i, G) == 0) {
            unsigned j = i, k = 0;
            while (LLVM_LIKELY(out_degree(j, G) != 0)) {
                const auto e = out_edge(j, G);
                G[j] = pathCount;
                G[e] = ++k;
                j = target(e, G);
            }
            // If this path is not pipeline sink, add an edge to
            // record its final distance.
            if (j <= LastKernel) {
                G[j] = pathCount;
                add_edge(j, PipelineInput, ++k, G);
            }
            ++pathCount;
        }
    }

    // TODO: fix this so we aren't initializing multiple values
    // simultaneously in the constructor.
    mTerminationSignals.resize(pathCount, nullptr);

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(PipelineOutput, G) > 0);
    assert ("termination graph construction error?"
            && out_degree(PipelineOutput, G) == 0);

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addTerminationProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addTerminationProperties(BuilderRef b) {
    const auto n = mTerminationSignals.size();
    IntegerType * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < n; ++i) {
        mPipelineKernel->addInternalScalar(sizeTy, TERMINATION_PREFIX + std::to_string(i));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initiallyTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::initiallyTerminated(BuilderRef b) {
    const auto pathId = mTerminationGraph[mKernelIndex];
    b->setKernel(mPipelineKernel);
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(pathId));
    b->setKernel(mKernel);
    Value * signal = b->CreateLoad(ptr, true);
    mTerminationSignals[pathId] = signal;
    mTerminatedInitially = signal;
    return hasKernelTerminated(b, mKernelIndex);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const unsigned kernel) const {
    // any pipeline input streams are considered produced by the P_{in} vertex.
    if (LLVM_UNLIKELY(kernel == PipelineInput)) {
        return mPipelineKernel->isFinal();
    } else {
        const auto pathId = mTerminationGraph[kernel];
        assert(pathId < mTerminationSignals.size());
        Value * const terminated = mTerminationSignals[pathId];
        const auto nextKernel = child(kernel, mTerminationGraph);
        const auto isLastLink = out_degree(nextKernel, mTerminationGraph) == 0;
        const auto comparison = isLastLink ? ICmpInst::ICMP_EQ : ICmpInst::ICMP_UGE;
        return b->CreateICmp(comparison, terminated, getTerminationSignal(b, kernel));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
Constant * PipelineCompiler::getTerminationSignal(BuilderRef b, const unsigned kernel) const {
    assert (kernel > 0);
    assert (out_degree(kernel, mTerminationGraph) > 0);
    const auto e = out_edge(kernel, mTerminationGraph);
    const auto k = mTerminationGraph[e];
    assert ("termination signal cannot be 0!" && k > 0);
    assert ("termination signal must be one greater than prior signal!" &&
            ((in_degree(kernel, mTerminationGraph) == 0 && (k == 1)) ||
             (mTerminationGraph[in_edge(kernel, mTerminationGraph)] == (k - 1))));
    return b->getSize(k);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::isClosed(BuilderRef b, const unsigned inputPort) {
    if (mIsInputClosed[inputPort] == nullptr) {
        const auto buffer = getInputBufferVertex(inputPort);
        const auto producer = parent(buffer, mBufferGraph);
        mIsInputClosed[inputPort] = hasKernelTerminated(b, producer);
    }
    return mIsInputClosed[inputPort];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateTerminationSignal
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateTerminationSignal(Value * const signal) {
    mTerminationSignals[mTerminationGraph[mKernelIndex]] = signal;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::setTerminated(BuilderRef b) const {
    b->setKernel(mPipelineKernel);
    const auto pathId = mTerminationGraph[mKernelIndex];
    Value * const ptr = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(pathId));
    Constant * const signal = getTerminationSignal(b, mKernelIndex);
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        Value * const priorSignal = b->CreateLoad(ptr, true);
        Value * const expectedPriorSignal = ConstantExpr::getSub(signal, b->getSize(1));
        Value * const valid = b->CreateICmpEQ(priorSignal, expectedPriorSignal);
        const auto prefix = makeKernelName(mKernelIndex);
        b->CreateAssert(valid, prefix + " prior termination signal is invalid");
    }

    b->CreateStore(signal, ptr, true);
    b->setKernel(mKernel);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadTerminationSignals
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::loadTerminationSignals(BuilderRef b) {
//    const auto n = mTerminationSignals.size();
//    for (unsigned i = 0; i < n; ++i) {
//        Value * const signal = b->getScalarField(TERMINATION_PREFIX + std::to_string(i));
////        mTerminationSignalPtr[i] = b->CreateAlloca(signal->getType());
////        b->CreateStore(signal, mTerminationSignalPtr[i]);
//        mTerminationSignals[i] = signal;
//    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storeTerminationSignals
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::storeTerminationSignals(BuilderRef b) {
//    const auto n = mTerminationSignals.size();
//    for (unsigned i = 0; i < n; ++i) {
//        Value * const global = b->getScalarFieldPtr(TERMINATION_PREFIX + std::to_string(i));
//        b->CreateStore(mTerminationSignals[i], global);
//    }
}

}

#endif // TERMINATION_LOGIC_HPP
