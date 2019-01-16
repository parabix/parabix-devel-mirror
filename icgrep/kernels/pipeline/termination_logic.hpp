#ifndef TERMINATION_LOGIC_HPP
#define TERMINATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#include <boost/graph/topological_sort.hpp>
#include <boost/graph/edmonds_karp_max_flow.hpp>

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

    using Vertex = TerminationGraph::vertex_descriptor;
    using Edge = TerminationGraph::edge_descriptor;
    using VertexVector = std::vector<Vertex>;

    const auto numOfCalls = mPipelineKernel->getCallBindings().size();
    const auto firstCall = mPipelineOutput + 1;
    const auto lastCall = firstCall + numOfCalls;
    const auto n = mPipelineOutput + 1;

    // TODO: if the lower bound of an input is 0 or a the input is zero-extended,
    // how would this affect termination?

    TerminationGraph G(n);

    // 1) copy and summarize producer -> consumer relations from the buffer graph
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        for (auto buffer : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto bufferVertex = target(buffer, mBufferGraph);
            for (auto consumer : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
                const auto j = target(consumer, mBufferGraph);
                add_edge(i, j, G);
            }
        }
    }

    // 2) copy and summarize any output scalars of the pipeline or any calls
    for (unsigned i = mPipelineOutput; i < lastCall; ++i) {
        for (auto relationship : make_iterator_range(in_edges(i, mScalarDependencyGraph))) {
            const auto relationshipVertex = source(relationship, mScalarDependencyGraph);
            for (auto producer : make_iterator_range(in_edges(relationshipVertex, mScalarDependencyGraph))) {
                const auto kernel = source(producer, mScalarDependencyGraph);
                assert ("cannot occur" && kernel != mPipelineOutput);
                add_edge(kernel, mPipelineOutput, G);
            }
        }
    }

    // 3) remove any incoming edges to a kernel that must explicitly terminate;
    // 4) create a k_i -> P_out edge for every kernel with a side effect attribute
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i];
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::MustExplicitlyTerminate))) {
            clear_in_edges(i, G);
        }
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::SideEffecting))) {
            add_edge(i, mPipelineOutput, G);
        }
    }

    // generate a transitive closure
    VertexVector ordering;
    ordering.reserve(n);
    topological_sort(G, std::back_inserter(ordering));

    for (unsigned u : ordering) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (auto f : make_iterator_range(out_edges(u, G))) {
                add_edge(s, target(f, G), G);
            }
        }
    }

    // then take the transitive reduction
    dynamic_bitset<> sources(n, false);
    for (unsigned u = mPipelineOutput; u--; ) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            sources.set(source(e, G), true);
        }
        for (auto e : make_iterator_range(out_edges(u, G))) {
            remove_in_edge_if(target(e, G), [&G, &sources](const Edge f) {
                return sources.test(source(f, G));
            }, G);
        }
        sources.reset();
    }

    // 5) remove any incoming edges to a kernel that may terminate;
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i];
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::CanTerminateEarly))) {
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

    // Record the path distance to indicate the value to check
    unsigned pathCount = 0;
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
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
            if (j < mLastKernel) {
                G[j] = pathCount;
                add_edge(j, mPipelineInput, ++k, G);
            }
            ++pathCount;
        }
    }

    // TODO: fix this so we aren't initializing multiple values
    // simultaneously in the constructor.
    mTerminationSignals.resize(pathCount, nullptr);

    assert ("a pipeline with no sinks ought to produce no observable data"
            && in_degree(mPipelineOutput, G) > 0);
    assert ("termination graph construction error?"
            && out_degree(mPipelineOutput, G) == 0);

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
    Value * const signal = b->CreateLoad(ptr, true);
    mTerminationSignals[pathId] = signal;
    mTerminatedInitially = signal;
    return hasKernelTerminated(b, mKernelIndex);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasKernelTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasKernelTerminated(BuilderRef b, const unsigned kernel) const {
    // any pipeline input streams are considered produced by the P_{in} vertex.
    if (LLVM_UNLIKELY(kernel == 0)) {
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
 * @brief producerTerminated
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::producerTerminated(BuilderRef b, const unsigned inputPort) const {
    const auto buffer = getInputBufferVertex(inputPort);
    const auto producer = parent(buffer, mBufferGraph);
    return hasKernelTerminated(b, producer);
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
    b->setKernel(mKernel);
    b->CreateStore(getTerminationSignal(b, mKernelIndex), ptr, true);
}

}

#endif // TERMINATION_LOGIC_HPP
