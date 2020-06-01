#ifndef CONSUMER_ANALYSIS_HPP
#define CONSUMER_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

// TODO: rework consumer logic for external I/O.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makeConsumerGraph() {

    mConsumerGraph = ConsumerGraph(LastStreamSet + 1);

    flat_set<unsigned> observedGlobalPortIds;

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        // copy the producing edge
        const auto pe = in_edge(streamSet, mBufferGraph);
        const BufferPort & br = mBufferGraph[pe];
        const auto producer = source(pe, mBufferGraph);
        add_edge(producer, streamSet, ConsumerEdge{br.Port, 0, ConsumerEdge::None}, mConsumerGraph);

        // If we have no consumers, we do not want to update the consumer count on exit
        // as we would then have to retain a scalar for it.
        if (LLVM_UNLIKELY(out_degree(streamSet, mBufferGraph) == 0)) {
            continue;
        }

        auto lastConsumer = PipelineInput;
        auto index = 0U;
        // flag the production rate as ignorable by inserting it upfront
        observedGlobalPortIds.insert(br.GlobalPortId);
        for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const BufferPort & br = mBufferGraph[ce];
            const auto consumer = target(ce, mBufferGraph);
            if (LLVM_UNLIKELY(consumer == PipelineOutput && producer != PipelineInput)) {
                continue;
            }
            // check if any consumer has a rate we have not yet observed
            lastConsumer = std::max<unsigned>(lastConsumer, consumer);
            #ifndef TEST_ALL_CONSUMERS
            if (observedGlobalPortIds.insert(br.GlobalPortId).second) {
            #endif
                add_edge(streamSet, consumer, ConsumerEdge{br.Port, ++index, ConsumerEdge::UpdatePhi}, mConsumerGraph);
            #ifndef TEST_ALL_CONSUMERS
            }
            #endif
        }
        observedGlobalPortIds.clear();

        // Is the production rate guaranteed to be equivalent to all of the
        // consumption rates?
        if (LLVM_UNLIKELY(lastConsumer == PipelineInput)) {
            continue;
        }

        // Although we may already know the final consumed item count prior
        // to executing the last consumer, we need to defer writing the final
        // consumed item count until the very last consumer reads the data.

        ConsumerGraph::edge_descriptor e;
        bool exists;
        std::tie(e, exists) = edge(streamSet, lastConsumer, mConsumerGraph);
        const auto flags = ConsumerEdge::WriteConsumedCount;
        if (exists) {
            ConsumerEdge & cn = mConsumerGraph[e];
            cn.Flags |= flags;
        } else {
            add_edge(streamSet, lastConsumer, ConsumerEdge{br.Port, 0, flags}, mConsumerGraph);
        }
    }

#if 0

    auto & out = errs();

    out << "digraph \"ConsumerGraph_" << mPipelineKernel->getName() << "\" {\n";
    for (auto v : make_iterator_range(vertices(mConsumerGraph))) {
        out << "v" << v << " [label=\"" << v << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(mConsumerGraph))) {
        const auto s = source(e, mConsumerGraph);
        const auto t = target(e, mConsumerGraph);
        out << "v" << s << " -> v" << t <<
               " [label=\"";
        const ConsumerEdge & c = mConsumerGraph[e];
        if (c.Flags & ConsumerEdge::UpdatePhi) {
            out << 'U';
        }
        if (c.Flags & ConsumerEdge::WriteConsumedCount) {
            out << 'W';
        }
        if (c.Flags & ConsumerEdge::UpdateExternalCount) {
            out << 'E';
        }
        out << "\"];\n";
    }

    out << "}\n\n";
    out.flush();

#endif

}

}

#endif // CONSUMER_ANALYSIS_HPP
