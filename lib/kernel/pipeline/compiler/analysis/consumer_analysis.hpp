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
        const BufferRateData & br = mBufferGraph[pe];
        const auto producer = source(pe, mBufferGraph);
        add_edge(producer, streamSet, ConsumerEdge{br.Port, 0, ConsumerEdge::None}, mConsumerGraph);

        // If we have no consumers, we do not want to update the consumer count on exit
        // as we would then have to retain a scalar for it. Initially
        auto flags = ConsumerEdge::None;
        auto lastConsumer = producer;

        if (LLVM_LIKELY(out_degree(streamSet, mBufferGraph) > 0)) {
            unsigned index = 0;
            // flag the production rate as ignorable by inserting it upfront
            observedGlobalPortIds.insert(br.GlobalPortId);
            for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
                const BufferRateData & br = mBufferGraph[ce];
                const auto consumer = target(ce, mBufferGraph);
                lastConsumer = std::max(lastConsumer, consumer);
                // check if any consumer has a rate we have not yet observed
                if (observedGlobalPortIds.insert(br.GlobalPortId).second) {
                    add_edge(streamSet, consumer, ConsumerEdge{br.Port, ++index, ConsumerEdge::UpdatePhi}, mConsumerGraph);
                    flags = ConsumerEdge::WriteFinalCount;
                }
            }
            observedGlobalPortIds.clear();
        }

        // Although we may already know the final consumed item count prior
        // to executing the last consumer, we need to defer writing the final
        // consumed item count until the very last consumer reads the data.
        // Make sure the final consumer is told to write the count out.

        ConsumerGraph::edge_descriptor e;
        bool exists;
        std::tie(e, exists) = edge(streamSet, lastConsumer, mConsumerGraph);

        if (exists) {
            mConsumerGraph[e].Flags = ConsumerEdge::UpdateAndWrite;
        } else {
            add_edge(streamSet, lastConsumer, ConsumerEdge{br.Port, 0, flags}, mConsumerGraph);
        }
    }

}

}

#endif // CONSUMER_ANALYSIS_HPP
