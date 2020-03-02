#ifndef CONSUMER_ANALYSIS_HPP
#define CONSUMER_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

// TODO: rework consumer logic for external I/O.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    ConsumerGraph G(LastStreamSet + 1);

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        // copy the producing edge
        const auto pe = in_edge(streamSet, mBufferGraph);
        const BufferRateData & br = mBufferGraph[pe];
        const auto producer = source(pe, mBufferGraph);
        const auto globalPortId = br.GlobalPortId;
        add_edge(producer, streamSet, ConsumerEdge{br.Port, 0}, G);
        unsigned index = 0;
        for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[ce];
            if (br.GlobalPortId != globalPortId) {
                add_edge(streamSet, target(ce, mBufferGraph), ConsumerEdge{br.Port, ++index}, G);
            }
        }
    }
    return G;
}

}

#endif // CONSUMER_ANALYSIS_HPP
