#ifndef IO_ANALYSIS_HPP
#define IO_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernelInputCheckGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makeKernelIOGraph() {

#if 0
    // The root kernel of a partition is resposible for determining how many strides the
    // remaining kernels within the partition will execute. The reason for this is only
    // the root kernel may accept input at a non-Fixed rate or have input rate attributes
    // that may alter the dataflow rates within the partition.

    flat_set<unsigned> knownGlobalRateIds;

    mIOCheckGraph = IOCheckGraph(num_vertices(mBufferGraph));

    for (auto start = FirstKernel; start <= LastKernel; ) {

        const auto pid = KernelPartitionId[start];
        auto end = start + 1;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != pid) {
                break;
            }
        }

        // Linked rates have equivalent processed/produced/consumed counts throughout the
        // lifetime of the program. We can avoid redundant checks by ensuring any rate we
        // assess is unique. However, any port with non-linear I/O could have different
        // intermediary item counts but despite having linked final item counts.

//        auto untestedPortId = [&](const BufferGraph::edge_descriptor e) {
//            const BufferRateData & br = mBufferGraph[e];
//            return knownGlobalRateIds.insert(br.GlobalPortId).second;
//        };


        auto addInputCheck = [&](const size_t kernel, const BufferGraph::edge_descriptor e) {
            const BufferRateData & br = mBufferGraph[e];
            if (knownGlobalRateIds.insert(br.GlobalPortId).second) {
                const auto streamSet = source(e, mBufferGraph);
                add_edge(kernel, streamSet, br, mIOCheckGraph);
            }
        };

        auto addOutputCheck = [&](const size_t kernel, const BufferGraph::edge_descriptor e) {
            const BufferRateData & br = mBufferGraph[e];
            if (knownGlobalRateIds.insert(br.GlobalPortId).second) {
                const auto streamSet = target(e, mBufferGraph);
                add_edge(streamSet, kernel, br, mIOCheckGraph);
            }
        };

        knownGlobalRateIds.clear();

        for (const auto e : make_iterator_range(in_edges(pid, mPartitioningGraph))) {
            const PartitioningGraphEdge & check = mPartitioningGraph[e];
            const auto f = getInput(check.Kernel, check.Port);
            const BufferRateData & br = mBufferGraph[f];
            if (knownGlobalRateIds.insert(br.GlobalPortId).second) {
                const auto streamSet = source(e, mBufferGraph);
                add_edge(start, streamSet, br, mIOCheckGraph);
            }


            // IOCheckEdge



            addInputCheck(start, check.Kernel, getInput(check.Kernel, check.Port));
        }

        for (auto kernel = start + 1U; kernel < end; ++kernel) {

            // If a kernel has non-linear input, we'll have to test the non-linear input
            // ports to determine how many strides are in a particular sub-segment.

            // TODO: if an input has an Add(k), we'll need to check the input stream.
            // We can only directly accept the partition # of strides if there is no
            // potential change of input even in the final stride even if the kernel has
            // strictly linear input.


            for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
                const auto streamSet = source(e, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (bn.NonLocal || !bn.Linear) {
                    addInputCheck(kernel, e);
                }
            }

            for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
                const auto streamSet = target(e, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];
                if (bn.isOwned() && (bn.NonLocal || !bn.Linear)) {
                    addOutputCheck(kernel, e);
                }
            }


        }

        start = end;
    }
#endif
}

}

#endif // IO_ANALYSIS_HPP
