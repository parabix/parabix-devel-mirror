#ifndef IO_ANALYSIS_HPP
#define IO_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernelInputCheckGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::makeKernelInputCheckGraph() const {

    // The root kernel of a partition is resposible for determining how many strides the
    // remaining kernels within the partition will execute. The reason for this is only
    // the root kernel may accept input at a non-Fixed rate or have input rate attributes
    // that may alter the dataflow rates within the partition.

    flat_set<unsigned> knownLinkedRateIds;


    using InputGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, BufferRateData>;

    InputGraph G(num_vertices(mBufferGraph));

    for (auto start = FirstKernel; start <= LastKernel; ) {

        const auto pid = KernelPartitionId[start];
        auto end = start + 1;
        for (; end <= LastKernel; ++end) {
            if (KernelPartitionId[end] != pid) {
                break;
            }
        }

        knownLinkedRateIds.clear();

        auto addCheck = [&](const size_t kernel, const BufferGraph::edge_descriptor e) {
            const BufferRateData & br = mBufferGraph[e];
            const auto streamSet = source(e, mBufferGraph);
            add_edge(kernel, streamSet, br, G);
        };

        for (const auto e : make_iterator_range(in_edges(pid, mPartitioningGraph))) {
            const PartitioningGraphEdge & check = mPartitioningGraph[e];
            addCheck(start, getInput(check.Kernel, check.Port));
        }

        for (auto kernel = start; kernel < end; ++kernel) {


            // If a kernel has non-linear input, we'll have to test the non-linear input
            // ports to determine how many strides are in a particular sub-segment.

            if (mayHaveNonLinearIO(kernel)) {
                for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {

                    // TODO: we should be able to characterize linked I/O ports better to reflect the
                    // potential itermediary states. This would require reasoning about the symbolic
                    // state of the item counts over time for any data patterns.

                    const auto streamSet = source(e, mBufferGraph);
                    const BufferNode & bn = mBufferGraph[streamSet];

                    // Linked rates have equivalent processed/produced/consumed counts throughout the
                    // lifetime of the program. We can avoid redundant checks by ensuring any rate we
                    // assess is unique. However, any port with non-linear I/O could have different
                    // intermediary item counts but despite having linked final item counts.

                    if (!bn.Linear || knownLinkedRateIds.insert(br.LinkedPortId).second) {
                        addCheck(kernel, e);
                    }
                }
            }
        }

        start = end;
    }


}

#endif

}

#endif // IO_ANALYSIS_HPP
