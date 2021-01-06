#ifndef BUFFER_SIZE_ANALYSIS_HPP
#define BUFFER_SIZE_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

#ifdef PERMIT_BUFFER_MEMORY_REUSE

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineBufferLayout
 *
 * Given our buffer graph, we want to identify the best placement to maximize memory reuse byway of minimizing
 * the total memory required. Although this assumes that the memory-aware scheduling algorithm was first called,
 * it does not actually use any data from it. The reason for this disconnection is to enable us to explore
 * the impact of static memory allocation independent of the chosen scheduling algorithm.
 *
 * The following is a genetic algorithm that adapts Gergov's incremental 2-approx from "Algorithms for
 * Compile time memory optimization" (1999) to generate some initial layout candidates then attempts
 * to refine them.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determineBufferLayout() {

    // Construct the weighted interval graph for our local streamsets

    struct Trail {
        double Weight;
        double Pheromone;
    };

    using IntervalGraph = adjacency_list<hash_setS, vecS, undirectedS, no_property, Trail>;

    const auto n = LastStreamSet - FirstStreamSet + 1U;

    IntervalGraph G(n);
    std::vector<size_t> weight(n, 0);

    BEGIN_SCOPED_REGION

    const auto firstKernel = out_degree(PipelineInput, mBufferGraph) == 0 ? FirstKernel : PipelineInput;
    const auto lastKernel = in_degree(PipelineOutput, mBufferGraph) == 0 ? LastKernel : PipelineOutput;

    // The buffer graph is constructed in order of how the compiler will structure the pipeline program.
    // We compute the interval graph based purely on the invocation order entailed by it.

    std::vector<int> remaining(n, 0);

    for (auto kernel = firstKernel; kernel <= lastKernel; ++kernel) {

        for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            if (bn.Locality == BufferLocality::Local) {
                // determine the number of bytes this streamset requires
//                const BufferPort & producerRate = mBufferGraph[output];
//                const Binding & outputRate = producerRate.Binding;
//                ConstantInt * const typeSize = cast<ConstantInt>(ConstantExpr::getSizeOf(outputRate.getType()));
//                const auto typeWidth = typeSize->getLimitedValue();
//                const Kernel * const kernelObj = getKernel(kernel);



                MaximumNumOfStrides[kernel];




                // record how many consumers exist before the streamset memory can be reused
                remaining[streamSet - FirstStreamSet] = out_degree(streamSet, mBufferGraph);
            }
        }

        for (unsigned i = 1; i < n; ++i) {
            if (remaining[i] > 0) {
                for (unsigned j = 0; j < i; ++j) {
                    if (remaining[j] > 0) {
                        add_edge(i, j, G);
                    }
                }
            }
        }

        for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            // NOTE: remaining may go below 0 for non local streamsets
            remaining[streamSet - FirstStreamSet]--;
        }



    }

    END_SCOPED_REGION








}

#else // #ifndef PERMIT_BUFFER_MEMORY_REUSE

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineBufferLayout
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::determineBufferLayout() {


}

#endif

}

#endif // BUFFER_SIZE_ANALYSIS_HPP
