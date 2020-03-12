#ifndef INPUT_TRUNCATION_ANALYSIS_HPP
#define INPUT_TRUNCATION_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

void PipelineAnalysis::makeInputTruncationGraph() {

    mInputTruncationGraph = InputTruncationGraph(LastKernel + 1);

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);

            const BufferNode & bn = mBufferGraph[streamSet];
            const StreamSetBuffer * const buffer = bn.Buffer;

            // If this streamset has 0 streams and is not extensible, it exists only to have a
            // produced/processed count. Ignore it even if its a non-local buffer.
            if (LLVM_UNLIKELY(buffer->isEmptySet())) {
                continue;
            }

            const BufferRateData & br = mBufferGraph[e];

            const Binding & input = br.Binding;
            const ProcessingRate & rate = input.getRate();

            if (LLVM_LIKELY(rate.isFixed())) {

                // TODO: if this kernel is the *sole* consumer of this buffer, we do not need a
                // temporary buffer to copy to and can directly clear the input data of the
                // original buffer.

                InputTruncation it(br.Port);

                if (out_degree(streamSet, mBufferGraph) == 1) {
                    it.CreateTemporaryBuffer = false;
                }

                add_edge(kernel, kernel, it, mInputTruncationGraph);

            }

        }
    }
}

}

#endif // INPUT_TRUNCATION_ANALYSIS_HPP
