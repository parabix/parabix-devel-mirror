#ifndef INPUT_TRUNCATION_ANALYSIS_HPP
#define INPUT_TRUNCATION_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

void PipelineAnalysis::makeInputTruncationGraph() {

    mInputTruncationGraph = InputTruncationGraph(LastStreamSet + 1);

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        const auto partitionId = KernelPartitionId[kernel];

        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);

            const BufferNode & bn = mBufferGraph[streamSet];
            if (LLVM_UNLIKELY(bn.NonLocal)) {

                const StreamSetBuffer * const buffer = bn.Buffer;

                // If this streamset has 0 streams and is not extensible, it exists only to have a
                // produced/processed count. Ignore it even if its a non-local buffer.
                if (LLVM_UNLIKELY(buffer->isEmptySet())) {
                    continue;
                }

                // A non-local buffer could still be produced by a kernel within the same partition
                // as the current kernel.
                const auto producer = parent(streamSet, mBufferGraph);
                if (KernelPartitionId[producer] == partitionId) {
                    continue;
                }

                // TODO: if this kernel is the *sole* consumer of this buffer, we do not need a
                // temporary buffer to copy to and can directly clear the input data of the
                // original buffer.

            }
    }

    }
}

}

#endif // INPUT_TRUNCATION_ANALYSIS_HPP
