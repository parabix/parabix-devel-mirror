#ifndef ADD_ANALYSIS_HPP
#define ADD_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeAddGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::annotateBufferGraphWithAddAttributes() {

    SmallVector<int, 64> transitiveAdd(LastStreamSet - FirstStreamSet + 1);

    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        int minAddK = 0;
        if (LLVM_LIKELY(in_degree(i, mBufferGraph) > 0)) {
            bool noPrincipal = true;
            minAddK = std::numeric_limits<int>::max();
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto streamSet = source(e, mBufferGraph);
                int k = transitiveAdd[streamSet - FirstStreamSet];
                BufferPort & br = mBufferGraph[e];
                k += br.Add;
                k -= br.Truncate;
                minAddK = std::min(minAddK, k);
                if (LLVM_UNLIKELY(br.IsPrincipal)) {
                    minAddK = k;
                    noPrincipal = false;
                    break;
                }
                br.TransitiveAdd = k;
            }

            if (LLVM_LIKELY(noPrincipal)) {
                for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                    BufferPort & br = mBufferGraph[e];
                    br.TransitiveAdd -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                    BufferPort & br = mBufferGraph[e];
                    br.TransitiveAdd = 0;
                }
            }
        }

        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            BufferPort & br = mBufferGraph[e];
            auto k = minAddK;
            k += br.Add;
            k -= br.Truncate;
            br.TransitiveAdd = k;
            const auto streamSet = target(e, mBufferGraph);
            transitiveAdd[streamSet - FirstStreamSet] = k;
        }
    }

    // Scan through the I/O for each kernel to see if some transitive
    // add relationship imposes an overflow requirement on a buffer


    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        int maxK = 0;
        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const BufferPort & rate = mBufferGraph[e];
            maxK = std::max(maxK, rate.TransitiveAdd);
        }
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferPort & rate = mBufferGraph[e];
            maxK = std::max(maxK, rate.TransitiveAdd);
        }
        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            bn.MaxAdd = std::max<unsigned>(bn.MaxAdd, maxK);
        }
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            bn.MaxAdd = std::max<unsigned>(bn.MaxAdd, maxK);
        }
    }



}

}

#endif // ADD_ANALYSIS_HPP
