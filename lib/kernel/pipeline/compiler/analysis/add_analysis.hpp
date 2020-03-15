#ifndef ADD_ANALYSIS_HPP
#define ADD_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeAddGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::annotateBufferGraphWithAddAttributes() {
    // TODO: this should generate formulas to take roundup into account

    // TODO: this doesn't handle fixed I/O rate conversions correctly. E.g., 2 input items to 3 output items.

    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        int minAddK = 0;
        if (LLVM_LIKELY(in_degree(i, mBufferGraph) > 0)) {
            bool noPrincipal = true;
            minAddK = std::numeric_limits<int>::max();
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto streamSet = source(e, mBufferGraph);
                const BufferNode & bn = mBufferGraph[streamSet];

                BufferRateData & br = mBufferGraph[e];
                const Binding & input = br.Binding;

                int k = bn.Add;
                bool isPrincipal = false;
                for (const Attribute & attr : input.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::Add:
                            k += attr.amount();
                            break;
                        case AttrId::Truncate:
                            k -= attr.amount();
                            break;
                        case AttrId::Principal:
                            isPrincipal = true;
                            break;
                        default: break;
                    }
                }
                minAddK = std::min(minAddK, k);
                if (LLVM_UNLIKELY(isPrincipal)) {
                    minAddK = k;
                    noPrincipal = false;
                    break;
                }
                br.Add = k;
            }

            if (LLVM_LIKELY(noPrincipal)) {
                for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                    BufferRateData & br = mBufferGraph[e];
                    br.Add -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                    BufferRateData & br = mBufferGraph[e];
                    br.Add = 0;
                }
            }
        }


        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            BufferRateData & br = mBufferGraph[e];
            const Binding & output = br.Binding;

            auto k = minAddK;
            for (const Attribute & attr : output.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Add:
                        k += attr.amount();
                        break;
                    case AttrId::Truncate:
                        k -= attr.amount();
                        break;
                    default: break;
                }
            }
            br.Add = k;

            const auto streamSet = target(e, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            bn.Add = k;
        }
    }

    // Scan through the I/O for each kernel to see if some transitive
    // add relationship imposes an overflow requirement on a buffer


    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {
        int maxK = 0;
        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const BufferRateData & rate = mBufferGraph[e];
            maxK = std::max(maxK, rate.Add);
        }
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferRateData & rate = mBufferGraph[e];
            maxK = std::max(maxK, rate.Add);
        }
        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const auto streamSet = source(e, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            bn.Add = std::max<unsigned>(bn.Add, maxK);
        }
        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const auto streamSet = target(e, mBufferGraph);
            BufferNode & bn = mBufferGraph[streamSet];
            bn.Add = std::max<unsigned>(bn.Add, maxK);
        }
    }



}

}

#endif // ADD_ANALYSIS_HPP
