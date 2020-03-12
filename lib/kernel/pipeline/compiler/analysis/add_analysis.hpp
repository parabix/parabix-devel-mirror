#ifndef ADD_ANALYSIS_HPP
#define ADD_ANALYSIS_HPP

#include "pipeline_analysis.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeAddGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::makeAddGraph() {
    // TODO: this should generate formulas to take roundup into account

    // TODO: this doesn't handle fixed I/O rate conversions correctly. E.g., 2 input items to 3 output items.

    mAddGraph = AddGraph(LastStreamSet + 1);

    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        int minAddK = 0;
        if (LLVM_LIKELY(in_degree(i, mBufferGraph) > 0)) {
            bool noPrincipal = true;

            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto buffer = source(e, mBufferGraph);
                const BufferRateData & br = mBufferGraph[e];
                const Binding & input = br.Binding;

                int k = mAddGraph[buffer];
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
                if (LLVM_UNLIKELY(isPrincipal)) {
                    minAddK = k;
                    noPrincipal = false;
                }

                add_edge(buffer, i, k, mAddGraph);
            }

            if (LLVM_LIKELY(noPrincipal)) {
                minAddK = std::numeric_limits<int>::max();
                for (const auto e : make_iterator_range(in_edges(i, mAddGraph))) {
                    const int k = mAddGraph[e];
                    minAddK = std::min(minAddK, k);
                }
                for (const auto e : make_iterator_range(in_edges(i, mAddGraph))) {
                    mAddGraph[e] -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(i, mAddGraph))) {
                    mAddGraph[e] = 0;
                }
            }

        }

        mAddGraph[i] = minAddK;

        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto buffer = target(e, mBufferGraph);
            const BufferRateData & br = mBufferGraph[e];
            const Binding & output = br.Binding;

            int k = minAddK;
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


            mAddGraph[buffer] = k;
            add_edge(i, buffer, k, mAddGraph);
        }
    }

    #if 0
    auto & out = errs();
    out << "digraph AddGraph {\n";
    for (const auto v : make_iterator_range(vertices(mAddGraph))) {
        out << "v" << v << " [label=\"" << v << " (" << (int)mAddGraph[v] << ")\"];\n";
    }
    for (const auto e : make_iterator_range(edges(mAddGraph))) {
        const auto s = source(e, mAddGraph);
        const auto t = target(e, mAddGraph);

        graph_traits<BufferGraph>::edge_descriptor f;
        bool found;
        std::tie(f, found) = edge(s, t, mBufferGraph); assert (found);
        const BufferRateData & br = mBufferGraph[f];

        out << "v" << s << " -> v" << t << " [label=\"";
        if (br.Port.Type == PortType::Input) {
            out << "I";
        } else {
            out << "O";
        }
        out << br.Port.Number;

        const int k = mAddGraph[e];
        if (k > 0) {
            out << " +" << k;
        } else if (k < 0) {
            out << " " << k;
        }
        out << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
    #endif

}

}

#endif // ADD_ANALYSIS_HPP
