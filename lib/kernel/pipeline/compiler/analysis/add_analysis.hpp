#ifndef ADD_ANALYSIS_HPP
#define ADD_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeAddGraph
 ** ------------------------------------------------------------------------------------------------------------- */
AddGraph PipelineCompiler::makeAddGraph() const {
    // TODO: this should generate formulas to take roundup into account

    // TODO: this doesn't handle fixed I/O rate conversions correctly. E.g., 2 input items to 3 output items.

    AddGraph G(LastStreamSet + 1);
    for (auto i = PipelineInput; i <= PipelineOutput; ++i) {
        int minAddK = 0;
        if (LLVM_LIKELY(in_degree(i, mBufferGraph) > 0)) {
            bool noPrincipal = true;

            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const auto buffer = source(e, mBufferGraph);
                const BufferRateData & br = mBufferGraph[e];
                const Binding & input = br.Binding;

                int k = G[buffer];
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

                add_edge(buffer, i, k, G);
            }

            if (LLVM_LIKELY(noPrincipal)) {
                minAddK = std::numeric_limits<int>::max();
                for (const auto e : make_iterator_range(in_edges(i, G))) {
                    minAddK = std::min<int>(minAddK, G[e]);
                }
                for (const auto e : make_iterator_range(in_edges(i, G))) {
                    G[e] -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(i, G))) {
                    G[e] = 0;
                }
            }

        }

        G[i] = minAddK;

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


            G[buffer] = k;
            add_edge(i, buffer, k, G);
        }
    }

    #if 0
    auto & out = errs();
    out << "digraph AddGraph {\n";
    for (const auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << " (" << (int)G[v] << ")\"];\n";
    }
    for (const auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        const BufferRateData & r = mBufferGraph[edge(s, t, mBufferGraph).first];
        out << "v" << s << " -> v" << t << " [label=\"" << r.Port.Number << ": " << (int)G[e] << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
    #endif

    return G;
}

}

#endif // ADD_ANALYSIS_HPP
