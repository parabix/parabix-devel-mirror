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
    for (auto kernel = PipelineInput; kernel <= PipelineOutput; ++kernel) {
        int minAddK = 0;
        if (LLVM_LIKELY(in_degree(kernel, mStreamGraph) > 0)) {
            bool noPrincipal = true;

            for (const auto e : make_iterator_range(in_edges(kernel, mStreamGraph))) {
                const auto binding = source(e, mBufferGraph);
                const RelationshipNode & rn = mStreamGraph[binding];
                assert (rn.Type == RelationshipNode::IsBinding);
                const Binding & input = rn.Binding;

                const auto f = first_in_edge(binding, mStreamGraph);
                const auto streamSet = source(f, mStreamGraph);
                assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
                assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
                assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

                int k = G[streamSet];
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

                add_edge(streamSet, kernel, k, G);
            }

            if (LLVM_LIKELY(noPrincipal)) {
                minAddK = std::numeric_limits<int>::max();
                for (const auto e : make_iterator_range(in_edges(kernel, G))) {
                    minAddK = std::min<int>(minAddK, G[e]);
                }
                for (const auto e : make_iterator_range(in_edges(kernel, G))) {
                    G[e] -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(kernel, G))) {
                    G[e] = 0;
                }
            }

        }

        G[kernel] = minAddK;

        for (const auto e : make_iterator_range(out_edges(kernel, mStreamGraph))) {

            const auto binding = target(e, mBufferGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const Binding & output = rn.Binding;

            assert (rn.Type == RelationshipNode::IsBinding);
            const auto f = out_edge(binding, mStreamGraph);
            assert (mStreamGraph[f].Reason != ReasonType::Reference);
            const auto streamSet = target(f, mStreamGraph);
            assert (streamSet >= FirstStreamSet && streamSet <= LastStreamSet);

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

            G[streamSet] = k;
            add_edge(kernel, streamSet, k, G);
        }
    }

    #if 1
    auto & out = errs();
    out << "digraph AddGraph {\n";
    for (const auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << " (" << (int)G[v] << ")\"];\n";
    }
    for (const auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << " [label=\"" << (int)G[e] << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
    #endif

    return G;
}

}

#endif // ADD_ANALYSIS_HPP
