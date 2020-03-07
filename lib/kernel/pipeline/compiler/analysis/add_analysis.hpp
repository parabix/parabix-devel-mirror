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

                int k = mAddGraph[streamSet];
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

                add_edge(streamSet, kernel, k, mAddGraph);
            }

            if (LLVM_LIKELY(noPrincipal)) {
                minAddK = std::numeric_limits<int>::max();
                for (const auto e : make_iterator_range(in_edges(kernel, mAddGraph))) {
                    minAddK = std::min<int>(minAddK, mAddGraph[e]);
                }
                for (const auto e : make_iterator_range(in_edges(kernel, mAddGraph))) {
                    mAddGraph[e] -= minAddK;
                }
            } else {
                for (const auto e : make_iterator_range(in_edges(kernel, mAddGraph))) {
                    mAddGraph[e] = 0;
                }
            }

        }

        mAddGraph[kernel] = minAddK;

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

            mAddGraph[streamSet] = k;
            add_edge(kernel, streamSet, k, mAddGraph);
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
        out << "v" << s << " -> v" << t << " [label=\"" << (int)mAddGraph[e] << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
    #endif

}

}

#endif // ADD_ANALYSIS_HPP
