#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace pablo {

class BooleanReassociationPass {
public:
    using VertexData = std::pair<PabloAST::ClassTypeId, PabloAST *>;
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexData, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;
    using Map = std::unordered_map<const PabloAST *, Vertex>;
    using ScopeMap = boost::container::flat_map<const PabloBlock *, Statement *>;
    static bool optimize(PabloFunction & function);
protected:
    inline BooleanReassociationPass() {}
    void processScopes(PabloFunction & function);
    PabloBlock * processScopes(PabloFunction & f, PabloBlock * const block);
    PabloBlock * processScope(PabloFunction & f, PabloBlock * const block);
    void summarizeAST(PabloBlock * const block, Graph & G) const;
    static void summarizeGraph(Graph & G, std::vector<Vertex> & mapping);
    void resolveNestedUsages(const Vertex u, PabloAST * expr, PabloBlock * const block, Graph & G, Map & M, const Statement * const ignoreIfThis = nullptr) const;
    void redistributeAST(Graph & G) const;
    PabloBlock * rewriteAST(PabloFunction & f, PabloBlock * const block, Graph & G);
    static PabloAST * createTree(const PabloBlock * const block, PabloBlock * const newScope, const Vertex u, Graph & G);
    static Vertex getSummaryVertex(PabloAST * expr, Graph & G, Map & M, const PabloBlock * const block);
    static Vertex addSummaryVertex(const PabloAST::ClassTypeId typeId, Graph & G);
private:
    ScopeMap mResolvedScopes;
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
