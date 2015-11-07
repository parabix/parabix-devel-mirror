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
    void resolveScopes(PabloFunction & function);
    void resolveScopes(PabloBlock &block);
    void processScopes(PabloFunction & function);
    void processScopes(PabloFunction & function, PabloBlock & block);
    void processScope(PabloFunction &, PabloBlock & block);
    void summarizeAST(PabloBlock & block, Graph & G, Map & M) const;
    static void summarizeGraph(const PabloBlock & block, Graph & G, std::vector<Vertex> & mapping, Map &M);
    void resolveUsages(const Vertex u, PabloAST * expr, PabloBlock & block, Graph & G, Map & M, const Statement * const ignoreIfThis = nullptr) const;
    void redistributeAST(const PabloBlock & block, Graph & G, Map & M) const;
    void rewriteAST(PabloBlock & block, Graph & G);
    static PabloAST * createTree(PabloBlock & block, const Vertex u, Graph & G);
    static Vertex getSummaryVertex(PabloAST * expr, Graph & G, Map & M, const PabloBlock & block);
    static Vertex addSummaryVertex(const PabloAST::ClassTypeId typeId, Graph & G);
private:
    ScopeMap mResolvedScopes;
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
