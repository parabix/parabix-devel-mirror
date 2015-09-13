#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>

namespace pablo {

class BooleanReassociationPass {
public:
    using VertexData = std::pair<PabloAST::ClassTypeId, PabloAST *>;
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexData, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;
    using Map = std::unordered_map<const PabloAST *, Vertex>;

    static bool optimize(PabloFunction & function);
protected:
    inline BooleanReassociationPass() {}
    void resolveScopes(PabloFunction & function);
    void resolveScopes(PabloBlock &block);
    void processScopes(PabloFunction & function);
    void processScopes(PabloFunction & function, PabloBlock & block);
    void processScope(PabloFunction & function, PabloBlock & block);
    void summarizeAST(PabloBlock & block, Graph & G) const;
    static void summarizeGraph(Graph & G, std::vector<Vertex> & mapping);
    void resolveUsages(const Vertex u, PabloAST * expr, PabloBlock & block, Graph & G, Map & M, Statement * ignoreIfThis = nullptr) const;
    void redistributeAST(const PabloBlock & block, Graph & G) const;
public:
    static bool isOptimizable(const VertexData & data);
    static bool isMutable(const VertexData & data, const PabloBlock &block);
    static bool isNonEscaping(const VertexData & data);
    static bool isSameType(const VertexData & data1, const VertexData & data2);
private:
    boost::container::flat_map<PabloBlock *, Statement *> mResolvedScopes;
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
