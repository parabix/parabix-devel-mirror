#ifndef USEANALYSIS_H
#define USEANALYSIS_H

#include <pablo/codegenstate.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <vector>
#include <set>

namespace pablo {

using namespace boost;

class UseAnalysis {
    typedef adjacency_list<hash_setS, vecS, bidirectionalS, property<vertex_name_t, PabloAST*>> UseDefGraph;
    typedef graph_traits<UseDefGraph>::vertex_descriptor Vertex;
    typedef graph_traits<UseDefGraph>::vertex_iterator VertexIterator;
    typedef graph_traits<UseDefGraph>::in_edge_iterator InEdgeIterator;
    typedef graph_traits<UseDefGraph>::out_edge_iterator OutEdgeIterator;
    typedef std::unordered_map<const PabloAST*, Vertex> UseDefMap;
    typedef std::set<Statement *> PredecessorSet;
public:
    static bool optimize(PabloBlock & block);
private:
    void cse(PabloBlock & block);
    Statement * findInsertionPointFor(const Vertex v, PabloBlock & block);
    Statement * findLastStatement(const PredecessorSet & predecessors, StatementList & statements);
    void dce();
    void gatherUseDefInformation(const StatementList & statements);
    void gatherUseDefInformation(const Vertex parent, const StatementList & statements);
    void gatherUseDefInformation(const Vertex parent, const PabloAST * expr);
    Vertex find(const PabloAST * const node);
private:
    UseDefGraph       mUseDefGraph;
    UseDefMap         mUseDefMap;
};

}

#endif // USEANALYSIS_H
