#ifndef USEANALYSIS_H
#define USEANALYSIS_H

#include <pablo/codegenstate.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace pablo {

using namespace boost;

class UseAnalysis
{
    typedef adjacency_list<hash_setS, vecS, directedS, property<vertex_name_t, PabloAST*>> UseDefGraph;
    typedef UseDefGraph::vertex_descriptor Vertex;
    typedef std::unordered_map<PabloAST*, Vertex> UseDefMap;
public:
    static void optimize(PabloBlock & block);
private:
    void traverse(const Vertex parent, StatementList & statements);
    void traverse(const Vertex parent, PabloAST * expr);
    Vertex find(const PabloAST * const node);
    UseAnalysis();
private:
    UseDefGraph       mUseDefGraph;
    UseDefMap         mUseDefMap;
    const Vertex      mRoot;
};

}

#endif // USEANALYSIS_H
