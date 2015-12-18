#ifndef SCHEDULINGPREPASS_H
#define SCHEDULINGPREPASS_H

#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>

namespace pablo {

class PabloFunction;
class PabloBlock;
class Statement;
class PabloAST;

class SchedulingPrePass {
public:
    using Graph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;
    using Map = std::unordered_map<const PabloAST *, Vertex>;
    using ScopeMap = boost::container::flat_map<const PabloBlock *, Statement *>;
    using ReadyPair = std::pair<unsigned, Vertex>;
    using ReadySet = std::vector<ReadyPair>;
public:
    static bool optimize(PabloFunction & function);
protected:
    void schedule(PabloFunction & function);
    void schedule(PabloBlock * const block);
    void resolveNestedUsages(Statement * const root, Statement * const stmt, Graph & G, Map & M, PabloBlock * const block);
    SchedulingPrePass() = default;
private:
    ScopeMap mResolvedScopes;
};


}


#endif // SCHEDULINGPREPASS_H
