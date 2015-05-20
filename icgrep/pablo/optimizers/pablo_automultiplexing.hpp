#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <unordered_map>
#include <pablo/analysis/bdd/bdd.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/edge_list.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <stdint.h>

namespace pablo {

class AutoMultiplexing {

    using CharacterizationMap = boost::container::flat_map<PabloAST *, bdd::BDD>;
    using ConstraintGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::directedS>;
    using PathGraph = boost::adjacency_matrix<boost::undirectedS>;
    using SubsetGraph = boost::edge_list<std::pair<unsigned, unsigned>>;
    using IndexMap = boost::container::flat_map<PabloAST *, unsigned>;

public:
    static void optimize(PabloBlock & block);
protected:
    bdd::Engine AutoMultiplexing::initialize(const std::vector<Var *> & vars, const PabloBlock & entry);


private:
    AutoMultiplexing();

    CharacterizationMap     mCharacterizationMap;
    PathGraph               mPathGraph;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;

    bool                    mTestForConjunctionContradictions;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
