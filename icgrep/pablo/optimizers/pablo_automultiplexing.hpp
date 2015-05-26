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
#include <random>
#include <stdint.h>

namespace pablo {

class AutoMultiplexing {

    using CharacterizationMap = boost::container::flat_map<PabloAST *, bdd::BDD>;
    using ConstraintGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::directedS>;
    using PathGraph = boost::adjacency_matrix<boost::undirectedS>;
    using SubsetGraph = boost::edge_list<std::pair<unsigned, unsigned>>;
    using MappingGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using IndependentSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS, unsigned>;
    using Advances = std::vector<Advance *>;

    using RNG = std::mt19937;
    using RNGDistribution = std::uniform_int_distribution<RNG::result_type>;

    using Vertex = ConstraintGraph::vertex_descriptor;

    using IndependentSet = std::vector<Vertex>;

public:
    static void optimize(PabloBlock & block);
protected:
    bdd::Engine initialize(const std::vector<Var *> & vars, const PabloBlock & entry);
    void characterize(bdd::Engine & engine, const PabloBlock & entry);
    bool generateMultiplexSets(RNG & rng);
    void addMultiplexSet(const IndependentSet & set);
    void approxMaxWeightIndependentSet(RNG & rng);
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets();
    void topologicalSort();

private:
    AutoMultiplexing();

    CharacterizationMap     mCharacterizationMap;
    PathGraph               mPathGraph;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;
    Advances                mAdvance;
    MappingGraph            mMappingGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
