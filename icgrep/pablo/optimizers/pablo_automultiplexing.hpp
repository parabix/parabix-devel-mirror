#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <queue>
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

    using CharacterizationMap = boost::container::flat_map<const PabloAST *, bdd::BDD>;
    using ConstraintGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using PathGraph = boost::adjacency_matrix<boost::undirectedS>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using IndependentSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS, unsigned>;
    using SubsetGraph = std::vector<std::pair<MultiplexSetGraph::vertex_descriptor, MultiplexSetGraph::vertex_descriptor>>;
    using Advances = std::vector<Advance *>;
    using TopologicalSortGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, Statement *>;
    using TopologicalSortQueue = std::queue<TopologicalSortGraph::vertex_descriptor>;
    using TopologicalSortMap = boost::container::flat_map<PabloAST *, TopologicalSortGraph::vertex_descriptor>;

    using RNG = std::mt19937;
    using RNGDistribution = std::uniform_int_distribution<RNG::result_type>;

    using Vertex = ConstraintGraph::vertex_descriptor;

    using IndependentSet = std::vector<Vertex>;

public:
    static void optimize(const std::vector<Var *> & input, PabloBlock & entry);
protected:
    bdd::Engine initialize(const std::vector<Var *> & vars, const PabloBlock & entry);
    void characterize(bdd::Engine & engine, PabloBlock &entry);
    void createMultiplexSetGraph();
    bool generateMultiplexSets(RNG & rng);    
    void addMultiplexSet(const IndependentSet & set);
    void approxMaxWeightIndependentSet(RNG & rng);
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets();
    void topologicalSort(PabloBlock & entry) const;
    void topologicalSort(TopologicalSortGraph & G, TopologicalSortQueue & Q, TopologicalSortMap & M, Statement * ip, Statement * first) const;

private:
    AutoMultiplexing();

    CharacterizationMap     mCharacterizationMap;
    PathGraph               mPathGraph;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;
    Advances                mAdvance;
    MultiplexSetGraph       mMultiplexSetGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
