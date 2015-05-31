#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/edge_list.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class AutoMultiplexing {

    using CharacterizationMap = boost::container::flat_map<const PabloAST *, DdNode *>;
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
    void initialize(const std::vector<Var *> & vars, const PabloBlock & entry);
    void characterize(PabloBlock & entry);    
    void createMultiplexSetGraph();
    bool generateMultiplexSets(RNG & rng);    
    void addMultiplexSet(const IndependentSet & set);
    void approxMaxWeightIndependentSet(RNG & rng);
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets() const;
    void topologicalSort(PabloBlock & entry) const;
    inline AutoMultiplexing()
    : mPathGraph(0) {
    }
private:
    bool isZero(DdNode * x) const;
    DdNode * And(DdNode * x, DdNode * y);
    DdNode * Or(DdNode * x, DdNode * y);
    DdNode * Xor(DdNode * x, DdNode * y);
    DdNode * Not(DdNode * x);
    DdNode * Ite(DdNode * x, DdNode * y, DdNode * z);
    bool noSatisfyingAssignment(DdNode * x);
    void shutdown();
private:
    DdManager *             mManager;
    CharacterizationMap     mCharacterizationMap;
    PathGraph               mPathGraph;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;
    Advances                mAdvance;
    MultiplexSetGraph       mMultiplexSetGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
