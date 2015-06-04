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
    using IndependentSetGraph = boost::adjacency_list<boost::hash_setS, boost::listS, boost::undirectedS, std::pair<unsigned, MultiplexSetGraph::vertex_descriptor>>;
    using ChosenSets = std::vector<MultiplexSetGraph::vertex_descriptor>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using Advances = std::vector<Advance *>;

    using RNG = std::mt19937;
    using RNGDistribution = std::uniform_int_distribution<RNG::result_type>;

    using Vertex = ConstraintGraph::vertex_descriptor;

    using IndependentSet = std::vector<Vertex>;

public:
    static bool optimize(const std::vector<Var *> & input, PabloBlock & entry);
protected:
    void initialize(const std::vector<Var *> & vars, const PabloBlock & entry);
    void characterize(PabloBlock & entry);
    bool notTransitivelyDependant(const PathGraph::vertex_descriptor i, const PathGraph::vertex_descriptor j) const;
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
    DdNode * Zero() const;
    DdNode * One() const;
    bool isZero(DdNode * const x) const;
    DdNode * And(DdNode * const x, DdNode * const y);
    DdNode * Or(DdNode * const x, DdNode * const y);
    DdNode * Xor(DdNode * const x, DdNode * const y);
    DdNode * Not(DdNode * x) const;
    DdNode * Ite(DdNode * const x, DdNode * const y, DdNode * const z);
    bool noSatisfyingAssignment(DdNode * const x);
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
