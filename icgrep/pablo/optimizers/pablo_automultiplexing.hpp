#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/edge_list.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class AutoMultiplexing {

    using CharacterizationMap = boost::container::flat_map<const PabloAST *, DdNode *>;
    using ConstraintGraph = boost::adjacency_matrix<boost::directedS>;
    using PathGraph = boost::adjacency_matrix<boost::undirectedS>;
    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;
    using RealDistribution = std::uniform_real_distribution<double>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, boost::no_property, double>;
    using IndependentSetGraph = boost::adjacency_matrix<boost::undirectedS, std::pair<int, int>>;
    using ChosenSet = boost::container::flat_set<MultiplexSetGraph::vertex_descriptor>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using Advances = std::vector<Advance *>;
    using IndependentSet = std::vector<ConstraintGraph::vertex_descriptor>;

public:
    static bool optimize(const std::vector<Var *> & input, PabloBlock & entry);
protected:
    void initialize(const std::vector<Var *> & vars, const PabloBlock & entry);
    void characterize(PabloBlock & entry);
    bool notTransitivelyDependant(const PathGraph::vertex_descriptor i, const PathGraph::vertex_descriptor j) const;
    void createMultiplexSetGraph();
    bool generateMultiplexSets(RNG & rng, unsigned k = 1);
    void addMultiplexSet(const IndependentSet & N, const IndependentSet & M);
    void selectMultiplexSets(RNG &);
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets() const;
    void topologicalSort(PabloBlock & entry) const;
    inline AutoMultiplexing()
    : mVariables(0)
    , mConstraintGraph(0)
    {
    }
private:
    DdNode * Zero() const;
    DdNode * One() const;
    bool isZero(DdNode * const x) const;
    DdNode * And(DdNode * const x, DdNode * const y);
    DdNode * Intersect(DdNode * const x, DdNode * const y);
    DdNode * Or(DdNode * const x, DdNode * const y);
    DdNode * Xor(DdNode * const x, DdNode * const y);
    DdNode * Not(DdNode * x) const;
    DdNode * Ite(DdNode * const x, DdNode * const y, DdNode * const z);
    bool noSatisfyingAssignment(DdNode * const x);
    void shutdown();
private:
    DdManager *             mManager;
    unsigned                mVariables;
    CharacterizationMap     mCharacterizationMap;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;
    Advances                mAdvance;    
    MultiplexSetGraph       mMultiplexSetGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
