#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class PabloBuilder;

class AutoMultiplexing {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, DdNode *>;
    using ConstraintGraph = boost::adjacency_matrix<boost::directedS>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using IndependentSetGraph = boost::adjacency_matrix<boost::undirectedS, std::pair<int, int>>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    // the Advance pointer, input BDD and the BDD variable of the i-th Advance
    using AdvanceMap = boost::container::flat_map<const Statement *, unsigned>;
    using AdvanceVector = std::vector<std::tuple<Advance *, DdNode *, DdNode *>>;
    using IndependentSet = std::vector<ConstraintVertex>;

public:
    static bool optimize(const std::vector<Var *> & input, PabloBlock & entry);
protected:
    void initialize(const std::vector<Var *> & vars, PabloBlock & entry);
    void characterize(PabloBlock & block);
    DdNode * characterize(Statement * const stmt);
    DdNode * characterize(Advance * adv, DdNode * input);
    bool notTransitivelyDependant(const ConstraintVertex i, const ConstraintVertex j) const;
    bool generateMultiplexSets(RNG & rng, unsigned k = 1);
    void addMultiplexSet(const IndependentSet & N, const IndependentSet & M);
    void selectMultiplexSets(RNG &);
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets() const;
    void simplify(const std::vector<PabloAST *> & variables, const unsigned m, PabloBuilder & block) const;
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
    DdNode * Or(DdNode * const x, DdNode * const y);
    DdNode * Xor(DdNode * const x, DdNode * const y);
    DdNode * Not(DdNode * x) const;
    DdNode * Ite(DdNode * const x, DdNode * const y, DdNode * const z);
    DdNode * NewVar();
    bool noSatisfyingAssignment(DdNode * const x);
    void shutdown();
private:
    DdManager *             mManager;
    unsigned                mVariables;
    CharacterizationMap     mCharacterizationMap;
    ConstraintGraph         mConstraintGraph;
    SubsetGraph             mSubsetGraph;
    AdvanceMap              mAdvanceMap;
    AdvanceVector           mAdvance;
    MultiplexSetGraph       mMultiplexSetGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
