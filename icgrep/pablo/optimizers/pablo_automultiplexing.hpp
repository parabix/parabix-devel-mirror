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
#include <llvm/ADT/DenseMap.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

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

    struct SubsitutionMap {
        SubsitutionMap(SubsitutionMap * parent = nullptr) : mParent(parent) {}
        PabloAST * test(const DdNode * node, PabloAST * stmt) {
            PabloAST * replacement = find(node);
            if (LLVM_LIKELY(replacement == nullptr)) {
                mMap.insert(std::make_pair(node, stmt));
            }
            return replacement;
        }
        PabloAST * find(const DdNode * node) const {
            auto f = mMap.find(node);
            if (LLVM_LIKELY(f == mMap.end())) {
                PabloAST * replacement = nullptr;
                if (mParent == nullptr) {
                    replacement = mParent->find(node);
                }
                return replacement;
            }
            return f->second;
        }
        void insert(const DdNode * node, PabloAST * stmt) {
            mMap.insert(std::make_pair(node, stmt));
        }
    private:
        const SubsitutionMap * const mParent;
        llvm::DenseMap<const DdNode *, PabloAST *> mMap;
    };

public:
    static bool optimize(const std::vector<Var *> & input, PabloBlock & entry);
protected:
    void initialize(const std::vector<Var *> & vars, PabloBlock & entry);
    void characterize(PabloBlock & block);
    DdNode * characterize(Statement * const stmt, const bool throwUncharacterizedOperandError);
    DdNode * characterize(Advance * adv, DdNode * input);
    void reevaluate(Next * next, DdNode * value);
    void minimize(PabloBlock & entry);
    void minimize(PabloBlock & block, SubsitutionMap & parent);

    bool notTransitivelyDependant(const ConstraintVertex i, const ConstraintVertex j) const;
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
