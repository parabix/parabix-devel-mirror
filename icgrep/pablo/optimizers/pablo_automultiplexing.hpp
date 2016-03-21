#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <util/slab_allocator.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>

typedef int BDD;

namespace pablo {

class PabloBuilder;
class PabloFunction;

class MultiplexingPass {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, BDD>;

    using ConstraintGraph = boost::adjacency_matrix<boost::directedS, boost::no_property, bool>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using Constraints = std::vector<ConstraintVertex>;

    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;

    using CandidateGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS>;
    using Candidates = std::vector<CandidateGraph::vertex_descriptor>;

    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;

    using CliqueGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS>;
    using CliqueSet = boost::container::flat_set<CliqueGraph::vertex_descriptor>;
    using CliqueSets = boost::container::flat_set<std::vector<CliqueGraph::vertex_descriptor>>;

    using AdvanceVector = std::vector<Advance *>;
    using AdvanceRank = std::vector<int>;
    using AdvanceVariable = std::vector<BDD>;

public:

    static bool optimize(PabloFunction & function, const bool independent = false);
    #ifdef PRINT_TIMING_INFORMATION
    using seed_t = RNG::result_type;
    static seed_t SEED;
    static unsigned NODES_ALLOCATED;
    static unsigned NODES_USED;
    #endif
protected:

    unsigned initialize(PabloFunction & function, const bool independent);
    void initializeBaseConstraintGraph(PabloBlock * const block, const unsigned statements, const unsigned advances);
    void initializeAdvanceDepth(PabloBlock * const block, const unsigned advances) ;

    void characterize(PabloBlock * const block);
    BDD characterize(Statement * const stmt);
    BDD characterize(Advance * const adv, const BDD Ik);
    bool independent(const ConstraintVertex i, const ConstraintVertex j) const;
    bool exceedsWindowSize(const ConstraintVertex i, const ConstraintVertex j) const;

    void generateUsageWeightingGraph();
    CliqueSets findMaximalCliques(const CliqueGraph & G);
    void findMaximalCliques(const CliqueGraph & G, CliqueSet & R, CliqueSet && P, CliqueSet && X, CliqueSets & S);

    bool generateCandidateSets();
    void addCandidateSet(const Constraints & S);
    void updateCandidateSet(ConstraintVertex * const begin, ConstraintVertex * const end);
    void selectCandidateSet(const unsigned n, const unsigned k, const unsigned m, const Constraints & S, ConstraintVertex * const element);

    void selectMultiplexSetsGreedy();
    void selectMultiplexSetsWorkingSet();

    void removePotentialCycles(const CandidateGraph::vertex_descriptor u);
    bool dependent(const ConstraintVertex i, const ConstraintVertex j) const;

    void eliminateSubsetConstraints();
    void doTransitiveReductionOfSubsetGraph();

    Candidates orderMultiplexSet(const CandidateGraph::vertex_descriptor u);
    void multiplexSelectedSets(PabloFunction & function);

    static void rewriteAST(PabloBlock * const block);

    BDD & get(const PabloAST * const expr);

    inline MultiplexingPass(const RNG::result_type seed)
    : mTestConstrainedAdvances(true)
    , mSubsetImplicationsAdhereToWindowingSizeConstraint(false)
    , mVariables(0)
    , mRNG(seed)
    , mConstraintGraph(0)
    , mAdvance(0, nullptr)
    , mAdvanceRank(0, 0)
    , mAdvanceNegatedVariable(0, 0)
    {

    }

private:
    const bool                  mTestConstrainedAdvances;
    const bool                  mSubsetImplicationsAdhereToWindowingSizeConstraint;
    unsigned                    mVariables;
    RNG                         mRNG;
    CharacterizationMap         mCharacterization;
    ConstraintGraph             mConstraintGraph;   
    AdvanceVector               mAdvance;
    AdvanceRank                 mAdvanceRank;
    AdvanceVariable             mAdvanceNegatedVariable;
    SubsetGraph                 mSubsetGraph;
    CliqueGraph                 mUsageGraph;
    CandidateGraph              mCandidateGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
