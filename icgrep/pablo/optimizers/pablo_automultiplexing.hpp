#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>

typedef int BDD;

namespace pablo {

class PabloBuilder;
class PabloFunction;
struct OrderingVerifier;

class MultiplexingPass {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, BDD>;
    using ConstraintGraph = boost::adjacency_matrix<boost::directedS>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using SubsetEdgeIterator = boost::graph_traits<SubsetGraph>::edge_iterator;
    using CliqueGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS>;
    using CliqueSet = boost::container::flat_set<CliqueGraph::vertex_descriptor>;
    using CliqueSets = boost::container::flat_set<std::vector<CliqueGraph::vertex_descriptor>>;

    using AdvanceVector = std::vector<Advance *>;
    using AdvanceDepth = std::vector<int>;
    using AdvanceVariable = std::vector<BDD>;
    using VertexVector = std::vector<ConstraintVertex>;

public:

    static bool optimize(PabloFunction & function, const unsigned limit = std::numeric_limits<unsigned>::max(), const unsigned maxSelections = 100, const unsigned windowSize = 1, const bool independent = false);

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
    void addCandidateSet(const VertexVector & S);
    void selectMultiplexSets();

    void eliminateSubsetConstraints();
    void doTransitiveReductionOfSubsetGraph();

    void multiplexSelectedIndependentSets(PabloFunction & function);

    static void topologicalSort(PabloFunction & function);
    static void topologicalSort(PabloBlock * block, OrderingVerifier & parent);

    BDD & get(const PabloAST * const expr);

    inline MultiplexingPass(const RNG::result_type seed, const unsigned limit, const unsigned maxSelections, const unsigned windowSize)
    : mMultiplexingSetSizeLimit(limit)
    , mMaxMultiplexingSetSelections(maxSelections)
    , mWindowSize(windowSize)
    , mTestConstrainedAdvances(true)
    , mVariables(0)
    , mRNG(seed)
    , mConstraintGraph(0)
    , mAdvance(0, nullptr)
    , mAdvanceDepth(0, 0)
    , mAdvanceNegatedVariable(0, 0)
    {

    }

private:
    const unsigned              mMultiplexingSetSizeLimit;
    const unsigned              mMaxMultiplexingSetSelections;
    const unsigned              mWindowSize;
    const bool                  mTestConstrainedAdvances;
    unsigned                    mVariables;
    RNG                         mRNG;
    CharacterizationMap         mCharacterization;
    ConstraintGraph             mConstraintGraph;   
    AdvanceVector               mAdvance;
    AdvanceDepth                mAdvanceDepth;
    AdvanceVariable             mAdvanceNegatedVariable;
    SubsetGraph                 mSubsetGraph;
    CliqueGraph                 mUsageWeightingGraph;
    MultiplexSetGraph           mMultiplexSetGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
