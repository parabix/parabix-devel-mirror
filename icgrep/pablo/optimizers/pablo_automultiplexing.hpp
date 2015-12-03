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

typedef int BDD;

namespace pablo {

class PabloBuilder;
class PabloFunction;

class MultiplexingPass {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, BDD>;
    using ConstraintGraph = boost::adjacency_matrix<boost::directedS>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using AdvanceDepth = std::vector<int>;
    using AdvanceAttributes = std::vector<std::pair<Advance *, BDD>>; // the Advance pointer and its respective base BDD variable
    using VertexVector = std::vector<ConstraintVertex>;
    using ScopeMap = boost::container::flat_map<const PabloBlock *, Statement *>;

public:
    static bool optimize(PabloFunction & function, const unsigned limit = std::numeric_limits<unsigned>::max(), const unsigned maxSelections = 100, const unsigned windowSize = 10, const bool independent = false);
protected:
    unsigned initialize(PabloFunction & function, const bool independent);
    void initializeBaseConstraintGraph(PabloBlock * const block, const unsigned statements, const unsigned advances);
    void initializeAdvanceDepth(PabloBlock * const block, const unsigned advances) ;

    void characterize(PabloBlock * const block);
    BDD characterize(Statement * const stmt);
    BDD characterize(Advance * const adv, const BDD Ik);
    bool independent(const ConstraintVertex i, const ConstraintVertex j) const;
    bool exceedsWindowSize(const ConstraintVertex i, const ConstraintVertex j) const;
    bool generateCandidateSets(RNG & rng);

    void addCandidateSet(const VertexVector & S, RNG & rng);
    void selectMultiplexSets(RNG & rng);
    void doTransitiveReductionOfSubsetGraph();
    void eliminateSubsetConstraints();
    void multiplexSelectedIndependentSets(PabloFunction & function);
    static void topologicalSort(PabloFunction & function);
    BDD & get(const PabloAST * const expr);

    inline MultiplexingPass(const unsigned limit, const unsigned maxSelections, const unsigned windowSize)
    : mMultiplexingSetSizeLimit(limit)
    , mMaxMultiplexingSetSelections(maxSelections)
    , mWindowSize(windowSize)
    , mTestConstrainedAdvances(true)
    , mVariables(0)
    , mConstraintGraph(0)
    , mAdvanceDepth(0, 0)
    {

    }

private:
    const unsigned              mMultiplexingSetSizeLimit;
    const unsigned              mMaxMultiplexingSetSelections;
    const unsigned              mWindowSize;
    const bool                  mTestConstrainedAdvances;
    unsigned                    mVariables;
    CharacterizationMap         mCharacterization;
    ConstraintGraph             mConstraintGraph;
    AdvanceDepth                mAdvanceDepth;
    SubsetGraph                 mSubsetGraph;
    AdvanceAttributes           mAdvanceAttributes;
    MultiplexSetGraph           mMultiplexSetGraph;
    ScopeMap                    mResolvedScopes;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
