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

class AutoMultiplexing {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, BDD>;
    using ConstraintGraph = boost::adjacency_matrix<boost::directedS>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;
    using MultiplexSetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;
    using AdvanceAttributes = std::vector<std::tuple<Advance *, BDD, BDD>>; // the Advance pointer, input BDD, the base BDD variable
    using VertexVector = std::vector<ConstraintVertex>;
    using ScopeMap = boost::container::flat_map<const PabloBlock *, Statement *>;

public:
    static bool optimize(PabloFunction & function, const unsigned limit = std::numeric_limits<unsigned>::max(), const unsigned maxSelections = 100);
protected:
    unsigned initialize(PabloFunction & function);
    void characterize(PabloBlock & block);
    BDD characterize(Statement * const stmt);
    BDD characterize(Advance * const adv, const BDD Ik);
    bool independent(const ConstraintVertex i, const ConstraintVertex j) const;
    bool generateCandidateSets(RNG & rng);
    void addCandidateSet(const VertexVector & S, RNG & rng);
    void selectMultiplexSets(RNG &);
    void transitiveReductionOfSubsetGraph() ;
    void applySubsetConstraints();
    void multiplexSelectedIndependentSets(PabloFunction & function);
    static void topologicalSort(PabloFunction & function);

    inline AutoMultiplexing(const unsigned limit, const unsigned maxSelections)
    : mLimit(limit)
    , mMaxSelections(maxSelections)
    , mVariables(0)
    , mConstraintGraph(0)
    {
    }

private:
    unsigned                    mLimit;
    const unsigned              mMaxSelections;
    unsigned                    mVariables;
    CharacterizationMap         mCharacterizationMap;
    ConstraintGraph             mConstraintGraph;
    SubsetGraph                 mSubsetGraph;
    AdvanceAttributes           mAdvanceAttributes;
    MultiplexSetGraph           mMultiplexSetGraph;
    ScopeMap                    mResolvedScopes;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
