#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <util/slab_allocator.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/type_traits/ice.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>
#include <llvm/ADT/DenseSet.h>
#include <llvm/ADT/SmallVector.h>
#include <z3.h>
#include <stack>

namespace pablo {

class PabloBuilder;
class PabloFunction;

class MultiplexingPass {

    using CharacterizationRef = std::pair<Z3_ast, unsigned>;
    using CharacterizationMap = llvm::DenseMap<const PabloAST *, CharacterizationRef>;

    using ConstraintGraph = boost::adjacency_matrix<boost::undirectedS, Advance *>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using Constraints = std::vector<ConstraintVertex>;
    using ConstraintMap = boost::container::flat_map<Advance *, ConstraintVertex>;

    using RNG = std::mt19937;
    using IntDistribution = std::uniform_int_distribution<RNG::result_type>;

    using CandidateGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;
    using Candidates = std::vector<CandidateGraph::vertex_descriptor>;

    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;

    using CliqueGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::undirectedS>;
    using CliqueSet = boost::container::flat_set<CliqueGraph::vertex_descriptor>;
    using CliqueSets = boost::container::flat_set<std::vector<CliqueGraph::vertex_descriptor>>;

    using AdvanceVector = std::vector<Advance *>;
    using AdvanceRank = std::vector<int>;
    using AdvanceVariable = std::vector<Z3_ast>;

public:

    static bool optimize(PabloFunction & function);

protected:

    unsigned initialize(PabloFunction & function);
    Statement * initialize(Statement * const initial);

    void reset();

    void optimize();
    void optimize(PabloBlock * const block);
    Z3_ast characterize(Statement * const stmt);
    Z3_ast characterize(Advance * const adv, Z3_ast Ik);
    void multiplex(PabloBlock * const block, Statement * const begin, Statement * const end);

    bool generateCandidateSets(Statement * const begin, Statement * const end);
    void generateCandidateSets(Z3_context ctx, Z3_solver solver, const std::vector<std::pair<unsigned, unsigned>> & S, const std::vector<Z3_ast> & N);

    void selectMultiplexSetsGreedy();

    void eliminateSubsetConstraints();
    void doTransitiveReductionOfSubsetGraph();

    void multiplexSelectedSets(PabloBlock * const block, Statement * const begin, Statement * const end);


    Z3_ast make(const PabloAST * const expr);
    Z3_ast add(const PabloAST * const expr, Z3_ast node);
    Z3_ast & get(const PabloAST * const expr, const bool deref = false);
    bool equals(Z3_ast a, Z3_ast b);

    MultiplexingPass(PabloFunction & f, const RNG::result_type seed, Z3_context context, Z3_solver solver);

private:

    Z3_context                  mContext;
    Z3_solver                   mSolver;
    PabloFunction &             mFunction;
    RNG                         mRNG;

    CharacterizationMap         mCharacterization;
    ConstraintGraph             mConstraintGraph;

    AdvanceVariable             mNegatedAdvance;
    SubsetGraph                 mSubsetGraph;
    CandidateGraph              mCandidateGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
