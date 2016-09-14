#ifndef PABLO_AUTOMULTIPLEXING_HPP
#define PABLO_AUTOMULTIPLEXING_HPP

#include <pablo/codegenstate.h>
#include <util/slab_allocator.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/type_traits/ice.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_set.hpp>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>
#include <z3.h>

namespace pablo {

class PabloBuilder;
class PabloFunction;

class MultiplexingPass {

    using CharacterizationRef = std::pair<Z3_ast, size_t>;
    using CharacterizationMap = llvm::DenseMap<const PabloAST *, CharacterizationRef>;

    enum class ConstraintType : uint8_t {
        Dependency,
        Inclusive
    };

    using ConstraintGraph = boost::adjacency_matrix<boost::undirectedS, Advance *, ConstraintType>;
    using ConstraintVertex = ConstraintGraph::vertex_descriptor;
    using Constraints = std::vector<ConstraintVertex>;

    using CandidateGraph = boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS>;
    using Candidates = std::vector<CandidateGraph::vertex_descriptor>;

    using SubsetGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS>;

    using AdvanceVariable = std::vector<Z3_ast>;

public:

    static bool optimize(PabloFunction & function);

protected:

    unsigned initialize(PabloFunction & function);
    Statement * initialize(Statement * const initial);

    void reset();

    void optimize();
    void optimize(PabloBlock * const block);
    Z3_ast characterize(const Statement * const stmt, const bool deref = true);
    Z3_ast characterize(const Advance * const adv, Z3_ast Ik);
    void multiplex(PabloBlock * const block, Statement * const begin, Statement * const end);

    bool generateCandidateSets(Statement * const begin, Statement * const end);
    void generateCandidateSets(Z3_context ctx, Z3_solver solver, const std::vector<std::pair<unsigned, unsigned>> & S, const std::vector<Z3_ast> & N);

    void selectMultiplexSetsGreedy();

    void eliminateSubsetConstraints();
    void doTransitiveReductionOfSubsetGraph();

    void multiplexSelectedSets(PabloBlock * const block, Statement * const begin, Statement * const end);


    Z3_ast makeVar();
    Z3_ast add(const PabloAST * const expr, Z3_ast node, const size_t refs);
    Z3_ast & get(const PabloAST * const expr, const bool deref = false);
    bool equals(Z3_ast a, Z3_ast b);

    MultiplexingPass(PabloFunction & f, Z3_context context, Z3_solver solver);

private:

    Z3_context                  mContext;
    Z3_solver                   mSolver;
    PabloFunction &             mFunction;

    CharacterizationMap         mCharacterization;
    ConstraintGraph             mConstraintGraph;

    AdvanceVariable             mNegatedAdvance;
    SubsetGraph                 mSubsetGraph;
    CandidateGraph              mCandidateGraph;
};

}

#endif // PABLO_AUTOMULTIPLEXING_HPP
