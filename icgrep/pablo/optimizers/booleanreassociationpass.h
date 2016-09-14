#ifndef BOOLEANREASSOCIATIONPASS_H
#define BOOLEANREASSOCIATIONPASS_H

#include <pablo/codegenstate.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/container/flat_map.hpp>
#include <z3.h>

namespace pablo {

class BooleanReassociationPass {
public:
    using VertexData = std::tuple<PabloAST::ClassTypeId, PabloAST *, Z3_ast>;
    using Graph = boost::adjacency_list<boost::vecS, boost::vecS, boost::bidirectionalS, VertexData, PabloAST *>;
    using Vertex = Graph::vertex_descriptor;
    using VertexMap = std::unordered_map<Z3_ast, Vertex>;
    using StatementMap = boost::container::flat_map<PabloAST *, Vertex>;
    using DistributionGraph = boost::adjacency_list<boost::hash_setS, boost::vecS, boost::bidirectionalS, Vertex>;

    static bool optimize(PabloFunction & function);

protected:

    struct CharacterizationMap {
        using ForwardMap = boost::container::flat_map<PabloAST *, Z3_ast>;
        using BackwardMap = boost::container::flat_map<Z3_ast, PabloAST *>;
        using iterator = ForwardMap::const_iterator;
        Z3_ast add(PabloAST * const expr, Z3_ast node, const bool forwardOnly = false);
        Z3_ast get(PabloAST * const expr) const;
        PabloAST * findKey(Z3_ast const node) const;
        iterator begin() const { return mForward.cbegin(); }
        iterator end() const { return mForward.cend(); }
        inline CharacterizationMap() : mPredecessor(nullptr) {}
        inline CharacterizationMap(CharacterizationMap & predecessor) : mPredecessor(&predecessor) {}
        CharacterizationMap * predecessor() const { return mPredecessor; }
    private:
        CharacterizationMap * const     mPredecessor;
        ForwardMap                      mForward;
        BackwardMap                     mBackward;
    };

protected:

    BooleanReassociationPass(Z3_context ctx, Z3_params params, Z3_tactic tactic, PabloFunction & f);
    bool processScopes(PabloFunction & function);
    void processScopes(PabloBlock * const block, CharacterizationMap & map);
    void distributeScope(PabloBlock * const block, CharacterizationMap & C);

    void transformAST(CharacterizationMap & C, Graph & G);
    void resolveNestedUsages(PabloAST * const expr, const Vertex u, CharacterizationMap &C, StatementMap & S, Graph & G, const Statement * const ignoreIfThis = nullptr) const;

    bool contractGraph(Graph & G) const;

    bool reduceVertex(const Vertex u, CharacterizationMap & C, VertexMap & M, Graph & G, const bool use_expensive_simplification) const;
    bool reduceGraph(CharacterizationMap & C, VertexMap & M, Graph & G) const;

    bool factorGraph(const PabloAST::ClassTypeId typeId, Graph & G, std::vector<Vertex> & factors) const;
    bool factorGraph(Graph & G) const;

    static Vertex makeVertex(const PabloAST::ClassTypeId typeId, PabloAST * const expr, Graph & G, Z3_ast node = nullptr);
    static Vertex makeVertex(const PabloAST::ClassTypeId typeId, PabloAST * const expr, StatementMap & M, Graph & G, Z3_ast node = nullptr);
    static Vertex makeVertex(const PabloAST::ClassTypeId typeId, PabloAST * const expr, CharacterizationMap & C, StatementMap & M, Graph & G);
    void removeVertex(const Vertex u, StatementMap & M, Graph & G) const;
    void removeVertex(const Vertex u, Graph & G) const;

    bool redistributeGraph(CharacterizationMap & C, VertexMap & M, Graph & G) const;

    bool rewriteAST(CharacterizationMap & C, VertexMap &M, Graph & G);

    Statement * characterize(Statement * const stmt, CharacterizationMap & map);

    Z3_ast simplify(Z3_ast node, bool use_expensive_minimization = false) const;

    Z3_ast makeVar() const;

private:
    PabloBlock *                mBlock;
    Z3_context const            mContext;
    Z3_params const             mParams;
    Z3_tactic const             mTactic;
    Z3_ast                      mInFile;
    PabloFunction &             mFunction;
    bool                        mModified;
};

}

#endif // BOOLEANREASSOCIATIONPASS_H
