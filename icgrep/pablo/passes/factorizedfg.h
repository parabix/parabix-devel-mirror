#ifndef FACTORIZEDFG_H
#define FACTORIZEDFG_H

#include <boost/container/flat_map.hpp>
#include <boost/circular_buffer.hpp>
#include <vector>
#include <set>

namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;
class Not;
class Statement;
class PabloAST;

class FactorizeDFG {
    using ScopeDepth = boost::container::flat_map<const PabloBlock *, unsigned>;
    using ScopeSet = std::vector<PabloBlock *>;
    using NodeSet = std::vector<PabloAST *>;
    // using Variadics = std::vector<Variadic *>;
    using Variadics = boost::circular_buffer<Variadic *>;

    using VertexSet = std::vector<PabloAST *>;
    using Biclique = std::pair<VertexSet, VertexSet>; // [{Operands}, {Users}]
    // using BicliqueSet = boost::container::flat_set<Biclique>;
    using BicliqueSet = std::vector<Biclique>;
    using CheckSet = boost::container::flat_set<PabloAST *>;
public:
    static void transform(PabloFunction & function);
protected:    
    void enumerateScopeDepth(const PabloFunction & function);
    void enumerateScopeDepth(const PabloBlock * const block, const unsigned depth);
    static void deMorgansExpansion(Not * const var, PabloBlock * const block);
    static void deMorgansExpansion(PabloBlock * const block);
    void factor(PabloFunction & function) const;
    // static void factor(PabloBlock * const block, BicliqueSet & bicliques);
    // static void enumerateBicliques(Variadic * const var, BicliqueSet & bicliques);
    static BicliqueSet findAllFactoringsOf(Variadic * const var);
    static void independentCliqueSets(BicliqueSet & bicliques);
    void processBicliques(BicliqueSet & bicliques) const;
    // void factor(PabloBlock * const block, BicliqueSet & vars) const;
    void factor(PabloBlock * const block, Variadics & vars) const;
    void factor(Variadic * const var, Variadics & Q) const;
    PabloBlock * findInsertionScope(const NodeSet & users) const;
    CheckSet makeCheckSet(PabloBlock * const scope, const NodeSet & values) const;
    Statement * firstIn(PabloBlock * const scope, Statement * stmt, const NodeSet & operands) const;
    Statement * lastIn(PabloBlock * const scope, Statement * stmt, const NodeSet & users) const;
    PabloBlock * findInsertionPoint(const NodeSet & operands, const NodeSet & users) const;
    void lower(PabloFunction & function) const;
    void lower(PabloBlock * const block) const;
    Statement * lower(Variadic * const var, PabloBlock * block) const;
    unsigned scopeDepthOf(const PabloBlock * const block) const;
    unsigned scopeDepthOf(const PabloAST * const expr) const;
    static void ensureLegality(PabloBlock * const block);
    FactorizeDFG() : mNumOfVariadics(0) {}
private:
    unsigned        mNumOfVariadics;
    ScopeDepth      mScopeDepth;
};

}

#endif // FACTORIZEDFG_H
