#ifndef FACTORIZEDFG_H
#define FACTORIZEDFG_H


#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <pablo/pabloAST.h>
#include <vector>
#include <deque>

namespace pablo {

class PabloFunction;
class PabloBlock;
class Variadic;
class Statement;
class PabloAST;

class FactorizeDFG {

    using ScopeDepth = boost::container::flat_map<const PabloBlock *, unsigned>;
    using ObjectSet = std::vector<PabloAST *>;
    using ScopeUsers = std::pair<PabloBlock *, ObjectSet>;
    using Biclique = std::pair<ObjectSet, ObjectSet>; // [{Operands}, {Users}]
    using BicliqueSet = std::vector<Biclique>;
    using CheckSet = boost::container::flat_set<PabloAST *>;
    using LiveSet = std::deque<PabloAST *>;
    using TypeId = PabloAST::ClassTypeId;
public:
    static void transform(PabloFunction & function);
protected:    
    void initialize(PabloFunction &function);
    void initialize(PabloBlock * const block, const unsigned depth);

    unsigned scopeDepthOf(const PabloBlock * const block) const;
    unsigned scopeDepthOf(const PabloAST * const expr) const;

    CheckSet makeCheckSet(PabloBlock * const scope, const ObjectSet & values) const;
    Statement * firstIn(PabloBlock * const scope, Statement * const initial, const ObjectSet & operands) const;
    Statement * lastIn(PabloBlock * const scope, Statement * const initial, const ObjectSet & users) const;
    Variadic * factorize(const TypeId typeId, PabloBlock * const scope, ObjectSet & operands, ObjectSet & users) const;
    static BicliqueSet independentFactoringSets(BicliqueSet && factoringSets, const unsigned side);
    static BicliqueSet enumerateFactoringSets(Variadic * const var);
    static BicliqueSet enumerateFactoringSets(ObjectSet params, PabloBlock * const entryScope, const TypeId typeId);
    bool processFactoringSets(const TypeId typeId, PabloBlock * const scope, BicliqueSet && factoringSets) const;
    PabloBlock * findInsertionScope(const ObjectSet & users) const;
    bool processFactoringSets(const TypeId typeId, BicliqueSet && factoringSets, ObjectSet & factorings) const;
    bool factor(PabloBlock * const block);
    void factor(PabloFunction & function, const TypeId typeId);
    void factor(PabloFunction & function);

//    void rematerialize(PabloBlock * const block, LiveSet & priorSet);
//    void rematerialize(PabloFunction & function);

    void elevate(PabloFunction & function) const;
    void elevate(PabloBlock * const block) const;
    void elevate(Variadic * const var, PabloBlock * block) const;

    void lower(PabloFunction & function) const;
    void lower(PabloBlock * const block) const;
    PabloAST * lower(Variadic * const var, PabloBlock * block) const;

    FactorizeDFG() = default;

private:
    ScopeDepth              mScopeDepth;
};

}

#endif // FACTORIZEDFG_H
