#ifndef FACTORIZEDFG_H
#define FACTORIZEDFG_H

#include <boost/container/flat_map.hpp>
#include <vector>

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
    using ASTVector = std::vector<PabloAST *>;
    struct OrderingMap {
        unsigned of(const PabloAST * const expr) const {
            auto f = mMap.find(expr);
            if (f == mMap.end()) {
                return mParent ? mParent->of(expr) : 0;
            }
            return f->second;
        }
        inline void enumerate(const PabloAST * const expr) {
            mMap.emplace(expr, ++mOrderingCount);
        }
        OrderingMap() :  mParent(nullptr), mOrderingCount(0) {}
        OrderingMap(OrderingMap * const parent) :  mParent(parent), mOrderingCount(parent->mOrderingCount) {}
        ~OrderingMap() { if (mParent) { mParent->mOrderingCount = mOrderingCount; } }
    private:
        OrderingMap * const mParent;
        unsigned mOrderingCount;
        boost::container::flat_map<const PabloAST *, unsigned> mMap;
    };

public:
    static void transform(PabloFunction & function);
protected:    
    void enumerateScopeDepth(const PabloFunction & function);
    void enumerateScopeDepth(const PabloBlock * const block, const unsigned depth);
    void cse(PabloBlock * const block);
    void cse(Variadic * const var);
    PabloBlock * chooseInsertionScope(const ASTVector & users);
    void findInsertionPoint(const ASTVector & operands, PabloBlock * const scope);
    void lower(PabloFunction & function);
//    void lower(PabloBlock * const block, OrderingMap & parent);
//    PabloAST * lower(Variadic * const var, PabloBlock * block, OrderingMap & order);
    void lower(PabloBlock * const block);
    Statement * lower(Variadic * const var, PabloBlock * block);
    FactorizeDFG() = default;
private:
    ScopeDepth  mScopeDepth;
};

}

#endif // FACTORIZEDFG_H
