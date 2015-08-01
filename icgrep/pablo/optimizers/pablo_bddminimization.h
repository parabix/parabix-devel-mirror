#ifndef PABLO_BDDMINIMIZATION_H
#define PABLO_BDDMINIMIZATION_H

#include <llvm/ADT/DenseMap.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class PabloAST;
class PabloBlock;
class PabloFunction;
class PabloBuilder;
class Statement;

class BDDMinimizationPass {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, DdNode *>;
    using StatementVector = std::vector<PabloAST *>;

    struct SubsitutionMap {
        SubsitutionMap(SubsitutionMap * parent = nullptr) : mParent(parent) {}

        PabloAST * operator [](const DdNode * node) const {
            auto f = mMap.find(node);
            if (f == mMap.end()) {
                return mParent ? mParent->operator [](node) : nullptr;
            }
            return f->second;
        }

        void insert(const DdNode * node, PabloAST * stmt) {
            mMap.insert(std::make_pair(node, stmt));
        }
    private:
        const SubsitutionMap * const mParent;
        llvm::DenseMap<const DdNode *, PabloAST *> mMap;
    };

public:
    static bool optimize(PabloFunction & function);
protected:
    static void promoteCrossBlockReachingDefs(const PabloFunction & function);
    void initialize(const PabloFunction & function);
    void characterizeAndEliminateLogicallyEquivalentStatements(PabloFunction & function);
    void characterizeAndEliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent);
    DdNode * characterizeAndEliminateLogicallyEquivalentStatements(const Statement * const stmt);
    void simplifyAST(PabloFunction & function);
    void simplifyAST(PabloBuilder & block);
    void simplifyAST(Statement *stmt, Statement * const value, PabloBuilder & builder);
    PabloAST * simplifyAST(DdNode * const f, PabloBuilder & builder);
    PabloAST * makeCoverAST(DdNode * const f, PabloBuilder & builder);
private:
    DdNode * Zero() const;
    DdNode * One() const;
    bool isZero(DdNode * const x) const;
    DdNode * And(DdNode * const x, DdNode * const y);
    DdNode * Intersect(DdNode * const x, DdNode * const y);
    DdNode * Or(DdNode * const x, DdNode * const y);
    DdNode * Xor(DdNode * const x, DdNode * const y);
    DdNode * Not(DdNode * x) const;
    DdNode * Ite(DdNode * const x, DdNode * const y, DdNode * const z);
    DdNode * NewVar(const PabloAST * expr);
    bool noSatisfyingAssignment(DdNode * const x);
    void shutdown();
private:
    DdManager *                     mManager;
    std::vector<PabloAST *>         mVariables;
    std::vector<const Statement *>  mPromotions;
    CharacterizationMap             mCharacterizationMap;
};

}

#endif // PABLO_BDDMINIMIZATION_H
