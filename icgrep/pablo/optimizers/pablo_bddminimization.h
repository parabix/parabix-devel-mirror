#ifndef PABLO_BDDMINIMIZATION_H
#define PABLO_BDDMINIMIZATION_H

#include <pablo/codegenstate.h>
#include <slab_allocator.h>
#include <queue>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/edge_list.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <random>
#include <stdint.h>
#include <llvm/ADT/DenseMap.h>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class BDDMinimizationPass {

    using CharacterizationMap = llvm::DenseMap<const PabloAST *, DdNode *>;

    struct SubsitutionMap {
        SubsitutionMap(SubsitutionMap * parent = nullptr) : mParent(parent) {}

        PabloAST * operator [](const DdNode * node) const {
            auto f = mMap.find(node);
            if (f == mMap.end()) {
                return mParent ? mParent->find(node) : nullptr;
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
    void initialize(PabloFunction & function);
    void characterize(const PabloBlock &block);
    DdNode * characterize(const Statement * const stmt);
    void eliminateLogicallyEquivalentStatements(PabloBlock & entry);
    void eliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent);
    void simplify(PabloBlock & entry);
    PabloAST * simplify(DdNode * bdd);

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
    DdNode * NewVar();
    bool noSatisfyingAssignment(DdNode * const x);
    void shutdown();
private:
    DdManager *             mManager;
    unsigned                mVariables;
    CharacterizationMap     mCharacterizationMap;
};


#endif // PABLO_BDDMINIMIZATION_H
