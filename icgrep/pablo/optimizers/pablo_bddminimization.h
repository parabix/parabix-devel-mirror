#ifndef PABLO_BDDMINIMIZATION_H
#define PABLO_BDDMINIMIZATION_H

#include <boost/container/flat_map.hpp>
#include <vector>

struct DdManager; // forward declare of the CUDD manager
struct DdNode;

namespace pablo {

class PabloAST;
class PabloBlock;
class PabloFunction;
class PabloBuilder;
class Statement;

class BDDMinimizationPass {

    using CharacterizationMap = boost::container::flat_map<const PabloAST *, DdNode *>;
    using Terminals = std::vector<Statement *>;

    struct SubsitutionMap {
        SubsitutionMap(SubsitutionMap * parent = nullptr) : mParent(parent) {}

        PabloAST * get(const DdNode * node) const {
            auto f = mMap.find(node);
            if (f == mMap.end()) {
                return mParent ? mParent->get(node) : nullptr;
            }
            return f->second;
        }

        void insert(const DdNode * node, PabloAST * stmt) {
            mMap.emplace(node, stmt);
        }
    private:
        const SubsitutionMap * const mParent;
        boost::container::flat_map<const DdNode *, PabloAST *> mMap;
    };

public:
    static bool optimize(PabloFunction & function);
protected:
    void initialize(PabloFunction & function);
    void eliminateLogicallyEquivalentStatements(PabloFunction & function);
    void eliminateLogicallyEquivalentStatements(PabloBlock & block, SubsitutionMap & parent);
    void eliminateLogicallyEquivalentStatements(Statement * const stmt, SubsitutionMap & map);
    std::pair<DdNode *, bool> characterize(Statement * const stmt);
    void identifyHiddenContradicionsAndTautologies(PabloBlock & block);
private:
    DdNode * Zero() const;
    DdNode * One() const;
    bool nonConstant(DdNode * const x) const;
    DdNode * And(DdNode * const x, DdNode * const y);
    DdNode * Intersect(DdNode * const x, DdNode * const y);
    DdNode * Or(DdNode * const x, DdNode * const y);
    DdNode * Xor(DdNode * const x, DdNode * const y);
    DdNode * Not(DdNode * x) const;
    DdNode * Ite(DdNode * const x, DdNode * const y, DdNode * const z);
    DdNode * NewVar(const PabloAST *);
    bool noSatisfyingAssignment(DdNode * const x);
    void shutdown();
private:
    DdManager *                     mManager;
    unsigned                        mVariables;
    CharacterizationMap             mCharacterizationMap;
};

}

#endif // PABLO_BDDMINIMIZATION_H
