#ifndef PABLO_BDDMINIMIZATION_H
#define PABLO_BDDMINIMIZATION_H

#include <boost/container/flat_map.hpp>
#include <vector>

typedef int BDD;

namespace pablo {

class PabloAST;
class PabloBlock;
class PabloFunction;
class Statement;

class BDDMinimizationPass {

    using CharacterizationMap = boost::container::flat_map<const PabloAST *, BDD>;

    struct SubsitutionMap {
        SubsitutionMap(SubsitutionMap * parent = nullptr) : mParent(parent) {}

        PabloAST * get(const BDD node) const {
            auto f = mMap.find(node);
            if (f == mMap.end()) {
                return mParent ? mParent->get(node) : nullptr;
            }
            return f->second;
        }

        void insert(const BDD node, PabloAST * stmt) {
            mMap.emplace(node, stmt);
        }
    private:
        const SubsitutionMap * const mParent;
        boost::container::flat_map<BDD, PabloAST *> mMap;
    };

public:
    static bool optimize(PabloFunction & function);
protected:
    void initialize(PabloFunction & function);
    void eliminateLogicallyEquivalentStatements(PabloFunction & function);
    void eliminateLogicallyEquivalentStatements(PabloBlock * const block, SubsitutionMap & parent);
    void eliminateLogicallyEquivalentStatements(Statement * const stmt, SubsitutionMap & map);
    std::pair<BDD, bool> characterize(Statement * const stmt);
    BDD & get(const PabloAST * const expr);
private:
    unsigned                        mVariables;
    CharacterizationMap             mCharacterizationMap;
};

}

#endif // PABLO_BDDMINIMIZATION_H
