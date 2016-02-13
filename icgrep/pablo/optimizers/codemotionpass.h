#ifndef PABLO_CODESINKING_HPP
#define PABLO_CODESINKING_HPP

#include <vector>
#include <algorithm>

namespace pablo {

class PabloFunction;
class PabloBlock;
class Statement;
class While;
class Variadic;

class CodeMotionPass {
    struct ScopeSet : public std::vector<PabloBlock *> {
        inline bool insert(PabloBlock * block) {
            const auto i = std::lower_bound(begin(), end(), block);
            if (i == end() || *i != block) {
                std::vector<PabloBlock *>::insert(i, block);
                return true;
            }
            return false;
        }
        inline bool count(PabloBlock * block) {
            const auto i = std::lower_bound(begin(), end(), block);
            return (i != end() && *i == block);
        }
    };
public:
    static bool optimize(PabloFunction & function);
protected:
    static void movement(PabloBlock * const block);
    static bool isAcceptableTarget(Statement *stmt, ScopeSet & scopeSet, const PabloBlock * const block);
    static void sink(PabloBlock * const block);
    static void hoistLoopInvariants(While * loop);

    static void reschedule(PabloBlock * const block);
};

}

#endif // PABLO_CODESINKING_HPP
