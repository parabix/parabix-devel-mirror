#ifndef PABLO_CODESINKING_HPP
#define PABLO_CODESINKING_HPP

#include <pablo/codegenstate.h>
#include <vector>
#include <algorithm>

namespace pablo {

class PabloFunction;

class CodeSinking {
    struct ScopeSet : public std::vector<PabloBlock *> {
        inline bool insert(PabloBlock * block) {
            const auto i = std::lower_bound(begin(), end(), block);
            if (i == end() || *i != block) {
                std::vector<PabloBlock *>::insert(i, block);
                assert (std::is_sorted(begin(), end()));
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
    void sink(PabloBlock & block);
    CodeSinking() { }
};

}

#endif // PABLO_CODESINKING_HPP
