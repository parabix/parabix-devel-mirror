#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/pe_next.h>

namespace pablo {

bool Assign::isConstant() const {
    if (LLVM_UNLIKELY(isOutputAssignment())) {
        return false;
    }
    for (const PabloAST * inst : users()) {
        if (isa<Next>(inst)) {
            return false;
        }
        if (isa<If>(inst)) {
            // if this Assign is the condition of an If node but not a defined var,
            // then this Assign is a "constant".
            if (cast<If>(inst)->getCondition() == this) {
                const auto & dv = cast<If>(inst)->getDefined();
                if (LLVM_LIKELY(std::find(dv.begin(), dv.end(), this) == dv.end())) {
                    continue;
                }
            }
            return false;
        }
    }
    return true;
}

}

