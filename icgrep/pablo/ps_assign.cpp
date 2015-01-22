#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/pe_next.h>

namespace pablo {

bool Assign::superfluous() const {
    if (LLVM_UNLIKELY(isOutputAssignment())) {
        // If this Assign is an assignment to an output variable, it cannot be superfluous.
        return false;
    }
    for (const PabloAST * inst : users()) {       
        if (isa<Next>(inst)) {
            // If this Assign has a Next node, it cannot be superfluous.
            return false;
        }
        if (isa<If>(inst)) {
            // If this Assign is a defined variable of an If node, it cannot be superfluous.
            const auto & dv = cast<If>(inst)->getDefined();
            if (LLVM_UNLIKELY(std::find(dv.begin(), dv.end(), this) != dv.end())) {
                return false;
            }
        }
    }
    return true;
}

}

