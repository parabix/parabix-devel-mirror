#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/pe_next.h>

namespace pablo {

bool Assign::isConstant() const {
    if (LLVM_UNLIKELY(isOutputAssignment())) {
        return false;
    }
    for (const PabloAST * inst : users()) {
        if (isa<Next>(inst) || isa<If>(inst)) {
            return false;
        }
    }
    return true;
}

}

