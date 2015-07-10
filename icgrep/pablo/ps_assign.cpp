#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/pe_next.h>
#include <pablo/function.h>

namespace pablo {

bool Assign::superfluous() const {
    for (const PabloAST * inst : users()) {       
        if (isa<Next>(inst) || isa<PabloFunction>(inst) || isa<If>(inst)) {
            return false;
        }
    }
    return true;
}

}

