/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_not.h"
#include "pe_all.h"

namespace pablo {

PabloE * make_not(PabloE * expr) {
    if (All * all = dyn_cast<All>(expr)) {
        return make_all(!all->getValue());
    }
    else if (Not * pe_not = dyn_cast<Not>(expr)) {
        return pe_not->getExpr();
    }
    return new Not(expr);
}

}
