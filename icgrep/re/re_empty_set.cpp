/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_empty_set.h"
#include <re/re_alt.h>

namespace re {

bool isEmptySet(RE * const re) {
    return llvm::isa<Alt>(re) && llvm::cast<Alt>(re)->empty();
}

RE * makeEmptySet() {
    return makeAlt();
}

}
