/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

#include <llvm/Support/Compiler.h>

namespace re {

class RE;

RE * simplifyRE(RE * re);

RE * removeUnneededCaptures(RE * r);

}

#endif // RE_SIMPLIFIER_H
