#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

#include <llvm/Support/Compiler.h>

namespace re {

class RE;

RE * simplifyRE(RE * re);

RE * removeUnneededCaptures(RE * r);

}

#endif // RE_SIMPLIFIER_H
