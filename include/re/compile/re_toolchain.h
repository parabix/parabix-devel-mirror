/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H

#include <map>
#include <set>
#include <string>
#include <llvm/Support/Compiler.h>

namespace llvm { namespace cl { class OptionCategory; } }
namespace pablo { class PabloKernel; class PabloAST; }
namespace re { class RE; class CC;}
namespace re { class Name; class Start; class End; class CC; class Seq; class Alt;
    class Rep; class Intersect; class Diff; class Range; class Group;
    class Assertion;
}

namespace re {

RE * resolveModesAndExternalSymbols(RE * r, bool globallyCaseInsensitive = false);

RE * excludeUnicodeLineBreak(RE * r);

RE * regular_expression_passes(RE * r);
    
}
#endif
