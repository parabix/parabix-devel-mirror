/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PATTERN_COMPILER_H
#define PATTERN_COMPILER_H

#include <re/re_re.h>
#include <re/re_seq.h>
#include <cc/cc_compiler.h>

#include <string>
#include <list>
#include <map>

namespace pablo {
class PabloBlock;
class PabloKernel;
class PabloAST;
class Assign;
class Var;
}

namespace re {

class Pattern_Compiler {
public:

    Pattern_Compiler(pablo::PabloKernel & kernel);

    void compile(const std::vector<std::string> & patterns, pablo::PabloBuilder & pb, pablo::PabloAST *basisBits[], int dist, unsigned optPosition, int stepSize);

private:

    pablo::PabloKernel & mKernel;
};

}

#endif // PATTERN_COMPILER_H
