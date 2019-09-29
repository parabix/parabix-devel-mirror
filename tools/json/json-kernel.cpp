/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "json-kernel.h"
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <pablo/bixnum/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/re_name_resolve.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>

using namespace pablo;
using namespace kernel;

void JSONMarker::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * marker = getOutputStreamVar("marker");
    Var * callouts = getOutputStreamVar("callouts");
    Var * err = getOutputStreamVar("err");
}

void ValidateJSONString::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * strCallouts = getOutputStreamVar("strCallouts");
    Var * err = getOutputStreamVar("err");
}