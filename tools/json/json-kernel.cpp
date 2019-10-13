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

// This enum MUST reflect the type Lex on json.pablo file
enum Lex {
    lCurly = 0,
    rCurly,
    lBracket,
    rBracket,
    colon,
    comma,
    dQuote,
    hyphen,
    digit,
    backslash,
    n, // # first letter of null
    f, // # first letter of false
    t, // # first letter of true
    ws
};

void JSONStringMarker::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * const strMarker = getOutputStreamVar("marker");

    // keeping the names as the ones in paper PGJS (Lemire)
    PabloAST * B = lex[Lex::backslash];
    PabloAST * E = pb.createRepeat(1, pb.getInteger(0xAAAAAAAAAAAAAAAA, 64)); // constant
    PabloAST * O = pb.createRepeat(1, pb.getInteger(0x5555555555555555, 64)); // constant

    // identify 'starts' - backslashes not preceded by backslashes
    // paper does S = B & ~(B << 1), but we can't Advance(-1)
    PabloAST * notB = pb.createNot(B);
    PabloAST * S = pb.createAnd(B, pb.createAdvance(notB, 1));
    
    // paper does S & E, but we advanced notB by 1, so it became S & O
    PabloAST * ES = pb.createAnd(S, O);
    // we don't have add, so we will have to use ScanThru on ES
    // eg.:          ES = ..............1............
    //                B = ..............1111.........
    // ScanThru(ES, EB) = ..................1........
    // This way we don't need to filter after the sum :)
    PabloAST * EC = pb.createScanThru(ES, B);
    // Checking with odd instead of even because we're one step ahead
    PabloAST * OD1 = pb.createAnd(EC, pb.createNot(O));
    // inverted even/odd
    PabloAST * OS = pb.createAnd(S, E);
    // no add/clean again
    PabloAST * OC = pb.createScanThru(OS, B);
     // Checking with odd instead of even because we're one step ahead
    PabloAST * OD2 = pb.createAnd(OC, O);
    PabloAST * OD = pb.createOr(OD1, OD2);

    // There is a bug on the quotes
    PabloAST * Q = lex[Lex::dQuote];
    PabloAST * QEq = pb.createAnd(Q, pb.createNot(OD));

    pb.createAssign(pb.createExtract(strMarker, pb.getInteger(0)), QEq);
}

void JSONKeywordSpan::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * const strMarker = getOutputStreamVar("marker");
}

void ValidateJSONString::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * strCallouts = getOutputStreamVar("strCallouts");
    Var * err = getOutputStreamVar("err");
}