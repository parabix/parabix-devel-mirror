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

enum KwMarker {
    kwNull = 0,
    kwTrue,
    kwFalse,
};

enum KwLex {
    nMarker = 0,
    tMarker,
    fMarker
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

void JSONKeywordMarker::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    PabloAST * strSpan = getInputStreamSet("strSpan")[0];
    Var * const kwMarker = getOutputStreamVar("kwMarker");
    Var * const kwLex = getOutputStreamVar("kwLex");

    // null
    PabloAST * U = ccc.compileCC(re::makeByte(0x75));
    PabloAST * L = ccc.compileCC(re::makeByte(0x6C));
    // true
    PabloAST * R = ccc.compileCC(re::makeByte(0x72));
    PabloAST * E = ccc.compileCC(re::makeByte(0x65));
    // false
    PabloAST * A = ccc.compileCC(re::makeByte(0x61));
    PabloAST * S = ccc.compileCC(re::makeByte(0x73));

    PabloAST * notStrSpan = pb.createNot(strSpan);
    PabloAST * N = pb.createAnd(notStrSpan, lex[Lex::n]);
    PabloAST * T = pb.createAnd(notStrSpan, lex[Lex::t]);
    PabloAST * F = pb.createAnd(notStrSpan, lex[Lex::f]);

    PabloAST * advNU = pb.createAnd(U, pb.createAdvance(N, 1));
    PabloAST * advNUL = pb.createAnd(L, pb.createAdvance(advNU, 1));
    Var * seqNULL = pb.createVar("null", pb.createZeroes());
    auto itNUL = pb.createScope();
    pb.createIf(advNUL, itNUL);
    {
        PabloAST * advNULL = itNUL.createAnd(L, itNUL.createAdvance(advNUL, 1));
        itNUL.createAssign(seqNULL, advNULL);
    }

    PabloAST * advTR = pb.createAnd(R, pb.createAdvance(T, 1));
    PabloAST * advTRU = pb.createAnd(U, pb.createAdvance(advTR, 1));
    Var * seqTRUE = pb.createVar("true", pb.createZeroes());
    auto itTRU = pb.createScope();
    pb.createIf(advTRU, itTRU);
    {
        PabloAST * advTRUE = pb.createAnd(E, pb.createAdvance(advTRU, 1));
        itTRU.createAssign(seqTRUE, advTRUE);
    }

    PabloAST * advFA = pb.createAnd(A, pb.createAdvance(F, 1));
    PabloAST * advFAL = pb.createAnd(L, pb.createAdvance(advFA, 1));
    PabloAST * advFALS = pb.createAnd(S, pb.createAdvance(advFAL, 1));
    Var * seqFALSE = pb.createVar("false", pb.createZeroes());
    auto itFALS = pb.createScope();
    pb.createIf(advFALS, itFALS);
    {
        PabloAST * advFALSE = pb.createAnd(E, pb.createAdvance(advFALS, 1));
        itFALS.createAssign(seqFALSE, advFALSE);
    }

    pb.createAssign(pb.createExtract(kwMarker, pb.getInteger(KwMarker::kwNull)), seqNULL);
    pb.createAssign(pb.createExtract(kwMarker, pb.getInteger(KwMarker::kwFalse)), seqFALSE);
    pb.createAssign(pb.createExtract(kwMarker, pb.getInteger(KwMarker::kwTrue)), seqTRUE);
    pb.createAssign(pb.createExtract(kwLex, pb.getInteger(KwLex::nMarker)), N);
    pb.createAssign(pb.createExtract(kwLex, pb.getInteger(KwLex::tMarker)), T);
    pb.createAssign(pb.createExtract(kwLex, pb.getInteger(KwLex::fMarker)), F);
}

void JSONKeywordSpan::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> marker = getInputStreamSet("kwMarker");
    std::vector<PabloAST *> lex = getInputStreamSet("kwLex");
    Var * const kwSpan = getOutputStreamVar("kwSpan");
    Var * const kwErr = getOutputStreamVar("kwErr");

    Var * seqNULL = pb.createVar("null", marker[KwMarker::kwNull]);
    for (auto i = 1; i < 4; i++) {
        PabloAST * lookAhead = pb.createLookahead(marker[KwMarker::kwNull], i);
        pb.createAssign(seqNULL, pb.createOr(seqNULL, lookAhead));
    }

    Var * seqTRUE = pb.createVar("true", marker[KwMarker::kwTrue]);
    for (auto i = 1; i < 4; i++) {
        PabloAST * lookAhead = pb.createLookahead(marker[KwMarker::kwTrue], i);
        pb.createAssign(seqTRUE, pb.createOr(seqTRUE, lookAhead));
    }

    Var * seqFALSE = pb.createVar("false", marker[KwMarker::kwFalse]);
    for (auto i = 1; i < 5; i++) {
        PabloAST * lookAhead = pb.createLookahead(marker[KwMarker::kwFalse], i);
        pb.createAssign(seqFALSE, pb.createOr(seqFALSE, lookAhead));
    }

    PabloAST * lookAheadN = pb.createLookahead(marker[KwMarker::kwNull], 3);
    PabloAST * invalidN = pb.createXor(lex[KwLex::nMarker], lookAheadN);
    PabloAST * lookAheadT = pb.createLookahead(marker[KwMarker::kwTrue], 3);
    PabloAST * invalidT = pb.createXor(lex[KwLex::tMarker], lookAheadT);
    PabloAST * lookAheadF = pb.createLookahead(marker[KwMarker::kwFalse], 4);
    PabloAST * invalidF = pb.createXor(lex[KwLex::fMarker], lookAheadF);
    PabloAST * err = pb.createOr3(invalidN, invalidF, invalidT);

    pb.createAssign(pb.createExtract(kwSpan, pb.getInteger(KwMarker::kwNull)), seqNULL);
    pb.createAssign(pb.createExtract(kwSpan, pb.getInteger(KwMarker::kwTrue)), seqTRUE);
    pb.createAssign(pb.createExtract(kwSpan, pb.getInteger(KwMarker::kwFalse)), seqFALSE);
    pb.createAssign(pb.createExtract(kwErr, pb.getInteger(0)), err);
}

void JSONNumberSpan::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    PabloAST * strSpan = getInputStreamSet("strSpan")[0];
    Var * const nbrLex = getOutputStreamVar("nbrLex");
    Var * const nbrSpan = getOutputStreamVar("nbrSpan");
    Var * const nbrErr = getOutputStreamVar("nbrErr");

    PabloAST * alleE = pb.createOr(ccc.compileCC(re::makeByte(0x45)), ccc.compileCC(re::makeByte(0x65)));
    PabloAST * allDot = ccc.compileCC(re::makeByte(0x2E));
    PabloAST * allPlusMinus = pb.createOr(lex[Lex::hyphen], ccc.compileCC(re::makeByte(0x2B)));

    PabloAST * notStrSpan = pb.createNot(strSpan);
    PabloAST * hyphen = pb.createAnd(notStrSpan, lex[Lex::hyphen]);
    PabloAST * digit = pb.createAnd(notStrSpan, lex[Lex::digit]);
    PabloAST * eE = pb.createAnd(notStrSpan, alleE);
    PabloAST * dot = pb.createAnd(notStrSpan, allDot);
    PabloAST * plusMinus = pb.createAnd(notStrSpan, allPlusMinus);

    PabloAST * nondigit = pb.createNot(digit);
    PabloAST * nonDigitNorEe = pb.createAnd(nondigit, pb.createNot(eE));
    PabloAST * advHyphen = pb.createAnd(hyphen, pb.createAdvance(nonDigitNorEe, 1));

    PabloAST * nonDigitEePlusMinus = pb.createAnd(nonDigitNorEe, pb.createNot(plusMinus));
    PabloAST * nonDigitEePlusMinusDot = pb.createAnd(nonDigitEePlusMinus, pb.createNot(dot));
    PabloAST * advDigit = pb.createAnd(digit, pb.createAdvance(nonDigitEePlusMinusDot, 1));
    PabloAST * beginNbr = pb.createOr(advDigit, advHyphen);
    pb.createAssign(pb.createExtract(nbrLex, pb.getInteger(0)), beginNbr);

    PabloAST * errDot = pb.createAnd(pb.createAdvance(dot, 1), nondigit);
    PabloAST * errPlusMinus = pb.createAnd(pb.createAdvance(plusMinus, 1), nondigit);
    PabloAST * eENotPlusMinus = pb.createAnd(pb.createAdvance(eE, 1), pb.createNot(plusMinus));
    PabloAST * erreENotPlusMinus = pb.createAnd(eENotPlusMinus, nondigit);
    PabloAST * err = pb.createOr3(errDot, errPlusMinus, erreENotPlusMinus);
    pb.createAssign(pb.createExtract(nbrErr, pb.getInteger(0)), err);

    PabloAST * fstPartNbr = pb.createIntrinsicCall(Intrinsic::InclusiveSpan, {beginNbr, digit});
    PabloAST * sndPartNbr = pb.createIntrinsicCall(Intrinsic::InclusiveSpan, {eE, digit});
    PabloAST * trdPartNbr = pb.createIntrinsicCall(Intrinsic::InclusiveSpan, {dot, digit});
    PabloAST * finalNbr = pb.createOr3(fstPartNbr, sndPartNbr, trdPartNbr);
    pb.createAssign(pb.createExtract(nbrSpan, pb.getInteger(0)), finalNbr);
}

void ValidateJSONString::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> lex = getInputStreamSet("lex");
    Var * strCallouts = getOutputStreamVar("strCallouts");
    Var * err = getOutputStreamVar("err");
}