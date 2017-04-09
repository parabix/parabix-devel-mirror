/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "linebreak_kernel.h"
#include <re/re_cc.h>
#include <re/re_toolchain.h> 
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <cc/cc_compiler.h>
#include <pablo/builder.hpp>
#include <llvm/IR/Module.h>

#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

#define UNICODE_LINE_BREAK (!AlgorithmOptionIsSet(DisableUnicodeLineBreak))

LineBreakKernelBuilder::LineBreakKernelBuilder (
IDISA::IDISA_Builder * iBuilder
, unsigned basisBitsCount)
: PabloKernel(iBuilder, "Parabix:lb_kernel", {Binding{iBuilder->getStreamSetTy(basisBitsCount), "basis"}}, {Binding{iBuilder->getStreamSetTy(1), "linebreak", Add1()}}) {

    CC_Compiler ccc(this, getInput(0));
    auto & builder = ccc.getBuilder();
    
    PabloAST * LineBreak = nullptr;
    PabloAST * LF = ccc.compileCC("LF", makeCC(0x0A), builder);
    PabloAST * CR = ccc.compileCC(makeCC(0x0D));
    PabloAST * LF_VT_FF_CR = ccc.compileCC(makeCC(0x0A, 0x0D));

    Zeroes * const zero = builder.createZeroes();
    Var * crlf = builder.createVar("crlf", zero);
    PabloBuilder crb = PabloBuilder::Create(builder);
#ifndef USE_LOOKAHEAD_CRLF
    PabloAST * cr1 = crb.createAdvance(CR, 1, "cr1");
    crb.createAssign(crlf, crb.createAnd(cr1, LF));
#else
    PabloAST * lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    crb.createAssign(crlf, crb.createAnd(CR, lookaheadLF));
#endif
    builder.createIf(CR, crb);
    
    Var * NEL_LS_PS = builder.createVar("NEL_LS_PS", zero);

    PabloAST * u8pfx = ccc.compileCC(makeCC(0xC0, 0xFF));
    PabloBuilder it = PabloBuilder::Create(builder);
    builder.createIf(u8pfx, it);
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF), it);

    //
    // Two-byte sequences
    Var * NEL = it.createVar("NEL", zero);
    PabloBuilder it2 = PabloBuilder::Create(it);
    it2.createAssign(NEL, it2.createAnd(it2.createAdvance(ccc.compileCC(makeCC(0xC2), it2), 1), ccc.compileCC(makeCC(0x85), it2)));
    it.createIf(u8pfx2, it2);

    //
    // Three-byte sequences

    Var * LS_PS = it.createVar("LS_PS", zero);
    PabloBuilder it3 = PabloBuilder::Create(it);
    it.createIf(u8pfx3, it3);
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(ccc.compileCC(makeCC(0xE2), it3), 1), ccc.compileCC(makeCC(0x80), it3));
    it3.createAssign(LS_PS, it3.createAnd(it3.createAdvance(E2_80, 1), ccc.compileCC(makeCC(0xA8,0xA9), it3)));
    it.createAssign(NEL_LS_PS, it.createOr(NEL, LS_PS));


    PabloAST * LB_chars = builder.createOr(LF_VT_FF_CR, NEL_LS_PS);
    PabloAST * UnicodeLineBreak = builder.createAnd(LB_chars, builder.createNot(crlf));  // count the CR, but not CRLF

    PabloAST * lb = UNICODE_LINE_BREAK ? UnicodeLineBreak : LF;
    PabloAST * unterminatedLineAtEOF = builder.createAtEOF(builder.createAdvance(builder.createNot(LB_chars), 1));
    LineBreak = builder.createOr(lb, unterminatedLineAtEOF);
    PabloAST * const r = builder.createExtract(getOutput(0), builder.getInteger(0));
    builder.createAssign(r, LineBreak);
#ifdef USE_LOOKAHEAD_CRLF
    setLookAhead(1);
#endif
}
