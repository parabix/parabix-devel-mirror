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
#include <kernels/kernel_builder.h>

#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

LineFeedKernelBuilder::LineFeedKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned codeUnitWidth)
: PabloKernel(b, "lf" + std::to_string(codeUnitWidth),
// input
#ifdef USE_DIRECT_LF_BUILDER
{Binding{b->getStreamSetTy(1, codeUnitWidth), "byteStream", FixedRate(), Principal()}},
#else
{Binding{b->getStreamSetTy(codeUnitWidth), "basis", FixedRate(), Principal()}},
#endif
// output
{Binding{b->getStreamSetTy(1), "lf"}}) {

}

void LineFeedKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    #ifdef USE_DIRECT_LF_BUILDER
    cc::Direct_CC_Compiler ccc(this, pb.createExtract(getInputStreamVar("byteStream"), pb.getInteger(0)));
    #else
    cc::Parabix_CC_Compiler ccc(this, getInputStreamSet("basis"));
    #endif
    PabloAST * LF = ccc.compileCC("LF", makeByte(0x0A), pb);
    pb.createAssign(pb.createExtract(getOutput(0), 0), LF);
}

LineBreakKernelBuilder::LineBreakKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned basisBitsCount)
: PabloKernel(b, "lb" + std::to_string(basisBitsCount),
// inputs
{Binding{b->getStreamSetTy(basisBitsCount), "basis", FixedRate(), Principal()}
,Binding{b->getStreamSetTy(1), "lf", FixedRate(), LookAhead(1)}},
// outputs
{Binding{b->getStreamSetTy(1), "linebreak", FixedRate(), Add1()}
,Binding{b->getStreamSetTy(1), "cr+lf"}}) {

}

void LineBreakKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(this, getInputStreamSet("basis"));

    Integer * const ZERO = pb.getInteger(0);

    PabloAST * const LF = pb.createExtract(getInput(1), ZERO, "LF");
    PabloAST * const CR = ccc.compileCC(makeByte(0x0D));
    PabloAST * const LF_VT_FF_CR = ccc.compileCC("LF,VT,FF,CR", makeByte(0x0A, 0x0D), pb);
    Var * const LineBreak = pb.createVar("LineBreak", LF_VT_FF_CR);

    // Remove the CR of any CR+LF
    Var * const CRLF = pb.createVar("CRLF", pb.createZeroes());
    auto crb = pb.createScope();
    pb.createIf(CR, crb);
    PabloAST * const lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    PabloAST * const crlf = crb.createAnd(CR, lookaheadLF);
    crb.createAssign(CRLF, crlf);
    PabloAST * removedCRLF = crb.createAnd(LineBreak, crb.createNot(CRLF));
    crb.createAssign(LineBreak, removedCRLF);
    // Record the CR marker of any CR+LF
    pb.createAssign(pb.createExtract(getOutput(1), ZERO), CRLF);

    // Check for Unicode Line Breaks
    PabloAST * u8pfx = ccc.compileCC(makeByte(0xC0, 0xFF));
    auto it = pb.createScope();
    pb.createIf(u8pfx, it);
    PabloAST * u8pfx2 = ccc.compileCC(makeByte(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(makeByte(0xE0, 0xEF), it);

    // Two-byte sequences
    auto it2 = it.createScope();
    it.createIf(u8pfx2, it2);
    PabloAST * NEL = it2.createAnd(it2.createAdvance(ccc.compileCC(makeByte(0xC2), it2), 1), ccc.compileCC(makeByte(0x85), it2), "NEL");
    it2.createAssign(LineBreak, it2.createOr(LineBreak, NEL));

    // Three-byte sequences
    auto it3 = it.createScope();
    it.createIf(u8pfx3, it3);
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(ccc.compileCC(makeByte(0xE2), it3), 1), ccc.compileCC(makeByte(0x80), it3));
    PabloAST * LS_PS = it3.createAnd(it3.createAdvance(E2_80, 1), ccc.compileCC(makeByte(0xA8,0xA9), it3), "LS_PS");
    it3.createAssign(LineBreak, it3.createOr(LineBreak, LS_PS));

    PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(pb.createNot(LineBreak), 1), "unterminatedLineAtEOF");
    pb.createAssign(pb.createExtract(getOutput(0), ZERO), pb.createOr(LineBreak, unterminatedLineAtEOF, "EOL"));
}
