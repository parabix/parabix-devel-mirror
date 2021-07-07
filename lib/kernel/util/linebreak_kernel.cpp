/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/linebreak_kernel.h>

#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <pablo/builder.hpp>
#include <re/adt/re_cc.h>
#include <re/adt/re_name.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/ucd/ucd_compiler.hpp>
#include <idisa/idisa_builder.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/core/callback.h>
#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;
using namespace IDISA;

std::string sourceShape(StreamSet * s) {
    return std::to_string(s->getNumElements()) + "x" + std::to_string(s->getFieldWidth());
}

Bindings makeInputScalarBindings(Scalar * signalNullObject) {
    if (signalNullObject) {
        return {Binding{"handler_address", signalNullObject}};
    } else {
        return {};
    }
}

std::string EOF_annotation(UnterminatedLineAtEOF eofMode) {
    if (eofMode == UnterminatedLineAtEOF::Add1) return "+UnterminatedLinesAtEOF";
    return "";
}

std::string NullModeAnnotation(NullCharMode nullMode) {
    if (nullMode == NullCharMode::Break) return "+Null";
    if (nullMode == NullCharMode::Abort) return "+AbortOnNull";
    return "";
}

Bindings makeOutputBreakBindings(UnterminatedLineAtEOF eofMode, StreamSet * lb) {
    if (eofMode == UnterminatedLineAtEOF::Add1) {
        return {Binding{"LB", lb, FixedRate(), Add1()}};
    }
    return {Binding{"LB", lb}};
}

UnixLinesKernelBuilder::UnixLinesKernelBuilder(BuilderRef b,
                                               StreamSet * Basis,
                                               StreamSet * LineEnds,
                                               UnterminatedLineAtEOF eofMode,
                                               NullCharMode nullMode,
                                               Scalar * signalNullObject)
: PabloKernel(b, "UnixLines" + sourceShape(Basis) + EOF_annotation(eofMode) + NullModeAnnotation(nullMode),
              {Binding{"basis", Basis}},
              makeOutputBreakBindings(eofMode, LineEnds),
              makeInputScalarBindings(signalNullObject), {}),
mEOFmode(eofMode),
mNullMode(nullMode) {
    if (nullMode == NullCharMode::Abort) {
        addAttribute(MayFatallyTerminate());
        addAttribute(SideEffecting());
    }
}

void UnixLinesKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<CC_Compiler> ccc;
    if (getInputStreamSet("basis").size() == 1) {
        ccc = std::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    if (mNullMode == NullCharMode::Abort) {
        pb.createTerminateAt(ccc->compileCC(makeCC(0, &cc::Byte)), pb.getInteger(0));
    }
    CC * breakCC = makeByte(0x0A);
    if (mNullMode == NullCharMode::Break) {
        breakCC = makeCC(breakCC, makeCC(0, &cc::Byte));
    }
    PabloAST * LB = ccc->compileCC(breakCC);
    if (mEOFmode == UnterminatedLineAtEOF::Add1) {
        PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(pb.createNot(LB), 1), "unterminatedLineAtEOF");
        LB = pb.createOr(LB, unterminatedLineAtEOF);
    }
    pb.createAssign(pb.createExtract(getOutput(0), 0), LB);
}

/*  Helper class to provide a LF stream as input to UnicodeLines logic,
    necessary so that LookAhead operations on the stream are available
    for CRLF processing.  */

class LineFeedKernelBuilder final : public pablo::PabloKernel {
public:
    LineFeedKernelBuilder(BuilderRef b, StreamSet * Basis, StreamSet * LineFeedStream);
protected:
    void generatePabloMethod() override;
    const unsigned mNumOfStreams;
    const unsigned mStreamFieldWidth;
};

LineFeedKernelBuilder::LineFeedKernelBuilder(BuilderRef b, StreamSet * Basis, StreamSet * LineFeedStream)
: PabloKernel(b, "lf" + sourceShape(Basis),
              // input
{Binding{"basis", Basis}},
              // output
{Binding{"lf", LineFeedStream}}),
mNumOfStreams(Basis->getNumElements()),
mStreamFieldWidth(Basis->getFieldWidth())
{
}

void LineFeedKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<CC_Compiler> ccc;
    if (mNumOfStreams == 1) {
        ccc = std::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    PabloAST * LF = ccc->compileCC("LF", makeByte(0x0A), pb);
    pb.createAssign(pb.createExtract(getOutput(0), 0), LF);
}

/* The kernel that implements the core UnicodeLines logic */

class UnicodeLinesKernelBuilder final : public pablo::PabloKernel {
public:
    UnicodeLinesKernelBuilder(BuilderRef b,
                              StreamSet * Basis,
                              StreamSet * LF,
                              StreamSet * UnicodeLB,
                              StreamSet * u8index,
                              UnterminatedLineAtEOF m = UnterminatedLineAtEOF::Ignore,
                              NullCharMode nullMode = NullCharMode::Data,
                              Scalar * signalNullObject = nullptr);
protected:
    void generatePabloMethod() override;
    const UnterminatedLineAtEOF mEOFmode;
    const NullCharMode mNullMode;
};

UnicodeLinesKernelBuilder::UnicodeLinesKernelBuilder(BuilderRef b,
                                                     StreamSet * Basis,
                                                     StreamSet * LF,
                                                     StreamSet * LineEnds,
                                                     StreamSet * u8index,
                                                     UnterminatedLineAtEOF eofMode,
                                                     NullCharMode nullMode,
                                                     Scalar * signalNullObject)
: PabloKernel(b, "UnicodeLB" + sourceShape(Basis) + EOF_annotation(eofMode) + NullModeAnnotation(nullMode),
              {Binding{"basis", Basis},
                  Binding{"lf", LF, FixedRate(), LookAhead(1)}},
              makeOutputBreakBindings(eofMode, LineEnds),
              makeInputScalarBindings(signalNullObject), {}),
mEOFmode(eofMode),
mNullMode(nullMode) {
    if (eofMode == UnterminatedLineAtEOF::Add1) {
        getOutputStreamSetBindings().emplace_back("u8index", u8index, FixedRate(), Add1());
    } else {
        getOutputStreamSetBindings().emplace_back("u8index", u8index);
    }
    if (nullMode == NullCharMode::Abort) {
        addAttribute(CanTerminateEarly());
        addAttribute(MayFatallyTerminate());
        addAttribute(SideEffecting());
    }
}

void UnicodeLinesKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<CC_Compiler> ccc;
    if (getInputStreamSet("basis").size() == 1) {
        ccc = std::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    if (mNullMode == NullCharMode::Abort) {
        pb.createTerminateAt(ccc->compileCC(makeCC(0, &cc::Byte)), pb.getInteger(0));
    }
    PabloAST * const LF = pb.createExtract(getInput(1), pb.getInteger(0), "LF");
    PabloAST * const CR = ccc->compileCC(makeByte(0x0D));
    PabloAST * const LF_VT_FF_CR = ccc->compileCC("LF,VT,FF,CR", makeByte(0x0A, 0x0D), pb);
    Var * const LineBreak = pb.createVar("LineBreak", LF_VT_FF_CR);

    // Remove the CR of any CR+LF
    Var * const CR_before_LF = pb.createVar("CR_before_LF", pb.createZeroes());
    auto crb = pb.createScope();
    pb.createIf(CR, crb);
    PabloAST * const lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    crb.createAssign(CR_before_LF, crb.createAnd(CR, lookaheadLF));
    PabloAST * removedCRLF = crb.createXor(LineBreak, CR_before_LF);
    crb.createAssign(LineBreak, removedCRLF);

    Zeroes * const ZEROES = pb.createZeroes();
    PabloAST * const u8pfx = ccc->compileCC(makeByte(0xC0, 0xFF));

    Var * const nonFinal = pb.createVar("nonFinal", u8pfx);
    Var * const u8invalid = pb.createVar("u8invalid", ZEROES);
    Var * const valid_pfx = pb.createVar("valid_pfx", u8pfx);

    auto it = pb.createScope();
    pb.createIf(u8pfx, it);
    PabloAST * const u8pfx2 = ccc->compileCC(makeByte(0xC2, 0xDF), it);
    PabloAST * const u8pfx3 = ccc->compileCC(makeByte(0xE0, 0xEF), it);
    PabloAST * const u8pfx4 = ccc->compileCC(makeByte(0xF0, 0xF4), it);
    PabloAST * const u8suffix = ccc->compileCC("u8suffix", makeByte(0x80, 0xBF), it);

    //
    // Two-byte sequences
    Var * const anyscope = it.createVar("anyscope", ZEROES);
    auto it2 = it.createScope();
    it.createIf(u8pfx2, it2);
    it2.createAssign(anyscope, it2.createAdvance(u8pfx2, 1));
    PabloAST * NEL = it2.createAnd(it2.createAdvance(ccc->compileCC(makeByte(0xC2), it2), 1), ccc->compileCC(makeByte(0x85), it2), "NEL");
    it2.createAssign(LineBreak, it2.createOr(LineBreak, NEL));


    //
    // Three-byte sequences
    Var * const EF_invalid = it.createVar("EF_invalid", ZEROES);
    auto it3 = it.createScope();
    it.createIf(u8pfx3, it3);
    PabloAST * const u8scope32 = it3.createAdvance(u8pfx3, 1);
    it3.createAssign(nonFinal, it3.createOr(nonFinal, u8scope32));
    PabloAST * const u8scope33 = it3.createAdvance(u8pfx3, 2);
    PabloAST * const u8scope3X = it3.createOr(u8scope32, u8scope33);
    it3.createAssign(anyscope, it3.createOr(anyscope, u8scope3X));
    PabloAST * const E0_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xE0), it3), 1), ccc->compileCC(makeByte(0x80, 0x9F), it3));
    PabloAST * const ED_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xED), it3), 1), ccc->compileCC(makeByte(0xA0, 0xBF), it3));
    PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
    it3.createAssign(EF_invalid, EX_invalid);
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xE2), it3), 1), ccc->compileCC(makeByte(0x80), it3));
    PabloAST * LS_PS = it3.createAnd(it3.createAdvance(E2_80, 1), ccc->compileCC(makeByte(0xA8,0xA9), it3), "LS_PS");
    it3.createAssign(LineBreak, it3.createOr(LineBreak, LS_PS));

    //
    // Four-byte sequences
    auto it4 = it.createScope();
    it.createIf(u8pfx4, it4);
    PabloAST * const u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * const u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * const u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    PabloAST * const u8scope4nonfinal = it4.createOr(u8scope42, u8scope43);
    it4.createAssign(nonFinal, it4.createOr(nonFinal, u8scope4nonfinal));
    PabloAST * const u8scope4X = it4.createOr(u8scope4nonfinal, u8scope44);
    it4.createAssign(anyscope, it4.createOr(anyscope, u8scope4X));
    PabloAST * const F0_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF0), it4), 1), ccc->compileCC(makeByte(0x80, 0x8F), it4));
    PabloAST * const F4_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF4), it4), 1), ccc->compileCC(makeByte(0x90, 0xBF), it4));
    PabloAST * const FX_invalid = it4.createOr(F0_invalid, F4_invalid);
    it4.createAssign(EF_invalid, it4.createOr(EF_invalid, FX_invalid));

    //
    // Invalid cases
    PabloAST * const legalpfx = it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * const mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * const pfx_invalid = it.createXor(valid_pfx, legalpfx);
    it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
    PabloAST * const u8valid = it.createNot(u8invalid, "u8valid");
    //
    //
    it.createAssign(nonFinal, it.createAnd(nonFinal, u8valid));
    pb.createAssign(nonFinal, pb.createOr(nonFinal, CR_before_LF));

    Var * const u8index = getOutputStreamVar("u8index");
    PabloAST * u8final = pb.createNot(nonFinal);
    pb.createAssign(pb.createExtract(u8index, pb.getInteger(0)), u8final);
    PabloAST * notLB = pb.createNot(LineBreak);
    if (mEOFmode == UnterminatedLineAtEOF::Add1) {
        PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(notLB, 1), "unterminatedLineAtEOF");
        pb.createAssign(LineBreak, pb.createOr(LineBreak, unterminatedLineAtEOF));
    }
    pb.createAssign(pb.createExtract(getOutputStreamVar("LB"), pb.getInteger(0)), LineBreak);
}

void UnicodeLinesLogic(const std::unique_ptr<kernel::ProgramBuilder> & P,
                       StreamSet * Basis,
                       StreamSet * UnicodeLB,
                       StreamSet * u8index,
                       UnterminatedLineAtEOF m,
                       NullCharMode nullMode,
                       Scalar * signalNullObject) {
    StreamSet * const LF = P->CreateStreamSet();
    P->CreateKernelCall<LineFeedKernelBuilder>(Basis, LF);
    Kernel * k = P->CreateKernelCall<UnicodeLinesKernelBuilder>
         (Basis, LF, UnicodeLB, u8index, m, nullMode, signalNullObject);
    if (nullMode == NullCharMode::Abort) {
        k->link("signal_dispatcher", kernel::signal_dispatcher);
    }
}

NullDelimiterKernel::NullDelimiterKernel(BuilderRef b,
                                         StreamSet * Source,
                                         StreamSet * Terminators,
                                         UnterminatedLineAtEOF eofMode)
: PabloKernel(b, "nullDelim" + sourceShape(Source) + EOF_annotation(eofMode),
              {Binding{"Source", Source}},
              makeOutputBreakBindings(eofMode, Terminators),
              {}, {}),
mEOFmode(eofMode) {}

void NullDelimiterKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<CC_Compiler> ccc;
    if (getInputStreamSet("Source").size() == 1) {
        ccc = std::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    PabloAST * NUL = ccc->compileCC("NUL", makeByte(0x0), pb);
    if (mEOFmode == UnterminatedLineAtEOF::Add1) {
        PabloAST * unterminatedRecordAtEOF = pb.createAtEOF(pb.createAdvance(pb.createNot(NUL), 1), "unterminatedRecordAtEOF");
        NUL = pb.createOr(NUL, unterminatedRecordAtEOF);
    }
    pb.createAssign(pb.createExtract(getOutput(0), 0), NUL);
}

LineStartsKernel::LineStartsKernel(BuilderRef b, StreamSet * LineEnds, StreamSet * LineStarts)
: PabloKernel(b, "LineStarts",
              {Binding{"LineEnds", LineEnds}},
              {Binding{"LineStarts", LineStarts}}) {}

void LineStartsKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * lineEnds = getInputStreamSet("LineEnds")[0];
    // Line starts are the positions after every line end, as well as the initial position.
    PabloAST * lineStarts = pb.createInFile(pb.createNot(pb.createAdvance(pb.createNot(lineEnds), 1)));
    pb.createAssign(pb.createExtract(getOutputStreamVar("LineStarts"), 0), lineStarts);
}

LineSpansKernel::LineSpansKernel(BuilderRef b, StreamSet * LineStarts, StreamSet * LineEnds, StreamSet * LineSpans)
: PabloKernel(b, "LineSpans",
              {Binding{"LineStarts", LineStarts}, Binding{"LineEnds", LineEnds, FixedRate(), Principal()}},
              {Binding{"LineSpans", LineSpans}}) {}

void LineSpansKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * lineStarts = getInputStreamSet("LineStarts")[0];
    PabloAST * lineEnds = getInputStreamSet("LineEnds")[0];
    PabloAST * lineSpans = pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {lineStarts, lineEnds});
    pb.createAssign(pb.createExtract(getOutputStreamVar("LineSpans"), 0), lineSpans);
}

