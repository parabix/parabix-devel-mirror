/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/linebreak_kernel.h>

#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <pablo/pe_debugprint.h>
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
#include <kernel/util/debug_display.h>
#include <pablo/pe_debugprint.h>

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
        addAttribute(CanTerminateEarly());
        addAttribute(MayFatallyTerminate());
        addAttribute(SideEffecting());
    }
}

void UnixLinesKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    /*std::unique_ptr<CC_Compiler> ccc;
    if (getInputStreamSet("basis").size() == 1) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }*/
    std::unique_ptr<cc::CC_Compiler> ccc_u16_hi;
    std::unique_ptr<cc::CC_Compiler> ccc_u16_lo;

    //input
    std::vector<PabloAST *> u16bytes = getInputStreamSet("basis");
    //separate the 2 bytes into hi and lo 8 bits
    std::vector<PabloAST *> hiByte(u16bytes.begin()+ u16bytes.size()/2, u16bytes.end());
    std::vector<PabloAST *> loByte(u16bytes.begin(), u16bytes.begin()+ u16bytes.size()/2);
    //works only with bitStream
    ccc_u16_hi = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), hiByte);
    ccc_u16_lo = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), loByte);

    if (mNullMode == NullCharMode::Abort) {
        PabloAST * Abort_hiByte = ccc_u16_hi->compileCC(makeByte(0x0));
        PabloAST * Abort_loByte = ccc_u16_lo->compileCC(makeByte(0x0));
        pb.createTerminateAt(pb.createAnd(Abort_hiByte, Abort_loByte), pb.getInteger(0));
    }
    CC * breakCC = makeByte(0x0A);
    if (mNullMode == NullCharMode::Break) {
        breakCC = makeCC(breakCC, makeCC(0, &cc::Byte));
    }
    PabloAST * LB = ccc_u16_lo->compileCC(breakCC);
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
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
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
    std::unique_ptr<cc::CC_Compiler> ccc_u16_hi;
    std::unique_ptr<cc::CC_Compiler> ccc_u16_lo;

    //input
    std::vector<PabloAST *> u16bytes = getInputStreamSet("basis");
    //separate the 2 bytes into hi and lo 8 bits
    std::vector<PabloAST *> hiByte(u16bytes.begin()+ u16bytes.size()/2, u16bytes.end());
    std::vector<PabloAST *> loByte(u16bytes.begin(), u16bytes.begin()+ u16bytes.size()/2);
    //works only with bitStream
    ccc_u16_hi = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), hiByte);
    ccc_u16_lo = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), loByte);

    if (mNullMode == NullCharMode::Abort) {
        pb.createTerminateAt(ccc_u16_lo->compileCC(makeCC(0, &cc::Byte)), pb.getInteger(0));
    }
    PabloAST * const LF = pb.createExtract(getInput(1), pb.getInteger(0), "LF");
    PabloAST * const CR = ccc_u16_lo->compileCC(makeByte(0x0D)); //Mark all CR
    PabloAST * const LF_VT_FF_CR = ccc_u16_lo->compileCC("LF,VT,FF,CR", makeByte(0x0A, 0x0D), pb); 
    //Mark all lo bytes in range 0x0A and 0x0D
    Var * const LineBreak = pb.createVar("LineBreak", LF_VT_FF_CR);

    // Remove the CR of any CR+LF
    Var * const CR_before_LF = pb.createVar("CR_before_LF", pb.createZeroes());
    auto crb = pb.createScope();
    pb.createIf(CR, crb);
    PabloAST * const lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    crb.createAssign(CR_before_LF, crb.createAnd(CR, lookaheadLF));
    PabloAST * removedCRLF = crb.createXor(LineBreak, CR_before_LF);
    crb.createAssign(LineBreak, removedCRLF);

    PabloAST * NEL = ccc_u16_lo->compileCC(makeByte(0x85));
    pb.createAssign(LineBreak, pb.createOr(LineBreak, NEL));

    PabloAST * LS_PS_hiByte = ccc_u16_hi->compileCC(makeByte(0x20));
    PabloAST * LS_PS_loByte = ccc_u16_lo->compileCC(makeByte(0x28,0x29));
    PabloAST * LS_PS = pb.createAnd(LS_PS_hiByte,LS_PS_loByte);
    pb.createAssign(LineBreak, pb.createOr(LineBreak, LS_PS));

    PabloAST * const u16sur_1 = ccc_u16_hi->compileCC(makeByte(0xD8, 0xDB));
    PabloAST * const u16sur_2 = ccc_u16_hi->compileCC(makeByte(0xDC, 0xDF)); 
    //mark all low bytes
    PabloAST * const u16lo = pb.createOnes(); //Mark all lo bytes
    //mark valid surrogate pair
    PabloAST * const u16_sur = pb.createOr(u16sur_1, u16sur_2);
    //mark the prefix of valid surrogate pair
    PabloAST * const u16sur_final = pb.createAnd(u16_sur, u16sur_1);
    PabloAST * const u16valid = pb.createNot(u16sur_final, "u16prefix");
    //mark all 2 byte code units and final code unit of valid surrogare pairs
    Var * const u16valid_final = pb.createVar("u16valid_final", pb.createZeroes());
    pb.createAssign(u16valid_final, pb.createAnd(u16lo, u16valid));
    //pb.createDebugPrint(u16valid_final, "u16valid_final");

    // TODO: Invalid cases -  check for invalid surrogare pairs
    pb.createAssign(u16valid_final, pb.createAnd(u16valid_final, pb.createNot(CR_before_LF)));
    //output
    Var * const u8index = getOutputStreamVar("u8index");
    pb.createAssign(pb.createExtract(u8index, pb.getInteger(0)), u16valid_final);

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
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
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
              {Binding{"LineStarts", LineStarts}, Binding{"LineEnds", LineEnds}},
              {Binding{"LineSpans", LineSpans}}) {}

void LineSpansKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * lineStarts = getInputStreamSet("LineStarts")[0];
    PabloAST * lineEnds = getInputStreamSet("LineEnds")[0];
    PabloAST * lineSpans = pb.createIntrinsicCall(pablo::Intrinsic::InclusiveSpan, {lineStarts, lineEnds});
    pb.createAssign(pb.createExtract(getOutputStreamVar("LineSpans"), 0), lineSpans);
}

