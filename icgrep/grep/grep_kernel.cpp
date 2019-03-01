/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <cc/alphabet.h>
#include <re/printer_re.h>
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_toolchain.h>
#include <re/re_reverse.h>
#include <grep/grep_engine.h>
#include <pablo/codegenstate.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <pablo/pe_infile.h>
#include <pablo/boolean.h>
#include <pablo/pe_count.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_pack.h>
#include <cc/cc_compiler.h>         // for CC_Compiler
#include <cc/cc_compiler_target.h>
#include <cc/alphabet.h>
#include <cc/multiplex_CCs.h>
#include <re/re_compiler.h>
#include <UCD/ucd_compiler.hpp>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

using ClassTypeId = pablo::PabloAST::ClassTypeId;
using op3_pair_t = cc::CC_Compiler::op3_pair_t;

UnicodeLineBreakKernel::UnicodeLineBreakKernel(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb,
              "UTF8_LB",
              {Binding{kb->getStreamSetTy(8), "basis"}, Binding{kb->getStreamSetTy(1), "lf", FixedRate(), LookAhead(1)}},
              {Binding{kb->getStreamSetTy(1, 1), "UTF8_LB", FixedRate()}}) {
}

void UnicodeLineBreakKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
    UCD::UCDCompiler ucdCompiler(ccc);
    Name * breakChars = re::makeName("breakChars", makeCC(makeCC(makeCC(0x0A, 0x0D), makeCC(0x85)), makeCC(0x2028,0x2029)));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(breakChars, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(breakChars);
    if (f == nameMap.end()) llvm::report_fatal_error("UnicodeLineBreakKernel compilation failure");
    PabloAST * breakStream = f-> second;
    PabloAST * const LF = pb.createExtract(getInput(1), pb.getInteger(0), "LF");
    PabloAST * const CR = ccc.compileCC(makeByte(0x0D));
    Var * const CR_before_LF = pb.createVar("CR_before_LF", pb.createZeroes());
    auto crb = pb.createScope();
    pb.createIf(CR, crb);
    PabloAST * const lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    crb.createAssign(CR_before_LF, crb.createAnd(CR, lookaheadLF));
    breakStream = pb.createXor(breakStream, CR_before_LF);  // Remove CR_before_LF from breakStream
    Var * const UTF8_LB = getOutputStreamVar("UTF8_LB");
    pb.createAssign(pb.createExtract(UTF8_LB, pb.getInteger(0)), breakStream);
}

void UTF8_index::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("source"));
    }

    Zeroes * const ZEROES = pb.createZeroes();
    PabloAST * const u8pfx = ccc->compileCC(makeByte(0xC0, 0xFF));
    const op3_pair_t op3 = std::make_pair(ClassTypeId::Or, ClassTypeId::Or);

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

    //
    // Three-byte sequences
    Var * const EF_invalid = it.createVar("EF_invalid", ZEROES);
    auto it3 = it.createScope();
    it.createIf(u8pfx3, it3);
    PabloAST * const u8scope32 = it3.createAdvance(u8pfx3, 1);
    it3.createAssign(nonFinal, it3.createOr(nonFinal, u8scope32));
    PabloAST * const u8scope33 = it3.createAdvance(u8pfx3, 2);
    it3.createAssign(anyscope, ccc->createCCOp3(op3, anyscope, u8scope32, u8scope33, it3));
    PabloAST * const E0_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xE0), it3), 1), ccc->compileCC(makeByte(0x80, 0x9F), it3));
    PabloAST * const ED_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xED), it3), 1), ccc->compileCC(makeByte(0xA0, 0xBF), it3));
    PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
    it3.createAssign(EF_invalid, EX_invalid);

    //
    // Four-byte sequences
    auto it4 = it.createScope();
    it.createIf(u8pfx4, it4);
    PabloAST * const u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * const u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * const u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(nonFinal, ccc->createCCOp3(op3, nonFinal, u8scope42, u8scope43, it4));
    PabloAST * const u8scope4X = ccc->createCCOp3(op3, u8scope42, u8scope43, u8scope44, it4);
    it4.createAssign(anyscope, it4.createOr(anyscope, u8scope4X));
    PabloAST * const F0_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF0), it4), 1), ccc->compileCC(makeByte(0x80, 0x8F), it4));
    PabloAST * const F4_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF4), it4), 1), ccc->compileCC(makeByte(0x90, 0xBF), it4));
    it4.createAssign(EF_invalid, ccc->createCCOp3(op3, EF_invalid, F0_invalid, F4_invalid, it4));

    //
    // Invalid cases
    PabloAST * const legalpfx = ccc->createCCOp3(op3, u8pfx2, u8pfx3, u8pfx4, it);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * const mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * const pfx_invalid = it.createXor(valid_pfx, legalpfx);
    it.createAssign(u8invalid, ccc->createCCOp3(op3, pfx_invalid, mismatch, EF_invalid, it));
    PabloAST * const u8valid = it.createNot(u8invalid, "u8valid");
    //
    //
    it.createAssign(nonFinal, it.createAnd(nonFinal, u8valid));
    //pb.createAssign(nonFinal, pb.createOr(nonFinal, CRLF));
    //PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(pb.createNot(LineBreak), 1), "unterminatedLineAtEOF");

    Var * const u8index = getOutputStreamVar("u8index");
    PabloAST * u8final = pb.createInFile(pb.createNot(nonFinal));
    pb.createAssign(pb.createExtract(u8index, pb.getInteger(0)), u8final);
}

UTF8_index::UTF8_index(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * Source, StreamSet * u8index)
: PabloKernel(kb, "UTF8_index_" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()),
// input
{Binding{"source", Source}},
// output
{Binding{"u8index", u8index}}) {

}

void RequiredStreams_UTF8::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("source"));
    }

    PabloAST * const LF = pb.createExtract(getInput(1), pb.getInteger(0), "LF");
    PabloAST * const CR = ccc->compileCC(makeByte(0x0D));
    PabloAST * const LF_VT_FF_CR = ccc->compileCC("LF,VT,FF,CR", makeByte(0x0A, 0x0D), pb);
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


    Zeroes * const ZEROES = pb.createZeroes();
    PabloAST * const u8pfx = ccc->compileCC(makeByte(0xC0, 0xFF));
    const op3_pair_t opOrAnd3 = std::make_pair(ClassTypeId::Or, ClassTypeId::And);
    const op3_pair_t opOr3 = std::make_pair(ClassTypeId::Or, ClassTypeId::Or);

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
    PabloAST * NEL1 = it2.createAdvance(ccc->compileCC(makeByte(0xC2), it2), 1);
    PabloAST * NEL2 = ccc->compileCC(makeByte(0x85), it2);
    it2.createAssign(LineBreak, ccc->createCCOp3(opOrAnd3, LineBreak, NEL1, NEL2, it2));

    //
    // Three-byte sequences
    Var * const EF_invalid = it.createVar("EF_invalid", ZEROES);
    auto it3 = it.createScope();
    it.createIf(u8pfx3, it3);
    PabloAST * const u8scope32 = it3.createAdvance(u8pfx3, 1);
    it3.createAssign(nonFinal, it3.createOr(nonFinal, u8scope32));
    PabloAST * const u8scope33 = it3.createAdvance(u8pfx3, 2);
    it3.createAssign(anyscope, ccc->createCCOp3(opOr3, anyscope, u8scope32, u8scope33, it3));
    PabloAST * const E0_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xE0), it3), 1), ccc->compileCC(makeByte(0x80, 0x9F), it3));
    PabloAST * const ED_invalid = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xED), it3), 1), ccc->compileCC(makeByte(0xA0, 0xBF), it3));
    PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
    it3.createAssign(EF_invalid, EX_invalid);
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(ccc->compileCC(makeByte(0xE2), it3), 1), ccc->compileCC(makeByte(0x80), it3));
    PabloAST * LS_PS1 = it3.createAdvance(E2_80, 1);
    PabloAST * LS_PS2 = ccc->compileCC(makeByte(0xA8,0xA9), it3);
    it3.createAssign(LineBreak, ccc->createCCOp3(opOrAnd3, LineBreak, LS_PS1, LS_PS2, it3));

    //
    // Four-byte sequences
    auto it4 = it.createScope();
    it.createIf(u8pfx4, it4);
    PabloAST * const u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * const u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * const u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(nonFinal, ccc->createCCOp3(opOr3, nonFinal, u8scope42, u8scope43, it4));
    PabloAST * const u8scope4X = ccc->createCCOp3(opOr3, u8scope42, u8scope43, u8scope44, it4);
    it4.createAssign(anyscope, it4.createOr(anyscope, u8scope4X));
    PabloAST * const F0_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF0), it4), 1), ccc->compileCC(makeByte(0x80, 0x8F), it4));
    PabloAST * const F4_invalid = it4.createAnd(it4.createAdvance(ccc->compileCC(makeByte(0xF4), it4), 1), ccc->compileCC(makeByte(0x90, 0xBF), it4));
    it4.createAssign(EF_invalid, ccc->createCCOp3(opOr3, EF_invalid, F0_invalid, F4_invalid, it4));

    //
    // Invalid cases
    PabloAST * const legalpfx = ccc->createCCOp3(opOr3, u8pfx2, u8pfx3, u8pfx4, it);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * const mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * const pfx_invalid = it.createXor(valid_pfx, legalpfx);
    it.createAssign(u8invalid, ccc->createCCOp3(opOr3, pfx_invalid, mismatch, EF_invalid, it));
    PabloAST * const u8valid = it.createNot(u8invalid, "u8valid");
    //
    //
    it.createAssign(nonFinal, it.createAnd(nonFinal, u8valid));
    pb.createAssign(nonFinal, pb.createOr(nonFinal, CRLF));

    Var * const u8index = getOutputStreamVar("u8index");
    PabloAST * u8final = pb.createNot(nonFinal);
    pb.createAssign(pb.createExtract(u8index, pb.getInteger(0)), u8final);
    pb.createAssign(pb.createExtract(getOutputStreamVar("UnicodeLB"), pb.getInteger(0)), LineBreak);
}

RequiredStreams_UTF8::RequiredStreams_UTF8(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * Source, StreamSet * LineFeedStream, StreamSet * RequiredStreams, StreamSet * UnicodeLB)
: PabloKernel(kb, "RequiredStreams_UTF8" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()),
// input
{Binding{"source", Source},
 Binding{"lf", LineFeedStream, FixedRate(), LookAhead(1)}},
// output
{Binding{"u8index", RequiredStreams, FixedRate(), Add1()},
 Binding{"UnicodeLB", UnicodeLB, FixedRate()}}) {

}

void RequiredStreams_UTF16::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
    const op3_pair_t opAndOr3 = std::make_pair(ClassTypeId::And, ClassTypeId::Or);

    PabloAST * u16hi_hi_surrogate = ccc.compileCC(makeCC(0xD800, 0xDBFF, &cc::UTF16));    //u16hi_hi_surrogate = [\xD8-\xDB]
    PabloAST * u16hi_lo_surrogate = ccc.compileCC(makeCC(0xDC00, 0xDFFF, &cc::UTF16));    //u16hi_lo_surrogate = [\xDC-\xDF]

    PabloAST * invalidTemp = pb.createAdvance(u16hi_hi_surrogate, 1, "InvalidTemp");
    PabloAST * u16invalid = pb.createXor(invalidTemp, u16hi_lo_surrogate, "u16invalid");

    PabloAST * u16valid = pb.createNot(u16invalid, "u16valid");
    PabloAST * nonFinal = pb.createAnd(u16hi_hi_surrogate, u16valid, "nonfinal");

    PabloAST * fstUTF16 = ccc.compileCC(makeCC(0x0000, 0xD7FF, &cc::UTF16));
    PabloAST * sndUTF16 = ccc.compileCC(makeCC(0xE000, 0xFFFF, &cc::UTF16));
    PabloAST * u16single = ccc.createCCOp3(opAndOr3, pb.createNot(u16invalid), fstUTF16, sndUTF16, pb);

    PabloAST * const nonFinalCodeUnits = pb.createExtract(getInput(1), pb.getInteger(0));
    PabloAST * const initial = pb.createOr(u16single, u16hi_hi_surrogate, "initial");
    PabloAST * const final = pb.createNot(ccc.createCCOp3(opAndOr3, u16hi_hi_surrogate, u16invalid, nonFinalCodeUnits, pb), "final");

    Var * const required = getOutputStreamVar("required");
    pb.createAssign(pb.createExtract(required, pb.getInteger(0)), initial);
    pb.createAssign(pb.createExtract(required, pb.getInteger(1)), nonFinal);
    pb.createAssign(pb.createExtract(required, pb.getInteger(2)), final);

}

RequiredStreams_UTF16::RequiredStreams_UTF16(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb, "RequiredStreams_UTF16",
// inputs
{Binding{kb->getStreamSetTy(8), "basis"}},
// output
{Binding{kb->getStreamSetTy(3), "required", FixedRate(), Add1()}}) {

}

void GrepKernelOptions::setIndexingAlphabet(const cc::Alphabet * a) {mIndexingAlphabet = a;}
void GrepKernelOptions::setRE(RE * e) {mRE = e;}
void GrepKernelOptions::setPrefixRE(RE * e) {mPrefixRE = e;}
void GrepKernelOptions::setSource(StreamSet * s) {mSource = s;}
void GrepKernelOptions::setResults(StreamSet * r) {mResults = r;}
void GrepKernelOptions::addExternal(std::string name, StreamSet * strm) {
    mExternals.emplace_back(name, strm);
}
void GrepKernelOptions::addAlphabet(std::shared_ptr<cc::Alphabet> a, StreamSet * basis) {
    mAlphabets.emplace_back(a, basis);
}

Bindings GrepKernelOptions::streamSetInputBindings() {
    Bindings inputs;
    inputs.emplace_back("basis", mSource);
    for (const auto & e : mExternals) {
        inputs.emplace_back(e.first, e.second);
    }
    for (const auto & a : mAlphabets) {
        inputs.emplace_back(a.first->getName() + "_basis", a.second);
    }
    return inputs;
}

Bindings GrepKernelOptions::streamSetOutputBindings() {
    return {Binding{"matches", mResults, FixedRate(), Add1()}};
}

Bindings GrepKernelOptions::scalarInputBindings() {
    return {};
}

Bindings GrepKernelOptions::scalarOutputBindings() {
    return {};
}

std::string GrepKernelOptions::getSignature() {
    if (mSignature == "") {
        mSignature = std::to_string(mSource->getNumElements()) + "x" + std::to_string(mSource->getFieldWidth());
        mSignature += "/" + mIndexingAlphabet->getName();
        for (auto e: mExternals) {
            mSignature += "_" + e.first;
        }
        for (auto a: mAlphabets) {
            mSignature += "_" + a.first->getName();
        }
        if (mPrefixRE) {
            mSignature += ":" + Printer_RE::PrintRE(mPrefixRE);
        }
        mSignature += ":" + Printer_RE::PrintRE(mRE);
    }
    return mSignature;
}

ICGrepKernel::ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::unique_ptr<GrepKernelOptions> options)
: PabloKernel(b, "ic" + getStringHash(options->getSignature()),
options->streamSetInputBindings(),
options->streamSetOutputBindings(),
options->scalarInputBindings(),
options->scalarOutputBindings()),
mOptions(std::move(options)) {
    addAttribute(InfrequentlyUsed());
}

std::string ICGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mOptions->getSignature();
}

void ICGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    //cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"), mOptions->mBasisSetNumbering);
    RE_Compiler re_compiler(getEntryScope(), *ccc.get(), *(mOptions->mIndexingAlphabet));
    for (const auto & e : mOptions->mExternals) {
        re_compiler.addPrecompiled(e.first, pb.createExtract(getInputStreamVar(e.first), pb.getInteger(0)));
    }
    for (const auto & a : mOptions->mAlphabets) {
        auto & alpha = a.first;
        auto mpx_basis = getInputStreamSet(alpha->getName() + "_basis");
        re_compiler.addAlphabet(alpha, mpx_basis);
    }
    if (mOptions->mPrefixRE) {
        PabloAST * const prefixMatches = re_compiler.compile(mOptions->mPrefixRE);
        Var * const final_matches = pb.createVar("final_matches", pb.createZeroes());
        PabloBlock * scope1 = getEntryScope()->createScope();
        pb.createIf(prefixMatches, scope1);

        PabloAST * u8bytes = pb.createExtract(getInput(0), pb.getInteger(0));
        PabloAST * nybbles[2];
        nybbles[0] = scope1->createPackL(scope1->getInteger(8), u8bytes);
        nybbles[1] = scope1->createPackH(scope1->getInteger(8), u8bytes);

        PabloAST * bitpairs[4];
        for (unsigned i = 0; i < 2; i++) {
            bitpairs[2*i] = scope1->createPackL(scope1->getInteger(4), nybbles[i]);
            bitpairs[2*i + 1] = scope1->createPackH(scope1->getInteger(4), nybbles[i]);
        }

        std::vector<PabloAST *> basis(8);
        for (unsigned i = 0; i < 4; i++) {
            basis[2*i] = scope1->createPackL(scope1->getInteger(2), bitpairs[i]);
            basis[2*i + 1] = scope1->createPackH(scope1->getInteger(2), bitpairs[i]);
        }

        cc::Parabix_CC_Compiler_Builder ccc(scope1, basis);
        RE_Compiler re_compiler(scope1, ccc, *(mOptions->mIndexingAlphabet));
        scope1->createAssign(final_matches, re_compiler.compile(mOptions->mRE, prefixMatches));
        Var * const output = getOutputStreamVar("matches");
        pb.createAssign(pb.createExtract(output, pb.getInteger(0)), final_matches);
    } else {
        PabloAST * const matches = re_compiler.compile(mOptions->mRE);
        Var * const output = getOutputStreamVar("matches");
        pb.createAssign(pb.createExtract(output, pb.getInteger(0)), matches);
    }
}

// Helper to compute stream set inputs to pass into PabloKernel constructor.
Bindings ByteBitGrepKernel::makeInputBindings(StreamSet * const basis, const Externals & externals) {
    Bindings inputs;
    inputs.emplace_back("basis", basis);
    for (const auto & e : externals) {
        inputs.emplace_back(e.first, e.second);
    }
    return inputs;
}


ByteBitGrepSignature::ByteBitGrepSignature(RE * prefix, RE * suffix)
: mPrefixRE(prefix)
, mSuffixRE(suffix)
, mSignature(Printer_RE::PrintRE(mPrefixRE) + Printer_RE::PrintRE(mSuffixRE) ) {
}

ByteBitGrepKernel::ByteBitGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & b, RE * const prefixRE, RE * const suffixRE, StreamSet * const Source, StreamSet * const matches, const Externals externals)
: ByteBitGrepSignature(prefixRE, suffixRE)
, PabloKernel(b, "bBc" + getStringHash(mSignature),
// inputs
makeInputBindings(Source, externals),
// output
{Binding{"matches", matches, FixedRate(), Add1()}}) {
    addAttribute(InfrequentlyUsed());
}

std::string ByteBitGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}


void ByteBitGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * u8bytes = pb.createExtract(getInput(0), pb.getInteger(0));
    cc::Direct_CC_Compiler dcc(getEntryScope(), u8bytes);
    RE_Compiler re_byte_compiler(getEntryScope(), dcc);

    const auto numOfInputs = getNumOfInputs();
    for (unsigned i = 1; i < numOfInputs; ++i) {
        const Binding & input = getInputStreamSetBinding(i);
        re_byte_compiler.addPrecompiled(input.getName(), pb.createExtract(getInputStreamVar(input.getName()), pb.getInteger(0)));
    }

    PabloAST * const prefixMatches = re_byte_compiler.compile(mPrefixRE);
    Var * const final_matches = pb.createVar("final_matches", pb.createZeroes());
    PabloBlock * scope1 = getEntryScope()->createScope();
    pb.createIf(prefixMatches, scope1);

    PabloAST * nybbles[2];
    nybbles[0] = scope1->createPackL(scope1->getInteger(8), u8bytes);
    nybbles[1] = scope1->createPackH(scope1->getInteger(8), u8bytes);

    PabloAST * bitpairs[4];
    for (unsigned i = 0; i < 2; i++) {
        bitpairs[2*i] = scope1->createPackL(scope1->getInteger(4), nybbles[i]);
        bitpairs[2*i + 1] = scope1->createPackH(scope1->getInteger(4), nybbles[i]);
    }

    std::vector<PabloAST *> basis(8);
    for (unsigned i = 0; i < 4; i++) {
        basis[2*i] = scope1->createPackL(scope1->getInteger(2), bitpairs[i]);
        basis[2*i + 1] = scope1->createPackH(scope1->getInteger(2), bitpairs[i]);
    }

    cc::Parabix_CC_Compiler_Builder ccc(scope1, basis);
    RE_Compiler re_compiler(scope1, ccc);
    scope1->createAssign(final_matches, re_compiler.compile(mSuffixRE, prefixMatches));
    Var * const output = getOutputStreamVar("matches");
    pb.createAssign(pb.createExtract(output, pb.getInteger(0)), final_matches);
}


void MatchedLinesKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * matchResults = pb.createExtract(getInputStreamVar("matchResults"), pb.getInteger(0));
    PabloAST * lineBreaks = pb.createExtract(getInputStreamVar("lineBreaks"), pb.getInteger(0));
    PabloAST * notLB = pb.createNot(lineBreaks);
    PabloAST * match_follow = pb.createMatchStar(matchResults, notLB);
    PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(notLB, 1), "unterminatedLineAtEOF");
    Var * const matchedLines = getOutputStreamVar("matchedLines");
    pb.createAssign(pb.createExtract(matchedLines, pb.getInteger(0)), pb.createAnd(match_follow, pb.createOr(lineBreaks, unterminatedLineAtEOF)));
}

MatchedLinesKernel::MatchedLinesKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches)
: PabloKernel(iBuilder, "MatchedLines",
// inputs
{Binding{"matchResults", OriginalMatches}
,Binding{"lineBreaks", LineBreakStream}},
// output
{Binding{"matchedLines", Matches, FixedRate(), Add1()}}) {

}


void InvertMatchesKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * input = iBuilder->loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = iBuilder->loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateXor(input, lbs);
    iBuilder->storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * OriginalMatches, StreamSet * LineBreakStream, StreamSet * Matches)
: BlockOrientedKernel(b, "Invert",
// Inputs
{Binding{"matchedLines", OriginalMatches},
 Binding{"lineBreaks", LineBreakStream}},
// Outputs
{Binding{"nonMatches", Matches}},
// Input/Output Scalars and internal state
{}, {}, {}) {

}


void PopcountKernel::generatePabloMethod() {
    auto pb = getEntryScope();
    const auto toCount = pb->createExtract(getInputStreamVar("toCount"), pb->getInteger(0));
    pablo::Var * countResult = getOutputScalarVar("countResult");

    pb->createAssign(countResult, pb->createCount(pb->createInFile(toCount)));
}

PopcountKernel::PopcountKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder, StreamSet * const toCount, Scalar * countResult)
: PabloKernel(iBuilder, "Popcount",
{Binding{"toCount", toCount}},
{},
{},
{Binding{"countResult", countResult}}) {

}


void AbortOnNull::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) {
    Module * const m = b->getModule();
    DataLayout DL(m);
    IntegerType * const intPtrTy = DL.getIntPtrType(m->getContext());
    Type * voidPtrTy = b->getVoidPtrTy();
    const auto blocksPerStride = getStride() / b->getBitBlockWidth();
    Constant * const BLOCKS_PER_STRIDE = b->getSize(blocksPerStride);
    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const strideLoop = b->CreateBasicBlock("strideLoop");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const nullByteDetection = b->CreateBasicBlock("nullByteDetection");
    BasicBlock * const nullByteFound = b->CreateBasicBlock("nullByteFound");
    BasicBlock * const finalStride = b->CreateBasicBlock("finalStride");
    BasicBlock * const segmentDone = b->CreateBasicBlock("segmentDone");

    Value * const numOfBlocks = b->CreateMul(numOfStrides, BLOCKS_PER_STRIDE);
    Value * itemsToDo = b->getAccessibleItemCount("byteData");
    //
    // Fast loop to prove that there are no null bytes in a multiblock region.
    // We repeatedly combine byte packs using a SIMD unsigned min operation
    // (implemented as a Select/ICmpULT combination).
    //
    Value * byteStreamBasePtr = b->getInputStreamBlockPtr("byteData", b->getSize(0), b->getSize(0));
    Value * outputStreamBasePtr = b->getOutputStreamBlockPtr("untilNull", b->getSize(0), b->getSize(0));

    //
    // We set up a a set of eight accumulators to accumulate the minimum byte
    // values seen at each position in a block.   The initial min value at
    // each position is 0xFF (all ones).
    Value * blockMin[8];
    for (unsigned i = 0; i < 8; i++) {
        blockMin[i] = b->fwCast(8, b->allOnes());
    }
    // If we're in the final block bypass the fast loop.
    b->CreateCondBr(mIsFinal, finalStride, strideLoop);

    b->SetInsertPoint(strideLoop);
    PHINode * const baseBlockIndex = b->CreatePHI(b->getSizeTy(), 2);
    baseBlockIndex->addIncoming(ConstantInt::get(baseBlockIndex->getType(), 0), entry);
    PHINode * const blocksRemaining = b->CreatePHI(b->getSizeTy(), 2);
    blocksRemaining->addIncoming(numOfBlocks, entry);
    for (unsigned i = 0; i < 8; i++) {
        Value * next = b->CreateBlockAlignedLoad(b->CreateGEP(byteStreamBasePtr, {baseBlockIndex, b->getSize(i)}));
        b->CreateBlockAlignedStore(next, b->CreateGEP(outputStreamBasePtr, {baseBlockIndex, b->getSize(i)}));
        next = b->fwCast(8, next);
        blockMin[i] = b->CreateSelect(b->CreateICmpULT(next, blockMin[i]), next, blockMin[i]);
    }
    Value * nextBlockIndex = b->CreateAdd(baseBlockIndex, ConstantInt::get(baseBlockIndex->getType(), 1));
    Value * nextRemaining = b->CreateSub(blocksRemaining, ConstantInt::get(blocksRemaining->getType(), 1));
    baseBlockIndex->addIncoming(nextBlockIndex, strideLoop);
    blocksRemaining->addIncoming(nextRemaining, strideLoop);
    b->CreateCondBr(b->CreateICmpUGT(nextRemaining, ConstantInt::getNullValue(blocksRemaining->getType())), strideLoop, stridesDone);

    b->SetInsertPoint(stridesDone);
    // Combine the 8 blockMin values.
    for (unsigned i = 0; i < 4; i++) {
        blockMin[i] = b->CreateSelect(b->CreateICmpULT(blockMin[i], blockMin[i+4]), blockMin[i], blockMin[i+4]);
    }
    for (unsigned i = 0; i < 2; i++) {
        blockMin[i] = b->CreateSelect(b->CreateICmpULT(blockMin[i], blockMin[i+4]), blockMin[i], blockMin[i+2]);
    }
    blockMin[0] = b->CreateSelect(b->CreateICmpULT(blockMin[0], blockMin[1]), blockMin[0], blockMin[1]);
    Value * anyNull = b->bitblock_any(b->simd_eq(8, blockMin[0], b->allZeroes()));

    b->CreateCondBr(anyNull, nullByteDetection, segmentDone);


    b->SetInsertPoint(finalStride);
    b->CreateMemCpy(b->CreatePointerCast(outputStreamBasePtr, voidPtrTy), b->CreatePointerCast(byteStreamBasePtr, voidPtrTy), itemsToDo, 1);
    b->CreateBr(nullByteDetection);

    b->SetInsertPoint(nullByteDetection);
    //  Find the exact location using memchr, which should be fast enough.
    //
    Value * ptrToNull = b->CreateMemChr(b->CreatePointerCast(byteStreamBasePtr, voidPtrTy), b->getInt32(0), itemsToDo);
    Value * ptrAddr = b->CreatePtrToInt(ptrToNull, intPtrTy);
    b->CreateCondBr(b->CreateICmpEQ(ptrAddr, ConstantInt::getNullValue(intPtrTy)), segmentDone, nullByteFound);

    // A null byte has been located; set the termination code and call the signal handler.
    b->SetInsertPoint(nullByteFound);
    Value * nullPosn = b->CreateSub(b->CreatePtrToInt(ptrToNull, intPtrTy), b->CreatePtrToInt(byteStreamBasePtr, intPtrTy));
    b->setTerminationSignal();
    Function * const dispatcher = m->getFunction("signal_dispatcher"); assert (dispatcher);
    Value * handler = b->getScalarField("handler_address");
    b->CreateCall(dispatcher, {handler, ConstantInt::get(b->getInt32Ty(), static_cast<unsigned>(grep::GrepSignal::BinaryFile))});
    b->CreateBr(segmentDone);

    b->SetInsertPoint(segmentDone);
    PHINode * const produced = b->CreatePHI(b->getSizeTy(), 3);
    produced->addIncoming(nullPosn, nullByteFound);
    produced->addIncoming(itemsToDo, stridesDone);
    produced->addIncoming(itemsToDo, nullByteDetection);
    Value * producedCount = b->getProducedItemCount("untilNull");
    producedCount = b->CreateAdd(producedCount, produced);
    b->setProducedItemCount("untilNull", producedCount);
}

AbortOnNull::AbortOnNull(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const InputStream, StreamSet * const OutputStream, Scalar * callbackObject)
: MultiBlockKernel(b, "AbortOnNull",
// inputs
{Binding{"byteData", InputStream, FixedRate(), Principal()}},
// outputs
{Binding{ "untilNull", OutputStream, FixedRate(), Deferred()}},
// input scalars
{Binding{"handler_address", callbackObject}},
{}, {}) {
    addAttribute(CanTerminateEarly());
}
