/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <boost/uuid/sha1.hpp>
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

inline static std::string sha1sum(const std::string & str) {
    char buffer[41];    // 40 hex-digits and the terminating null
    uint32_t digest[5]; // 160 bits in total
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.c_str(), str.size());
    sha1.get_digest(digest);
    snprintf(buffer, sizeof(buffer), "%.8x%.8x%.8x%.8x%.8x",
             digest[0], digest[1], digest[2], digest[3], digest[4]);
    return std::string(buffer);
}


UnicodeLineBreakKernel::UnicodeLineBreakKernel(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb,
              "UTF8_LB",
              {Binding{kb->getStreamSetTy(8), "basis"}, Binding{kb->getStreamSetTy(1), "lf", FixedRate(), LookAhead(1)}},
              {Binding{kb->getStreamSetTy(1, 1), "UTF8_LB", FixedRate()}}) {
}

void UnicodeLineBreakKernel::generatePabloMethod() {
        PabloBuilder pb(getEntryScope());
        cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
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
    Var * const CR_before_LF = pb.createVar("CR_before_LFCR_before_LF", pb.createZeroes());
    auto crb = pb.createScope();
    pb.createIf(CR, crb);
    PabloAST * const lookaheadLF = crb.createLookahead(LF, 1, "lookaheadLF");
    crb.createAssign(CR_before_LF, crb.createAnd(CR, lookaheadLF));
    breakStream = pb.createXor(breakStream, CR_before_LF);  // Remove CR_before_LF from breakStream
    Var * const UTF8_LB = getOutputStreamVar("UTF8_LB");
    pb.createAssign(pb.createExtract(UTF8_LB, pb.getInteger(0)), breakStream);
}

void RequiredStreams_UTF8::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    
    PabloAST * const LF = pb.createExtract(getInput(1), pb.getInteger(0), "LF");
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

    
    Zeroes * const ZEROES = pb.createZeroes();
    PabloAST * const u8pfx = ccc.compileCC(makeByte(0xC0, 0xFF));


    Var * const nonFinal = pb.createVar("nonFinal", u8pfx);
    Var * const u8invalid = pb.createVar("u8invalid", ZEROES);
    Var * const valid_pfx = pb.createVar("valid_pfx", u8pfx);

    auto it = pb.createScope();
    pb.createIf(u8pfx, it);
    PabloAST * const u8pfx2 = ccc.compileCC(makeByte(0xC2, 0xDF), it);
    PabloAST * const u8pfx3 = ccc.compileCC(makeByte(0xE0, 0xEF), it);
    PabloAST * const u8pfx4 = ccc.compileCC(makeByte(0xF0, 0xF4), it);
    PabloAST * const u8suffix = ccc.compileCC("u8suffix", makeByte(0x80, 0xBF), it);
    
    //
    // Two-byte sequences
    Var * const anyscope = it.createVar("anyscope", ZEROES);
    auto it2 = it.createScope();
    it.createIf(u8pfx2, it2);
    it2.createAssign(anyscope, it2.createAdvance(u8pfx2, 1));
    PabloAST * NEL = it2.createAnd(it2.createAdvance(ccc.compileCC(makeByte(0xC2), it2), 1), ccc.compileCC(makeByte(0x85), it2), "NEL");
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
    PabloAST * const E0_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(makeByte(0xE0), it3), 1), ccc.compileCC(makeByte(0x80, 0x9F), it3));
    PabloAST * const ED_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(makeByte(0xED), it3), 1), ccc.compileCC(makeByte(0xA0, 0xBF), it3));
    PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
    it3.createAssign(EF_invalid, EX_invalid);
    PabloAST * E2_80 = it3.createAnd(it3.createAdvance(ccc.compileCC(makeByte(0xE2), it3), 1), ccc.compileCC(makeByte(0x80), it3));
    PabloAST * LS_PS = it3.createAnd(it3.createAdvance(E2_80, 1), ccc.compileCC(makeByte(0xA8,0xA9), it3), "LS_PS");
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
    PabloAST * const F0_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(makeByte(0xF0), it4), 1), ccc.compileCC(makeByte(0x80, 0x8F), it4));
    PabloAST * const F4_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(makeByte(0xF4), it4), 1), ccc.compileCC(makeByte(0x90, 0xBF), it4));
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
    pb.createAssign(nonFinal, pb.createOr(nonFinal, CRLF));
    //PabloAST * unterminatedLineAtEOF = pb.createAtEOF(pb.createAdvance(pb.createNot(LineBreak), 1), "unterminatedLineAtEOF");
    
    Var * const required = getOutputStreamVar("nonFinal");
    pb.createAssign(pb.createExtract(required, pb.getInteger(0)), nonFinal);
    pb.createAssign(pb.createExtract(getOutputStreamVar("UnicodeLB"), pb.getInteger(0)), LineBreak);//pb.createOr(LineBreak, unterminatedLineAtEOF, "EOL"));
}

RequiredStreams_UTF8::RequiredStreams_UTF8(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb, "RequiredStreams_UTF8",
// input
{Binding{kb->getStreamSetTy(8), "basis"},
 Binding{kb->getStreamSetTy(1), "lf", FixedRate(), LookAhead(1)}},
// output
{Binding{kb->getStreamSetTy(1), "nonFinal", FixedRate()},
 Binding{kb->getStreamSetTy(1), "UnicodeLB", FixedRate()}}) {

}

void RequiredStreams_UTF16::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    
    PabloAST * u16hi_hi_surrogate = ccc.compileCC(makeCC(0xD800, 0xDBFF, &cc::UTF16));    //u16hi_hi_surrogate = [\xD8-\xDB]
    PabloAST * u16hi_lo_surrogate = ccc.compileCC(makeCC(0xDC00, 0xDFFF, &cc::UTF16));    //u16hi_lo_surrogate = [\xDC-\xDF]
    
    PabloAST * invalidTemp = pb.createAdvance(u16hi_hi_surrogate, 1, "InvalidTemp");
    PabloAST * u16invalid = pb.createXor(invalidTemp, u16hi_lo_surrogate, "u16invalid");

    PabloAST * u16valid = pb.createNot(u16invalid, "u16valid");
    PabloAST * nonFinal = pb.createAnd(u16hi_hi_surrogate, u16valid, "nonfinal");

    PabloAST * u16single_temp = pb.createOr(ccc.compileCC(makeCC(0x0000, 0xD7FF, &cc::UTF16)), ccc.compileCC(makeCC(0xE000, 0xFFFF, &cc::UTF16)));
    PabloAST * u16single = pb.createAnd(u16single_temp, pb.createNot(u16invalid));

    PabloAST * const nonFinalCodeUnits = pb.createExtract(getInput(1), pb.getInteger(0));
    PabloAST * const initial = pb.createOr(u16single, u16hi_hi_surrogate, "initial");
    PabloAST * const final = pb.createNot(pb.createOr(pb.createOr(u16hi_hi_surrogate, u16invalid), nonFinalCodeUnits), "final");

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

ICGrepSignature::ICGrepSignature(re::RE * const re_ast)
: mRE(re_ast)
, mSignature(Printer_RE::PrintRE(mRE)) {
    
}

// Helper to compute stream set inputs to pass into PabloKernel constructor.
inline std::vector<Binding> icGrepInputs(const std::unique_ptr<kernel::KernelBuilder> & b,
                                         const std::vector<std::string> & externals,
                                         const std::vector<cc::Alphabet *> & alphabets) {
    std::vector<Binding> streamSetInputs = {
        Binding{b->getStreamSetTy(8), "basis"},
    };
    for (auto & e : externals) {
        streamSetInputs.push_back(Binding{b->getStreamSetTy(1, 1), e});
    }
    for (const auto & alphabet : alphabets) {
        unsigned basis_size = cast<cc::MultiplexedAlphabet>(alphabet)->getMultiplexedCCs().size();
        streamSetInputs.push_back(Binding{b->getStreamSetTy(basis_size, 1), alphabet->getName() + "_basis"});
    }
    return streamSetInputs;
}

ICGrepKernel::ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & b, RE * const re, std::vector<std::string> externals, std::vector<cc::Alphabet *> alphabets)
: ICGrepSignature(re)
, PabloKernel(b, "ic" + sha1sum(mSignature),
// inputs
icGrepInputs(b, externals, alphabets),
// output
{Binding{b->getStreamSetTy(1, 1), "matches", FixedRate(), Add1()}})
, mExternals(externals)
, mAlphabets(alphabets) {
}

std::string ICGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}

void ICGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    RE_Compiler re_compiler(getEntryScope(), ccc);
    for (auto & e : mExternals) {
        re_compiler.addPrecompiled(e, pb.createExtract(getInputStreamVar(e), pb.getInteger(0)));
    }
    for (auto a : mAlphabets) {
        auto mpx_basis = getInputStreamSet(a->getName() + "_basis");
        re_compiler.addAlphabet(a, mpx_basis);
    }
    PabloAST * const matches = re_compiler.compile(mRE);
    Var * const output = getOutputStreamVar("matches");
    pb.createAssign(pb.createExtract(output, pb.getInteger(0)), matches);
}


ByteGrepSignature::ByteGrepSignature(RE * re)
: mRE(re)
, mSignature(Printer_RE::PrintRE(re) ) {
}

ByteGrepKernel::ByteGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & b, RE * const re, std::vector<std::string> externals)
: ByteGrepSignature(re)
, PabloKernel(b, "byteGrep" + sha1sum(mSignature),
              // inputs
{Binding{b->getStreamSetTy(1, 8), "byteData"}},
              // output
{Binding{b->getStreamSetTy(1, 1), "matches", FixedRate(), Add1()}})
, mExternals(externals) {
    for (auto & e : externals) {
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(1, 1), e});
    }
}

std::string ByteGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}


void ByteGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * u8bytes = pb.createExtract(getInput(0), pb.getInteger(0));
    cc::Direct_CC_Compiler dcc(getEntryScope(), u8bytes);
    RE_Compiler re_byte_compiler(getEntryScope(), dcc);
    for (auto & e : mExternals) {
        re_byte_compiler.addPrecompiled(e, pb.createExtract(getInputStreamVar(e), pb.getInteger(0)));
    }
    PabloAST * const matches = re_byte_compiler.compile(mRE);
    
    Var * const output = getOutputStreamVar("matches");
    pb.createAssign(pb.createExtract(output, pb.getInteger(0)), matches);
}

// Helper to compute stream set inputs to pass into PabloKernel constructor.
inline std::vector<Binding> byteBitGrepInputs(const std::unique_ptr<kernel::KernelBuilder> & b,
                                              const std::vector<std::string> & externals) {
    std::vector<Binding> streamSetInputs = {
        Binding{b->getStreamSetTy(1, 8), "bytedata"},
    };
    for (auto & e : externals) {
        streamSetInputs.push_back(Binding{b->getStreamSetTy(1, 1), e});
    }
    return streamSetInputs;
}

ByteBitGrepSignature::ByteBitGrepSignature(RE * prefix, RE * suffix)
: mPrefixRE(prefix)
, mSuffixRE(suffix)
, mSignature(Printer_RE::PrintRE(mPrefixRE) + Printer_RE::PrintRE(mSuffixRE) ) {
}

ByteBitGrepKernel::ByteBitGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & b, RE * const prefixRE, RE * const suffixRE, std::vector<std::string> externals)
: ByteBitGrepSignature(prefixRE, suffixRE)
, PabloKernel(b, "bBc" + sha1sum(mSignature),
              // inputs
              byteBitGrepInputs(b, externals),
              // output
{Binding{b->getStreamSetTy(1, 1), "matches", FixedRate(), Add1()}})
, mExternals(externals) {
}

std::string ByteBitGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}


void ByteBitGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * u8bytes = pb.createExtract(getInput(0), pb.getInteger(0));
    cc::Direct_CC_Compiler dcc(getEntryScope(), u8bytes);
    RE_Compiler re_byte_compiler(getEntryScope(), dcc);
    for (auto & e : mExternals) {
        re_byte_compiler.addPrecompiled(e, pb.createExtract(getInputStreamVar(e), pb.getInteger(0)));
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
        // The subtraction 7-bit is because of the confusion between
        // little-endian and big-endian bit numbering of bytes.
        // We should fix this, switching to little-endian numbering throughout.
        basis[7-2*i] = scope1->createPackL(scope1->getInteger(2), bitpairs[i]);
        basis[7-(2*i + 1)] = scope1->createPackH(scope1->getInteger(2), bitpairs[i]);
    }
    
    cc::Parabix_CC_Compiler ccc(scope1, basis);
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

MatchedLinesKernel::MatchedLinesKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: PabloKernel(iBuilder, "MatchedLines",
// inputs
{Binding{iBuilder->getStreamSetTy(1), "matchResults"}
,Binding{iBuilder->getStreamSetTy(1), "lineBreaks"}},
// output
{Binding{iBuilder->getStreamSetTy(1), "matchedLines", FixedRate(), Add1()}}) {

}


void InvertMatchesKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * input = iBuilder->loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = iBuilder->loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateXor(input, lbs);
    iBuilder->storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder)
: BlockOrientedKernel("Invert",
// Inputs
{Binding{builder->getStreamSetTy(1, 1), "matchedLines"}, Binding{builder->getStreamSetTy(1, 1), "lineBreaks"}},
// Outputs
{Binding{builder->getStreamSetTy(1, 1), "nonMatches"}},
// Input/Output Scalars and internal state
{}, {}, {}) {

}


void PopcountKernel::generatePabloMethod() {
    auto pb = this->getEntryScope();
    const auto toCount = pb->createExtract(getInputStreamVar("toCount"), pb->getInteger(0));
    pablo::Var * countResult = getOutputScalarVar("countResult");
    
    pb->createAssign(countResult, pb->createCount(pb->createInFile(toCount)));
}

PopcountKernel::PopcountKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: PabloKernel(iBuilder, "Popcount",
{Binding{iBuilder->getStreamSetTy(1), "toCount"}},
{},
{},
{Binding{iBuilder->getSizeTy(), "countResult"}}) {

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
    Value * availItems = b->getAvailableItemCount("bytedata");
    //
    // Fast loop to prove that there are no null bytes in a multiblock region.
    // We repeatedly combine byte packs using a SIMD unsigned min operation
    // (implemented as a Select/ICmpULT combination).
    //
    Value * byteStreamBasePtr = b->getInputStreamBlockPtr("bytedata", b->getSize(0), b->getSize(0));
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
    b->CreateMemCpy(b->CreatePointerCast(outputStreamBasePtr, voidPtrTy), b->CreatePointerCast(byteStreamBasePtr, voidPtrTy), availItems, 1);
    b->CreateBr(nullByteDetection);
    
    b->SetInsertPoint(nullByteDetection);
    //  Find the exact location using memchr, which should be fast enough.
    //
    Value * ptrToNull = b->CreateMemChr(b->CreatePointerCast(byteStreamBasePtr, voidPtrTy), b->getInt32(0), availItems);
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
    produced->addIncoming(availItems, stridesDone);
    produced->addIncoming(availItems, nullByteDetection);
    Value * producedCount = b->getProducedItemCount("untilNull");
    producedCount = b->CreateAdd(producedCount, produced);
    b->setProducedItemCount("untilNull", producedCount);
}

AbortOnNull::AbortOnNull(const std::unique_ptr<kernel::KernelBuilder> & b)
: MultiBlockKernel("AbortOnNull",
                   // inputs
{Binding{b->getStreamSetTy(1, 8), "bytedata"}},
                   // outputs
{Binding{b->getStreamSetTy(1, 8), "untilNull", FixedRate(), Deferred()}},
                   // input scalars
{Binding{b->getIntAddrTy(), "handler_address"}},
{}, {}) {
    addAttribute(CanTerminateEarly());
}
