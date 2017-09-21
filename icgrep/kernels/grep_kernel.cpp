/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grep_kernel.h"
#include <boost/uuid/sha1.hpp>
#include <re/printer_re.h>
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <pablo/boolean.h>
#include <pablo/pe_count.h>
#include <pablo/pe_matchstar.h>
#include "cc/cc_compiler.h"         // for CC_Compiler

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

RegularExpressionOptimizer::RegularExpressionOptimizer(re::RE * const re_ast)
: mRE(regular_expression_passes(re_ast))
, mSignature(Printer_RE::PrintRE(mRE)) {

}

void RequiredStreams_UTF8::generatePabloMethod() {
    
    cc::CC_Compiler ccc(this, getInput(0));
    auto & pb = ccc.getBuilder();
    Zeroes * const zero = pb.createZeroes();
    PabloAST * LF = ccc.compileCC("LF", makeCC(0x0A), pb);
    PabloAST * CR = ccc.compileCC(makeCC(0x0D));

    Var * crlf = pb.createVar("crlf", zero);
    PabloBuilder crb = PabloBuilder::Create(pb);
    PabloAST * cr1 = crb.createAdvance(CR, 1, "cr1");
    crb.createAssign(crlf, crb.createAnd(cr1, LF));
    pb.createIf(CR, crb);
   
    Var * u8invalid = pb.createVar("u8invalid", zero);
    Var * valid_pfx = pb.createVar("valid_pfx", zero);
    Var * nonFinal = pb.createVar("nonfinal", zero);
    PabloAST * u8pfx = ccc.compileCC(makeCC(0xC0, 0xFF));
    
    PabloBuilder it = PabloBuilder::Create(pb);

    pb.createIf(u8pfx, it);
    PabloAST * u8pfx2 = ccc.compileCC(makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = ccc.compileCC(makeCC(0xF0, 0xF4), it);
    PabloAST * u8suffix = ccc.compileCC("u8suffix", makeCC(0x80, 0xBF), it);
    
    //
    // Two-byte sequences
    Var * u8scope22 = it.createVar("u8scope22", zero);
    PabloBuilder it2 = PabloBuilder::Create(it);
    it2.createAssign(u8scope22, it2.createAdvance(u8pfx2, 1));
    it.createIf(u8pfx2, it2);
    //
    // Three-byte sequences
    
    Var * u8scope32 = it.createVar("u8scope32", zero);
    Var * u8scope3X = it.createVar("u8scope3X", zero);
    Var * EX_invalid = it.createVar("EX_invalid", zero);
    PabloBuilder it3 = PabloBuilder::Create(it);
    it.createIf(u8pfx3, it3);
    it3.createAssign(u8scope32, it3.createAdvance(u8pfx3, 1));
    PabloAST * u8scope33 = it3.createAdvance(u8pfx3, 2);
    it3.createAssign(u8scope3X, it3.createOr(u8scope32, u8scope33));
    PabloAST * E0_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(makeCC(0xE0), it3), 1), ccc.compileCC(makeCC(0x80, 0x9F), it3));
    PabloAST * ED_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(makeCC(0xED), it3), 1), ccc.compileCC(makeCC(0xA0, 0xBF), it3));
    it3.createAssign(EX_invalid, it3.createOr(E0_invalid, ED_invalid));
   
    //
    // Four-byte sequences
    Var * u8scope4nonfinal = it.createVar("u8scope4nonfinal", zero);
    Var * u8scope4X = it.createVar("u8scope4X", zero);
    Var * FX_invalid = it.createVar("FX_invalid", zero);
    PabloBuilder it4 = PabloBuilder::Create(it);
    it.createIf(u8pfx4, it4);
    PabloAST * u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(u8scope4nonfinal, it4.createOr(u8scope42, u8scope43));
    it4.createAssign(u8scope4X, it4.createOr(u8scope4nonfinal, u8scope44));
    PabloAST * F0_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(makeCC(0xF0), it4), 1), ccc.compileCC(makeCC(0x80, 0x8F), it4));
    PabloAST * F4_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(makeCC(0xF4), it4), 1), ccc.compileCC(makeCC(0x90, 0xBF), it4));
    it4.createAssign(FX_invalid, it4.createOr(F0_invalid, F4_invalid));
    
    //
    // Invalid cases
    PabloAST * anyscope = it.createOr(u8scope22, it.createOr(u8scope3X, u8scope4X));
    PabloAST * legalpfx = it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * EF_invalid = it.createOr(EX_invalid, FX_invalid);
    PabloAST * pfx_invalid = it.createXor(u8pfx, legalpfx);
    it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
    PabloAST * u8valid = it.createNot(u8invalid, "u8valid");
    //
    //
    
    it.createAssign(valid_pfx, it.createAnd(u8pfx, u8valid));
    it.createAssign(nonFinal, it.createAnd(it.createOr(it.createOr(u8pfx, u8scope32), u8scope4nonfinal), u8valid));
    
    PabloAST * u8single = pb.createAnd(ccc.compileCC(makeCC(0x00, 0x7F)), pb.createNot(u8invalid));
    
    Var * const required = getOutputStreamVar("required");
    pb.createAssign(pb.createExtract(required, pb.getInteger(0)), pb.createOr(u8single, valid_pfx, "initial"));
    pb.createAssign(pb.createExtract(required, pb.getInteger(1)), nonFinal);
    pb.createAssign(pb.createExtract(required, pb.getInteger(2)), pb.createNot(pb.createOr(nonFinal, u8invalid), "final"));
    pb.createAssign(pb.createExtract(required, pb.getInteger(3)), crlf);
}

RequiredStreams_UTF8::RequiredStreams_UTF8(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb, "RequiredStreams_UTF8",               
              {Binding{kb->getStreamSetTy(8), "basis"}}, 
              {Binding{kb->getStreamSetTy(4), "required"}},
              {},
              {}) {
}

void RequiredStreams_UTF16::generatePabloMethod() {
    
    cc::CC_Compiler ccc(this, getInput(0));
    auto & pb = ccc.getBuilder();
    
    PabloAST * LF = ccc.compileCC("LF", makeCC(0x000A), pb);
    PabloAST * CR = ccc.compileCC("CR", makeCC(0x000D), pb);
    PabloAST * cr1 = pb.createAdvance(CR, 1, "cr1");
    
    PabloAST * u16hi_hi_surrogate = ccc.compileCC(makeCC(0xD800, 0xDBFF));    //u16hi_hi_surrogate = [\xD8-\xDB]
    PabloAST * u16hi_lo_surrogate = ccc.compileCC(makeCC(0xDC00, 0xDFFF));    //u16hi_lo_surrogate = [\xDC-\xDF]
    
    PabloAST * invalidTemp = pb.createAdvance(u16hi_hi_surrogate, 1, "InvalidTemp");
    PabloAST * u16invalid = pb.createXor(invalidTemp, u16hi_lo_surrogate, "u16invalid");
    PabloAST * u16valid = pb.createNot(u16invalid, "u16valid");
    
    PabloAST * u16single_temp = pb.createOr(ccc.compileCC(makeCC(0x0000, 0xD7FF)), ccc.compileCC(makeCC(0xE000, 0xFFFF)));
    PabloAST * u16single = pb.createAnd(u16single_temp, pb.createNot(u16invalid));

    Var * const required = getOutputStreamVar("required");
    pb.createAssign(pb.createExtract(required, pb.getInteger(0)), pb.createOr(u16single, u16hi_hi_surrogate, "initial"));
    pb.createAssign(pb.createExtract(required, pb.getInteger(1)), pb.createAnd(u16hi_hi_surrogate, u16valid, "nonfinal"));
    pb.createAssign(pb.createExtract(required, pb.getInteger(2)), pb.createNot(pb.createOr(u16hi_hi_surrogate, u16invalid), "final"));
    pb.createAssign(pb.createExtract(required, pb.getInteger(3)), pb.createAnd(cr1, LF, "crlf"));
}

RequiredStreams_UTF16::RequiredStreams_UTF16(const std::unique_ptr<kernel::KernelBuilder> & kb)
: PabloKernel(kb, "RequiredStreams_UTF16",               
              {Binding{kb->getStreamSetTy(16), "basis"}}, 
              {Binding{kb->getStreamSetTy(4), "required"}},
              {},
              {}) {
}


ICGrepKernel::ICGrepKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, RE * const re, unsigned numOfCharacterClasses)
: RegularExpressionOptimizer(re)
, PabloKernel(iBuilder,
              "ic" + sha1sum(mSignature),
              {Binding{iBuilder->getStreamSetTy(numOfCharacterClasses), "basis"},
               Binding{iBuilder->getStreamSetTy(1, 1), "linebreak"},
               Binding{iBuilder->getStreamSetTy(4, 1), "required"}},

              {Binding{iBuilder->getStreamSetTy(1, 1), "matches"}}) {

}

std::string ICGrepKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}

void ICGrepKernel::generatePabloMethod() {
    PabloAST * const match_post = re2pablo_compiler(this, mRE);
    PabloBlock * const pb = getEntryBlock();
    Var * const output = getOutputStreamVar("matches");
    pb->createAssign(pb->createExtract(output, pb->getInteger(0)), match_post);
}

void MatchedLinesKernel::generatePabloMethod() {
    auto pb = this->getEntryBlock();
    PabloAST * matchResults = pb->createExtract(getInputStreamVar("matchResults"), pb->getInteger(0));
    PabloAST * lineBreaks = pb->createExtract(getInputStreamVar("lineBreaks"), pb->getInteger(0));
    PabloAST * notLB = pb->createNot(lineBreaks);
    PabloAST * match_follow = pb->createMatchStar(matchResults, notLB);
    Var * const matchedLines = getOutputStreamVar("matchedLines");
    pb->createAssign(pb->createExtract(matchedLines, pb->getInteger(0)), pb->createAnd(match_follow, lineBreaks));
}

MatchedLinesKernel::MatchedLinesKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: PabloKernel(iBuilder, "MatchedLines",
              {Binding{iBuilder->getStreamSetTy(1), "matchResults"}, Binding{iBuilder->getStreamSetTy(1), "lineBreaks"}},
              {Binding{iBuilder->getStreamSetTy(1), "matchedLines"}},
              {},
              {}) {
}


void InvertMatchesKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    Value * input = iBuilder->loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = iBuilder->loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateXor(input, lbs);
    iBuilder->storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(const std::unique_ptr<kernel::KernelBuilder> & builder)
: BlockOrientedKernel("Invert", {Binding{builder->getStreamSetTy(1, 1), "matchedLines"}, Binding{builder->getStreamSetTy(1, 1), "lineBreaks"}}, {Binding{builder->getStreamSetTy(1, 1), "nonMatches"}}, {}, {}, {}) {
    setNoTerminateAttribute(true);    
}


void PopcountKernel::generatePabloMethod() {
    auto pb = this->getEntryBlock();
    const auto toCount = pb->createExtract(getInputStreamVar("toCount"), pb->getInteger(0));
    pablo::Var * countResult = getOutputScalarVar("countResult");
    pb->createAssign(countResult, pb->createCount(toCount));
}

PopcountKernel::PopcountKernel (const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: PabloKernel(iBuilder, "Popcount",
              {Binding{iBuilder->getStreamSetTy(1), "toCount"}},
              {},
              {},
              {Binding{iBuilder->getSizeTy(), "countResult"}}) {
}

