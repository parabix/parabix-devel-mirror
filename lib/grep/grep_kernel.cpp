/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <grep/grep_kernel.h>

#include <grep/grep_engine.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/codegenstate.h>
#include <toolchain/pablo_toolchain.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <pablo/pe_infile.h>
#include <pablo/pe_advance.h>
#include <pablo/boolean.h>
#include <pablo/pe_count.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_pack.h>
#include <pablo/pe_debugprint.h>
#include <re/adt/printer_re.h>
#include <re/adt/re_cc.h>
#include <re/adt/re_name.h>
#include <re/alphabet/alphabet.h>
#include <re/toolchain/toolchain.h>
#include <re/transforms/re_reverse.h>
#include <re/analysis/collect_ccs.h>
#include <re/transforms/exclude_CC.h>
#include <re/transforms/re_multiplex.h>
#include <kernel/unicode/UCD_property_kernel.h>
#include <re/analysis/re_name_gather.h>
#include <re/unicode/grapheme_clusters.h>
#include <re/unicode/re_name_resolve.h>
#include <kernel/unicode/charclasses.h>
#include <re/cc/cc_compiler.h>         // for CC_Compiler
#include <re/cc/cc_compiler_target.h>
#include <re/cc/multiplex_CCs.h>
#include <re/compile/re_compiler.h>

using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

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


    Var * const nonFinal = pb.createVar("nonFinal", u8pfx);
    Var * const u8invalid = pb.createVar("u8invalid", ZEROES);
    Var * const valid_pfx = pb.createVar("valid_pfx", u8pfx);

    auto it = pb.createScope();
    pb.createIf(u8pfx, it);
    PabloAST * const u8pfx2 = ccc->compileCC(makeByte(0xC2, 0xDF), it);
    PabloAST * const u8pfx3 = ccc->compileCC(makeByte(0xE0, 0xEF), it);
    PabloAST * const u8pfx4 = ccc->compileCC(makeByte(0xF0, 0xF4), it);

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
    PabloAST * const u8scope3X = it3.createOr(u8scope32, u8scope33);
    it3.createAssign(anyscope, it3.createOr(anyscope, u8scope3X));

    PabloAST * const advE0 = it3.createAdvance(ccc->compileCC(makeByte(0xE0), it3), 1, "advEO");
    PabloAST * const range80_9F = ccc->compileCC(makeByte(0x80, 0x9F), it3);
    PabloAST * const E0_invalid = it3.createAnd(advE0, range80_9F, "E0_invalid");

    PabloAST * const advED = it3.createAdvance(ccc->compileCC(makeByte(0xED), it3), 1, "advED");
    PabloAST * const rangeA0_BF = ccc->compileCC(makeByte(0xA0, 0xBF), it3);
    PabloAST * const ED_invalid = it3.createAnd(advED, rangeA0_BF, "ED_invalid");

    PabloAST * const EX_invalid = it3.createOr(E0_invalid, ED_invalid);
    it3.createAssign(EF_invalid, EX_invalid);

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
    PabloAST * const u8suffix = ccc->compileCC("u8suffix", makeByte(0x80, 0xBF), it);
    PabloAST * const mismatch = it.createXor(anyscope, u8suffix);
    //
    PabloAST * const pfx_invalid = it.createXor(valid_pfx, legalpfx);
    it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
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

UTF8_index::UTF8_index(BuilderRef kb, StreamSet * Source, StreamSet * u8index)
: PabloKernel(kb, "UTF8_index_" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()),
// input
{Binding{"source", Source}},
// output
{Binding{"u8index", u8index}}) {

}

//=UTF-16 indexing kernel=
void UTF16_index::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc_u16_hi;
    std::unique_ptr<cc::CC_Compiler> ccc_u16_lo;

    //input
    std::vector<PabloAST *> u16bytes = getInputStreamSet("source");
    //separate the 2 bytes into hi and lo 8 bits
    std::vector<PabloAST *> hiByte(u16bytes.begin()+ u16bytes.size()/2, u16bytes.end());
    std::vector<PabloAST *> loByte(u16bytes.begin(), u16bytes.begin()+ u16bytes.size()/2);
    //works only with bitStream
    ccc_u16_hi = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), hiByte);
    ccc_u16_lo = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), loByte);

    PabloAST * const u16sur_1 = ccc_u16_hi->compileCC(makeByte(0xD8, 0xDB));
    PabloAST * const u16sur_2 = ccc_u16_hi->compileCC(makeByte(0xDC, 0xDF)); 
    //mark all low bytes
    PabloAST * const u16lo = ccc_u16_lo->compileCC(makeByte(0x0, 0xFF));
    //mark valid surrogate pair
    PabloAST * const u16_sur = pb.createOr(u16sur_1, u16sur_2);
    //pb.createDebugPrint(u16_sur, "u16sur");
    //mark the prefix of valid surrogate pair
    PabloAST * const u16sur_final = pb.createAnd(u16_sur, u16sur_1);
    //pb.createDebugPrint(u16sur_final, "u16sur_final");
    
    PabloAST * const u16valid = pb.createNot(u16sur_final, "u16prefix");
    //pb.createDebugPrint(u16valid, "u16prefix");
    //mark all 2 byte code units and final code unit of valid surrogare pairs
    PabloAST * const u16valid_final = pb.createAnd(u16lo, u16valid);
    //pb.createDebugPrint(u16valid_final, "u16valid_final");

    //output
    Var * const u16index = getOutputStreamVar("u16index");
    pb.createAssign(pb.createExtract(u16index, pb.getInteger(0)), u16valid_final);
}

UTF16_index::UTF16_index(BuilderRef kb, StreamSet * Source, StreamSet * u16index)
: PabloKernel(kb, "UTF16_index_" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()),
// input
{Binding{"source", Source}},
// output
{Binding{"u16index", u16index}}) {

}
//=UTF-16 indexing kernel=

void GrepKernelOptions::setIndexingTransformer(EncodingTransformer * encodingTransformer, StreamSet * idx) {
    mEncodingTransformer = encodingTransformer;
    mIndexStream = idx;
}

void GrepKernelOptions::setRE(RE * e) {mRE = e;}
void GrepKernelOptions::setPrefixRE(RE * e) {mPrefixRE = e;}
void GrepKernelOptions::setSource(StreamSet * s) {mSource = s;}
void GrepKernelOptions::setCombiningStream(GrepCombiningType t, StreamSet * toCombine){
    mCombiningType = t;
    mCombiningStream = toCombine;
}
void GrepKernelOptions::setResults(StreamSet * r) {mResults = r;}

void GrepKernelOptions::addAlphabet(std::shared_ptr<cc::Alphabet> a, StreamSet * basis) {
    mAlphabets.emplace_back(a, basis);
}

void GrepKernelOptions::addExternal(std::string name, StreamSet * strm, int offset, int lgth, StreamSet * indexStrm) {
    if ((lgth != 1) || (indexStrm != nullptr)) {
        llvm::report_fatal_error("Length and index stream parameters for grep externals not yet supported.");
    }
    if (offset == 0) {
        if (mSource) {
            mExternals.emplace_back(name, strm, FixedRate(), ZeroExtended());
        } else {
            mExternals.emplace_back(name, strm);
        }
    } else {
        if (mSource) {
            std::initializer_list<Attribute> attrs{ZeroExtended(), LookAhead(offset)};
            mExternals.emplace_back(name, strm, FixedRate(), attrs);
        } else {
            mExternals.emplace_back(name, strm, FixedRate(), LookAhead(offset));
        }
    }
}

Bindings GrepKernelOptions::streamSetInputBindings() {
    Bindings inputs;
    if (mSource) {
        inputs.emplace_back(mCodeUnitAlphabet->getName() + "_basis", mSource);
    }
    for (const auto & a : mAlphabets) {
        inputs.emplace_back(a.first->getName() + "_basis", a.second);
    }
    for (const auto & a : mExternals) {
        inputs.emplace_back(a);
    }
    if (mEncodingTransformer) {
        inputs.emplace_back("mIndexing", mIndexStream);
    }
    if (mCombiningType != GrepCombiningType::None) {
        inputs.emplace_back("toCombine", mCombiningStream, FixedRate(), Add1());
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

GrepKernelOptions::GrepKernelOptions(const cc::Alphabet * codeUnitAlphabet, re::EncodingTransformer * encodingTransformer)
: mCodeUnitAlphabet(codeUnitAlphabet)
, mEncodingTransformer(encodingTransformer) {

}

std::string GrepKernelOptions::makeSignature() {
    std::string tmp;
    raw_string_ostream sig(tmp);
    if (mSource) {
        sig << mSource->getNumElements() << 'x' << mSource->getFieldWidth();
        if (mSource->getFieldWidth() == 8) {
            sig << ':' << grep::ByteCClimit;
        }
        sig << '/' << mCodeUnitAlphabet->getName();
    }
    if (mEncodingTransformer) {
        sig << ':' << mEncodingTransformer->getIndexingAlphabet()->getName();
    }
    for (const auto & e : mExternals) {
        sig << '_' << e.getName();
    }
    for (const auto & a: mAlphabets) {
        sig << '_' << a.first->getName();
    }
    if (mCombiningType == GrepCombiningType::Exclude) {
        sig << "&~";
    } else if (mCombiningType == GrepCombiningType::Include) {
        sig << "|=";
    }
    if (mPrefixRE) {
        sig << ':' << Printer_RE::PrintRE(mPrefixRE);
    }
    sig << ':' << Printer_RE::PrintRE(mRE);
    sig.flush();
    return tmp;
}

ICGrepKernel::ICGrepKernel(BuilderRef b, std::unique_ptr<GrepKernelOptions> && options)
: PabloKernel(b, AnnotateWithREflags("ic") + getStringHash(options->makeSignature()),
options->streamSetInputBindings(),
options->streamSetOutputBindings(),
options->scalarInputBindings(),
options->scalarOutputBindings()),
mOptions(std::move(options)),
mSignature(mOptions->makeSignature()) {
    addAttribute(InfrequentlyUsed());    
}

StringRef ICGrepKernel::getSignature() const {
    return mSignature;
}

void ICGrepKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    RE_Compiler re_compiler(getEntryScope(), mOptions->mCodeUnitAlphabet);
    if (mOptions->mSource) {
        std::vector<pablo::PabloAST *> basis_set = getInputStreamSet(mOptions->mCodeUnitAlphabet->getName() + "_basis");
        re_compiler.addAlphabet(mOptions->mCodeUnitAlphabet, basis_set);
    }
    for (unsigned i = 0; i < mOptions->mAlphabets.size(); i++) {
        auto & alpha = mOptions->mAlphabets[i].first;
        auto basis = getInputStreamSet(alpha->getName() + "_basis");
        re_compiler.addAlphabet(alpha, basis);
    }
    if (mOptions->mEncodingTransformer) {
        PabloAST * idxStrm = pb.createExtract(getInputStreamVar("mIndexing"), pb.getInteger(0));
        re_compiler.addIndexingAlphabet(mOptions->mEncodingTransformer, idxStrm);
    }
    for (const auto & e : mOptions->mExternals) {
        PabloAST * extStrm = pb.createExtract(getInputStreamVar(e.getName()), pb.getInteger(0));
        re_compiler.addPrecompiled(e.getName(), RE_Compiler::Marker(extStrm));
    }
    Var * const final_matches = pb.createVar("final_matches", pb.createZeroes());
    if (mOptions->mPrefixRE) {
        RE_Compiler::Marker prefixMatches = re_compiler.compileRE(mOptions->mPrefixRE);
        PabloBlock * scope1 = getEntryScope()->createScope();
        pb.createIf(prefixMatches.stream(), scope1);

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
        RE_Compiler re_compiler(scope1, mOptions->mCodeUnitAlphabet);
        re_compiler.addAlphabet(mOptions->mCodeUnitAlphabet, basis);
        for (unsigned i = 0; i < mOptions->mAlphabets.size(); i++) {
            auto & alpha = mOptions->mAlphabets[i].first;
            auto basis = getInputStreamSet(alpha->getName() + "_basis");
            re_compiler.addAlphabet(alpha, basis);
        }
        if (mOptions->mEncodingTransformer) {
            PabloAST * idxStrm = pb.createExtract(getInputStreamVar("mIndexing"), pb.getInteger(0));
            re_compiler.addIndexingAlphabet(mOptions->mEncodingTransformer, idxStrm);
        }
        for (const auto & e : mOptions->mExternals) {
            PabloAST * extStrm = pb.createExtract(getInputStreamVar(e.getName()), pb.getInteger(0));
            re_compiler.addPrecompiled(e.getName(), RE_Compiler::Marker(extStrm));
        }
        RE_Compiler::Marker matches = re_compiler.compileRE(mOptions->mRE, prefixMatches);
        PabloAST * matchResult = matches.stream();
        if (matches.offset() == 0) matchResult = scope1->createAdvance(matchResult, scope1->getInteger(1));
        scope1->createAssign(final_matches, matchResult);
    } else {
        RE_Compiler::Marker matches = re_compiler.compileRE(mOptions->mRE);
        PabloAST * matchResult = matches.stream();
        if (matches.offset() == 0) matchResult = pb.createAdvance(matchResult, 1);
        pb.createAssign(final_matches, matchResult);
    }
    Var * const output = pb.createExtract(getOutputStreamVar("matches"), pb.getInteger(0));
    PabloAST * value = nullptr;
    if (mOptions->mCombiningType == GrepCombiningType::None) {
        value = final_matches;
    } else {
        PabloAST * toCombine = pb.createExtract(getInputStreamVar("toCombine"), pb.getInteger(0));
        if (mOptions->mCombiningType == GrepCombiningType::Exclude) {
            value = pb.createAnd(toCombine, pb.createNot(final_matches), "toCombine");
        } else {
            value = pb.createOr(toCombine, final_matches, "toCombine");
        }
    }
    pb.createAssign(output, value);
}

void MatchedLinesKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    auto matchResults = getInputStreamSet("matchResults");
    PabloAST * lineBreaks = pb.createExtract(getInputStreamVar("lineBreaks"), pb.getInteger(0));
    PabloAST * notLB = pb.createNot(lineBreaks);
    PabloAST * match_follow = pb.createMatchStar(matchResults.back(), notLB);
    Var * const matchedLines = getOutputStreamVar("matchedLines");
    pb.createAssign(pb.createExtract(matchedLines, pb.getInteger(0)), pb.createAnd(match_follow, lineBreaks, "matchedLines"));
}

MatchedLinesKernel::MatchedLinesKernel (BuilderRef iBuilder, StreamSet * Matches, StreamSet * LineBreakStream, StreamSet * MatchedLines)
: PabloKernel(iBuilder, "MatchedLines" + std::to_string(Matches->getNumElements()),
// inputs
{Binding{"matchResults", Matches}
,Binding{"lineBreaks", LineBreakStream, FixedRate()}},
// output
{Binding{"matchedLines", MatchedLines}}) {

}

void InvertMatchesKernel::generateDoBlockMethod(BuilderRef iBuilder) {
    Value * input = iBuilder->loadInputStreamBlock("matchedLines", iBuilder->getInt32(0));
    Value * lbs = iBuilder->loadInputStreamBlock("lineBreaks", iBuilder->getInt32(0));
    Value * inverted = iBuilder->CreateAnd(iBuilder->CreateNot(input), lbs, "inverted");
    iBuilder->storeOutputStreamBlock("nonMatches", iBuilder->getInt32(0), inverted);
}

InvertMatchesKernel::InvertMatchesKernel(BuilderRef b, StreamSet * Matches, StreamSet * LineBreakStream, StreamSet * InvertedMatches)
: BlockOrientedKernel(b, "Invert" + std::to_string(Matches->getNumElements()),
// Inputs
{Binding{"matchedLines", Matches},
 Binding{"lineBreaks", LineBreakStream}},
// Outputs
{Binding{"nonMatches", InvertedMatches}},
// Input/Output Scalars and internal state
{}, {}, {}) {

}

FixedMatchPairsKernel::FixedMatchPairsKernel(BuilderRef b, unsigned length, StreamSet * MatchResults, StreamSet * MatchPairs)
: PabloKernel(b, "FixedMatchPairsKernel" + std::to_string(MatchResults->getNumElements()) + "x1_by" + std::to_string(length),
{Binding{"MatchResults", MatchResults, FixedRate(1), LookAhead(length)}}, {Binding{"MatchPairs", MatchPairs}}),
mMatchLength(length) {}

void FixedMatchPairsKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    Var * matchResults = getInputStreamVar("MatchResults");
    PabloAST * matchFollows = pb.createExtract(matchResults, pb.getInteger(0));
    Var * matchPairsVar = getOutputStreamVar("MatchPairs");
    PabloAST * starts = pb.createLookahead(matchFollows, mMatchLength);
    PabloAST * disjoint = pb.createXor(starts, matchFollows);
    starts = pb.createAnd(starts, disjoint);
    matchFollows = pb.createAnd(matchFollows, disjoint);
    pb.createAssign(pb.createExtract(matchPairsVar, 0), starts);
    pb.createAssign(pb.createExtract(matchPairsVar, 1), matchFollows);
}

void PopcountKernel::generatePabloMethod() {
    auto pb = getEntryScope();
    const auto toCount = pb->createExtract(getInputStreamVar("toCount"), pb->getInteger(0));
    pablo::Var * countResult = getOutputScalarVar("countResult");

    pb->createAssign(countResult, pb->createCount(pb->createInFile(toCount)));
}

PopcountKernel::PopcountKernel (BuilderRef iBuilder, StreamSet * const toCount, Scalar * countResult)
: PabloKernel(iBuilder, "Popcount",
{Binding{"toCount", toCount}},
{},
{},
{Binding{"countResult", countResult}}) {

}


void AbortOnNull::generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) {
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
    b->CreateCondBr(b->isFinal(), finalStride, strideLoop);

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
    b->setFatalTerminationSignal();
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

AbortOnNull::AbortOnNull(BuilderRef b, StreamSet * const InputStream, StreamSet * const OutputStream, Scalar * callbackObject)
: MultiBlockKernel(b, "AbortOnNull",
// inputs
{Binding{"byteData", InputStream, FixedRate(), Principal()}},
// outputs
{Binding{ "untilNull", OutputStream, FixedRate(), Deferred()}},
// input scalars
{Binding{"handler_address", callbackObject}},
{}, {}) {
    addAttribute(CanTerminateEarly());
    addAttribute(MayFatallyTerminate());
}

ContextSpan::ContextSpan(BuilderRef b, StreamSet * const markerStream, StreamSet * const contextStream, unsigned before, unsigned after)
: PabloKernel(b, "ContextSpan-" + std::to_string(before) + "+" + std::to_string(after),
              // input
{Binding{"markerStream", markerStream, FixedRate(1), LookAhead(before)}},
              // output
{Binding{"contextStream", contextStream}}),
mBeforeContext(before), mAfterContext(after) {
}

void ContextSpan::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    Var * markerStream = pb.createExtract(getInputStreamVar("markerStream"), pb.getInteger(0));
    PabloAST * contextStart = pb.createLookahead(markerStream, pb.getInteger(mBeforeContext));
    unsigned lgth = mBeforeContext + 1 + mAfterContext;
    PabloAST * consecutive = contextStart;
    unsigned consecutiveCount = 1;
    for (unsigned i = 1; i <= lgth/2; i *= 2) {
        consecutiveCount += i;
        consecutive = pb.createOr(consecutive,
                                  pb.createAdvance(consecutive, i),
                                  "consecutive" + std::to_string(consecutiveCount));
    }
    if (consecutiveCount < lgth) {
        consecutive = pb.createOr(consecutive,
                                  pb.createAdvance(consecutive, lgth - consecutiveCount),
                                  "consecutive" + std::to_string(lgth));
    }
    pb.createAssign(pb.createExtract(getOutputStreamVar("contextStream"), pb.getInteger(0)), pb.createInFile(consecutive));
}

void kernel::GraphemeClusterLogic(const std::unique_ptr<ProgramBuilder> & P, UTF16_Transformer * t,
                          StreamSet * Source, StreamSet * U8index, StreamSet * GCBstream) {

    re::RE * GCB = re::generateGraphemeClusterBoundaryRule();
    GCB = re::resolveUnicodeNames(GCB);
    const auto GCB_Sets = re::collectCCs(GCB, cc::Unicode);
    auto GCB_mpx = std::make_shared<cc::MultiplexedAlphabet>("GCB_mpx", GCB_Sets);
    GCB = transformCCs(GCB_mpx, GCB);
    auto GCB_basis = GCB_mpx->getMultiplexedCCs();
    StreamSet * const GCB_Classes = P->CreateStreamSet(GCB_basis.size());
    P->CreateKernelCall<CharClassesKernel>(std::move(GCB_basis), Source, GCB_Classes);
    std::unique_ptr<GrepKernelOptions> options = make_unique<GrepKernelOptions>();
    options->setIndexingTransformer(t, U8index);
    options->setRE(GCB);
    options->setSource(GCB_Classes);
    options->addAlphabet(GCB_mpx, GCB_Classes);
    options->setResults(GCBstream);
    options->addExternal("UTF16_index", U8index);
    P->CreateKernelCall<ICGrepKernel>(std::move(options));
}
