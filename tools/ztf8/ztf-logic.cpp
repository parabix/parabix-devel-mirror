/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "ztf-logic.h"
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
#include <llvm/Support/raw_ostream.h>
#include <pablo/pe_debugprint.h>
#include <sstream>

using namespace pablo;
using namespace kernel;
using namespace llvm;

unsigned EncodingInfo::getLengthGroupNo(unsigned lgth) const {
    for (unsigned i = 0; i < byLength.size(); i++) {
        if ((byLength[i].lo <= lgth)  && (byLength[i].hi >= lgth)) {
            return i;
        }
    }
    llvm_unreachable("failed to locate length group info");
}

unsigned EncodingInfo::maxSymbolLength() const {
    unsigned maxSoFar = 0;
    for (unsigned i = 0; i < byLength.size(); i++) {
        if (byLength[i].hi >=  maxSoFar) {
            maxSoFar = byLength[i].hi;
        }
    }
    return maxSoFar;
}

unsigned EncodingInfo::maxEncodingBytes() const {
    unsigned enc_bytes = 0;
    for (auto g : byLength) {
        if (g.encoding_bytes > enc_bytes) enc_bytes = g.encoding_bytes;
    }
    return enc_bytes;
}

std::string EncodingInfo::uniqueSuffix() const {
    std::stringstream s;
    for (auto g : byLength) {
        s << "_" << g.lo << "_" << g.hi << ":" << g.encoding_bytes;
        s << "@" << std::hex << g.prefix_base << std::dec;
        s << ":" << g.hash_bits << ":" << g.length_extension_bits;
    }
    return s.str();
}

unsigned EncodingInfo::prefixLengthOffset(unsigned lgth) const {
    unsigned groupNo = getLengthGroupNo(lgth);
    auto g = byLength[groupNo];
    unsigned suffix_bits_avail = (g.encoding_bytes - 1) * 7;
    unsigned hash_ext_bits = g.hash_bits + g.length_extension_bits;
    return suffix_bits_avail < hash_ext_bits ? hash_ext_bits - suffix_bits_avail : 0;
}

WordMarkKernel::WordMarkKernel(BuilderRef kb, StreamSet * BasisBits, StreamSet * WordMarks)
: PabloKernel(kb, "WordMarks", {Binding{"source", BasisBits}}, {Binding{"WordMarks", WordMarks}}) { }

void WordMarkKernel::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::Name * word = re::makeName("word", re::Name::Type::UnicodeProperty);
    word = llvm::cast<re::Name>(re::resolveUnicodeNames(word));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(word, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(word);
    if (f == nameMap.end()) llvm::report_fatal_error("Cannot find word property");
    PabloAST * wordChar = f->second;
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordMarks"), pb.getInteger(0)), wordChar);
}

PhraseMarker::PhraseMarker(BuilderRef kb, StreamSet * WordMarks, StreamSet * PhraseMarks)
: PabloKernel(kb, "PhraseMarker", {Binding{"WordMarks", WordMarks}}, {Binding{"PhraseMarks", PhraseMarks}}) { }

void PhraseMarker::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    PabloAST * WordMarks = getInputStreamSet("WordMarks")[0];
    PabloAST * nonWord = pb.createNot(WordMarks);
    PabloAST * oddNonWord = pb.createEveryNth(nonWord, pb.getInteger(2));
    PabloAST * evenNonWord = pb.createIndexedAdvance(oddNonWord, nonWord, 1);
    //pb.createDebugPrint(evenNonWord, "evenNonWord");
    //pb.createDebugPrint(oddNonWord, "oddNonWord");

    PabloAST * phrasePos1 = pb.createIntrinsicCall(pablo::Intrinsic::SpanUpTo, {pb.createAdvance(evenNonWord, 1), evenNonWord});
    PabloAST * phrasePos2 = pb.createIntrinsicCall(pablo::Intrinsic::SpanUpTo, {pb.createAdvance(oddNonWord, 1), oddNonWord});

    PabloAST * oddPhrasePosExceptFirst = pb.createAnd(pb.createOr(oddNonWord, WordMarks), phrasePos1);
    PabloAST * evenPhrasePos = pb.createAnd(pb.createOr(evenNonWord, WordMarks), phrasePos2);
    //pb.createDebugPrint(oddPhrasePosExceptFirst, "oddPhrasePosExceptFirst");
    //pb.createDebugPrint(evenPhrasePos, "evenPhrasePos");

    pb.createAssign(pb.createExtract(getOutputStreamVar("PhraseMarks"), pb.getInteger(0)), pb.createOr(evenPhrasePos, WordMarks));
}

void ByteRun::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * excluded = getInputStreamSet("excluded")[0];

    PabloAST * mismatches = pb.createZeroes();
    for (unsigned i = 0; i < 8; i++) {
        mismatches = pb.createOr(mismatches,
                                 pb.createXor(basis[i], pb.createAdvance(basis[i], 1)),
                                 "mismatches_to_bit" + std::to_string(i));
    }
    PabloAST * matchesprior = pb.createInFile(pb.createNot(mismatches), "matchesprior");
    matchesprior = pb.createAnd(matchesprior, pb.createNot(excluded));
    pb.createAssign(pb.createExtract(getOutputStreamVar("runMask"), pb.getInteger(0)), matchesprior);
}

ZTF_ExpansionDecoder::ZTF_ExpansionDecoder(BuilderRef b,
                                           EncodingInfo & encodingScheme,
                                           StreamSet * const basis,
                                           StreamSet * insertBixNum)
: pablo::PabloKernel(b, "ZTF_ExpansionDecoder" + encodingScheme.uniqueSuffix(),
                     {Binding{"basis", basis, FixedRate(), LookAhead(encodingScheme.maxEncodingBytes() - 1)}},
                     {Binding{"insertBixNum", insertBixNum}}),
    mEncodingScheme(encodingScheme)  {}

void ZTF_ExpansionDecoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * ASCII_lookahead = pb.createNot(pb.createLookahead(basis[7], 1));
    for (unsigned i = 2; i < mEncodingScheme.maxEncodingBytes(); i++) {
        ASCII_lookahead = pb.createAnd(ASCII_lookahead, pb.createNot(pb.createLookahead(basis[7], pb.getInteger(i))));
    }
    BixNum insertLgth(4, pb.createZeroes());
    for (unsigned i = 0; i < mEncodingScheme.byLength.size(); i++) {
        LengthGroupInfo groupInfo = mEncodingScheme.byLength[i];
        unsigned lo = groupInfo.lo;
        unsigned hi = groupInfo.hi;
        unsigned suffix_bits_avail = (groupInfo.encoding_bytes - 1) * 7;
        unsigned hash_ext_bits = groupInfo.hash_bits + groupInfo.length_extension_bits;
        unsigned excess_bits = suffix_bits_avail < hash_ext_bits ? hash_ext_bits - suffix_bits_avail : 0;
        unsigned multiplier = 1<<excess_bits;
        unsigned base = groupInfo.prefix_base;
        unsigned next_base = base + multiplier * (hi - lo + 1);
        PabloAST * inGroup = pb.createAnd3(ASCII_lookahead, bnc.UGE(basis, base), bnc.ULT(basis, next_base));
        BixNum relative = bnc.HighBits(bnc.SubModular(basis, base), 8 - excess_bits);
        BixNum toInsert = bnc.AddModular(relative, lo - groupInfo.encoding_bytes);
        for (unsigned i = 0; i < 4; i++) {
            insertLgth[i] = pb.createSel(inGroup, toInsert[i], insertLgth[i], "insertLgth[" + std::to_string(i) + "]");
        }
    }
    Var * lengthVar = getOutputStreamVar("insertBixNum");
    for (unsigned i = 0; i < 4; i++) {
        pb.createAssign(pb.createExtract(lengthVar, pb.getInteger(i)), insertLgth[i]);
    }
}

ZTF_DecodeLengths::ZTF_DecodeLengths(BuilderRef b,
                                     EncodingInfo & encodingScheme,
                                     StreamSet * basisBits,
                                     StreamSet * groupStreams)
: PabloKernel(b, "ZTF_DecodeLengths" + encodingScheme.uniqueSuffix(),
              {Binding{"basisBits", basisBits}}, {Binding{"groupStreams", groupStreams}}),
    mEncodingScheme(encodingScheme) { }

void ZTF_DecodeLengths::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> basis = getInputStreamSet("basisBits");
    std::vector<PabloAST *> groupStreams(mEncodingScheme.byLength.size());
    PabloAST * ASCII = bnc.ULT(basis, 0x80);
    Var * groupStreamVar = getOutputStreamVar("groupStreams");
    for (unsigned i = 0; i < mEncodingScheme.byLength.size(); i++) {
        LengthGroupInfo groupInfo = mEncodingScheme.byLength[i];
        unsigned lo = groupInfo.lo;
        unsigned hi = groupInfo.hi;
        unsigned suffix_bits_avail = (groupInfo.encoding_bytes - 1) * 7;
        unsigned hash_ext_bits = groupInfo.hash_bits + groupInfo.length_extension_bits;
        unsigned excess_bits = suffix_bits_avail < hash_ext_bits ? hash_ext_bits - suffix_bits_avail : 0;
        unsigned multiplier = 1<<excess_bits;
        unsigned base = groupInfo.prefix_base;
        unsigned next_base = base + multiplier * (hi - lo + 1);
        PabloAST * inGroup = pb.createAnd(bnc.UGE(basis, base), bnc.ULT(basis, next_base));
        // std::string groupName = "lengthGroup" + std::to_string(lo) +  "_" + std::to_string(hi);
        groupStreams[i] = pb.createAnd(pb.createAdvance(inGroup, 1), ASCII);
        for (unsigned j = 2; j < mEncodingScheme.maxEncodingBytes(); j++) {
            groupStreams[i] = pb.createAnd(pb.createAdvance(groupStreams[i], 1), ASCII);
        }
        pb.createAssign(pb.createExtract(groupStreamVar, pb.getInteger(i)), groupStreams[i]);
    }
}

void ZTF_Symbols::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basisBits");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    pablo::PabloAST * wordChar = getInputStreamSet("wordChar")[0];
    // Find start bytes of word characters.
    PabloAST * ASCII = ccc.compileCC(re::makeCC(0x0, 0x7F));
    PabloAST * prefix2 = ccc.compileCC(re::makeCC(0xC2, 0xDF));
    PabloAST * prefix3 = ccc.compileCC(re::makeCC(0xE0, 0xEF));
    PabloAST * prefix4 = ccc.compileCC(re::makeCC(0xF0, 0xF4));
    PabloAST * wc1 = pb.createAnd(ASCII, wordChar);
    wc1 = pb.createOr(wc1, pb.createAnd(prefix2, pb.createLookahead(wordChar, 1)));
    wc1 = pb.createOr(wc1, pb.createAnd(prefix3, pb.createLookahead(wordChar, 2)));
    wc1 = pb.createOr(wc1, pb.createAnd(prefix4, pb.createLookahead(wordChar, 3)));
    //
    // ZTF Code symbols
    PabloAST * anyPfx = ccc.compileCC(re::makeCC(0xC0, 0xFF));
    PabloAST * ZTF_sym = pb.createAnd(pb.createAdvance(anyPfx, 1), ASCII);
    PabloAST * ZTF_prefix = pb.createAnd(anyPfx, pb.createNot(pb.createLookahead(basis[7], 1)));
    // Filter out ZTF code symbols from word characters.
    wc1 = pb.createAnd(wc1, pb.createNot(ZTF_sym));
    //
    PabloAST * wordStart = pb.createAnd(pb.createNot(pb.createAdvance(wordChar, 1)), wc1, "wordStart");
    // Nulls, Linefeeds and ZTF_symbols are also treated as symbol starts.
    PabloAST * LF = ccc.compileCC(re::makeByte(0x0A));
    PabloAST * Null = ccc.compileCC(re::makeByte(0x0));
    PabloAST * fileStart = pb.createNot(pb.createAdvance(pb.createOnes(), 1));
    PabloAST * symStart = pb.createOr3(wordStart, ZTF_prefix, pb.createOr3(LF, Null, fileStart));
    // The next character after a ZTF symbol or a line feed also starts a new symbol.
    symStart = pb.createOr(symStart, pb.createAdvance(pb.createOr(ZTF_sym, LF), 1), "symStart");
    //
    // runs are the bytes after a start symbol until the next symStart byte.
    pablo::PabloAST * runs = pb.createInFile(pb.createNot(symStart));
    pb.createAssign(pb.createExtract(getOutputStreamVar("symbolRuns"), pb.getInteger(0)), runs);
}

ZTF_SymbolEncoder::ZTF_SymbolEncoder(BuilderRef b,
                      EncodingInfo & encodingScheme,
                      StreamSet * const basis,
                      StreamSet * bixHash,
                      StreamSet * extractionMask,
                      StreamSet * runIdx,
                      StreamSet * encoded)
    : pablo::PabloKernel(b, "ZTF_SymbolEncoder" + encodingScheme.uniqueSuffix(),
                         {Binding{"basis", basis},
                             Binding{"bixHash", bixHash, FixedRate(), LookAhead(encodingScheme.maxEncodingBytes() - 1)},
                             Binding{"extractionMask", extractionMask},
                             Binding{"runIdx", runIdx}},
                         {Binding{"encoded", encoded}}),
    mEncodingScheme(encodingScheme) {}

void ZTF_SymbolEncoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    std::vector<PabloAST *> bixHash = getInputStreamSet("bixHash");
    PabloAST * extractionMask = getInputStreamSet("extractionMask")[0];
    BixNum runIdx = getInputStreamSet("runIdx");
    std::vector<PabloAST *> encoded = basis;
    Var * encodedVar = getOutputStreamVar("encoded");
    //  ZTF symbol prefixes are inserted at the position of the first 1 bit
    //  following a series of 0 bits in the extraction mask.
    PabloAST * ZTF_prefix = pb.createAnd(extractionMask, pb.createAdvance(pb.createNot(extractionMask), 1), "ZTF_prefix");
    // ZTF prefixes depend on the length group, but always have the high 2 bits set in each case.
    encoded[7] = pb.createOr(ZTF_prefix, basis[7]);
    encoded[6] = pb.createOr(ZTF_prefix, basis[6]);
    // There may be more than one suffix.  Number them in reverse order.
    std::vector<PabloAST *> ZTF_suffix(mEncodingScheme.maxEncodingBytes() - 1, pb.createZeroes());
    for (unsigned i = 0; i < mEncodingScheme.byLength.size(); i++) {
        LengthGroupInfo groupInfo = mEncodingScheme.byLength[i];
        unsigned suffix_bytes = groupInfo.encoding_bytes - 1;
        // The run index counts from the second position of the run starting with 0,
        // so the length is 2 + the runIndex value at the end of the symbol.
        // At the ZTF_prefix position, the offset is higher by the byte length - 1.
        unsigned offset = groupInfo.encoding_bytes + 1;
        unsigned lo = groupInfo.lo;
        unsigned hi = groupInfo.hi;
        PabloAST * inGroup = pb.createAnd(bnc.UGE(runIdx, lo - offset), bnc.ULE(runIdx, hi - offset));
        inGroup = pb.createAnd(inGroup, ZTF_prefix, "inGroup" + std::to_string(lo) + "_" + std::to_string(hi));
        unsigned suffix_bits_avail = (groupInfo.encoding_bytes - 1) * 7;
        unsigned hash_ext_bits = groupInfo.hash_bits + groupInfo.length_extension_bits;
        unsigned excess_bits = suffix_bits_avail < hash_ext_bits ? hash_ext_bits - suffix_bits_avail : 0;
        unsigned multiplier = 1<<excess_bits;
        BixNum base = bnc.ZeroExtend(bnc.Create(groupInfo.prefix_base & 0x3F), 6); // only low 6 bits needed
        BixNum lenOffset = bnc.SubModular(bnc.ZeroExtend(runIdx, 6), lo - offset);
        BixNum value = bnc.AddModular(base, bnc.MulModular(lenOffset, multiplier));
        // Cannot handle length-extension bits yet.
        for (unsigned j = 0; j < excess_bits - groupInfo.length_extension_bits; j++) {
            value[j] = pb.createLookahead(bixHash[j + suffix_bits_avail], groupInfo.encoding_bytes - 1);
        }
        for (unsigned j = 0; j < 6; j++) {
            encoded[j] = pb.createSel(inGroup, value[j], encoded[j], "encoded[" + std::to_string(j) + "]");
        }
        for (unsigned j = 0; j < suffix_bytes; j++) {
            ZTF_suffix[j] = pb.createOr(ZTF_suffix[j], pb.createAdvance(inGroup, suffix_bytes - j));
        }
    }
    // Final suffix positions receive bits 0 through 7
    for (unsigned i = 0; i < 7; i++)  {
        encoded[i] = pb.createSel(ZTF_suffix[0], bixHash[i], encoded[i]);
    }
    encoded[7] = pb.createAnd(encoded[7], pb.createNot(ZTF_suffix[0]));
    //
    // Other suffix positions receive higher numbered bits.
    for (unsigned i = 1; i < mEncodingScheme.maxEncodingBytes() - 1; i++) {
        for (unsigned j = 0; j < 7; j++)  {
            if ((j + 7 * i) < mEncodingScheme.MAX_HASH_BITS) {
                encoded[j] = pb.createSel(ZTF_suffix[i], pb.createLookahead(bixHash[j + 7 * i], i), encoded[j]);
            } else {
                encoded[j] = pb.createAnd(encoded[j], pb.createNot(ZTF_suffix[i]));
            }
        }
        encoded[7] = pb.createAnd(encoded[7], pb.createNot(ZTF_suffix[i]));
    }
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(encodedVar, pb.getInteger(i)), encoded[i]);
    }
}

void ZTF_SymbolEnds::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * run = getInputStreamSet("symbolRuns")[0];
    PabloAST * overflow = getInputStreamSet("overflow")[0];
    PabloAST * runFinal = pb.createAnd(run, pb.createNot(pb.createLookahead(run, 1)));
    runFinal = pb.createAnd(runFinal, pb.createNot(overflow));
    pb.createAssign(pb.createExtract(getOutputStreamVar("symbolEnds"), pb.getInteger(0)), runFinal);
}

std::string LengthSelectorSuffix(EncodingInfo & encodingScheme, unsigned groupNo, StreamSet * bixNum) {
    auto g = encodingScheme.byLength[groupNo];
    auto elems = bixNum->getNumElements();
    return encodingScheme.uniqueSuffix() + ":" + std::to_string(g.lo) + "_" + std::to_string(g.hi) + "_" + std::to_string(elems);
}

LengthGroupSelector::LengthGroupSelector(BuilderRef b,
                           EncodingInfo & encodingScheme,
                           unsigned groupNo,
                           StreamSet * symbolRun,
                           StreamSet * const lengthBixNum,
                           StreamSet * overflow,
                           StreamSet * selected,
                           StreamSet * longSymSequence)
: PabloKernel(b, "LengthGroupSelector" + LengthSelectorSuffix(encodingScheme, groupNo, lengthBixNum),
              {Binding{"symbolRun", symbolRun, FixedRate(), LookAhead(1)},
                  Binding{"lengthBixNum", lengthBixNum},
                  Binding{"overflow", overflow}},
              {Binding{"selected", selected},
              Binding{"longSymSequence", longSymSequence}}), mEncodingScheme(encodingScheme), mGroupNo(groupNo) { }

void LengthGroupSelector::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * run = getInputStreamSet("symbolRun")[0];
    std::vector<PabloAST *> lengthBixNum = getInputStreamSet("lengthBixNum");
    PabloAST * overflow = getInputStreamSet("overflow")[0];
    PabloAST * runFinal = pb.createAnd(run, pb.createNot(pb.createLookahead(run, 1)));
    runFinal = pb.createAnd(runFinal, pb.createNot(overflow));
    Var * groupStreamVar = getOutputStreamVar("selected");
    Var * symSequenceVar = getOutputStreamVar("longSymSequence");
    LengthGroupInfo groupInfo = mEncodingScheme.byLength[mGroupNo];
    // Run index codes count from 0 on the 2nd byte of a symbol.
    // So the length is 2 more than the bixnum.
    unsigned offset = 2;
    unsigned lo = groupInfo.lo;
    unsigned hi = groupInfo.hi;
    std::string groupName = "lengthGroup" + std::to_string(lo) +  "_" + std::to_string(hi);
    PabloAST * groupStream = pb.createAnd3(bnc.UGE(lengthBixNum, lo - offset), bnc.ULE(lengthBixNum, hi - offset), runFinal, groupName);
    PabloAST * longestSymSequence = pb.createZeroes();
    for (unsigned i = 1; i <= lo; i++) {
        PabloAST * symbolSequence = pb.createAnd(groupStream, pb.createAdvance(groupStream, lo*i));
        longestSymSequence = pb.createOr(longestSymSequence, symbolSequence);
    }
    pb.createAssign(pb.createExtract(groupStreamVar, pb.getInteger(0)), groupStream);
    pb.createAssign(pb.createExtract(symSequenceVar, pb.getInteger(0)), longestSymSequence);
}


LengthSorter::LengthSorter(BuilderRef b,
                           EncodingInfo & encodingScheme,
                           StreamSet * symbolRun, StreamSet * const lengthBixNum,
                           StreamSet * overflow,
                           StreamSet * groupStreams)
: PabloKernel(b, "LengthSorter" + std::to_string(lengthBixNum->getNumElements()) + "x1:" + encodingScheme.uniqueSuffix(),
              {Binding{"symbolRun", symbolRun, FixedRate(), LookAhead(1)},
                  Binding{"lengthBixNum", lengthBixNum},
                  Binding{"overflow", overflow}},
              {Binding{"groupStreams", groupStreams}}), mEncodingScheme(encodingScheme) { }

void LengthSorter::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * run = getInputStreamSet("symbolRun")[0];
    std::vector<PabloAST *> lengthBixNum = getInputStreamSet("lengthBixNum");
    PabloAST * overflow = getInputStreamSet("overflow")[0];
    PabloAST * runFinal = pb.createAnd(run, pb.createNot(pb.createLookahead(run, 1)));
    runFinal = pb.createAnd(runFinal, pb.createNot(overflow));
    std::vector<PabloAST *> groupStreams(mEncodingScheme.byLength.size());
    Var * groupStreamVar = getOutputStreamVar("groupStreams");
    for (unsigned i = 0; i < mEncodingScheme.byLength.size(); i++) {
        // Run index codes count from 0 on the 2nd byte of a symbol.
        // So the length is 2 more than the bixnum.
        unsigned offset = 2;
        LengthGroupInfo groupInfo = mEncodingScheme.byLength[i];
        unsigned lo = groupInfo.lo;
        unsigned hi = groupInfo.hi;
        std::string groupName = "lengthGroup" + std::to_string(lo) +  "_" + std::to_string(hi);
        groupStreams[i] = pb.createAnd3(bnc.UGE(lengthBixNum, lo - offset), bnc.ULE(lengthBixNum, hi - offset), runFinal, groupName);
        pb.createAssign(pb.createExtract(groupStreamVar, pb.getInteger(i)), groupStreams[i]);
    }
}
