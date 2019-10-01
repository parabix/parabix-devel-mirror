/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/streamutils/deletion.h>                      // for DeletionKernel
#include <kernel/io/source_kernel.h>
#include <kernel/basis/p2s_kernel.h>
#include <kernel/basis/s2p_kernel.h>                    // for S2PKernel
#include <kernel/io/stdout_kernel.h>                 // for StdOutKernel_
#include <kernel/streamutils/pdep_kernel.h>
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <pablo/pablo_kernel.h>                    // for PabloKernel
#include <toolchain/pablo_toolchain.h>
#include <pablo/parse/pablo_source_kernel.h>
#include <pablo/parse/pablo_parser.h>
#include <pablo/parse/simple_lexer.h>
#include <pablo/parse/rd_parser.h>
#include <re/adt/re_name.h>
#include <re/adt/re_re.h>
#include <grep/grep_kernel.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/resolve_properties.h>
#include <re/unicode/re_name_resolve.h>
#include <pablo/bixnum/bixnum.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/toolchain.h>
#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/streamset.h>
#include <kernel/streamutils/run_index.h>
#include <kernel/streamutils/streams_merge.h>
#include <kernel/util/bixhash.h>
#include <kernel/util/debug_display.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>
#include <fcntl.h>
#include <iostream>
#include <iomanip>
#include <kernel/pipeline/pipeline_builder.h>
#include "ztf-kernel.h"
#include "ztf-scan.h"

using namespace pablo;
using namespace parse;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory ztfHashOptions("ztfHash Options", "ZTF-Hash options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztfHashOptions));
static cl::opt<bool> Decompression("d", cl::desc("Decompress from ZTF-Runs to UTF-8."), cl::cat(ztfHashOptions), cl::init(false));
static cl::alias DecompressionAlias("decompress", cl::desc("Alias for -d"), cl::aliasopt(Decompression));

class ZTF_HashMarks : public PabloKernel {
public:
    ZTF_HashMarks(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basisBits, StreamSet * mark3_4, StreamSet * mark5_8, StreamSet * mark9_16)
    : PabloKernel(kb, "ZTF_HashMarks",
                  {Binding{"basisBits", basisBits}},
                  {Binding{"mark3_4", mark3_4}, Binding{"mark5_8", mark5_8}, Binding{"mark9_16", mark9_16}}) { }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_HashMarks::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basisBits");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    PabloAST * ASCII = ccc.compileCC(re::makeCC(0x0, 0x7F));
    PabloAST * mark3_4 = pb.createAnd(pb.createAdvance(ccc.compileCC(re::makeCC(0xC2, 0xC5)), 1), ASCII, "mark3_4");
    PabloAST * mark5_8 = pb.createAnd(pb.createAdvance(ccc.compileCC(re::makeCC(0xC6, 0xCD)), 1), ASCII, "mark5_8");
    PabloAST * mark9_16 = pb.createAnd(pb.createAdvance(ccc.compileCC(re::makeCC(0xCE, 0xDD)), 1), ASCII, "mark9_16");
    pb.createAssign(pb.createExtract(getOutputStreamVar("mark3_4"), pb.getInteger(0)), mark3_4);
    pb.createAssign(pb.createExtract(getOutputStreamVar("mark5_8"), pb.getInteger(0)), mark5_8);
    pb.createAssign(pb.createExtract(getOutputStreamVar("mark9_16"), pb.getInteger(0)), mark9_16);
}

class ZTF_Symbols : public PabloKernel {
public:
    ZTF_Symbols(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basisBits, StreamSet * wordChar, StreamSet * symbolRuns)
    : PabloKernel(kb, "ZTF_Symbols",
                  {Binding{"basisBits", basisBits, FixedRate(1), LookAhead(1)},
                      Binding{"wordChar", wordChar, FixedRate(1), LookAhead(3)}},
                  {Binding{"symbolRuns", symbolRuns}}) { }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

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
    PabloAST * ZTF_sym = pb.createAnd(pb.createAdvance(prefix2, 1), ASCII);
    PabloAST * ZTF_prefix = pb.createAnd(prefix2, pb.createNot(pb.createLookahead(basis[7], 1)));
    // Filter out ZTF code symbols from word characters.
    wc1 = pb.createAnd(wc1, pb.createNot(ZTF_sym));
    //
    PabloAST * wordStart = pb.createAnd(pb.createNot(pb.createAdvance(wordChar, 1)), wc1, "wordStart");
    // Nulls, Linefeeds and ZTF_symbols are also treated as symbol starts.
    PabloAST * LF = ccc.compileCC(re::makeByte(0x0A));
    PabloAST * Null = ccc.compileCC(re::makeByte(0x0));
    PabloAST * symStart = pb.createOr3(wordStart, ZTF_prefix, pb.createOr(LF, Null));
    // The next character after a ZTF symbol or a line feed also starts a new symbol.
    symStart = pb.createOr(symStart, pb.createAdvance(pb.createOr(ZTF_sym, LF), 1), "symStart");
    //
    // runs are the bytes after a start symbol until the next symStart byte.
    pablo::PabloAST * runs = pb.createInFile(pb.createNot(symStart));
    pb.createAssign(pb.createExtract(getOutputStreamVar("symbolRuns"), pb.getInteger(0)), runs);
}


class ZTF_SymbolEncoder final: public PabloKernel {
public:
    ZTF_SymbolEncoder(const std::unique_ptr<kernel::KernelBuilder> & b,
                      StreamSet * const basis, StreamSet * bixHash, StreamSet * extractionMask, StreamSet * runIdx, StreamSet * encoded)
    : PabloKernel(b, "ZTF_SymbolEncoder", {Binding{"basis", basis},
        Binding{"bixHash", bixHash, FixedRate(), LookAhead(1)},
        Binding{"extractionMask", extractionMask},
        Binding{"runIdx", runIdx}},
                  {Binding{"encoded", encoded}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_SymbolEncoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    std::vector<PabloAST *> bixHash = getInputStreamSet("bixHash");
    PabloAST * extractionMask = getInputStreamSet("extractionMask")[0];
    std::vector<PabloAST *> runIdx = getInputStreamSet("runIdx");
    std::vector<PabloAST *> encoded(8);
    Var * encodedVar = getOutputStreamVar("encoded");
    //  ZTF symbol prefixes are inserted at the position of the first 1 bit
    //  following a series of 0 bits in the extraction mask.
    PabloAST * ZTF_prefix = pb.createAnd(extractionMask, pb.createAdvance(pb.createNot(extractionMask), 1));
    PabloAST * ZTF_suffix = pb.createAdvance(ZTF_prefix, 1);
    // Other extracted positions keep their data bits from the basis bit stream.
    // PabloAST * unencoded = pb.createXor(extractionMask, pb.createOr(ZTF_prefix, ZTF_suffix));
    // The ZTF prefix is formed by from the prefix base 0xC0 and adding twice
    // the compression count, plus the high bit of the hash Value.
    BixNum compCount = bnc.AddModular(runIdx, 1);
    PabloAST * highHashBit = pb.createLookahead(bixHash[7], 1);
    //
    // The high bit of the ZTF symbol is 1 for a prefix, 0 for a suffix or the basis[7] value.
    encoded[7] = pb.createAnd(pb.createOr(ZTF_prefix, basis[7]), pb.createNot(ZTF_suffix));
    // The second high bit of the ZTF symbol is 1 for a prefix, bixHash[6] for a suffix or the basis[6] value.
    encoded[6] = pb.createOr(ZTF_prefix, pb.createSel(ZTF_suffix, bixHash[6], basis[6]));
    // The third high bit of the ZTF symbol is 0 for a prefix, bixHash[5] for a suffix or the basis[5] value.
    encoded[5] = pb.createAnd(pb.createSel(ZTF_suffix, bixHash[5], basis[5]), pb.createNot(ZTF_prefix));
    // Bits 1 through 4 are selected from the compressionCount, basis or bixHash.
    encoded[4] = pb.createSel(ZTF_prefix, compCount[3], pb.createSel(ZTF_suffix, bixHash[4], basis[4]));
    encoded[3] = pb.createSel(ZTF_prefix, compCount[2], pb.createSel(ZTF_suffix, bixHash[3], basis[3]));
    encoded[2] = pb.createSel(ZTF_prefix, compCount[1], pb.createSel(ZTF_suffix, bixHash[2], basis[2]));
    encoded[1] = pb.createSel(ZTF_prefix, compCount[0], pb.createSel(ZTF_suffix, bixHash[1], basis[1]));
    // The low bit uses the highHashBit in case of a prefix.
    encoded[0] = pb.createSel(ZTF_prefix, highHashBit, pb.createSel(ZTF_suffix, bixHash[0], basis[0]));
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(encodedVar, pb.getInteger(i)), encoded[i]);
    }
}


/*
 This kernel decodes the insertion length for two-byte ZTF code symbols
 in the range 0xC2-0xDF.   The insertion length is the number of zero
 bytes to insert so that, after insertion the zeroes together with the
 encoded symbol can be replaced by the dictionary symbol of the appropriate
 length.
 
 The following table shows the pattern of the insertion lengths.
 0xC2, 0xC3   final length 3, insertion length 1
 0xC4, 0xC5   final length 4, insertion length 2
 0xC6, 0xC7   final length 5, insertion length 3
 ...
 0xDE, 0xDF   final length 17, insertion length 15
 
 As it turns out, the insertion length calculation is very simple for
 the given symbols: simply using bits 1 through 4 of the basis stream.
 */

class ZTF_ExpansionDecoder final: public PabloKernel {
public:
    ZTF_ExpansionDecoder(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const basis, StreamSet * insertBixNum)
    : PabloKernel(b, "ZTF_ExpansionDecoder", {Binding{"basis", basis, FixedRate(), LookAhead(1)}}, {Binding{"insertBixNum", insertBixNum}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void ZTF_ExpansionDecoder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    std::unique_ptr<cc::CC_Compiler> ccc;
    ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), basis);
    PabloAST * ASCII_lookahead = pb.createNot(pb.createLookahead(basis[7], 1));
    PabloAST * const ZTF_Sym = pb.createAnd(ccc->compileCC(re::makeByte(0xC2, 0xDF)), ASCII_lookahead, "ZTF_sym");
    Var * lengthVar = getOutputStreamVar("insertBixNum");
    for (unsigned i = 0; i < 4; i++) {
        pb.createAssign(pb.createExtract(lengthVar, pb.getInteger(i)), pb.createAnd(ZTF_Sym, basis[i+1]));
    }
}


class LengthGroups final: public PabloKernel {
public:
    LengthGroups(const std::unique_ptr<kernel::KernelBuilder> & b,
                 StreamSet * symbolRun, StreamSet * const lengthBixNum,
                 StreamSet * overflow,
                 StreamSet * lengthGroup3_4,
                 StreamSet * lengthGroup5_8,
                 StreamSet * lengthGroup9_16)
    : PabloKernel(b, "LengthGroups",
                     {Binding{"symbolRun", symbolRun, FixedRate(), LookAhead(1)},
                      Binding{"lengthBixNum", lengthBixNum},
                      Binding{"overflow", overflow}},
                     {Binding{"lengthGroup3_4", lengthGroup3_4}, Binding{"lengthGroup5_8", lengthGroup5_8}, Binding{"lengthGroup9_16", lengthGroup9_16}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

void LengthGroups::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * run = getInputStreamSet("symbolRun")[0];
    std::vector<PabloAST *> lengthBixNum = getInputStreamSet("lengthBixNum");
    PabloAST * overflow = getInputStreamSet("overflow")[0];
    PabloAST * runFinal = pb.createAnd(run, pb.createNot(pb.createLookahead(run, 1)));
    runFinal = pb.createAnd(runFinal, pb.createNot(overflow));
    // Run index codes count from 0 on the 2nd byte of a symbol.
    // So the length is 2 more than the bixnum.
    PabloAST * group3_4 = pb.createAnd3(bnc.UGE(lengthBixNum, 1), bnc.ULE(lengthBixNum, 2), runFinal, "group3_4");
    PabloAST * group5_8 = pb.createAnd3(bnc.UGE(lengthBixNum, 3), bnc.ULE(lengthBixNum, 6), runFinal, "group5_8");
    PabloAST * group9_16 = pb.createAnd3(bnc.UGE(lengthBixNum, 7), bnc.ULE(lengthBixNum, 14), runFinal, "group9_16");
    pb.createAssign(pb.createExtract(getOutputStreamVar("lengthGroup3_4"), pb.getInteger(0)), group3_4);
    pb.createAssign(pb.createExtract(getOutputStreamVar("lengthGroup5_8"), pb.getInteger(0)), group5_8);
    pb.createAssign(pb.createExtract(getOutputStreamVar("lengthGroup9_16"), pb.getInteger(0)), group9_16);
}

typedef void (*ztfHashFunctionType)(uint32_t fd);

ztfHashFunctionType ztfHash_compression_gen (CPUDriver & driver) {

    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});

    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const codeUnitStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, codeUnitStream);

    StreamSet * const u8basis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(codeUnitStream, u8basis);
    
    StreamSet * WordChars = P->CreateStreamSet(1);
    P->CreateKernelCall<WordMarkKernel>(u8basis, WordChars);
    StreamSet * const symbolRuns = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_Symbols>(u8basis, WordChars, symbolRuns);

    StreamSet * const runIndex = P->CreateStreamSet(4);
    StreamSet * const overflow = P->CreateStreamSet(1);
    P->CreateKernelCall<RunIndex>(symbolRuns, runIndex, overflow);

    StreamSet * const bixHashes = P->CreateStreamSet(8);
    P->CreateKernelCall<BixHash>(u8basis, symbolRuns, bixHashes);
    
    StreamSet * const hashBytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(bixHashes, hashBytes);
    
    StreamSet * const lgthBytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(runIndex, lgthBytes);
    
    StreamSet * const lengthGroup3_4 = P->CreateStreamSet(1);
    StreamSet * const lengthGroup5_8 = P->CreateStreamSet(1);
    StreamSet * const lengthGroup9_16 = P->CreateStreamSet(1);
    P->CreateKernelCall<LengthGroups>(symbolRuns, runIndex, overflow, lengthGroup3_4, lengthGroup5_8, lengthGroup9_16);
    
    StreamSet * const extractionMask3_4 = P->CreateStreamSet(1);
    StreamSet * const extractionMask5_8 = P->CreateStreamSet(1);
    StreamSet * const extractionMask9_16 = P->CreateStreamSet(1);
    P->CreateKernelCall<LengthGroupCompressionMask>(LengthGroup{3, 4}, lengthGroup3_4, lgthBytes, codeUnitStream, hashBytes, extractionMask3_4);
    P->CreateKernelCall<LengthGroupCompressionMask>(LengthGroup{5, 8}, lengthGroup5_8, lgthBytes, codeUnitStream, hashBytes, extractionMask5_8);
    P->CreateKernelCall<LengthGroupCompressionMask>(LengthGroup{9, 16}, lengthGroup9_16, lgthBytes, codeUnitStream, hashBytes, extractionMask9_16);
    StreamSet * const combinedMask = P->CreateStreamSet(1);
    P->CreateKernelCall<StreamsIntersect>(std::vector<StreamSet *>{extractionMask3_4, extractionMask5_8, extractionMask9_16}, combinedMask);
    StreamSet * const encoded = P->CreateStreamSet(8);
    P->CreateKernelCall<ZTF_SymbolEncoder>(u8basis, bixHashes, combinedMask, runIndex, encoded);
    
    StreamSet * const ZTF_basis = P->CreateStreamSet(8);
    FilterByMask(P, combinedMask, encoded, ZTF_basis);

    StreamSet * const ZTF_bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ZTF_basis, ZTF_bytes);
    P->CreateKernelCall<StdOutKernel>(ZTF_bytes);
    return reinterpret_cast<ztfHashFunctionType>(P->compile());
}

ztfHashFunctionType ztfHash_decompression_gen (CPUDriver & driver) {
    auto & b = driver.getBuilder();
    Type * const int32Ty = b->getInt32Ty();
    auto P = driver.makePipeline({Binding{int32Ty, "fd"}});
    Scalar * const fileDescriptor = P->getInputScalar("fd");

    // Source data
    StreamSet * const source = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, source);
    StreamSet * const ztfHashBasis = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(source, ztfHashBasis);
    StreamSet * const ztfInsertionLengths = P->CreateStreamSet(4);
    P->CreateKernelCall<ZTF_ExpansionDecoder>(ztfHashBasis, ztfInsertionLengths);
    StreamSet * const ztfRunSpreadMask = InsertionSpreadMask(P, ztfInsertionLengths);

    StreamSet * const ztfHash_u8_Basis = P->CreateStreamSet(8);
    SpreadByMask(P, ztfRunSpreadMask, ztfHashBasis, ztfHash_u8_Basis);

    StreamSet * const group3_4marks = P->CreateStreamSet(1);
    StreamSet * const group5_8marks = P->CreateStreamSet(1);
    StreamSet * const group9_16marks = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_HashMarks>(ztfHash_u8_Basis, group3_4marks, group5_8marks, group9_16marks);

    StreamSet * WordChars = P->CreateStreamSet(1);
    P->CreateKernelCall<WordMarkKernel>(ztfHash_u8_Basis, WordChars);

    StreamSet * const ztfHash_u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ztfHash_u8_Basis, ztfHash_u8bytes);
    
    StreamSet * const symbolRuns = P->CreateStreamSet(1);
    P->CreateKernelCall<ZTF_Symbols>(ztfHash_u8_Basis, WordChars, symbolRuns);

    StreamSet * const runIndex = P->CreateStreamSet(4);
    StreamSet * const overflow = P->CreateStreamSet(1);
    P->CreateKernelCall<RunIndex>(symbolRuns, runIndex, overflow);

    StreamSet * const bixHashes = P->CreateStreamSet(8);
    P->CreateKernelCall<BixHash>(ztfHash_u8_Basis, symbolRuns, bixHashes);
    
    StreamSet * const hashBytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(bixHashes, hashBytes);
    
    StreamSet * const lengthGroup3_4 = P->CreateStreamSet(1);
    StreamSet * const lengthGroup5_8 = P->CreateStreamSet(1);
    StreamSet * const lengthGroup9_16 = P->CreateStreamSet(1);
    P->CreateKernelCall<LengthGroups>(symbolRuns, runIndex, overflow, lengthGroup3_4, lengthGroup5_8, lengthGroup9_16);
    //P->CreateKernelCall<DebugDisplayKernel>("lengthGroup3_4", lengthGroup3_4);
    //P->CreateKernelCall<DebugDisplayKernel>("lengthGroup5_8", lengthGroup5_8);
    //P->CreateKernelCall<DebugDisplayKernel>("lengthGroup9_16", lengthGroup9_16);

    StreamSet * const lgthBytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(runIndex, lgthBytes);
    
    StreamSet * const u8bytes_34 = P->CreateStreamSet(1, 8);
    StreamSet * const u8bytes_58 = P->CreateStreamSet(1, 8);
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);
    // P->CreateKernelCall<DebugDisplayKernel>("ztfHash_u8bytes", ztfHash_u8bytes);
    P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{3, 4}, lengthGroup3_4, lgthBytes, group3_4marks, ztfHash_u8bytes, hashBytes, u8bytes_34);
    //P->CreateKernelCall<DebugDisplayKernel>("u8bytes_34", u8bytes_34);
    P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{5, 8}, lengthGroup5_8, lgthBytes, group5_8marks, u8bytes_34, hashBytes, u8bytes_58);
    // P->CreateKernelCall<DebugDisplayKernel>("u8bytes_58", u8bytes_58);
    P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{9, 16}, lengthGroup9_16, lgthBytes, group9_16marks, u8bytes_58, hashBytes, u8bytes);
    //P->CreateKernelCall<DebugDisplayKernel>("u8bytes", u8bytes);
    //P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{5, 8}, lengthGroup5_8, lgthBytes, hashMarks, ztfHash_u8bytes, hashBytes, u8bytes);

    P->CreateKernelCall<StdOutKernel>(u8bytes);
    return reinterpret_cast<ztfHashFunctionType>(P->compile());
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&ztfHashOptions, pablo_toolchain_flags(), codegen::codegen_flags()});
    
    CPUDriver pxDriver("ztfHash");
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        if (Decompression) {
            auto ztfHashDecompressionFunction = ztfHash_decompression_gen(pxDriver);
            ztfHashDecompressionFunction(fd);
        } else {
            auto ztfHashCompressionFunction = ztfHash_compression_gen(pxDriver);
            ztfHashCompressionFunction(fd);
        }
        close(fd);
    }
    return 0;
}
