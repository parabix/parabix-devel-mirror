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

using namespace pablo;
using namespace parse;
using namespace kernel;
using namespace llvm;
using namespace codegen;

static cl::OptionCategory ztfHashOptions("ztfHash Options", "ZTF-Hash options.");
static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(ztfHashOptions));
static cl::opt<bool> Decompression("d", cl::desc("Decompress from ZTF-Runs to UTF-8."), cl::cat(ztfHashOptions), cl::init(false));
static cl::alias DecompressionAlias("decompress", cl::desc("Alias for -d"), cl::aliasopt(Decompression));

static cl::opt<bool> DeferredAttribute("deferred", cl::desc("Use Deferred Attribute for decompression"), cl::cat(ztfHashOptions), cl::init(false));
static cl::opt<bool> DelayedAttribute("delayed", cl::desc("Use Delayed Attribute for decompression"), cl::cat(ztfHashOptions), cl::init(false));


class WordMarkKernel : public PabloKernel {
public:
    WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

WordMarkKernel::WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks)
: PabloKernel(kb, "WordMarks", {Binding{"source", BasisBits}}, {Binding{"WordMarks", WordMarks}}) { }

void WordMarkKernel::generatePabloMethod() {
    pablo::PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("source"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::Name * word = re::makeName("word", re::Name::Type::UnicodeProperty);
    word = cast<re::Name>(re::resolveUnicodeNames(word));
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(word, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(word);
    if (f == nameMap.end()) llvm::report_fatal_error("Cannot find word property");
    PabloAST * wordChar = f->second;
    pb.createAssign(pb.createExtract(getOutputStreamVar("WordMarks"), pb.getInteger(0)), wordChar);
}

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

const unsigned BITS_PER_BYTE = 8;
const unsigned SIZE_T_BITS = sizeof(size_t) * BITS_PER_BYTE;

struct ScanWordParameters {
    unsigned width;
    unsigned indexWidth;
    Type * const Ty;
    Type * const pointerTy;
    Constant * const WIDTH;
    Constant * const ix_MAXBIT;
    Constant * WORDS_PER_BLOCK;
    Constant * WORDS_PER_STRIDE;
    
    ScanWordParameters(const std::unique_ptr<KernelBuilder> & b, unsigned stride) :
#ifdef PREFER_NARROW_SCANWIDTH
    width(std::max(BITS_PER_BYTE, stride/SIZE_T_BITS)),
#else
    width(std::min(SIZE_T_BITS, stride/BITS_PER_BYTE)),
#endif
    indexWidth(stride/width),
    Ty(b->getIntNTy(width)),
    pointerTy(Ty->getPointerTo()),
    WIDTH(b->getSize(width)),
    ix_MAXBIT(b->getSize(indexWidth - 1)),
    WORDS_PER_BLOCK(b->getSize(b->getBitBlockWidth()/width)),
    WORDS_PER_STRIDE(b->getSize(indexWidth))
    {   //  The stride must be a power of 2 and a multiple of the BitBlock width.
        assert((((stride & (stride - 1)) == 0) && (stride >= b->getBitBlockWidth()) && (stride <= SIZE_T_BITS * SIZE_T_BITS)));
    }
};

struct LengthGroup {unsigned lo; unsigned hi;};

void checkLengthGroup(LengthGroup g) {
    assert((g.hi & (g.hi - 1)) == 0);
    assert((g.lo <= g.hi) && (g.lo > g.hi/2));
}

const unsigned HashTableEntries = 256;

unsigned hashTableSize(LengthGroup g) {
    unsigned numSubTables = (g.hi - g.lo + 1);
    return numSubTables * g.hi * HashTableEntries;
}

std::string lengthGroupStr(LengthGroup lengthGroup) {
    return std::to_string(lengthGroup.lo) + "-" + std::to_string(lengthGroup.hi);
}

Binding ByteDataBinding(LengthGroup lengthGroup, StreamSet * byteData) {
    if (DeferredAttribute) {
        return Binding{"byteData", byteData, FixedRate(), {Deferred()}};
    } else {
        return Binding{"byteData", byteData, FixedRate(), LookBehind(lengthGroup.hi)};
    }
}

class LengthGroupCompressionMask : public MultiBlockKernel {
public:
    LengthGroupCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                               LengthGroup lengthGroup,
                               StreamSet * symbolMarks,
                               StreamSet * symbolLengths,
                        StreamSet * const byteData, StreamSet * const hashValues, StreamSet * compressionMask, unsigned strideBlocks = 8)
    : MultiBlockKernel(b, "LengthGroupCompressionMask" + lengthGroupStr(lengthGroup) + (DeferredAttribute ? "deferred" : "lookBehind"),
                              {Binding{"symbolMarks", symbolMarks},
                                  Binding{"symbolLengths", symbolLengths},
                                  ByteDataBinding(lengthGroup, byteData),
                                  Binding{"hashValues", hashValues}},
                              {Binding{"compressionMask", compressionMask, BoundedRate(0,1)}}, {}, {},
                              {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                                  InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(lengthGroup)), "hashTable"}}),
    mLengthGroup(lengthGroup) {
        checkLengthGroup(lengthGroup);
        setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    LengthGroup mLengthGroup;
};

void LengthGroupCompressionMask::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_MAXBIT = b->getSize(SIZE_T_BITS - 1);
    Constant * sz_BLOCKWIDTH = b->getSize(b->getBitBlockWidth());
    Type * sizeTy = b->getSizeTy();
    Type * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();

    Type * halfLengthTy = b->getIntNTy(8 * mLengthGroup.hi/2);
    Type * halfSymPtrTy = halfLengthTy->getPointerTo();
    Constant * sz_HALF_SYM = b->getSize(mLengthGroup.hi/2);
    Constant * sz_MINLENGTH = b->getSize(mLengthGroup.lo);
    Constant * sz_MAXLENGTH = b->getSize(mLengthGroup.hi);
    Constant * sz_SUBTABLE = b->getSize(HashTableEntries * mLengthGroup.hi);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const stridePrecomputation = b->CreateBasicBlock("stridePrecomputation");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
    BasicBlock * const checkKey = b->CreateBasicBlock("checkKey");
    BasicBlock * const markCompression = b->CreateBasicBlock("markCompression");
    BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
    BasicBlock * const keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const updatePending = b->CreateBasicBlock("updatePending");
    BasicBlock * const compressionMaskDone = b->CreateBasicBlock("compressionMaskDone");

    Value * const initialPos = b->getProcessedItemCount("symbolMarks");
    Value * const avail = b->getAvailableItemCount("symbolMarks");
    Value * const initialProduced = b->getProducedItemCount("compressionMask");
    Value * pendingMask = b->CreateNot(b->getScalarField("pendingMaskInverted"));
    Value * producedPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialProduced), bitBlockPtrTy);
    //b->CallPrintInt("producedPtr", producedPtr);
    b->CreateStore(pendingMask, producedPtr);
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    Value * compressMaskPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", stridePos), bitBlockPtrTy);
    b->CreateBr(stridePrecomputation);

    // Precompute an index mask for one stride of the symbol marks stream.
    b->SetInsertPoint(stridePrecomputation);
    PHINode * const keyMaskAccum = b->CreatePHI(sizeTy, 2);
    keyMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, stridePrologue);
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    Value * keyBitBlock = b->loadInputStreamBlock("symbolMarks", sz_ZERO, strideBlockIndex);
    Value * const anyKey = b->simd_any(sw.width, keyBitBlock);
    Value * keyWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyKey), sizeTy);
    Value * keyMask = b->CreateOr(keyMaskAccum, b->CreateShl(keyWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "keyMask");
    // Initialize the compression mask.
    b->CreateBlockAlignedStore(b->allOnes(), b->CreateGEP(compressMaskPtr, strideBlockIndex));
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    keyMaskAccum->addIncoming(keyMask, stridePrecomputation);
    blockNo->addIncoming(nextBlockNo, stridePrecomputation);
    // Default initial compression mask is all ones (no zeroes => no compression).
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), stridePrecomputation, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    // Iterate through key symbols and update the hash table as appropriate.
    // As symbols are encountered, the hash value is retrieved from the
    // hashValues stream.   There are then three possibilities:
    //   1.  The hashTable has no entry for this hash value.
    //       In this case, the current symbol is copied into the table.
    //   2.  The hashTable has an entry for this hash value, and
    //       that entry is equal to the current symbol.    Mark the
    //       symbol for compression.
    //   3.  The hashTable has an entry for this hash value, but
    //       that entry is not equal to the current symbol.    Skip the
    //       symbol.
    //
    Value * keyWordBasePtr = b->getInputStreamBlockPtr("symbolMarks", sz_ZERO, strideBlockOffset);
    keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(keyMask, sz_ZERO), keysDone, keyProcessingLoop);

    b->SetInsertPoint(keyProcessingLoop);
    PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
    keyMaskPhi->addIncoming(keyMask, strideMasksReady);
    PHINode * const keyWordPhi = b->CreatePHI(sizeTy, 2);
    keyWordPhi->addIncoming(sz_ZERO, strideMasksReady);
    Value * keyWordIdx = b->CreateCountForwardZeroes(keyMaskPhi, "keyWordIdx");
    Value * nextKeyWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keyWordBasePtr, keyWordIdx)), sizeTy);
    Value * theKeyWord = b->CreateSelect(b->CreateICmpEQ(keyWordPhi, sz_ZERO), nextKeyWord, keyWordPhi);
    Value * keyWordPos = b->CreateAdd(stridePos, b->CreateMul(keyWordIdx, sw.WIDTH));
    Value * keyMarkPosInWord = b->CreateCountForwardZeroes(theKeyWord);
    Value * keyMarkPos = b->CreateAdd(keyWordPos, keyMarkPosInWord, "keyEndPos");
    //b->CallPrintInt("keyMarkPos", keyMarkPos);
    /* Determine the key length. */
    Value * const lgthPtr = b->getRawInputPointer("symbolLengths", keyMarkPos);
    Value * keyLength = b->CreateAdd(b->CreateZExt(b->CreateLoad(lgthPtr), sizeTy), sz_TWO);
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, sz_ONE));
    //b->CallPrintInt("keyLength", keyLength);
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, sz_HALF_SYM);
    //b->CallPrintInt("keyOffset", keyOffset);
    // Get the hash of this key.
    Value * const keyPtr = b->getRawInputPointer("hashValues", keyMarkPos);
    Value * keyHash = b->CreateZExt(b->CreateLoad(keyPtr), sizeTy, "keyHash");
    //b->CallPrintInt("keyHash", keyHash);
    // Starting with length 9, the length-based subtables are 16 * 256 = 4K each.
    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    //b->CallPrintInt("hashTableBasePtr", hashTableBasePtr);
    Value * hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength, sz_MINLENGTH), sz_SUBTABLE));
    //b->CallPrintInt("hashTablePtr", hashTablePtr);
    Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, sz_MAXLENGTH));
    // Use two 8-byte loads to get hash and symbol values.
    //b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    Value * tblPtr1 = b->CreateBitCast(tblEntryPtr, halfSymPtrTy);
    //b->CallPrintInt("tblPtr1", tblPtr1);
    Value * tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, keyOffset), halfSymPtrTy);
    //b->CallPrintInt("tblPtr2", tblPtr2);
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", b->getInt32(0), keyStartPos), halfSymPtrTy);
    //b->CallPrintInt("symPtr1", symPtr1);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->getInt32(0), b->CreateAdd(keyStartPos, keyOffset)), halfSymPtrTy);
    //b->CallPrintInt("symPtr2", symPtr2);
    // Check to see if the hash table entry is nonzero (already assigned).
    Value * sym1 = b->CreateLoad(symPtr1);
    //b->CallPrintInt("sym1", sym1);
    Value * sym2 = b->CreateLoad(symPtr2);
    //b->CallPrintInt("sym2", sym2);
    Value * entry1 = b->CreateLoad(tblPtr1);
    //b->CallPrintInt("entry1", entry1);
    Value * entry2 = b->CreateLoad(tblPtr2);
    //b->CallPrintInt("entry2", entry2);
    Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1, entry2), Constant::getNullValue(halfLengthTy));
    b->CreateCondBr(isEmptyEntry, storeKey, checkKey);
    b->SetInsertPoint(storeKey);
    // We have a new symbol that allows future occurrences of the symbol to
    // be compressed using the hash code.
    b->CreateStore(sym1, tblPtr1);
    b->CreateStore(sym2, tblPtr2);
    //b->CreateWriteCall(b->getInt32(STDERR_FILENO), symPtr1, keyLength);
    //b->CallPrintInt("keyHash", keyHash);
    //b->CallPrintInt("keyLength", keyLength);

    b->CreateBr(nextKey);

    b->SetInsertPoint(checkKey);
    // If the symbol is equal to the stored entry, it will be replace
    // by the ZTF compression code.  Prepare the compression mask by
    // zeroing out all symbol positions except the last two.
    Value * symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(entry1, sym1), b->CreateICmpEQ(entry2, sym2));
    //b->CallPrintInt("symIsEqEntry", symIsEqEntry);
    b->CreateCondBr(symIsEqEntry, markCompression, nextKey);

    b->SetInsertPoint(markCompression);
    // Compute a mask of bits to zero out.
    Value * maskLength = b->CreateZExt(b->CreateSub(keyLength, sz_TWO), sizeTy);
    Value * mask = b->CreateSub(b->CreateShl(sz_ONE, maskLength), sz_ONE);
    // Determine a base position from which both the keyStart and the keyEnd
    // are accessible within SIZE_T_BITS - 8, and which will not overflow
    // the buffer.
    Value * startBase = b->CreateSub(keyStartPos, b->CreateURem(keyStartPos, b->getSize(8)));
    Value * markBase = b->CreateSub(keyMarkPos, b->CreateURem(keyMarkPos, sz_BITS));
    Value * keyBase = b->CreateSelect(b->CreateICmpULT(startBase, markBase), startBase, markBase);
    Value * bitOffset = b->CreateSub(keyStartPos, keyBase);
    //b->CallPrintInt("bitOffset", bitOffset);
    mask = b->CreateShl(mask, bitOffset);
    //b->CallPrintInt("mask", mask);
    Value * const keyBasePtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", keyBase), sizeTy->getPointerTo());

    Value * initialMask = b->CreateLoad(keyBasePtr);
    //b->CallPrintInt("initialMask", initialMask);
    Value * updated = b->CreateAnd(initialMask, b->CreateNot(mask));
    //b->CallPrintInt("updated", updated);
    b->CreateStore(b->CreateAnd(updated, b->CreateNot(mask)), keyBasePtr);
    b->CreateBr(nextKey);

    b->SetInsertPoint(nextKey);
    Value * dropKey = b->CreateResetLowestBit(theKeyWord);
    Value * thisWordDone = b->CreateICmpEQ(dropKey, sz_ZERO);
    // There may be more keys in the key mask.
    Value * nextKeyMask = b->CreateSelect(thisWordDone, b->CreateResetLowestBit(keyMaskPhi), keyMaskPhi);
    BasicBlock * currentBB = b->GetInsertBlock();
    keyMaskPhi->addIncoming(nextKeyMask, currentBB);
    keyWordPhi->addIncoming(dropKey, currentBB);
    b->CreateCondBr(b->CreateICmpNE(nextKeyMask, sz_ZERO), keyProcessingLoop, keysDone);

    b->SetInsertPoint(keysDone);
    strideNo->addIncoming(nextStrideNo, keysDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // In the next segment, we may need to access byte data in the last
    // 16 bytes of this segment.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(mIsFinal, avail, b->CreateSub(avail, sz_MAXLENGTH));
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the last block mask, we do not include it as
    // produced, because we may need to update it in the event that there is
    // a compressible symbol starting in this segment and finishing in the next.
    Value * produced = b->CreateSelect(mIsFinal, avail, b->CreateSub(avail, sz_BLOCKWIDTH));
    b->setProducedItemCount("compressionMask", produced);
    b->CreateCondBr(mIsFinal, compressionMaskDone, updatePending);
    b->SetInsertPoint(updatePending);
    Value * pendingPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", produced), bitBlockPtrTy);
    //b->CallPrintInt("pendingPtr", pendingPtr);
    Value * lastMask = b->CreateBlockAlignedLoad(pendingPtr);
    b->setScalarField("pendingMaskInverted", b->CreateNot(lastMask));
    b->CreateBr(compressionMaskDone);
    b->SetInsertPoint(compressionMaskDone);
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

class LengthGroupDecompression : public MultiBlockKernel {
public:
    LengthGroupDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                             LengthGroup lengthGroup,
                             StreamSet * keyMarks,
                             StreamSet * symbolLengths,
                             StreamSet * const hashMarks, StreamSet * const byteData,
                             StreamSet * const hashValues,
                             StreamSet * const result, unsigned strideBlocks = 8)
    : MultiBlockKernel(b, "LengthGroupDecompression" + lengthGroupStr(lengthGroup) + (DeferredAttribute ? "deferred" : "lookBehind"),
                        {Binding{"keyMarks", keyMarks},
                            Binding{"symbolLengths", symbolLengths},
                            Binding{"hashMarks", hashMarks},
                            ByteDataBinding(lengthGroup, byteData),
                            Binding{"hashValues", hashValues},
                        },
                       {}, {}, {},
                       {InternalScalar{ArrayType::get(b->getInt8Ty(), lengthGroup.hi), "pendingOutput"},
                           // Hash table 8 length-based tables with 256 16-byte entries each.
                           InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(lengthGroup)), "hashTable"}}),
    mLengthGroup(lengthGroup) {
        checkLengthGroup(lengthGroup);
        setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
        if (DelayedAttribute) {
            mOutputStreamSets.emplace_back("result", result, FixedRate(), Delayed(mLengthGroup.hi) );
        } else {
            mOutputStreamSets.emplace_back("result", result, BoundedRate(0,1));
        }

    }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const numOfStrides) override;
    LengthGroup mLengthGroup;
};

void LengthGroupDecompression::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    ScanWordParameters sw(b, mStride);

    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_MAXBIT = b->getSize(SIZE_T_BITS - 1);
    Type * sizeTy = b->getSizeTy();

    Type * halfLengthTy = b->getIntNTy(8 * mLengthGroup.hi/2);
    Type * halfSymPtrTy = halfLengthTy->getPointerTo();
    Constant * sz_HALF_SYM = b->getSize(mLengthGroup.hi/2);
    Constant * sz_MINLENGTH = b->getSize(mLengthGroup.lo);
    Constant * sz_MAXLENGTH = b->getSize(mLengthGroup.hi);
    Constant * sz_SUBTABLE = b->getSize(HashTableEntries * mLengthGroup.hi);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const stridePrecomputation = b->CreateBasicBlock("stridePrecomputation");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
    BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
    BasicBlock * const keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const hashProcessingLoop = b->CreateBasicBlock("hashProcessingLoop");
    BasicBlock * const hashesDone = b->CreateBasicBlock("hashesDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");

    Value * const initialPos = b->getProcessedItemCount("keyMarks");
    Value * const initialProduced = b->getProducedItemCount("result");
    Value * const avail = b->getAvailableItemCount("keyMarks");
    // Copy pending output data.
    b->CreateMemCpy(b->getRawOutputPointer("result", initialProduced), b->getScalarFieldPtr("pendingOutput"), sz_MAXLENGTH, 1);
    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateSub(avail, initialPos);
    b->CreateMemCpy(b->getRawOutputPointer("result", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);
    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    //b->CallPrintInt("hashTableBasePtr", hashTableBasePtr);
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    b->CreateBr(stridePrecomputation);
    // Precompute index masks for one stride of the key result and line hash streams,
    // as well as a partial sum popcount of line numbers if line numbering is on.
    b->SetInsertPoint(stridePrecomputation);
    PHINode * const keyMaskAccum = b->CreatePHI(sizeTy, 2);
    keyMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const hashMaskAccum = b->CreatePHI(sizeTy, 2);
    hashMaskAccum->addIncoming(sz_ZERO, stridePrologue);
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, stridePrologue);
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    Value * keyBitBlock = b->loadInputStreamBlock("keyMarks", sz_ZERO, strideBlockIndex);
    Value * hashBitBlock = b->loadInputStreamBlock("hashMarks", sz_ZERO, strideBlockIndex);
    Value * const anyKey = b->simd_any(sw.width, keyBitBlock);
    Value * const anyHash = b->simd_any(sw.width, hashBitBlock);
    Value * keyWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyKey), sizeTy);
    Value * hashWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyHash), sizeTy);
    Value * keyMask = b->CreateOr(keyMaskAccum, b->CreateShl(keyWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "keyMask");
    Value * hashMask = b->CreateOr(hashMaskAccum, b->CreateShl(hashWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)), "hashMask");
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    keyMaskAccum->addIncoming(keyMask, stridePrecomputation);
    hashMaskAccum->addIncoming(hashMask, stridePrecomputation);
    blockNo->addIncoming(nextBlockNo, stridePrecomputation);
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), stridePrecomputation, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    // First iterate through the new keys and update the hash table as
    // appropriate.   Each key is hashed, and is entered into the hash
    // table if there is not already an entry for that hash code.
    Value * keyWordBasePtr = b->getInputStreamBlockPtr("keyMarks", sz_ZERO, strideBlockOffset);
    keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(keyMask, sz_ZERO), keysDone, keyProcessingLoop);

    b->SetInsertPoint(keyProcessingLoop);
    PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
    keyMaskPhi->addIncoming(keyMask, strideMasksReady);
    PHINode * const keyWordPhi = b->CreatePHI(sizeTy, 2);
    keyWordPhi->addIncoming(sz_ZERO, strideMasksReady);
    Value * keyWordIdx = b->CreateCountForwardZeroes(keyMaskPhi, "keyWordIdx");
    Value * nextKeyWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keyWordBasePtr, keyWordIdx)), sizeTy);
    Value * theKeyWord = b->CreateSelect(b->CreateICmpEQ(keyWordPhi, sz_ZERO), nextKeyWord, keyWordPhi);
    Value * keyWordPos = b->CreateAdd(stridePos, b->CreateMul(keyWordIdx, sw.WIDTH));
    Value * keyMarkPosInWord = b->CreateCountForwardZeroes(theKeyWord);
    Value * keyMarkPos = b->CreateAdd(keyWordPos, keyMarkPosInWord, "keyEndPos");
    //b->CallPrintInt("keyMarkPos", keyMarkPos);
    /* Determine the key length. */
    Value * const lgthPtr = b->getRawInputPointer("symbolLengths", keyMarkPos);
    Value * keyLength = b->CreateAdd(b->CreateZExt(b->CreateLoad(lgthPtr), sizeTy), sz_TWO);
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, sz_ONE));
    //b->CallPrintInt("keyLength", keyLength);
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, sz_HALF_SYM);
    //b->CallPrintInt("keyOffset", keyOffset);
    // Get the hash of this key.
    Value * const keyPtr = b->getRawInputPointer("hashValues", keyMarkPos);
    Value * keyHash = b->CreateZExt(b->CreateLoad(keyPtr), sizeTy);
    //b->CallPrintInt("keyHash", keyHash);
    Value * hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength, sz_MINLENGTH), sz_SUBTABLE));
    Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, sz_MAXLENGTH));
    // Use two 8-byte loads to get hash and symbol values.
    //b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    Value * tblPtr1 = b->CreateBitCast(tblEntryPtr, halfSymPtrTy);
    //b->CallPrintInt("tblPtr1", tblPtr1);
    Value * tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, keyOffset), halfSymPtrTy);
    //b->CallPrintInt("tblPtr2", tblPtr2);
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", b->getInt32(0), keyStartPos), halfSymPtrTy);
    //b->CallPrintInt("symPtr1", symPtr1);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->getInt32(0), b->CreateAdd(keyStartPos, keyOffset)), halfSymPtrTy);
    //b->CallPrintInt("symPtr2", symPtr2);
    // Check to see if the hash table entry is nonzero (already assigned).
    Value * sym1 = b->CreateLoad(symPtr1);
    //b->CallPrintInt("sym1", sym1);
    Value * sym2 = b->CreateLoad(symPtr2);
    //b->CallPrintInt("sym2", sym2);
    Value * entry1 = b->CreateLoad(tblPtr1);
    //b->CallPrintInt("entry1", entry1);
    Value * entry2 = b->CreateLoad(tblPtr2);
    //b->CallPrintInt("entry2", entry2);
    Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1, entry2), Constant::getNullValue(halfLengthTy));

    b->CreateCondBr(isEmptyEntry, storeKey, nextKey);
    b->SetInsertPoint(storeKey);
    // We have a new symbols that allows future occurrences of the symbol to
    // be compressed using the hash code.
    //b->CreateWriteCall(b->getInt32(STDERR_FILENO), symPtr1, keyLength);
    //b->CallPrintInt("keyHash", keyHash);
    //b->CallPrintInt("keyLength", keyLength);
    b->CreateStore(sym1, tblPtr1);
    b->CreateStore(sym2, tblPtr2);
    b->CreateBr(nextKey);

    b->SetInsertPoint(nextKey);
    Value * dropKey = b->CreateResetLowestBit(theKeyWord);
    Value * thisWordDone = b->CreateICmpEQ(dropKey, sz_ZERO);
    // There may be more keys in the key mask.
    Value * nextKeyMask = b->CreateSelect(thisWordDone, b->CreateResetLowestBit(keyMaskPhi), keyMaskPhi);
    BasicBlock * currentBB = b->GetInsertBlock();
    keyMaskPhi->addIncoming(nextKeyMask, currentBB);
    keyWordPhi->addIncoming(dropKey, currentBB);
    b->CreateCondBr(b->CreateICmpNE(nextKeyMask, sz_ZERO), keyProcessingLoop, keysDone);

    b->SetInsertPoint(keysDone);
    Value * hashWordBasePtr = b->getInputStreamBlockPtr("hashMarks", sz_ZERO, strideBlockOffset);
    hashWordBasePtr = b->CreateBitCast(hashWordBasePtr, sw.pointerTy);
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(hashMask, sz_ZERO), hashesDone, hashProcessingLoop);


    b->SetInsertPoint(hashProcessingLoop);
    PHINode * const hashMaskPhi = b->CreatePHI(sizeTy, 2);
    hashMaskPhi->addIncoming(hashMask, keysDone);
    PHINode * const hashWordPhi = b->CreatePHI(sizeTy, 2);
    hashWordPhi->addIncoming(sz_ZERO, keysDone);
    Value * hashWordIdx = b->CreateCountForwardZeroes(hashMaskPhi, "hashWordIdx");
    Value * nextHashWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(hashWordBasePtr, hashWordIdx)), sizeTy);
    Value * theHashWord = b->CreateSelect(b->CreateICmpEQ(hashWordPhi, sz_ZERO), nextHashWord, hashWordPhi);
    Value * hashWordPos = b->CreateAdd(stridePos, b->CreateMul(hashWordIdx, sw.WIDTH));
    Value * hashPosInWord = b->CreateCountForwardZeroes(theHashWord);
    Value * hashMarkPos = b->CreateAdd(hashWordPos, hashPosInWord, "hashMarkPos");
    Value * hashPfxPos = b->CreateSub(hashMarkPos, b->getSize(1));

    Value * const hashPfx = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", hashPfxPos)), sizeTy);
    //b->CallPrintInt("hashPfx", hashPfx);
    Value * const hashSfx = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", hashMarkPos)), sizeTy);
    //b->CallPrintInt("hashSfx", hashSfx);
    Value * symLength = b->CreateAdd(b->CreateURem(b->CreateUDiv(hashPfx, sz_TWO), sz_MAXLENGTH), sz_TWO);
    //b->CallPrintInt("symLength", symLength);
    Value * hashCode = b->CreateAdd(b->CreateMul(b->CreateURem(hashPfx, sz_TWO), b->getSize(128)), hashSfx, "hashCode");
    //b->CallPrintInt("hashCode", hashCode);
    Value * symStartPos = b->CreateSub(hashMarkPos, b->CreateSub(symLength, sz_ONE), "symStartPos");
    Value * symOffset = b->CreateSub(symLength, sz_HALF_SYM);

    hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(symLength, sz_MINLENGTH), sz_SUBTABLE));
    //b->CallPrintInt("hashTablePtr", hashTablePtr);
    tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(hashCode, sz_MAXLENGTH));
    // Use two 8-byte loads to get hash and symbol values.
    //b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    tblPtr1 = b->CreateBitCast(tblEntryPtr, halfSymPtrTy);
    //b->CallPrintInt("tblPtr1", tblPtr1);
    tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, symOffset), halfSymPtrTy);
    //b->CallPrintInt("tblPtr2", tblPtr2);
    entry1 = b->CreateLoad(tblPtr1);
    //b->CallPrintInt("entry1", entry1);
    entry2 = b->CreateLoad(tblPtr2);

    symPtr1 = b->CreateBitCast(b->getRawOutputPointer("result", b->getInt32(0), symStartPos), halfSymPtrTy);
    symPtr2 = b->CreateBitCast(b->getRawOutputPointer("result", b->getInt32(0), b->CreateAdd(symStartPos, symOffset)), halfSymPtrTy);
    b->CreateStore(entry1, symPtr1);
    b->CreateStore(entry2, symPtr2);

    Value * dropHash = b->CreateResetLowestBit(theHashWord);
    Value * hashWordDone = b->CreateICmpEQ(dropHash, sz_ZERO);
    // There may be more hashs in the hash mask.
    Value * nextHashMask = b->CreateSelect(hashWordDone, b->CreateResetLowestBit(hashMaskPhi), hashMaskPhi);
    BasicBlock * hashBB = b->GetInsertBlock();
    hashMaskPhi->addIncoming(nextHashMask, hashBB);
    hashWordPhi->addIncoming(dropHash, hashBB);
    b->CreateCondBr(b->CreateICmpNE(nextHashMask, sz_ZERO), hashProcessingLoop, hashesDone);

    b->SetInsertPoint(hashesDone);
    strideNo->addIncoming(nextStrideNo, hashesDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // If the segment ends in the middle of a 2-byte codeword, we need to
    // make sure that we still have access to the codeword in the next block.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(mIsFinal, avail, b->CreateSub(avail, sz_MAXLENGTH));
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the full input stream to output, there may
    // be an incomplete symbol at the end of this block.   Store the
    // data that may be overwritten as pending and set the produced item
    // count to that which is guaranteed to be correct.
    Value * guaranteedProduced = b->CreateSub(avail, sz_MAXLENGTH);
    b->CreateMemCpy(b->getScalarFieldPtr("pendingOutput"), b->getRawOutputPointer("result", guaranteedProduced), sz_MAXLENGTH, 1);
    if (!DelayedAttribute) {
        b->setProducedItemCount("result", b->CreateSelect(mIsFinal, avail, guaranteedProduced));
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
    
    StreamSet * const ztfHash_u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<P2SKernel>(ztfHash_u8_Basis, ztfHash_u8bytes);

    StreamSet * const u8bytes_34 = P->CreateStreamSet(1, 8);
    StreamSet * const u8bytes_58 = P->CreateStreamSet(1, 8);
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{3, 4}, lengthGroup3_4, lgthBytes, group3_4marks, ztfHash_u8bytes, hashBytes, u8bytes_34);
    //P->CreateKernelCall<DebugDisplayKernel>("u8bytes_34", u8bytes_34);
    P->CreateKernelCall<LengthGroupDecompression>(LengthGroup{5, 8}, lengthGroup5_8, lgthBytes, group5_8marks, u8bytes_34, hashBytes, u8bytes_58);
    //P->CreateKernelCall<DebugDisplayKernel>("u8bytes_58", u8bytes_58);
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
