/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "ztf-scan.h"
#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/intrusive/detail/math.hpp>
#include <sstream>

#if 0
#define DEBUG_PRINT(title,value) b->CallPrintInt(title, value)
#else
#define DEBUG_PRINT(title,value)
#endif

#if 0
#define CHECK_COMPRESSION_DECOMPRESSION_STORE
#endif

using namespace kernel;
using namespace llvm;

static cl::opt<bool> DeferredAttribute("deferred", cl::desc("Use Deferred attribute instead of Lookbehind for source data"), cl::init(false));
static cl::opt<bool> DelayedAttribute("delayed", cl::desc("Use Delayed Attribute instead of BoundedRate for output"), cl::init(true));
static cl::opt<bool> PrefixCheck("prefix-check-mode", cl::desc("Use experimental prefix check mode"), cl::init(false));

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

struct LengthGroupParameters {
    LengthGroupInfo groupInfo;
    Constant * MAX_HASH_BITS;
    Constant * SUFFIX_BITS;
    Constant * SUFFIX_MASK;
    unsigned const groupHalfLength;
    Type * halfLengthTy;
    Type * halfSymPtrTy;
    Constant * HALF_LENGTH;
    Constant * LO;
    Constant * HI;
    // All subtables are sized the same.
    Constant * SUBTABLE_SIZE;
    Constant * HASH_BITS;
    Constant * EXTENDED_BITS;
    Constant * HASH_MASK;
    Constant * ENC_BYTES;
    Constant * MAX_INDEX;
    Constant * PREFIX_BASE;
    Constant * LENGTH_MASK;
    Constant * EXTENSION_MASK;

    LengthGroupParameters(const std::unique_ptr<KernelBuilder> & b, EncodingInfo encodingScheme, unsigned groupNo) :
        groupInfo(encodingScheme.byLength[groupNo]),
        MAX_HASH_BITS(b->getSize(encodingScheme.MAX_HASH_BITS)),
        SUFFIX_BITS(b->getSize(7)),
        SUFFIX_MASK(b->getSize(0x7F)),
        groupHalfLength(1 << boost::intrusive::detail::floor_log2(groupInfo.lo)),
        halfLengthTy(b->getIntNTy(8 * groupHalfLength)),
        halfSymPtrTy(halfLengthTy->getPointerTo()),
        HALF_LENGTH(b->getSize(groupHalfLength)),
        LO(b->getSize(groupInfo.lo)),
        HI(b->getSize(groupInfo.hi)),
        // All subtables are sized the same.
        SUBTABLE_SIZE(b->getSize((1 << groupInfo.hash_bits) * groupInfo.hi)),
        HASH_BITS(b->getSize(groupInfo.hash_bits)),
        EXTENDED_BITS(b->getSize(std::max(groupInfo.hash_bits + groupInfo.length_extension_bits, (groupInfo.encoding_bytes -1 ) * 7))),
        HASH_MASK(b->getSize((1 << groupInfo.hash_bits) - 1)),
        ENC_BYTES(b->getSize(groupInfo.encoding_bytes)),
        MAX_INDEX(b->getSize(groupInfo.encoding_bytes - 1)),
        PREFIX_BASE(b->getSize(groupInfo.prefix_base)),
        LENGTH_MASK(b->getSize(2 * groupHalfLength - 1)),
        EXTENSION_MASK(b->getSize((1 << groupInfo.length_extension_bits) - 1)) {
            assert(groupInfo.hi <= (1 << (boost::intrusive::detail::floor_log2(groupInfo.lo) + 1)));
        }
};

unsigned hashTableSize(LengthGroupInfo g) {
    unsigned numSubTables = (g.hi - g.lo + 1);
    return numSubTables * g.hi * (1<<g.hash_bits);
}

std::string lengthRangeSuffix(EncodingInfo encodingScheme, unsigned lo, unsigned hi) {
    std::stringstream suffix;
    suffix << encodingScheme.uniqueSuffix() << "_" << lo << "_" << hi;
    if (DeferredAttribute) suffix << "deferred";
    if (DelayedAttribute) suffix << "_delayed";
    return suffix.str();
}

std::string lengthGroupSuffix(EncodingInfo encodingScheme, unsigned groupNo) {
    LengthGroupInfo g = encodingScheme.byLength[groupNo];
    return lengthRangeSuffix(encodingScheme, g.lo, g.hi);
}

Binding ByteDataBinding(unsigned max_length, StreamSet * byteData) {
    if (DeferredAttribute) {
        return Binding{"byteData", byteData, FixedRate(), { Deferred() }}; // , Linear()
    } else {
        return Binding{"byteData", byteData, FixedRate(), { LookBehind(max_length+1) }}; // , Linear()
    }
}

std::vector<Value *> initializeCompressionMasks(const std::unique_ptr<KernelBuilder> & b,
                                                ScanWordParameters & sw,
                                                Constant * sz_BLOCKS_PER_STRIDE,
                                                unsigned maskCount,
                                                Value * strideBlockOffset,
                                                Value * compressMaskPtr,
                                                BasicBlock * strideMasksReady) {
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Type * sizeTy = b->getSizeTy();
    std::vector<Value *> keyMasks(maskCount);
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const maskInitialization = b->CreateBasicBlock("maskInitialization");
    b->CreateBr(maskInitialization);
    b->SetInsertPoint(maskInitialization);
    std::vector<PHINode *> keyMaskAccum(maskCount);
    for (unsigned i = 0; i < maskCount; i++) {
        keyMaskAccum[i] = b->CreatePHI(sizeTy, 2);
        keyMaskAccum[i]->addIncoming(sz_ZERO, entryBlock);
    }
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, entryBlock);
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    for (unsigned i = 0; i < maskCount; i++) {
        Value * keyBitBlock = b->loadInputStreamBlock("symbolMarks", b->getSize(i), strideBlockIndex);
        Value * const anyKey = b->simd_any(sw.width, keyBitBlock);
        Value * keyWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyKey), sizeTy);
        keyMasks[i] = b->CreateOr(keyMaskAccum[i], b->CreateShl(keyWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)));
        keyMaskAccum[i]->addIncoming(keyMasks[i], maskInitialization);
    }
    // Initialize the compression mask.
    // Default initial compression mask is all ones (no zeroes => no compression).
    b->CreateBlockAlignedStore(b->allOnes(), b->CreateGEP(compressMaskPtr, strideBlockIndex));
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, maskInitialization);
    // Default initial compression mask is all ones (no zeroes => no compression).
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), maskInitialization, strideMasksReady);
    return keyMasks;
}

LengthGroupCompressionMask::LengthGroupCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                       EncodingInfo encodingScheme,
                                                       unsigned groupNo,
                                                       StreamSet * symbolMarks,
                                                       StreamSet * hashValues,
                                                       StreamSet * const byteData, StreamSet * compressionMask, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupCompressionMask" + lengthGroupSuffix(encodingScheme, groupNo) + (PrefixCheck ? "_prefix" : ""),
                   {Binding{"symbolMarks", symbolMarks},
                       Binding{"hashValues", hashValues},
                       ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)},
                   {}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo])), "hashTable"}}),
mEncodingScheme(encodingScheme), mGroupNo(groupNo) {
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, FixedRate(), Delayed(encodingScheme.maxSymbolLength()));
    } else {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, BoundedRate(0,1));
    }
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

void LengthGroupCompressionMask::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    LengthGroupParameters lg(b, mEncodingScheme, mGroupNo);
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_BLOCKWIDTH = b->getSize(b->getBitBlockWidth());
    Type * sizeTy = b->getSizeTy();
    Type * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * const tryStore = b->CreateBasicBlock("tryStore");
    BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
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
    b->CreateStore(pendingMask, producedPtr);
    Value * compressMaskPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialPos), bitBlockPtrTy);
    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);

    std::vector<Value *> keyMasks = initializeCompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, 1, strideBlockOffset, compressMaskPtr, strideMasksReady);
    Value * keyMask = keyMasks[0];

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
    /* Determine the key length. */
    Value * hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);
    Value * keyLength = b->CreateAdd(b->CreateLShr(hashValue, lg.MAX_HASH_BITS), sz_TWO, "keyLength");
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, sz_ONE), "keyStartPos");
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, lg.HALF_LENGTH);
    // Get the hash of this key.
    Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");
    Value * hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength, lg.LO), lg.SUBTABLE_SIZE));
    Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, lg.HI));
    // Use two 8-byte loads to get hash and symbol values.
    //b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    Value * tblPtr1 = b->CreateBitCast(tblEntryPtr, lg.halfSymPtrTy);
    Value * tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, keyOffset), lg.halfSymPtrTy);
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", keyStartPos), lg.halfSymPtrTy);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos, keyOffset)), lg.halfSymPtrTy);
    // Check to see if the hash table entry is nonzero (already assigned).
    Value * sym1 = b->CreateAlignedLoad(symPtr1, 1);
    Value * sym2 = b->CreateAlignedLoad(symPtr2, 1);
    Value * entry1 = b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr1);
    Value * entry2 = b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr2);
    Value * symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(entry1, sym1), b->CreateICmpEQ(entry2, sym2));
    Value * maskLength = nullptr;
    if (PrefixCheck) {
        BasicBlock * const tryPrefix = b->CreateBasicBlock("tryPrefix");
        BasicBlock * const havePrefix = b->CreateBasicBlock("havePrefix");
        b->CreateCondBr(symIsEqEntry, markCompression, tryPrefix);

        b->SetInsertPoint(tryPrefix);
        // We can try an immediate prefix if the last byte of the symbol is
        // an ASCII non-word character and the the length is greater than the
        // length group minimum.
        Value * lastByte = b->CreateTrunc(sym2, b->getInt8Ty());
        Value * x20_2F = b->CreateAnd(b->CreateICmpUGE(lastByte, b->getInt8(0x20)), b->CreateICmpULE(lastByte, b->getInt8(0x2F)));
        b->CreateCondBr(b->CreateAnd(b->CreateICmpUGT(keyLength, lg.LO), x20_2F), havePrefix, tryStore);

        b->SetInsertPoint(havePrefix);
        Value * pfxEndPos = b->CreateSub(keyMarkPos, sz_ONE);
        Value * pfxHashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", pfxEndPos)), sizeTy);
        Value * pfxLength = b->CreateSub(keyLength, sz_ONE);
        Value * pfxOffset = b->CreateSub(pfxLength, lg.HALF_LENGTH);
        Value * pfxHash = b->CreateAnd(pfxHashValue, lg.HASH_MASK, "pfxHash");
        Value * pfxTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(pfxLength, lg.LO), lg.SUBTABLE_SIZE));
        Value * pfxEntryPtr = b->CreateGEP(pfxTablePtr, b->CreateMul(pfxHash, lg.HI));
        Value * pfxPtr1 = b->CreateBitCast(pfxEntryPtr, lg.halfSymPtrTy);
        Value * pfxPtr2 = b->CreateBitCast(b->CreateGEP(pfxEntryPtr, pfxOffset), lg.halfSymPtrTy);
        Value * pfx1 = b->CreateMonitoredScalarFieldLoad("hashTable", pfxPtr1);
        Value * pfx2 = b->CreateMonitoredScalarFieldLoad("hashTable", pfxPtr2);
        // Only the second half of the symbol needs to be loaded.
        Value * symPfxPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos, pfxOffset)), lg.halfSymPtrTy);
        Value * pfxSym2 = b->CreateAlignedLoad(symPfxPtr2, 1);
        symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(pfx1, sym1), b->CreateICmpEQ(pfx2, pfxSym2));
        b->CreateCondBr(symIsEqEntry, markCompression, tryStore);

        b->SetInsertPoint(markCompression);
        PHINode * const compressLengthPhi = b->CreatePHI(sizeTy, 2);
        compressLengthPhi->addIncoming(keyLength, keyProcessingLoop);
        compressLengthPhi->addIncoming(pfxLength, havePrefix);
        maskLength = b->CreateZExt(b->CreateSub(compressLengthPhi, lg.ENC_BYTES), sizeTy);
    } else {
        b->CreateCondBr(symIsEqEntry, markCompression, tryStore);
        b->SetInsertPoint(markCompression);
        maskLength = b->CreateZExt(b->CreateSub(keyLength, lg.ENC_BYTES, "maskLength"), sizeTy);
    }
    // Compute a mask of bits, with zeroes marking positions to eliminate.
    // The entire symbols will be replaced, but we need to keep the required
    // number of positions for the encoded ZTF sequence.
    Value * mask = b->CreateSub(b->CreateShl(sz_ONE, maskLength), sz_ONE);
    // Determine a base position from which both the keyStart and the keyEnd
    // are accessible within SIZE_T_BITS - 8, and which will not overflow
    // the buffer.
    assert(SIZE_T_BITS - 8 > 2 * lg.groupHalfLength);
    Value * startBase = b->CreateSub(keyStartPos, b->CreateURem(keyStartPos, b->getSize(8)));
    Value * markBase = b->CreateSub(keyMarkPos, b->CreateURem(keyMarkPos, sz_BITS));
    Value * keyBase = b->CreateSelect(b->CreateICmpULT(startBase, markBase), startBase, markBase);
    Value * bitOffset = b->CreateSub(keyStartPos, keyBase);
    //b->CallPrintInt("bitOffset", bitOffset);
    mask = b->CreateShl(mask, bitOffset);
    //b->CallPrintInt("mask", mask);
    Value * const keyBasePtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", keyBase), sizeTy->getPointerTo());

    Value * initialMask = b->CreateAlignedLoad(keyBasePtr, 1);
    //b->CallPrintInt("initialMask", initialMask);
    Value * updated = b->CreateAnd(initialMask, b->CreateNot(mask));
    //b->CallPrintInt("updated", updated);
    b->CreateAlignedStore(b->CreateAnd(updated, b->CreateNot(mask)), keyBasePtr, 1);
    b->CreateBr(nextKey);

    b->SetInsertPoint(tryStore);
    Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1, entry2), Constant::getNullValue(lg.halfLengthTy));
    b->CreateCondBr(isEmptyEntry, storeKey, nextKey);

    b->SetInsertPoint(storeKey);
#ifdef CHECK_COMPRESSION_DECOMPRESSION_STORE
    b->CallPrintInt("hashCode", keyHash);
    b->CallPrintInt("keyStartPos", keyStartPos);
    b->CallPrintInt("keyLength", keyLength);
#endif
    // We have a new symbol that allows future occurrences of the symbol to
    // be compressed using the hash code.
    b->CreateMonitoredScalarFieldStore("hashTable", sym1, tblPtr1);
    b->CreateMonitoredScalarFieldStore("hashTable", sym2, tblPtr2);
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
        Value * processed = b->CreateSub(avail, lg.HI);
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

void initializeDecompressionMasks(const std::unique_ptr<KernelBuilder> & b,
                                  ScanWordParameters & sw,
                                  Constant * sz_BLOCKS_PER_STRIDE,
                                  unsigned maskCount,
                                  Value * strideBlockOffset,
                                  std::vector<Value *> & keyMasks,
                                  std::vector<Value *> & hashMasks,
                                  BasicBlock * strideMasksReady) {
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Type * sizeTy = b->getSizeTy();
    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const maskInitialization = b->CreateBasicBlock("maskInitialization");
    b->CreateBr(maskInitialization);
    b->SetInsertPoint(maskInitialization);
    std::vector<PHINode *> keyMaskAccum(maskCount);
    std::vector<PHINode *> hashMaskAccum(maskCount);
    for (unsigned i = 0; i < maskCount; i++) {
        keyMaskAccum[i] = b->CreatePHI(sizeTy, 2);
        hashMaskAccum[i] = b->CreatePHI(sizeTy, 2);
        keyMaskAccum[i]->addIncoming(sz_ZERO, entryBlock);
        hashMaskAccum[i]->addIncoming(sz_ZERO, entryBlock);
    }
    PHINode * const blockNo = b->CreatePHI(sizeTy, 2);
    blockNo->addIncoming(sz_ZERO, entryBlock);
    Value * strideBlockIndex = b->CreateAdd(strideBlockOffset, blockNo);
    for (unsigned i = 0; i < maskCount; i++) {
        Value * keyBitBlock = b->loadInputStreamBlock("keyMarks" + std::to_string(i), strideBlockIndex);
        Value * hashBitBlock = b->loadInputStreamBlock("hashMarks" + std::to_string(i), strideBlockIndex);
        Value * const anyKey = b->simd_any(sw.width, keyBitBlock);
        Value * const anyHash = b->simd_any(sw.width, hashBitBlock);
        Value * keyWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyKey), sizeTy);
        Value * hashWordMask = b->CreateZExtOrTrunc(b->hsimd_signmask(sw.width, anyHash), sizeTy);
        keyMasks[i] = b->CreateOr(keyMaskAccum[i], b->CreateShl(keyWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)));
        hashMasks[i] = b->CreateOr(hashMaskAccum[i], b->CreateShl(hashWordMask, b->CreateMul(blockNo, sw.WORDS_PER_BLOCK)));
        keyMaskAccum[i]->addIncoming(keyMasks[i], maskInitialization);
        hashMaskAccum[i]->addIncoming(hashMasks[i], maskInitialization);
    }
    Value * const nextBlockNo = b->CreateAdd(blockNo, sz_ONE);
    blockNo->addIncoming(nextBlockNo, maskInitialization);
    // Default initial compression mask is all ones (no zeroes => no compression).
    b->CreateCondBr(b->CreateICmpNE(nextBlockNo, sz_BLOCKS_PER_STRIDE), maskInitialization, strideMasksReady);
}


LengthGroupDecompression::LengthGroupDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                   EncodingInfo encodingScheme,
                                                   unsigned groupNo,
                                                   StreamSet * keyMarks,
                                                   StreamSet * hashValues,
                                                   StreamSet * const hashMarks, StreamSet * const byteData,
                                                   StreamSet * const result, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupDecompression" + lengthGroupSuffix(encodingScheme, groupNo),
                   {Binding{"keyMarks0", keyMarks},
                       Binding{"hashValues", hashValues},
                       Binding{"hashMarks0", hashMarks},
                       ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)
                   },
                   {}, {}, {},
                   {InternalScalar{ArrayType::get(b->getInt8Ty(), encodingScheme.byLength[groupNo].hi), "pendingOutput"},
                       // Hash table 8 length-based tables with 256 16-byte entries each.
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo])), "hashTable"}}),
    mEncodingScheme(encodingScheme), mGroupNo(groupNo) {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("result", result, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("result", result, BoundedRate(0,1));
    }
    if (!DelayedAttribute) {
        addInternalScalar(ArrayType::get(b->getInt8Ty(), encodingScheme.byLength[groupNo].hi), "pendingOutput");
    }
}

void LengthGroupDecompression::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    ScanWordParameters sw(b, mStride);
    LengthGroupParameters lg(b, mEncodingScheme, mGroupNo);
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Type * sizeTy = b->getSizeTy();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
    BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
    BasicBlock * const keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const hashProcessingLoop = b->CreateBasicBlock("hashProcessingLoop");
    BasicBlock * const lookupSym = b->CreateBasicBlock("lookupSym");
    BasicBlock * const nextHash = b->CreateBasicBlock("nextHash");
    BasicBlock * const hashesDone = b->CreateBasicBlock("hashesDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");

    Value * const initialPos = b->getProcessedItemCount("keyMarks0");
    Value * const avail = b->getAvailableItemCount("keyMarks0");

    if (!DelayedAttribute) {
        // Copy pending output data.
        Value * const initialProduced = b->getProducedItemCount("result");
        b->CreateMemCpy(b->getRawOutputPointer("result", initialProduced), b->getScalarFieldPtr("pendingOutput"), lg.HI, 1);
    }

    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateMul(numOfStrides, sz_STRIDE);
    b->CreateMemCpy(b->getRawOutputPointer("result", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);
    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);

    std::vector<Value *> keyMasks(1);
    std::vector<Value *> hashMasks(1);
    initializeDecompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, 1, strideBlockOffset, keyMasks, hashMasks, strideMasksReady);
    Value * keyMask = keyMasks[0];
    Value * hashMask = hashMasks[0];

    b->SetInsertPoint(strideMasksReady);
    // First iterate through the new keys and update the hash table as
    // appropriate.   Each key is hashed, and is entered into the hash
    // table if there is not already an entry for that hash code.
    Value * keyWordBasePtr = b->getInputStreamBlockPtr("keyMarks0", sz_ZERO, strideBlockOffset);
    keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
    DEBUG_PRINT("keyMask", keyMask);
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
    DEBUG_PRINT("keyMarkPos", keyMarkPos);
    /* Determine the key length. */
    Value * const hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);
    Value * keyLength = b->CreateAdd(b->CreateLShr(hashValue, lg.MAX_HASH_BITS), sz_TWO, "keyLength");
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, sz_ONE), "keyStartPos");
    DEBUG_PRINT("keyLength", keyLength);
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, lg.HALF_LENGTH);
    DEBUG_PRINT("keyOffset", keyOffset);
    // Get the hash of this key.
    Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");
    DEBUG_PRINT("keyHash", keyHash);
    Value * hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength, lg.LO), lg.SUBTABLE_SIZE));
    Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, lg.HI));
    // Use two 8-byte loads to get hash and symbol values.
    Value * tblPtr1 = b->CreateBitCast(tblEntryPtr, lg.halfSymPtrTy);
    Value * tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, keyOffset), lg.halfSymPtrTy);
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", keyStartPos), lg.halfSymPtrTy);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos, keyOffset)), lg.halfSymPtrTy);

    // Check to see if the hash table entry is nonzero (already assigned).
    Value * sym1 = b->CreateLoad(symPtr1);
    DEBUG_PRINT("sym1", sym1);
    Value * sym2 = b->CreateLoad(symPtr2);
    DEBUG_PRINT("sym2", sym2);
    Value * entry1 = b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr1);
    DEBUG_PRINT("entry1", entry1);
    Value * entry2 = b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr2);
    DEBUG_PRINT("entry2", entry2);
    Value * isEmptyEntry = b->CreateIsNull(b->CreateOr(entry1, entry2));
    b->CreateCondBr(isEmptyEntry, storeKey, nextKey);
    b->SetInsertPoint(storeKey);
    // We have a new symbols that allows future occurrences of the symbol to
    // be compressed using the hash code.
    //b->CreateWriteCall(b->getInt32(STDERR_FILENO), symPtr1, keyLength);
#ifdef CHECK_COMPRESSION_DECOMPRESSION_STORE
    b->CallPrintInt("hashCode", keyHash);
    b->CallPrintInt("keyStartPos", keyStartPos);
    b->CallPrintInt("keyLength", keyLength);
#endif
    b->CreateMonitoredScalarFieldStore("hashTable", sym1, tblPtr1);
    b->CreateMonitoredScalarFieldStore("hashTable", sym2, tblPtr2);
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
    Value * hashWordBasePtr = b->getInputStreamBlockPtr("hashMarks0", sz_ZERO, strideBlockOffset);
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
    DEBUG_PRINT("hashMarkPos", hashMarkPos);
    Value * hashPfxPos = b->CreateSub(hashMarkPos, lg.MAX_INDEX);
    Value * const hashPfx = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", hashPfxPos)), sizeTy);
    DEBUG_PRINT("hashPfx", hashPfx);
    // Build up a single encoded value from the ZTF code sequence.
    Value * encodedVal = b->CreateSub(hashPfx, lg.PREFIX_BASE, "encodedVal");
    Value * curPos = hashPfxPos;
    for (unsigned i = 1; i < lg.groupInfo.encoding_bytes; i++) {
        curPos = b->CreateAdd(curPos, sz_ONE);
        Value * suffixByte = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", curPos)), sizeTy);
        encodedVal = b->CreateOr(b->CreateShl(encodedVal, lg.SUFFIX_BITS), b->CreateAnd(suffixByte, lg.SUFFIX_MASK), "encodedVal");
    }
    Value * symLength = b->CreateAdd(b->CreateLShr(encodedVal, lg.EXTENDED_BITS), lg.LO, "symLength");
    Value * validLength = b->CreateAnd(b->CreateICmpUGE(symLength, lg.LO), b->CreateICmpULE(symLength, lg.HI));
    DEBUG_PRINT("symLength", symLength);
    b->CreateCondBr(validLength, lookupSym, nextHash);
    b->SetInsertPoint(lookupSym);
    Value * hashCode = b->CreateAnd(encodedVal, lg.HASH_MASK, "hashCode");
    DEBUG_PRINT("hashCode", hashCode);
    Value * symStartPos = b->CreateSub(hashMarkPos, b->CreateSub(symLength, sz_ONE), "symStartPos");
    Value * symOffset = b->CreateSub(symLength, lg.HALF_LENGTH);
    hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(symLength, lg.LO), lg.SUBTABLE_SIZE));
    tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(hashCode, lg.HI));
    // Use two 8-byte loads to get hash and symbol values.
    // b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    tblPtr1 = b->CreateBitCast(tblEntryPtr, lg.halfSymPtrTy);
    tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, symOffset), lg.halfSymPtrTy);
    entry1 = b->CreateAlignedLoad(tblPtr1, 1);
    entry2 = b->CreateAlignedLoad(tblPtr2, 1);
    DEBUG_PRINT("symStartPos", symStartPos);
    symPtr1 = b->CreateBitCast(b->getRawOutputPointer("result", symStartPos), lg.halfSymPtrTy);
    DEBUG_PRINT("symOffset", symOffset);
    symPtr2 = b->CreateBitCast(b->getRawOutputPointer("result", b->CreateAdd(symStartPos, symOffset)), lg.halfSymPtrTy);
    DEBUG_PRINT("entry1", entry1);
    b->CreateAlignedStore(entry1, symPtr1, 1);
    DEBUG_PRINT("entry2", entry2);
    b->CreateAlignedStore(entry2, symPtr2, 1);
    b->CreateBr(nextHash);
    b->SetInsertPoint(nextHash);
    Value * dropHash = b->CreateResetLowestBit(theHashWord);
    Value * hashMaskDone = b->CreateICmpEQ(dropHash, sz_ZERO);
    // There may be more hashs in the hash mask.
    Value * nextHashMask = b->CreateSelect(hashMaskDone, b->CreateResetLowestBit(hashMaskPhi), hashMaskPhi);
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
        Value * processed = b->CreateSub(avail, lg.HI);
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the full input stream to output, there may
    // be an incomplete symbol at the end of this block.   Store the
    // data that may be overwritten as pending and set the produced item
    // count to that which is guaranteed to be correct.
    if (!DelayedAttribute) {
        Value * guaranteedProduced = b->CreateSub(avail, lg.HI);
        b->CreateMemCpy(b->getScalarFieldPtr("pendingOutput"), b->getRawOutputPointer("result", guaranteedProduced), lg.HI, 1);
        b->setProducedItemCount("result", b->CreateSelect(mIsFinal, avail, guaranteedProduced));
    }
}

unsigned hashTableSize(EncodingInfo info, unsigned lgth) {
    LengthGroupInfo g = info.byLength[info.getLengthGroupNo(lgth)];
    return lgth * (1 << g.hash_bits);
}

//
// Length-based symbol operations - for symbol length known at LLVM compile time.
//
std::vector<Value *> MonitoredScalarLoadSymbol(const std::unique_ptr<KernelBuilder> & b, std::string scalarName, Value * sourcePtr, unsigned length) {
    unsigned load_length = 1 << boost::intrusive::detail::floor_log2(length);
    //b->CallPrintInt("sourcePtr" + std::to_string(length), sourcePtr);
    Type * loadPtrTy = b->getIntNTy(load_length * 8)->getPointerTo();
    Value * load1 = b->CreateMonitoredScalarFieldLoad(scalarName, b->CreateBitCast(sourcePtr, loadPtrTy));
    if (load_length == length) {
        return std::vector<Value *>{load1};
    }
    Constant * offset = b->getInt32(length - load_length);
    Value * srcPtr2 = b->CreateGEP(b->CreateBitCast(sourcePtr, b->getInt8PtrTy()), offset);
    Value * load2 = b->CreateMonitoredScalarFieldLoad(scalarName, b->CreateBitCast(srcPtr2, loadPtrTy));
    return std::vector<Value *>{load1, load2};
}

std::vector<Value *> loadSymbol(const std::unique_ptr<KernelBuilder> & b, Value * sourcePtr, unsigned length) {
    unsigned load_length = 1 << boost::intrusive::detail::floor_log2(length);
    Type * loadPtrTy = b->getIntNTy(load_length * 8)->getPointerTo();
    Value * load1 = b->CreateAlignedLoad(b->CreateBitCast(sourcePtr, loadPtrTy), 1);
    if (load_length == length) {
        return std::vector<Value *>{load1};
    }
    Constant * offset = b->getInt32(length - load_length);
    Value * srcPtr2 = b->CreateGEP(b->CreateBitCast(sourcePtr, b->getInt8PtrTy()), offset);
    Value * load2 = b->CreateAlignedLoad(b->CreateBitCast(srcPtr2, loadPtrTy), 1);
    return std::vector<Value *>{load1, load2};
}

void MonitoredScalarStoreSymbol(const std::unique_ptr<KernelBuilder> & b, std::string scalarName, std::vector<Value *> toStore, Value * ptr, unsigned length) {
    unsigned store_length = 1 << boost::intrusive::detail::floor_log2(length);
    //b->CallPrintInt("ptr" + std::to_string(length), ptr);
    Type * storePtrTy = b->getIntNTy(store_length * 8)->getPointerTo();
    b->CreateMonitoredScalarFieldStore(scalarName, toStore[0], b->CreateBitCast(ptr, storePtrTy));
    if (store_length == length) {
        return;
    }
    Constant * offset = b->getInt32(length - store_length);
    Value * ptr2 = b->CreateGEP(b->CreateBitCast(ptr, b->getInt8PtrTy()), offset);
    b->CreateMonitoredScalarFieldStore(scalarName, toStore[1], b->CreateBitCast(ptr2, storePtrTy));
}

void storeSymbol(const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> toStore, Value * ptr, unsigned length) {
    unsigned store_length = 1 << boost::intrusive::detail::floor_log2(length);
    Type * storePtrTy = b->getIntNTy(store_length * 8)->getPointerTo();
    b->CreateAlignedStore(toStore[0], b->CreateBitCast(ptr, storePtrTy), 1);
    if (store_length == length) {
        return;
    }
    Constant * offset = b->getInt32(length - store_length);
    Value * ptr2 = b->CreateGEP(b->CreateBitCast(ptr, b->getInt8PtrTy()), offset);
    b->CreateAlignedStore(toStore[1], b->CreateBitCast(ptr2, storePtrTy), 1);
}

Value * compareSymbols (const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> sym1, std::vector<Value *> sym2) {
    if (sym1.size() == 1) {
        return b->CreateICmpEQ(sym1[0], sym2[0]);
    }
    return b->CreateAnd(b->CreateICmpEQ(sym1[0], sym2[0]), b->CreateICmpEQ(sym1[1], sym2[1]));
}

Value * isNullSymbol (const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> sym) {
    Constant * symNull = Constant::getNullValue(sym[0]->getType());
    if (sym.size() == 1) {
        return b->CreateICmpEQ(sym[0], symNull);
    }
    return b->CreateICmpEQ(b->CreateOr(sym[0], sym[1]), symNull);
}

void generateKeyProcessingLoops(const std::unique_ptr<KernelBuilder> & b,
                                ScanWordParameters & sw,
                                EncodingInfo & encodingScheme,
                                unsigned lo,
                                std::vector<Value *> keyMasks,
                                Value * keyWordBasePtr,
                                Value * hashTablePtr,
                                Value * stridePos,
                                BasicBlock * keysDone) {

    BasicBlock * keyProcessingLoop = b->GetInsertBlock();
    BasicBlock * loopPredecssor = keyProcessingLoop->getUniquePredecessor();
    unsigned maskCount = keyMasks.size();
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Type * sizeTy = b->getSizeTy();

    for (unsigned i = 0; i < maskCount; i++) {
        unsigned length = lo + i;
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * sz_COMPRESSION_MASK = b->getSize((1 << (length - lg.groupInfo.encoding_bytes)) - 1);
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
        BasicBlock * const markCompression = b->CreateBasicBlock("markCompression");
        BasicBlock * const checkKey = b->CreateBasicBlock("checkKey");
        BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
        BasicBlock * loopExit;
        if (i == maskCount - 1) {
            loopExit = keysDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("keyProcessingLoop");
        }
        b->SetInsertPoint(keyProcessingLoop);
        PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
        keyMaskPhi->addIncoming(keyMasks[i], loopPredecssor);
        PHINode * const keyWordPhi = b->CreatePHI(sizeTy, 2);
        keyWordPhi->addIncoming(sz_ZERO, loopPredecssor);
        Value * keyWordIdx = b->CreateCountForwardZeroes(keyMaskPhi, "keyWordIdx");
        Value * nextKeyWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keyWordBasePtr, keyWordIdx)), sizeTy);
        Value * theKeyWord = b->CreateSelect(b->CreateICmpEQ(keyWordPhi, sz_ZERO), nextKeyWord, keyWordPhi);
        Value * keyWordPos = b->CreateAdd(stridePos, b->CreateMul(keyWordIdx, sw.WIDTH));
        Value * keyMarkPosInWord = b->CreateCountForwardZeroes(theKeyWord);
        Value * keyMarkPos = b->CreateAdd(keyWordPos, keyMarkPosInWord, "keyEndPos");
        Value * const hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);
        Value * keyStartPos = b->CreateSub(keyMarkPos, sz_MARK_OFFSET, "keyStartPos");
        Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");

        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, lg.HI));
        Value * symPtr = b->getRawInputPointer("byteData", b->getInt32(0), keyStartPos);
        // Check to see if the hash table entry is nonzero (already assigned).
        std::vector<Value *> sym = loadSymbol(b, symPtr, length);
        std::vector<Value *> entry = loadSymbol(b, tblEntryPtr, length);
        b->CreateCondBr(isNullSymbol(b, entry), storeKey, checkKey);

        b->SetInsertPoint(storeKey);
        // We have a new symbol that allows future occurrences of the symbol to
        // be compressed using the hash code.
        storeSymbol(b, sym, tblEntryPtr, length);
#ifdef CHECK_COMPRESSION_DECOMPRESSION_STORE
        b->CallPrintInt("hashCode", keyHash);
        b->CallPrintInt("keyStartPos", keyStartPos);
        b->CallPrintInt("keyLength", lg.HI);
#endif
        b->CreateBr(nextKey);

        b->SetInsertPoint(checkKey);
        // If the symbol is equal to the stored entry, it will be replace
        // by the ZTF compression code.  Prepare the compression mask by
        // zeroing out all symbol positions except the last two.
        Value * symIsEqEntry = compareSymbols(b, sym, entry);
        b->CreateCondBr(symIsEqEntry, markCompression, nextKey);

        b->SetInsertPoint(markCompression);
        // Determine a base position from which both the keyStart and the keyEnd
        // are accessible within SIZE_T_BITS - 8, and which will not overflow
        // the buffer.
        Value * startBase = b->CreateSub(keyStartPos, b->CreateURem(keyStartPos, b->getSize(8)));
        Value * markBase = b->CreateSub(keyMarkPos, b->CreateURem(keyMarkPos, sz_BITS));
        Value * keyBase = b->CreateSelect(b->CreateICmpULT(startBase, markBase), startBase, markBase);
        Value * bitOffset = b->CreateSub(keyStartPos, keyBase);
        Value * mask = b->CreateShl(sz_COMPRESSION_MASK, bitOffset);
        //b->CallPrintInt("mask", mask);
        Value * const keyBasePtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", keyBase), sizeTy->getPointerTo());

        Value * initialMask = b->CreateAlignedLoad(keyBasePtr, 1);
        //b->CallPrintInt("initialMask", initialMask);
        Value * updated = b->CreateAnd(initialMask, b->CreateNot(mask));
        //b->CallPrintInt("updated", updated);
        b->CreateAlignedStore(b->CreateAnd(updated, b->CreateNot(mask)), keyBasePtr, 1);
        b->CreateBr(nextKey);

        b->SetInsertPoint(nextKey);
        Value * dropKey = b->CreateResetLowestBit(theKeyWord);
        Value * thisWordDone = b->CreateICmpEQ(dropKey, sz_ZERO);
        // There may be more keys in the key mask.
        Value * nextKeyMask = b->CreateSelect(thisWordDone, b->CreateResetLowestBit(keyMaskPhi), keyMaskPhi);
        BasicBlock * currentBB = b->GetInsertBlock();
        keyMaskPhi->addIncoming(nextKeyMask, currentBB);
        keyWordPhi->addIncoming(dropKey, currentBB);
        loopPredecssor = currentBB;
        b->CreateCondBr(b->CreateICmpNE(nextKeyMask, sz_ZERO), keyProcessingLoop, loopExit);
        keyProcessingLoop = loopExit;
    }
    b->SetInsertPoint(keysDone);
}

FixedLengthCompressionMask::FixedLengthCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                       EncodingInfo encodingScheme,
                                                       unsigned length,
                                                       StreamSet * symbolMarks,
                                                       StreamSet * hashValues,
                                                       StreamSet * const byteData,
                                                       StreamSet * compressionMask, unsigned strideBlocks)
: MultiBlockKernel(b, "FixedLengthCompressionMask" + lengthRangeSuffix(encodingScheme, length, length + symbolMarks->getNumElements() - 1),
                   {Binding{"symbolMarks", symbolMarks},
                       Binding{"hashValues", hashValues},
                       ByteDataBinding(length, byteData)},
                   {}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme, length)), "hashTable"}}),
mEncodingScheme(encodingScheme), mLength(length)  {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, BoundedRate(0,1));
    }
}

void FixedLengthCompressionMask::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);

    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);

    Constant * sz_BLOCKWIDTH = b->getSize(b->getBitBlockWidth());
    Type * sizeTy = b->getSizeTy();
    Type * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const updatePending = b->CreateBasicBlock("updatePending");
    BasicBlock * const compressionMaskDone = b->CreateBasicBlock("compressionMaskDone");

    Value * const initialPos = b->getProcessedItemCount("symbolMarks");
    Value * const avail = b->getAvailableItemCount("symbolMarks");
    Value * const initialProduced = b->getProducedItemCount("compressionMask");
    Value * pendingMask = b->CreateNot(b->getScalarField("pendingMaskInverted"));
    Value * producedPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialProduced), bitBlockPtrTy);
    b->CreateStore(pendingMask, producedPtr);
    Value * compressMaskPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialPos), bitBlockPtrTy);
    Value * hashTablePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);

    std::vector<Value *> keyMasks = initializeCompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, 1, strideBlockOffset, compressMaskPtr, strideMasksReady);
    Value * keyMask = keyMasks[0];
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
    generateKeyProcessingLoops(b, sw, mEncodingScheme, mLength, keyMasks, keyWordBasePtr, hashTablePtr, stridePos, keysDone);

    b->SetInsertPoint(keysDone);
    strideNo->addIncoming(nextStrideNo, keysDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // In the next segment, we may need to access byte data in the last
    // 16 bytes of this segment.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(mIsFinal, avail, b->CreateSub(avail, b->getSize(16)));
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

void generateDecompKeyProcessingLoops(const std::unique_ptr<KernelBuilder> & b,
                                ScanWordParameters & sw,
                                EncodingInfo & encodingScheme,
                                unsigned lo,
                                std::vector<Value *> keyMasks,
                                Value * hashTablePtr,
                                Value * strideBlockOffset,
                                Value * stridePos,
                                BasicBlock * keysDone) {

    unsigned maskCount = keyMasks.size();
    Constant * sz_ZERO = b->getSize(0);
    Type * sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < maskCount; i++) {
        unsigned length = lo + i;
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * entryBlock = b->GetInsertBlock();
        BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
        BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
        BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
        BasicBlock * loopExit;
        if (i == maskCount - 1) {
            loopExit = keysDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("loopExit");
        }
        Value * keyWordBasePtr = b->getInputStreamBlockPtr("keyMarks" + std::to_string(i), sz_ZERO, strideBlockOffset);
        keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
        b->CreateUnlikelyCondBr(b->CreateICmpEQ(keyMasks[i], sz_ZERO), loopExit, keyProcessingLoop);

        b->SetInsertPoint(keyProcessingLoop);
        PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
        keyMaskPhi->addIncoming(keyMasks[i], entryBlock);
        PHINode * const keyWordPhi = b->CreatePHI(sizeTy, 2);
        keyWordPhi->addIncoming(sz_ZERO, entryBlock);
        Value * keyWordIdx = b->CreateCountForwardZeroes(keyMaskPhi, "keyWordIdx");
        Value * nextKeyWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keyWordBasePtr, keyWordIdx)), sizeTy);
        Value * theKeyWord = b->CreateSelect(b->CreateICmpEQ(keyWordPhi, sz_ZERO), nextKeyWord, keyWordPhi);
        Value * keyWordPos = b->CreateAdd(stridePos, b->CreateMul(keyWordIdx, sw.WIDTH));
        Value * keyMarkPosInWord = b->CreateCountForwardZeroes(theKeyWord);
        Value * keyMarkPos = b->CreateAdd(keyWordPos, keyMarkPosInWord, "keyEndPos");
        Value * const hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);
        Value * keyStartPos = b->CreateSub(keyMarkPos, sz_MARK_OFFSET, "keyStartPos");
        Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");

        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), b->getInt32(i), b->CreateMul(keyHash, lg.HI)});
        Value * symPtr = b->getRawInputPointer("byteData", b->getInt32(0), keyStartPos);
        // Check to see if the hash table entry is nonzero (already assigned).
        std::vector<Value *> sym = loadSymbol(b, symPtr, length);
        std::vector<Value *> entry = MonitoredScalarLoadSymbol(b, "hashTable", tblEntryPtr, length);
        b->CreateCondBr(isNullSymbol(b, entry), storeKey, nextKey);

        b->SetInsertPoint(storeKey);
        // We have a new symbol that allows future occurrences of the symbol to
        // be compressed using the hash code.
        MonitoredScalarStoreSymbol(b, "hashTable", sym, tblEntryPtr, length);
#ifdef CHECK_COMPRESSION_DECOMPRESSION_STORE
        b->CallPrintInt("hashCode", keyHash);
        b->CallPrintInt("keyStartPos", keyStartPos);
        b->CallPrintInt("keyLength", lg.HI);
#endif
        b->CreateBr(nextKey);

        b->SetInsertPoint(nextKey);
        Value * dropKey = b->CreateResetLowestBit(theKeyWord);
        Value * thisWordDone = b->CreateICmpEQ(dropKey, sz_ZERO);
        // There may be more keys in the key mask.
        Value * nextKeyMask = b->CreateSelect(thisWordDone, b->CreateResetLowestBit(keyMaskPhi), keyMaskPhi);
        BasicBlock * currentBB = b->GetInsertBlock();
        keyMaskPhi->addIncoming(nextKeyMask, currentBB);
        keyWordPhi->addIncoming(dropKey, currentBB);
        b->CreateCondBr(b->CreateICmpNE(nextKeyMask, sz_ZERO), keyProcessingLoop, loopExit);
        b->SetInsertPoint(loopExit);
    }
}

void generateHashProcessingLoops(const std::unique_ptr<KernelBuilder> & b,
                                 ScanWordParameters & sw,
                                 EncodingInfo & encodingScheme,
                                 unsigned lo,
                                 std::vector<Value *> hashMasks,
                                 Value * hashTablePtr,
                                 Value * strideBlockOffset,
                                 Value * stridePos,
                                 BasicBlock * hashesDone) {

    unsigned maskCount = hashMasks.size();
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Type * sizeTy = b->getSizeTy();

    for (unsigned i = 0; i < maskCount; i++) {
        unsigned length = lo + i;
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * entryBlock = b->GetInsertBlock();
        BasicBlock * const hashProcessingLoop = b->CreateBasicBlock("hashProcessingLoop");
        BasicBlock * loopExit;
        if (i == maskCount - 1) {
            loopExit = hashesDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("loopExit");
        }
        Value * hashWordBasePtr = b->getInputStreamBlockPtr("hashMarks" + std::to_string(i), sz_ZERO, strideBlockOffset);
        hashWordBasePtr = b->CreateBitCast(hashWordBasePtr, sw.pointerTy);
        b->CreateUnlikelyCondBr(b->CreateICmpEQ(hashMasks[i], sz_ZERO), loopExit, hashProcessingLoop);

        b->SetInsertPoint(hashProcessingLoop);
        PHINode * const hashMaskPhi = b->CreatePHI(sizeTy, 2);
        hashMaskPhi->addIncoming(hashMasks[i], entryBlock);
        PHINode * const hashWordPhi = b->CreatePHI(sizeTy, 2);
        hashWordPhi->addIncoming(sz_ZERO, entryBlock);
        Value * hashWordIdx = b->CreateCountForwardZeroes(hashMaskPhi, "hashWordIdx");
        Value * nextHashWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(hashWordBasePtr, hashWordIdx)), sizeTy);
        Value * theHashWord = b->CreateSelect(b->CreateICmpEQ(hashWordPhi, sz_ZERO), nextHashWord, hashWordPhi);
        Value * hashWordPos = b->CreateAdd(stridePos, b->CreateMul(hashWordIdx, sw.WIDTH));
        Value * hashPosInWord = b->CreateCountForwardZeroes(theHashWord);
        Value * hashMarkPos = b->CreateAdd(hashWordPos, hashPosInWord, "hashMarkPos");
        Value * hashPfxPos = b->CreateSub(hashMarkPos, lg.MAX_INDEX);

        Value * const hashPfx = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", hashPfxPos)), sizeTy);
        // Build up a single encoded value from the ZTF code sequence.
        Value * encodedVal = b->CreateSub(hashPfx, lg.PREFIX_BASE, "encodedVal");
        Value * curPos = hashPfxPos;
        for (unsigned j = 1; j < lg.groupInfo.encoding_bytes; j++) {
            curPos = b->CreateAdd(curPos, sz_ONE);
            Value * suffixByte = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("byteData", curPos)), sizeTy);
            encodedVal = b->CreateOr(b->CreateShl(encodedVal, lg.SUFFIX_BITS), b->CreateAnd(suffixByte, lg.SUFFIX_MASK), "encodedVal");
        }
        Value * hashCode = b->CreateAnd(encodedVal, lg.HASH_MASK, "hashCode");
        Value * extensionCode = b->CreateAnd(b->CreateLShr(encodedVal, lg.HASH_BITS), lg.EXTENSION_MASK, "extensionCode");
        Value * symStartPos = b->CreateSub(hashMarkPos, sz_MARK_OFFSET, "symStartPos");
        //b->CallPrintInt("hashCode", hashCode);
        //b->CallPrintInt("symStartPos", symStartPos);
        Value * tableNo = b->CreateAdd(b->getSize(i), extensionCode, "tableNo");
        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), tableNo, b->CreateMul(hashCode, lg.HI)});
        std::vector<Value *> dictSym = MonitoredScalarLoadSymbol(b, "hashTable", tblEntryPtr, length);
        Value * destPtr = b->getRawOutputPointer("result", b->getInt32(0), symStartPos);
        storeSymbol(b, dictSym, destPtr, length);

        Value * dropHash = b->CreateResetLowestBit(theHashWord);
        Value * hashMaskDone = b->CreateICmpEQ(dropHash, sz_ZERO);
        // There may be more hashs in the hash mask.
        Value * nextHashMask = b->CreateSelect(hashMaskDone, b->CreateResetLowestBit(hashMaskPhi), hashMaskPhi);
        BasicBlock * hashBB = b->GetInsertBlock();

        hashMaskPhi->addIncoming(nextHashMask, hashBB);
        hashWordPhi->addIncoming(dropHash, hashBB);
        b->CreateCondBr(b->CreateICmpNE(nextHashMask, sz_ZERO), hashProcessingLoop, loopExit);
        b->SetInsertPoint(loopExit);
    }
}

FixedLengthDecompression::FixedLengthDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                   EncodingInfo encodingScheme,
                                                   unsigned lo,
                                                   StreamSet * const byteData,
                                                   StreamSet * const hashValues,
                                                   std::vector<StreamSet *> keyMarks,
                                                   std::vector<StreamSet *> hashMarks,
                                                   StreamSet * const result, unsigned strideBlocks)
: MultiBlockKernel(b, "FixedLengthDecompression" + lengthRangeSuffix(encodingScheme, lo, lo + keyMarks.size() - 1),
                   {ByteDataBinding(lo, byteData), Binding{"hashValues", hashValues}}, {}, {}, {}, {}),
mEncodingScheme(encodingScheme), mLo(lo), mHi(lo + keyMarks.size() - 1)  {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    assert(keyMarks.size() == hashMarks.size() && "keyMarks and hashMarks vectors mismatched");
    for (unsigned i = 0; i < keyMarks.size(); i++) {
        mInputStreamSets.emplace_back("keyMarks" + std::to_string(i), keyMarks[i]);
    }
    for (unsigned i = 0; i < hashMarks.size(); i++) {
        mInputStreamSets.emplace_back("hashMarks" + std::to_string(i), hashMarks[i]);
    }
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("result", result, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("result", result, BoundedRate(0,1));
    }
    mInternalScalars.emplace_back(ArrayType::get(b->getInt8Ty(), encodingScheme.maxSymbolLength()), "pendingOutput");
    mSubTableSize = 0;
    for (unsigned i = 0; i < keyMarks.size(); i++) {
        unsigned lgth = lo + i;
        size_t subTableSize = hashTableSize(encodingScheme, lgth);
        if (subTableSize > mSubTableSize) mSubTableSize = subTableSize;
    }
    Type * subTableType = ArrayType::get(b->getInt8Ty(), mSubTableSize);
    mInternalScalars.emplace_back(ArrayType::get(subTableType, mHi - mLo + 1), "hashTable");
}

void FixedLengthDecompression::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    ScanWordParameters sw(b, mStride);
    Constant * sz_DELAYED = b->getSize(mEncodingScheme.maxSymbolLength());
    unsigned numOfMasks = mHi - mLo + 1;

    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Type * sizeTy = b->getSizeTy();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const hashesDone = b->CreateBasicBlock("hashesDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");

    Value * const initialPos = b->getProcessedItemCount("keyMarks0");
    Value * const avail = b->getAvailableItemCount("keyMarks0");
    if (!DelayedAttribute) {
        // Copy pending output data.
        Value * const initialProduced = b->getProducedItemCount("result");
        b->CreateMemCpy(b->getRawOutputPointer("result", initialProduced), b->getScalarFieldPtr("pendingOutput"), sz_DELAYED, 1);
    }
    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateMul(numOfStrides, sz_STRIDE);
    b->CreateMemCpy(b->getRawOutputPointer("result", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);
    Value * hashTablePtr = b->getScalarFieldPtr("hashTable");
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);

    std::vector<Value *> keyMasks(numOfMasks);
    std::vector<Value *> hashMasks(numOfMasks);
    initializeDecompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, numOfMasks, strideBlockOffset, keyMasks, hashMasks, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    // First iterate through the new keys and update the hash table as
    // appropriate.   Each key is hashed, and is entered into the hash
    // table if there is not already an entry for that hash code.
    generateDecompKeyProcessingLoops(b, sw, mEncodingScheme, mLo, keyMasks, hashTablePtr, strideBlockOffset, stridePos, keysDone);

    b->SetInsertPoint(keysDone);
    generateHashProcessingLoops(b, sw, mEncodingScheme, mLo, hashMasks, hashTablePtr, strideBlockOffset, stridePos, hashesDone);

    b->SetInsertPoint(hashesDone);
    strideNo->addIncoming(nextStrideNo, hashesDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // If the segment ends in the middle of a 2-byte codeword, we need to
    // make sure that we still have access to the codeword in the next block.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(mIsFinal, avail, b->CreateSub(avail, sz_DELAYED));
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the full input stream to output, there may
    // be an incomplete symbol at the end of this block.   Store the
    // data that may be overwritten as pending and set the produced item
    // count to that which is guaranteed to be correct.
    Value * guaranteedProduced = b->CreateSub(avail, sz_DELAYED);
    //b->CreateMemCpy(b->getScalarFieldPtr("pendingOutput"), b->getRawOutputPointer("result", guaranteedProduced), sz_DELAYED, 1);
    if (!DelayedAttribute) {
        b->setProducedItemCount("result", b->CreateSelect(mIsFinal, avail, guaranteedProduced));
    }
}
