/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <llvm/IR/Function.h>                      // for Function, Function...
#include <llvm/IR/Module.h>                        // for Module
#include <llvm/Support/CommandLine.h>              // for ParseCommandLineOp...
#include <llvm/Support/Debug.h>                    // for dbgs
#include <kernel/core/kernel_builder.h>
#include <kernel/core/streamset.h>
#include <llvm/ADT/StringRef.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/intrusive/detail/math.hpp>
#include "ztf-scan.h"

#if 0
#define DEBUG_PRINT(title,value) b->CallPrintInt(title, value)
#else
#define DEBUG_PRINT(title,value)
#endif

using namespace kernel;
using namespace llvm;

static cl::opt<bool> DeferredAttribute("deferred", cl::desc("Use Deferred attribute instead of Lookbehind for source data"), cl::init(false));
static cl::opt<bool> DelayedAttribute("delayed", cl::desc("Use Delayed Attribute instead of BoundedRate for output"), cl::init(true));

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
    Constant * HASH_MASK;
    Constant * ENC_BYTES;
    Constant * MAX_INDEX;
    Constant * PREFIX_BASE;

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
        HASH_BITS(b->getSize(encodingScheme.MAX_HASH_BITS)),
        HASH_MASK(b->getSize((1 << groupInfo.hash_bits) - 1)),
        ENC_BYTES(b->getSize(groupInfo.encoding_bytes)),
        MAX_INDEX(b->getSize(groupInfo.encoding_bytes - 1)),
        PREFIX_BASE(b->getSize(groupInfo.prefix_base)) {
            assert(groupInfo.hi <= (1 << (boost::intrusive::detail::floor_log2(groupInfo.lo) + 1)));
        }
};

unsigned hashTableSize(LengthGroupInfo g) {
    unsigned numSubTables = (g.hi - g.lo + 1);
    return numSubTables * g.hi * (1<<g.hash_bits);
}

std::string lengthGroupStr(LengthGroupInfo g) {
    return std::to_string(g.lo) + "-" + std::to_string(g.hi);
}

Binding ByteDataBinding(unsigned max_length, StreamSet * byteData) {
    if (DeferredAttribute) {
        return Binding{"byteData", byteData, FixedRate(), { Deferred() }}; // , Linear()
    } else {
        return Binding{"byteData", byteData, FixedRate(), { LookBehind(max_length+1) }}; // , Linear()
    }
}

LengthGroupCompressionMask::LengthGroupCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                       EncodingInfo encodingScheme,
                                                       unsigned groupNo,
                                                       StreamSet * symbolMarks,
                                                       StreamSet * hashValues,
                                                       StreamSet * const byteData, StreamSet * compressionMask, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupCompressionMask" + lengthGroupStr(encodingScheme.byLength[groupNo]) + (DeferredAttribute ? "deferred" : "lookBehind") + (DelayedAttribute ? "_delayed" : "_bounded"),
                   {Binding{"symbolMarks", symbolMarks},
                       Binding{"hashValues", hashValues},
                       ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)},
                   {Binding{"compressionMask", compressionMask, BoundedRate(0,1)}}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo])), "hashTable"}}),
    mEncodingScheme(encodingScheme), mGroupNo(groupNo) {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

void LengthGroupCompressionMask::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    LengthGroupParameters lg(b, mEncodingScheme, mGroupNo);
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_BLOCKWIDTH = b->getSize(b->getBitBlockWidth());
    Type * sizeTy = b->getSizeTy();
    Type * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();

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
    /* Determine the key length. */
    Value * const hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);
    Value * keyLength = b->CreateAdd(b->CreateLShr(hashValue, lg.MAX_HASH_BITS), lg.ENC_BYTES, "keyLength");
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, lg.MAX_INDEX), "keyStartPos");
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, lg.HALF_LENGTH);
    // Get the hash of this key.
    Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");
    //b->CallPrintInt("keyHash", keyHash);
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
    Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1, entry2), Constant::getNullValue(lg.halfLengthTy));
    b->CreateCondBr(isEmptyEntry, storeKey, checkKey);

    b->SetInsertPoint(storeKey);
    // We have a new symbol that allows future occurrences of the symbol to
    // be compressed using the hash code.
    b->CreateMonitoredScalarFieldStore("hashTable", sym1, tblPtr1);
    b->CreateMonitoredScalarFieldStore("hashTable", sym2, tblPtr2);
    b->CreateBr(nextKey);

    b->SetInsertPoint(checkKey);
    // If the symbol is equal to the stored entry, it will be replace
    // by the ZTF compression code.  Prepare the compression mask by
    // zeroing out all symbol positions except the last two.
    Value * symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(entry1, sym1), b->CreateICmpEQ(entry2, sym2));
    b->CreateCondBr(symIsEqEntry, markCompression, nextKey);

    b->SetInsertPoint(markCompression);
    // Compute a mask of bits, with zeroes marking positions to eliminate.
    // The entire symbols will be replaced, but we need to keep the required
    // number of positions for the encoded ZTF sequence.
    Value * maskLength = b->CreateZExt(b->CreateSub(keyLength, lg.ENC_BYTES), sizeTy);
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


LengthGroupDecompression::LengthGroupDecompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                   EncodingInfo encodingScheme,
                                                   unsigned groupNo,
                                                   StreamSet * keyMarks,
                                                   StreamSet * hashValues,
                                                   StreamSet * const hashMarks, StreamSet * const byteData,
                                                   StreamSet * const result, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupDecompression" + lengthGroupStr(encodingScheme.byLength[groupNo]) + (DeferredAttribute ? "deferred" : "lookBehind") + (DelayedAttribute ? "_delayed" : "_bounded"),
                   {Binding{"keyMarks", keyMarks},
                       Binding{"hashValues", hashValues},
                       Binding{"hashMarks", hashMarks},
                       ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)
                   },
                   {}, {}, {},
                   {InternalScalar{ArrayType::get(b->getInt8Ty(), encodingScheme.byLength[groupNo].hi), "pendingOutput"},
                       // Hash table 8 length-based tables with 256 16-byte entries each.
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo])), "hashTable"}}),
    mEncodingScheme(encodingScheme), mGroupNo(groupNo) {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("result", result, FixedRate(), Delayed(encodingScheme.byLength[groupNo].hi) );
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
    Type * sizeTy = b->getSizeTy();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const stridePrecomputation = b->CreateBasicBlock("stridePrecomputation");
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

    Value * const initialPos = b->getProcessedItemCount("keyMarks");
    Value * const avail = b->getAvailableItemCount("keyMarks");

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
    Value * keyLength = b->CreateAdd(b->CreateLShr(hashValue, lg.MAX_HASH_BITS), lg.ENC_BYTES, "keyLength");
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, lg.MAX_INDEX), "keyStartPos");
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
    Value * symLength = b->CreateAdd(b->CreateLShr(encodedVal, lg.HASH_BITS), lg.LO, "symLength");
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
