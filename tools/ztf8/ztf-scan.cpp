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
#include "ztf-scan.h"

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

Binding ByteDataBinding(unsigned max_length, StreamSet * byteData) {
    if (DeferredAttribute) {
        return Binding{"byteData", byteData, FixedRate(), { Deferred() }}; // , Linear()
    } else {
        return Binding{"byteData", byteData, FixedRate(), { LookBehind(max_length) }}; // , Linear()
    }
}

LengthGroupCompressionMask::LengthGroupCompressionMask(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                       LengthGroup lengthGroup,
                                                       StreamSet * symbolMarks,
                                                       StreamSet * symbolLengths,
                                                       StreamSet * const byteData, StreamSet * const hashValues, StreamSet * compressionMask, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupCompressionMask" + lengthGroupStr(lengthGroup) + (DeferredAttribute ? "deferred" : "lookBehind") + (DelayedAttribute ? "_delayed" : "_bounded"),
                   {Binding{"symbolMarks", symbolMarks},
                       Binding{"symbolLengths", symbolLengths},
                       ByteDataBinding(lengthGroup.hi, byteData),
                       Binding{"hashValues", hashValues}},
                   {Binding{"compressionMask", compressionMask, BoundedRate(0,1)}}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(lengthGroup)), "hashTable"}}),
mLengthGroup(lengthGroup) {
    checkLengthGroup(lengthGroup);
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

void LengthGroupCompressionMask::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
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
    b->CreateStore(pendingMask, producedPtr);
    Value * compressMaskPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialPos), bitBlockPtrTy);
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
    Value * const lgthPtr = b->getRawInputPointer("symbolLengths", keyMarkPos);
    Value * keyLength = b->CreateAdd(b->CreateZExt(b->CreateLoad(lgthPtr), sizeTy), sz_TWO);
    Value * keyStartPos = b->CreateSub(keyMarkPos, b->CreateSub(keyLength, sz_ONE));
    // keyOffset for accessing the final half of an entry.
    Value * keyOffset = b->CreateSub(keyLength, sz_HALF_SYM);
    // Get the hash of this key.
    Value * const keyPtr = b->getRawInputPointer("hashValues", keyMarkPos);
    Value * keyHash = b->CreateZExt(b->CreateLoad(keyPtr), sizeTy, "keyHash");
    // Starting with length 9, the length-based subtables are 16 * 256 = 4K each.
    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());
    Value * hashTablePtr = b->CreateGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength, sz_MINLENGTH), sz_SUBTABLE));
    Value * tblEntryPtr = b->CreateGEP(hashTablePtr, b->CreateMul(keyHash, sz_MAXLENGTH));
    // Use two 8-byte loads to get hash and symbol values.
    //b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    Value * tblPtr1 = b->CreateBitCast(tblEntryPtr, halfSymPtrTy);
    Value * tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, keyOffset), halfSymPtrTy);
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", keyStartPos), halfSymPtrTy);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos, keyOffset)), halfSymPtrTy);
    // Check to see if the hash table entry is nonzero (already assigned).
    Value * sym1 = b->CreateLoad(symPtr1);
    Value * sym2 = b->CreateLoad(symPtr2);
    Value * entry1 = b->CreateLoad(tblPtr1);
    Value * entry2 = b->CreateLoad(tblPtr2);
    Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1, entry2), Constant::getNullValue(halfLengthTy));
    b->CreateCondBr(isEmptyEntry, storeKey, checkKey);

    b->SetInsertPoint(storeKey);
    // We have a new symbol that allows future occurrences of the symbol to
    // be compressed using the hash code.
    b->CreateStore(sym1, tblPtr1);
    b->CreateStore(sym2, tblPtr2);
    b->CreateBr(nextKey);

    b->SetInsertPoint(checkKey);
    // If the symbol is equal to the stored entry, it will be replace
    // by the ZTF compression code.  Prepare the compression mask by
    // zeroing out all symbol positions except the last two.
    Value * symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(entry1, sym1), b->CreateICmpEQ(entry2, sym2));
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
        Value * processed = b->CreateSub(avail, sz_MAXLENGTH);
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
                                                   LengthGroup lengthGroup,
                                                   StreamSet * keyMarks,
                                                   StreamSet * symbolLengths,
                                                   StreamSet * const hashMarks, StreamSet * const byteData,
                                                   StreamSet * const hashValues,
                                                   StreamSet * const result, unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupDecompression" + lengthGroupStr(lengthGroup) + (DeferredAttribute ? "deferred" : "lookBehind") + (DelayedAttribute ? "_delayed" : "_bounded"),
                   {Binding{"keyMarks", keyMarks},
                       Binding{"symbolLengths", symbolLengths},
                       Binding{"hashMarks", hashMarks},
                       ByteDataBinding(lengthGroup.hi, byteData),
                       Binding{"hashValues", hashValues},
                   },
                   {}, {}, {},
                   {// Hash table 8 length-based tables with 256 16-byte entries each.
                    InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(lengthGroup)), "hashTable"}}),
mLengthGroup(lengthGroup) {
    checkLengthGroup(lengthGroup);
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("result", result, FixedRate(), Delayed(mLengthGroup.hi) );
    } else {
        mOutputStreamSets.emplace_back("result", result, BoundedRate(0,1));
    }
    if (!DelayedAttribute) {
        addInternalScalar(ArrayType::get(b->getInt8Ty(), lengthGroup.hi), "pendingOutput");
    }
}

void LengthGroupDecompression::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfStrides) {

    ScanWordParameters sw(b, mStride);

    Constant * sz_STRIDE = b->getSize(mStride);
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth());
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
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
    Value * const avail = b->getAvailableItemCount("keyMarks");

    if (!DelayedAttribute) {
        // Copy pending output data.
        Value * const initialProduced = b->getProducedItemCount("result");
        b->CreateMemCpy(b->getRawOutputPointer("result", initialProduced), b->getScalarFieldPtr("pendingOutput"), sz_MAXLENGTH, 1);
    }

    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateSub(avail, initialPos);
    b->CreateMemCpy(b->getRawOutputPointer("result", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, mStride);
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
    //b->CallPrintInt("keyMask", keyMask);
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
    Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", keyStartPos), halfSymPtrTy);
    //b->CallPrintInt("symPtr1", symPtr1);
    Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos, keyOffset)), halfSymPtrTy);
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
    // b->CallPrintInt("tblEntryPtr", tblEntryPtr);
    tblPtr1 = b->CreateBitCast(tblEntryPtr, halfSymPtrTy);
    //b->CallPrintInt("tblPtr1", tblPtr1);
    tblPtr2 = b->CreateBitCast(b->CreateGEP(tblEntryPtr, symOffset), halfSymPtrTy);
    //b->CallPrintInt("tblPtr2", tblPtr2);
    entry1 = b->CreateLoad(tblPtr1);
    //b->CallPrintInt("entry1", entry1);
    entry2 = b->CreateLoad(tblPtr2);

    //b->CallPrintInt("symStartPos", symStartPos);
    symPtr1 = b->CreateBitCast(b->getRawOutputPointer("result", symStartPos), halfSymPtrTy);

    //b->CallPrintInt("symOffset", symOffset);
    symPtr2 = b->CreateBitCast(b->getRawOutputPointer("result", b->CreateAdd(symStartPos, symOffset)), halfSymPtrTy);

    //b->CallPrintInt("entry1", entry1);
    b->CreateStore(entry1, symPtr1);

    //b->CallPrintInt("entry2", entry2);
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
        Value * processed = b->CreateSub(avail, sz_MAXLENGTH);
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the full input stream to output, there may
    // be an incomplete symbol at the end of this block.   Store the
    // data that may be overwritten as pending and set the produced item
    // count to that which is guaranteed to be correct.
    if (!DelayedAttribute) {
        Value * guaranteedProduced = b->CreateSub(avail, sz_MAXLENGTH);
        b->CreateMemCpy(b->getScalarFieldPtr("pendingOutput"), b->getRawOutputPointer("result", guaranteedProduced), sz_MAXLENGTH, 1);
        b->setProducedItemCount("result", b->CreateSelect(mIsFinal, avail, guaranteedProduced));
    }
}

