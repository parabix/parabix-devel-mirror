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

using BuilderRef = Kernel::BuilderRef;

struct ScanWordParameters {
    unsigned width;
    unsigned indexWidth;
    Type * const Ty;
    Type * const pointerTy;
    Constant * const WIDTH;
    Constant * const ix_MAXBIT;
    Constant * WORDS_PER_BLOCK;
    Constant * WORDS_PER_STRIDE;

    ScanWordParameters(BuilderRef b, unsigned stride) :
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
    Constant * PREFIX_LENGTH_OFFSET;
    Constant * LENGTH_MASK;
    Constant * EXTENSION_MASK;

    LengthGroupParameters(BuilderRef b, EncodingInfo encodingScheme, unsigned groupNo) :
        groupInfo(encodingScheme.byLength[groupNo]),
        MAX_HASH_BITS(b->getSize(encodingScheme.MAX_HASH_BITS)),
        SUFFIX_BITS(b->getSize(7)),
        SUFFIX_MASK(b->getSize(0x7F)),
        groupHalfLength(1UL << boost::intrusive::detail::floor_log2(groupInfo.lo)),
        halfLengthTy(b->getIntNTy(8U * groupHalfLength)),
        halfSymPtrTy(halfLengthTy->getPointerTo()),
        HALF_LENGTH(b->getSize(groupHalfLength)),
        LO(b->getSize(groupInfo.lo)),
        HI(b->getSize(groupInfo.hi)),
        // All subtables are sized the same.
        SUBTABLE_SIZE(b->getSize((1UL << groupInfo.hash_bits) * groupInfo.hi)),
        HASH_BITS(b->getSize(groupInfo.hash_bits)),
        EXTENDED_BITS(b->getSize(std::max((groupInfo.hash_bits + groupInfo.length_extension_bits), ((groupInfo.encoding_bytes - 1U) * 7U)))),
        HASH_MASK(b->getSize((1UL << ((groupInfo.hash_bits >> 1UL) * groupInfo.encoding_bytes)) - 1UL)),
        ENC_BYTES(b->getSize(groupInfo.encoding_bytes)),
        MAX_INDEX(b->getSize(groupInfo.encoding_bytes - 1UL)),
        PREFIX_BASE(b->getSize(groupInfo.prefix_base)),
        PREFIX_LENGTH_OFFSET(b->getSize(encodingScheme.prefixLengthOffset(groupInfo.lo))),
        LENGTH_MASK(b->getSize(2UL * groupHalfLength - 1UL)),
        EXTENSION_MASK(b->getSize((1UL << groupInfo.length_extension_bits) - 1UL)) {
            assert(groupInfo.hi <= (1UL << (boost::intrusive::detail::floor_log2(groupInfo.lo) + 1UL)));
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

std::vector<Value *> initializeCompressionMasks(BuilderRef b,
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
        Value * keyBitBlock = b->loadInputStreamBlock("symbolMarks" + (i > 0 ? std::to_string(i) : ""), sz_ZERO, strideBlockIndex);
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

LengthGroupCompression::LengthGroupCompression(BuilderRef b,
                                               EncodingInfo encodingScheme,
                                               unsigned groupNo,
                                               StreamSet * symbolMarks,
                                               StreamSet * hashValues,
                                               StreamSet * const byteData,
                                               StreamSet * compressionMask,
                                               StreamSet * encodedBytes,
                                               unsigned strideBlocks)
: MultiBlockKernel(b, "LengthGroupCompression" + lengthGroupSuffix(encodingScheme, groupNo) + (PrefixCheck ? "_prefix" : ""),
                   {Binding{"symbolMarks", symbolMarks},
                       Binding{"hashValues", hashValues},
                       ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)},
                   {}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                       InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo])), "hashTable"}}),
mEncodingScheme(encodingScheme), mGroupNo(groupNo) {
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, BoundedRate(0,1));
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, BoundedRate(0,1));
        addInternalScalar(ArrayType::get(b->getInt8Ty(), encodingScheme.byLength[groupNo].hi), "pendingOutput");
    }
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

void LengthGroupCompression::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    LengthGroupParameters lg(b, mEncodingScheme, mGroupNo);
    Constant * sz_DELAYED = b->getSize(mEncodingScheme.maxSymbolLength());
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
    if (!DelayedAttribute) {
        // Copy pending output data.
        Value * const initialProduced = b->getProducedItemCount("result");
        b->CreateMemCpy(b->getRawOutputPointer("encodedBytes", initialProduced), b->getScalarFieldPtr("pendingOutput"), sz_DELAYED, 1);
    }
    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateMul(numOfStrides, sz_STRIDE);
    b->CreateMemCpy(b->getRawOutputPointer("encodedBytes", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);
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
    Value * curPos = keyMarkPos;
    Value * curHash = keyHash;  // Add hash extension bits later.
    // Write the suffixes.
    for (unsigned i = 0; i < lg.groupInfo.encoding_bytes - 1; i++) {
        Value * ZTF_suffix = b->CreateTrunc(b->CreateAnd(curHash, lg.SUFFIX_MASK, "ZTF_suffix"), b->getInt8Ty());
        b->CreateStore(ZTF_suffix, b->getRawOutputPointer("encodedBytes", curPos));
        curPos = b->CreateSub(curPos, sz_ONE);
        curHash = b->CreateLShr(curHash, lg.SUFFIX_BITS);
    }
    // Now prepare the prefix - PREFIX_BASE + ... + remaining hash bits.
    Value * lgthBase = b->CreateShl(b->CreateSub(keyLength, lg.LO), lg.PREFIX_LENGTH_OFFSET);
    Value * ZTF_prefix = b->CreateAdd(b->CreateAdd(lg.PREFIX_BASE, lgthBase), curHash, "ZTF_prefix");
    b->CreateStore(b->CreateTrunc(ZTF_prefix, b->getInt8Ty()), b->getRawOutputPointer("encodedBytes", curPos));
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
    Value * produced = b->CreateSelect(b->isFinal(), avail, b->CreateSub(avail, sz_BLOCKWIDTH));
    b->setProducedItemCount("compressionMask", produced);
    b->CreateCondBr(b->isFinal(), compressionMaskDone, updatePending);
    b->SetInsertPoint(updatePending);
    Value * pendingPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", produced), bitBlockPtrTy);
    //b->CallPrintInt("pendingPtr", pendingPtr);
    Value * lastMask = b->CreateBlockAlignedLoad(pendingPtr);
    b->setScalarField("pendingMaskInverted", b->CreateNot(lastMask));
    b->CreateBr(compressionMaskDone);
    b->SetInsertPoint(compressionMaskDone);
}

void initializeDecompressionMasks(BuilderRef b,
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
        Value * keyBitBlock = b->loadInputStreamBlock("keyMarks" + std::to_string(i), sz_ZERO, strideBlockIndex);
        Value * hashBitBlock = b->loadInputStreamBlock("hashMarks" + std::to_string(i), sz_ZERO, strideBlockIndex);
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


LengthGroupDecompression::LengthGroupDecompression(BuilderRef b,
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

void LengthGroupDecompression::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {

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
        b->setProducedItemCount("result", b->CreateSelect(b->isFinal(), avail, guaranteedProduced));
    }
}

unsigned hashTableSize(EncodingInfo info, unsigned lgth) {
    LengthGroupInfo g = info.byLength[info.getLengthGroupNo(lgth)];
    return lgth * (1 << g.hash_bits);
}

//
// Length-based symbol operations - for symbol length known at LLVM compile time.
//
std::vector<Value *> MonitoredScalarLoadSymbol(BuilderRef b, std::string scalarName, Value * sourcePtr, unsigned length) {
    unsigned load_length = 1U << boost::intrusive::detail::floor_log2(length);
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

std::vector<Value *> loadSymbol(BuilderRef b, Value * sourcePtr, unsigned length) {
    unsigned load_length = 1U << boost::intrusive::detail::floor_log2(length);
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

void MonitoredScalarStoreSymbol(BuilderRef b, std::string scalarName, std::vector<Value *> toStore, Value * ptr, unsigned length) {
    unsigned store_length = 1U << boost::intrusive::detail::floor_log2(length);
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

void storeSymbol(BuilderRef b, std::vector<Value *> toStore, Value * ptr, unsigned length) {
    unsigned store_length = 1U << boost::intrusive::detail::floor_log2(length);
    Type * storePtrTy = b->getIntNTy(store_length * 8)->getPointerTo();
    b->CreateAlignedStore(toStore[0], b->CreateBitCast(ptr, storePtrTy), 1);
    if (store_length == length) {
        return;
    }
    Constant * offset = b->getInt32(length - store_length);
    Value * ptr2 = b->CreateGEP(b->CreateBitCast(ptr, b->getInt8PtrTy()), offset);
    b->CreateAlignedStore(toStore[1], b->CreateBitCast(ptr2, storePtrTy), 1);
}

Value * compareSymbols (BuilderRef b, std::vector<Value *> sym1, std::vector<Value *> sym2) {
    if (sym1.size() == 1) {
        return b->CreateICmpEQ(sym1[0], sym2[0]);
    }
    return b->CreateAnd(b->CreateICmpEQ(sym1[0], sym2[0]), b->CreateICmpEQ(sym1[1], sym2[1]));
}

Value * isNullSymbol (BuilderRef b, std::vector<Value *> sym) {
    Constant * symNull = Constant::getNullValue(sym[0]->getType());
    if (sym.size() == 1) {
        return b->CreateICmpEQ(sym[0], symNull);
    }
    return b->CreateICmpEQ(b->CreateOr(sym[0], sym[1]), symNull);
}

void generateKeyProcessingLoops(BuilderRef b,
                                const ScanWordParameters & sw,
                                const EncodingInfo & encodingScheme,
                                const unsigned lo,
                                const std::vector<Value *> & keyMasks,
                                Value * strideBlockOffset,
                                Value * stridePos,
                                BasicBlock * keysDone) {

    const auto maskCount = keyMasks.size();
    const auto maxLength = lo + maskCount - 1;
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Type * sizeTy = b->getSizeTy();
    for (unsigned length = lo; length <= maxLength; length++) {
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * LGTH_IDX = b->getInt32(length - lo);
        Constant * sz_COMPRESSION_MASK = b->getSize((1 << (length - lg.groupInfo.encoding_bytes)) - 1);
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * entryBlock = b->GetInsertBlock();
        BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
        BasicBlock * const markCompression = b->CreateBasicBlock("markCompression");
        BasicBlock * const tryStore = b->CreateBasicBlock("tryStore");
        BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
        BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
        BasicBlock * loopExit;
        if (length == maxLength) {
            loopExit = keysDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("loopExit");
        }
        Value * hashTablePtr = b->getScalarFieldPtr("hashTable");
        Value * extensionMapPtr = b->getScalarFieldPtr("prefixMapTable");
        Value * keyWordBasePtr = b->getInputStreamBlockPtr("symbolMarks" + (length > lo ? std::to_string(length-lo) : ""), sz_ZERO, strideBlockOffset);
        keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
        b->CreateUnlikelyCondBr(b->CreateICmpEQ(keyMasks[length-lo], sz_ZERO), loopExit, keyProcessingLoop);

        b->SetInsertPoint(keyProcessingLoop);
        PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
        keyMaskPhi->addIncoming(keyMasks[length-lo], entryBlock);
        PHINode * const keyWordPhi = b->CreatePHI(sizeTy, 2);
        keyWordPhi->addIncoming(sz_ZERO, entryBlock);
        Value * keyWordIdx = b->CreateCountForwardZeroes(keyMaskPhi, "keyWordIdx");

        Value * nextKeyWord = b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keyWordBasePtr, keyWordIdx)), sizeTy);
        Value * theKeyWord = b->CreateSelect(b->CreateICmpEQ(keyWordPhi, sz_ZERO), nextKeyWord, keyWordPhi);
        Value * keyWordPos = b->CreateAdd(stridePos, b->CreateMul(keyWordIdx, sw.WIDTH));
        Value * keyMarkPosInWord = b->CreateCountForwardZeroes(theKeyWord);
        Value * keyMarkPos = b->CreateAdd(keyWordPos, keyMarkPosInWord, "keyEndPos");
        Value * const hashValue = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", keyMarkPos)), sizeTy);

        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(b->CreateICmpUGE(keyMarkPos, sz_MARK_OFFSET),
                            "keyMarkPos (%d) is less than sz_MARK_OFFSET (%d)", keyMarkPos, sz_MARK_OFFSET);
        }

        Value * keyStartPos = b->CreateSub(keyMarkPos, sz_MARK_OFFSET, "keyStartPos");
        Value * keyHash = b->CreateAnd(hashValue, lg.HASH_MASK, "keyHash");
        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), LGTH_IDX, b->CreateMul(keyHash, lg.HI)});
        Value * symPtr = b->getRawInputPointer("byteData", b->getInt32(0), keyStartPos);

        // Check to see if the hash table entry is nonzero (already assigned).
        std::vector<Value *> sym = loadSymbol(b, symPtr, length);
        std::vector<Value *> entry = MonitoredScalarLoadSymbol(b, "hashTable", tblEntryPtr, length);

        Value * symIsEqEntry = compareSymbols(b, sym, entry);
        Value * curHash = keyHash;
        if (length == maxLength) {
            b->CreateCondBr(symIsEqEntry, markCompression, tryStore);
            b->SetInsertPoint(markCompression);
        } else {
            BasicBlock * const tryExtension = b->CreateBasicBlock("tryExtension");
            BasicBlock * const checkExtension = b->CreateBasicBlock("checkExtension");
            b->CreateCondBr(symIsEqEntry, markCompression, tryExtension);

            b->SetInsertPoint(tryExtension);
            // Check for an extension entry of which the current key is a prefix.
            // First load the extensionMap entry for the current hash value and check it.
            Value * extensionMapEntry = b->CreateGEP(extensionMapPtr, {b->getInt32(0), LGTH_IDX, keyHash});
            Value * extensionHash = b->CreateZExt(b->CreateMonitoredScalarFieldLoad("prefixMapTable", extensionMapEntry), sizeTy);
            b->CreateCondBr(b->CreateIsNull(extensionHash), tryStore, checkExtension);

            b->SetInsertPoint(checkExtension);
            // The extension entry is in the hash table for the next highest length.
            LengthGroupParameters eg(b, encodingScheme, encodingScheme.getLengthGroupNo(length+1));
            Value * extEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), b->getInt32(length - lo + 1), b->CreateMul(extensionHash, eg.HI)});
            std::vector<Value *> extEntry = MonitoredScalarLoadSymbol(b, "hashTable", extEntryPtr, length);
#if 0
            BasicBlock * const printExtension = b->CreateBasicBlock("printExtension");
            b->CreateCondBr(compareSymbols(b, sym, extEntry), printExtension, tryStore);
            b->SetInsertPoint(printExtension);
            b->CallPrintInt("extensionHash_" + std::to_string(length + 1) , extensionHash);
            b->CallPrintInt("keyHash", keyHash);
            b->CreateBr(markCompression);
#else
            b->CreateCondBr(compareSymbols(b, sym, extEntry), markCompression, tryStore);
#endif
            b->SetInsertPoint(markCompression);
            PHINode * const encodingHash = b->CreatePHI(sizeTy, 2);
            encodingHash->addIncoming(keyHash, keyProcessingLoop);
            encodingHash->addIncoming(extensionHash, checkExtension);
            PHINode * const extensionVal = b->CreatePHI(sizeTy, 2);
            extensionVal->addIncoming(sz_ZERO, keyProcessingLoop);
            extensionVal->addIncoming(sz_ONE, checkExtension);
            curHash = b->CreateAdd(encodingHash, b->CreateShl(extensionVal, lg.HASH_BITS));
        }
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
        Value * curPos = keyMarkPos;
        // Write the suffixes.
        for (unsigned i = 0; i < lg.groupInfo.encoding_bytes - 1; i++) {
            Value * ZTF_suffix = b->CreateTrunc(b->CreateAnd(curHash, lg.SUFFIX_MASK, "ZTF_suffix"), b->getInt8Ty());
            b->CreateStore(ZTF_suffix, b->getRawOutputPointer("encodedBytes", curPos));
            curPos = b->CreateSub(curPos, sz_ONE);
            curHash = b->CreateLShr(curHash, lg.SUFFIX_BITS);
        }
        // Now prepare the prefix - PREFIX_BASE + ... + remaining hash bits.
        Value * ZTF_prefix = b->CreateTrunc(b->CreateAdd(lg.PREFIX_BASE, curHash, "ZTF_prefix"), b->getInt8Ty());
        b->CreateStore(ZTF_prefix, b->getRawOutputPointer("encodedBytes", curPos));
        b->CreateBr(nextKey);

        b->SetInsertPoint(tryStore);
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
        // Now store a prefixMap entry, if possible.
        if (length > lo) {
            BasicBlock * const storePrefix = b->CreateBasicBlock("storePrefix");
            Value * priorPos =  b->CreateSub(keyMarkPos, sz_ONE);
            Value * prefixHash = b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues", priorPos)), sizeTy);
            prefixHash = b->CreateAnd(prefixHash, lg.HASH_MASK, "prefixHash");
            Value * extensionMapEntry = b->CreateGEP(extensionMapPtr, {b->getInt32(0), b->getInt32(length-lo-1), prefixHash});
            Value * storedHash = b->CreateMonitoredScalarFieldLoad("prefixMapTable", extensionMapEntry);
            b->CreateCondBr(b->CreateIsNull(storedHash), storePrefix, nextKey);
            b->SetInsertPoint(storePrefix);
            b->CreateMonitoredScalarFieldStore("prefixMapTable", b->CreateTrunc(keyHash, b->getInt16Ty()), extensionMapEntry);
        }
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

FixedLengthCompression::FixedLengthCompression(BuilderRef b,
                                               EncodingInfo encodingScheme,
                                               unsigned lo,
                                               StreamSet * const byteData,
                                               StreamSet * hashValues,
                                               std::vector<StreamSet *> symbolMarks,
                                               StreamSet * compressionMask,
                                               StreamSet * encodedBytes,
                                               unsigned strideBlocks)
: MultiBlockKernel(b, "FixedLengthCompression" + lengthRangeSuffix(encodingScheme, lo, lo + symbolMarks.size() - 1),
                   {ByteDataBinding(lo, byteData), Binding{"hashValues", hashValues}}, {}, {}, {}, {}),
mEncodingScheme(encodingScheme), mLo(lo), mHi(lo + symbolMarks.size() - 1)  {
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
    for (unsigned i = 0; i < symbolMarks.size(); i++) {
        mInputStreamSets.emplace_back("symbolMarks" + (i > 0 ? std::to_string(i) : ""), symbolMarks[i]);
    }
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, BoundedRate(0,1));
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, BoundedRate(0,1));
        addInternalScalar(ArrayType::get(b->getInt8Ty(), mHi), "pendingOutput");
    }
    mInternalScalars.emplace_back(b->getBitBlockType(), "pendingMaskInverted");
    mSubTableSize = 0;
    for (unsigned i = 0; i < symbolMarks.size(); i++) {
        unsigned lgth = lo + i;
        size_t subTableSize = hashTableSize(encodingScheme, lgth);
        if (subTableSize > mSubTableSize) mSubTableSize = subTableSize;
    }
    Type * subTableType = ArrayType::get(b->getInt8Ty(), mSubTableSize);
    mInternalScalars.emplace_back(ArrayType::get(subTableType, mHi - mLo + 1), "hashTable");
    Type * prefixMapTableType  = ArrayType::get(b->getInt16Ty(), 1 << encodingScheme.MAX_HASH_BITS);
    mInternalScalars.emplace_back(ArrayType::get(prefixMapTableType, mHi - mLo + 1), "prefixMapTable");
}

void FixedLengthCompression::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    unsigned numOfMasks = mHi - mLo + 1;

    Constant * sz_DELAYED = b->getSize(mEncodingScheme.maxSymbolLength());
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
    if (!DelayedAttribute) {
        // Copy pending output data.
        Value * const initialProduced = b->getProducedItemCount("result");
        b->CreateMemCpy(b->getRawOutputPointer("encodedBytes", initialProduced), b->getScalarFieldPtr("pendingOutput"), sz_DELAYED, 1);
    }
    // Copy all new input to the output buffer; this will be then
    // overwritten when and as necessary for decompression of ZTF codes.
    Value * toCopy = b->CreateMul(numOfStrides, sz_STRIDE);
    b->CreateMemCpy(b->getRawOutputPointer("encodedBytes", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);
    b->CreateBr(stridePrologue);

    b->SetInsertPoint(stridePrologue);
    // Set up the loop variables as PHI nodes at the beginning of each stride.
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    std::vector<Value *> keyMasks = initializeCompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, numOfMasks, strideBlockOffset, compressMaskPtr, strideMasksReady);
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
    generateKeyProcessingLoops(b, sw, mEncodingScheme, mLo, keyMasks, strideBlockOffset, stridePos, keysDone);

    b->SetInsertPoint(keysDone);
    strideNo->addIncoming(nextStrideNo, keysDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // In the next segment, we may need to access byte data in the last
    // 16 bytes of this segment.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(b->isFinal(), avail, b->CreateSub(avail, b->getSize(16)));
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the last block mask, we do not include it as
    // produced, because we may need to update it in the event that there is
    // a compressible symbol starting in this segment and finishing in the next.
    Value * produced = b->CreateSelect(b->isFinal(), avail, b->CreateSub(avail, sz_BLOCKWIDTH));
    b->setProducedItemCount("compressionMask", produced);
    b->CreateCondBr(b->isFinal(), compressionMaskDone, updatePending);
    b->SetInsertPoint(updatePending);
    Value * pendingPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", produced), bitBlockPtrTy);
    //b->CallPrintInt("pendingPtr", pendingPtr);
    Value * lastMask = b->CreateBlockAlignedLoad(pendingPtr);
    b->setScalarField("pendingMaskInverted", b->CreateNot(lastMask));
    b->CreateBr(compressionMaskDone);
    b->SetInsertPoint(compressionMaskDone);
}

void generateDecompKeyProcessingLoops(BuilderRef b,
                                const ScanWordParameters & sw,
                                const EncodingInfo & encodingScheme,
                                const unsigned lo,
                                const std::vector<Value *> & keyMasks,
                                Value * strideBlockOffset,
                                Value * stridePos,
                                BasicBlock * keysDone) {

    unsigned maskCount = keyMasks.size();
    unsigned maxLength = lo + maskCount - 1;
    Constant * sz_ZERO = b->getSize(0);
    Type * sizeTy = b->getSizeTy();
    for (unsigned length = lo; length <= maxLength; length++) {
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * LGTH_IDX = b->getInt32(length - lo);
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * entryBlock = b->GetInsertBlock();
        BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
        BasicBlock * const storeKey = b->CreateBasicBlock("storeKey");
        BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
        BasicBlock * loopExit;
        if (length == maxLength) {
            loopExit = keysDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("loopExit");
        }
        Value * keyWordBasePtr = b->getInputStreamBlockPtr("keyMarks" + std::to_string(length-lo), sz_ZERO, strideBlockOffset);
        keyWordBasePtr = b->CreateBitCast(keyWordBasePtr, sw.pointerTy);
        Value * hashTablePtr = b->getScalarFieldPtr("hashTable");
        b->CreateUnlikelyCondBr(b->CreateICmpEQ(keyMasks[length-lo], sz_ZERO), loopExit, keyProcessingLoop);

        b->SetInsertPoint(keyProcessingLoop);
        PHINode * const keyMaskPhi = b->CreatePHI(sizeTy, 2);
        keyMaskPhi->addIncoming(keyMasks[length-lo], entryBlock);
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

        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), LGTH_IDX, b->CreateMul(keyHash, lg.HI)});
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

void generateHashProcessingLoops(BuilderRef b,
                                 const ScanWordParameters & sw,
                                 const EncodingInfo & encodingScheme,
                                 const unsigned lo,
                                 const std::vector<Value *> & hashMasks,
                                 Value * strideBlockOffset,
                                 Value * stridePos,
                                 BasicBlock * hashesDone) {

    unsigned maskCount = hashMasks.size();
    unsigned maxLength = lo + maskCount - 1;
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Type * sizeTy = b->getSizeTy();

    for (unsigned length = lo; length <= maxLength; length++) {
        LengthGroupParameters lg(b, encodingScheme, encodingScheme.getLengthGroupNo(length));
        Constant * LGTH_IDX = b->getSize(length - lo);
        Constant * sz_MARK_OFFSET = b->getSize(length - 1);
        BasicBlock * entryBlock = b->GetInsertBlock();
        BasicBlock * const hashProcessingLoop = b->CreateBasicBlock("hashProcessingLoop");
        BasicBlock * loopExit;
        if (length == maxLength) {
            loopExit = hashesDone;
        } else {
            // when this key is done, process the next key.
            loopExit = b->CreateBasicBlock("loopExit");
        }
        Value * hashTablePtr = b->getScalarFieldPtr("hashTable");
        Value * hashWordBasePtr = b->getInputStreamBlockPtr("hashMarks" + std::to_string(length-lo), sz_ZERO, strideBlockOffset);
        hashWordBasePtr = b->CreateBitCast(hashWordBasePtr, sw.pointerTy);
        b->CreateUnlikelyCondBr(b->CreateICmpEQ(hashMasks[length-lo], sz_ZERO), loopExit, hashProcessingLoop);

        b->SetInsertPoint(hashProcessingLoop);
        PHINode * const hashMaskPhi = b->CreatePHI(sizeTy, 2);
        hashMaskPhi->addIncoming(hashMasks[length-lo], entryBlock);
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
        Value * tableNo = b->CreateAdd(LGTH_IDX, extensionCode, "tableNo");
#if 0
        BasicBlock * const printExtension = b->CreateBasicBlock("printExtension");
        BasicBlock * const continueDecode = b->CreateBasicBlock("continueDecode");
        b->CreateCondBr(b->CreateIsNull(extensionCode), continueDecode, printExtension);
        b->SetInsertPoint(printExtension);
        b->CallPrintInt("hashPfx", hashPfx);
        b->CallPrintInt("hashCode", hashCode);
        b->CallPrintInt("lg.HI", lg.HI);
        b->CallPrintInt("tableNo", tableNo);
        b->CreateBr(continueDecode);
        b->SetInsertPoint(continueDecode);
#endif
        Value * tblEntryPtr = b->CreateGEP(hashTablePtr, {b->getInt32(0), tableNo, b->CreateMul(hashCode, b->CreateAdd(lg.HI, extensionCode))});
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

FixedLengthDecompression::FixedLengthDecompression(BuilderRef b,
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

void FixedLengthDecompression::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {

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
    generateDecompKeyProcessingLoops(b, sw, mEncodingScheme, mLo, keyMasks, strideBlockOffset, stridePos, keysDone);

    b->SetInsertPoint(keysDone);
    generateHashProcessingLoops(b, sw, mEncodingScheme, mLo, hashMasks, strideBlockOffset, stridePos, hashesDone);

    b->SetInsertPoint(hashesDone);
    strideNo->addIncoming(nextStrideNo, hashesDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    // If the segment ends in the middle of a 2-byte codeword, we need to
    // make sure that we still have access to the codeword in the next block.
    if (DeferredAttribute) {
        Value * processed = b->CreateSelect(b->isFinal(), avail, b->CreateSub(avail, sz_DELAYED));
        b->setProcessedItemCount("byteData", processed);
    }
    // Although we have written the full input stream to output, there may
    // be an incomplete symbol at the end of this block.   Store the
    // data that may be overwritten as pending and set the produced item
    // count to that which is guaranteed to be correct.
    Value * guaranteedProduced = b->CreateSub(avail, sz_DELAYED);
    //b->CreateMemCpy(b->getScalarFieldPtr("pendingOutput"), b->getRawOutputPointer("result", guaranteedProduced), sz_DELAYED, 1);
    if (!DelayedAttribute) {
        b->setProducedItemCount("result", b->CreateSelect(b->isFinal(), avail, guaranteedProduced));
    }
}

PhraseCompression::PhraseCompression(BuilderRef b,
                                               EncodingInfo encodingScheme,
                                               unsigned groupNo,
                                               std::vector<StreamSet *> symbolMarks,
                                               std::vector<StreamSet *> hashValues,
                                               StreamSet * const byteData,
                                               StreamSet * compressionMask,
                                               StreamSet * encodedBytes,
                                               StreamSet * compSymSeq,
                                               unsigned strideBlocks)
: MultiBlockKernel(b, "PhraseCompression_" + std::to_string(symbolMarks.size()) + "_" + lengthGroupSuffix(encodingScheme, groupNo),
                   {ByteDataBinding(encodingScheme.byLength[groupNo].hi, byteData)},
                   {}, {}, {},
                   {InternalScalar{b->getBitBlockType(), "pendingMaskInverted"},
                    InternalScalar{ArrayType::get(b->getInt8Ty(), hashTableSize(encodingScheme.byLength[groupNo]/* *unsigned(symbolMarks.size())*/)), "hashTable"}}),
mEncodingScheme(encodingScheme), mGroupNo(groupNo), mNumSym(1) {
    for (unsigned i = 0; i < symbolMarks.size(); i++) {
        mInputStreamSets.emplace_back("symbolMarks" + (i > 0 ? std::to_string(i) : ""), symbolMarks[i]);
    }
    for (unsigned i = 0; i < hashValues.size(); i++) {
        mInputStreamSets.emplace_back("hashValues" + (i > 0 ? std::to_string(i) : ""), hashValues[i]);
    }
    if (DelayedAttribute) {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
        mOutputStreamSets.emplace_back("compSymSeq", compSymSeq, FixedRate(), Delayed(encodingScheme.maxSymbolLength()) );
    } else {
        mOutputStreamSets.emplace_back("compressionMask", compressionMask, BoundedRate(0,1));
        mOutputStreamSets.emplace_back("encodedBytes", encodedBytes, BoundedRate(0,1));
        mOutputStreamSets.emplace_back("compSymSeq", compSymSeq, BoundedRate(0,1));
        addInternalScalar(ArrayType::get(b->getInt8Ty(), encodingScheme.byLength[groupNo].hi), "pendingOutput");
    }
    setStride(std::min(b->getBitBlockWidth() * strideBlocks, SIZE_T_BITS * SIZE_T_BITS));
}

void PhraseCompression::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    ScanWordParameters sw(b, mStride);
    LengthGroupParameters lg(b, mEncodingScheme, mGroupNo);
    Constant * sz_DELAYED = b->getSize(mEncodingScheme.maxSymbolLength());
    Constant * sz_STRIDE = b->getSize(mStride);   //2048
    Constant * sz_BLOCKS_PER_STRIDE = b->getSize(mStride/b->getBitBlockWidth()); //8
    Constant * sz_ZERO = b->getSize(0);
    Constant * sz_ONE = b->getSize(1);
    Constant * sz_TWO = b->getSize(2);
    Constant * sz_BITS = b->getSize(SIZE_T_BITS);
    Constant * sz_BLOCKWIDTH = b->getSize(b->getBitBlockWidth());   //256
    ConstantInt * const i1_FALSE = b->getFalse();
    ConstantInt * const i1_TRUE = b->getTrue();

    Type * sizeTy = b->getSizeTy();
    Type * bitBlockPtrTy = b->getBitBlockType()->getPointerTo();
    Type * const boolTy = b->getInt1Ty();

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const stridePrologue = b->CreateBasicBlock("stridePrologue");
    BasicBlock * const strideMasksReady = b->CreateBasicBlock("strideMasksReady");
    BasicBlock * const keyProcessingLoop = b->CreateBasicBlock("keyProcessingLoop");
    BasicBlock * const keyExtractionLoop = b->CreateBasicBlock("keyExtractionLoop");

    BasicBlock * const proceed = b->CreateBasicBlock("proceed");
    BasicBlock * const findEntry = b->CreateBasicBlock("findEntry");
    BasicBlock * const storeEntryInfo = b->CreateBasicBlock("storeEntryInfo");
    BasicBlock * const tryStore = b->CreateBasicBlock("tryStore");
    BasicBlock * const storeEntry = b->CreateBasicBlock("storeEntry");
    BasicBlock * const markCompression = b->CreateBasicBlock("markCompression");
    BasicBlock * const nextKey = b->CreateBasicBlock("nextKey");
    BasicBlock * const checkOverlappingKey = b->CreateBasicBlock("checkOverlappingKey");
    BasicBlock * const keysDone = b->CreateBasicBlock("keysDone");
    BasicBlock * const stridesDone = b->CreateBasicBlock("stridesDone");
    BasicBlock * const updatePending = b->CreateBasicBlock("updatePending");
    BasicBlock * const compressionMaskDone = b->CreateBasicBlock("compressionMaskDone");

    //common to all the input streams
    Value * initialProduced = b->getProducedItemCount("compressionMask");
    Value * producedPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialProduced), bitBlockPtrTy);

    // test
    Value * initialCompressed = b->getProducedItemCount("compSymSeq");
    Value * cmpProducedPtr = b->CreateBitCast(b->getRawOutputPointer("compSymSeq", initialCompressed), bitBlockPtrTy);
    //b->CallPrintInt("initialCompressed", initialCompressed);

    Value * initialPos = b->getProcessedItemCount("symbolMarks");
    Value * avail = b->getAvailableItemCount("symbolMarks");
    Value * pendingMask = b->CreateNot(b->getScalarField("pendingMaskInverted"));
    b->CreateStore(pendingMask, producedPtr);
    // test
    b->CreateStore(pendingMask, cmpProducedPtr);
    Value * compressMaskPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", initialPos), bitBlockPtrTy);

    //output buffer shared between compression and decompression modes?
    Value * toCopy = b->CreateMul(numOfStrides, sz_STRIDE);
    b->CreateMemCpy(b->getRawOutputPointer("encodedBytes", initialPos), b->getRawInputPointer("byteData", initialPos), toCopy, 1);

    //start processing phrases in each stride in numOfStrides
    b->CreateBr(stridePrologue);
    b->SetInsertPoint(stridePrologue);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);

    //update/ initialize stride position based on # of elements processed per stride
    Value * stridePos = b->CreateAdd(initialPos, b->CreateMul(strideNo, sz_STRIDE));
    Value * strideBlockOffset = b->CreateMul(strideNo, sz_BLOCKS_PER_STRIDE);
    Value * nextStrideNo = b->CreateAdd(strideNo, sz_ONE);

    //compression masks generated once for q symbolMarks streams per stride
    std::vector<Value *> keyMasks;
    keyMasks = initializeCompressionMasks(b, sw, sz_BLOCKS_PER_STRIDE, mNumSym, strideBlockOffset, compressMaskPtr, strideMasksReady);

    b->SetInsertPoint(strideMasksReady);
    //initialize keyword and hashTable pointers
    std::vector<Value * > keywordBasePtr;
    for(unsigned k = 0; k < mNumSym; k++) {
        keywordBasePtr.push_back(b->getInputStreamBlockPtr("symbolMarks" + (k > 0 ? std::to_string(k) : ""), sz_ZERO, strideBlockOffset));
        keywordBasePtr[k] = b->CreateBitCast(keywordBasePtr[k], sw.pointerTy);
    }

    Value * hashTableBasePtr = b->CreateBitCast(b->getScalarFieldPtr("hashTable"), b->getInt8PtrTy());

    Value * allMask = keyMasks[0];
    for(unsigned k = 1; k < mNumSym; k++) {
        allMask = b->CreateOr(allMask, keyMasks[k]);
    }
    b->CreateUnlikelyCondBr(b->CreateICmpEQ(allMask, sz_ZERO), keysDone, keyProcessingLoop);
    b->SetInsertPoint(keyProcessingLoop);

    //keyProcessingLoop variables
    std::vector<PHINode * > keyMaskPhi(mNumSym);
    std::vector<PHINode * > keyWordPhi(mNumSym);
    std::vector<Value *> nextKeyMask;

    std::vector<Value * > keyWordIdx;
    std::vector<Value * > nextKeyWord;
    std::vector<Value * > theKeyWord;
    std::vector<Value * > keyWordPos;
    std::vector<Value * > keyMarkPosInWord;
    std::vector<Value * > keyMarkPos;
    std::vector<Value * > hashValue;
    std::vector<Value * > keyLength;
    std::vector<Value * > keyStartPos;
    std::vector<Value * > keyOffset;
    std::vector<Value * > keyHash;
    std::vector<Value * > hashTablePtr;
    std::vector<Value * > tableEntryPtr;

    SmallVector<PHINode *, 64> currentMask(mNumSym);

    for(unsigned k = 0; k < mNumSym; k++) {
        keyMaskPhi[k] = b->CreatePHI(sizeTy, 2);
        keyMaskPhi[k]->addIncoming(keyMasks[k], strideMasksReady);
        keyWordPhi[k] = b->CreatePHI(sizeTy, 2);
        keyWordPhi[k]->addIncoming(sz_ZERO, strideMasksReady);
    }
    /*for(unsigned k = 0; k < mNumSym; k++) {
        b->CallPrintRegister("keyMaskPhi - before"+std::to_string(k)+std::to_string(mNumSym), keyMaskPhi[k]);
        b->CallPrintInt("keyWordPhi - before"+std::to_string(k)+std::to_string(mNumSym), keyWordPhi[k]);
    }*/

    for (unsigned k = 0; k < mNumSym; k++) {
        PHINode * const curMaskCopy = b->CreatePHI(sizeTy, 2);
        Value * temp = keyMaskPhi[k];
        curMaskCopy->addIncoming(temp, strideMasksReady);
        currentMask[k] = curMaskCopy;
    }

    for(unsigned k = 0; k < mNumSym; k++) {
        //BasicBlock * const entry = b->GetInsertBlock();
        //BasicBlock * const maskDone = b->CreateBasicBlock();
        //b->CreateUnlikelyCondBr(b->CreateICmpEQ(currentMask[k], sz_ZERO), maskDone, keyExtractionLoop);
        //b->SetInsertPoint(keyExtractionLoop);
        // identify keyword/phrase in a stride, its start and end positions, length
        // get the index corresponding to first byte of the phrase
        keyWordIdx.push_back(b->CreateCountForwardZeroes(keyMaskPhi[k], "keyWordIdx"+std::to_string(k)));
        //b->CallPrintInt("keyWordIdx"+std::to_string(k)+std::to_string(mNumSym), keyWordIdx[k]);

        // load the keyword/phrase starting from the identified first byte index

        nextKeyWord.push_back(b->CreateZExtOrTrunc(b->CreateLoad(b->CreateGEP(keywordBasePtr[k], keyWordIdx[k])), sizeTy));
        // select appropriate keyword/phrase to process

        theKeyWord.push_back(b->CreateSelect(b->CreateICmpEQ(keyWordPhi[k], sz_ZERO), nextKeyWord[k], keyWordPhi[k]));
        keyWordPos.push_back(b->CreateAdd(stridePos, b->CreateMul(keyWordIdx[k], sw.WIDTH)));
        // get end  position/index of the keyword/phrase

        keyMarkPosInWord.push_back(b->CreateCountForwardZeroes(theKeyWord[k]));
        //b->CallPrintInt("keyWordPos"+std::to_string(k), keyWordPos[k]);
        //b->CallPrintInt("keyMarkPosInWord"+std::to_string(k), keyMarkPosInWord[k]);
        keyMarkPos.push_back(b->CreateAdd(keyWordPos[k], keyMarkPosInWord[k], "keyEndPos"+std::to_string(k)));

        // get the hashVal bytes corresponding to the length of the keyword/phrase
        hashValue.push_back(b->CreateZExt(b->CreateLoad(b->getRawInputPointer("hashValues" + (k > 0 ? std::to_string(k) : ""), keyMarkPos[k])), sizeTy));

        // calculate keyLength from hashValue's bixnum part
        //b->CallPrintInt("keyMarkPos"+std::to_string(k), keyMarkPos[k]);
        //b->CallPrintInt("hashValue"+std::to_string(k), hashValue[k]);
        keyLength.push_back(b->CreateAdd(b->CreateLShr(hashValue[k], lg.MAX_HASH_BITS), sz_TWO, "keyLength"+std::to_string(k)));

        // get start position of the keyword/phrase
        keyStartPos.push_back(b->CreateSub(keyMarkPos[k], b->CreateSub(keyLength[k], sz_ONE), "keyStartPos"+std::to_string(k)));

        // divide the key into 2 halves for equal bytes load
        keyOffset.push_back(b->CreateSub(keyLength[k], lg.HALF_LENGTH));
        //b->CallPrintInt("keyLength"+std::to_string(k), keyLength[k]);
        //b->CallPrintInt("keyOffset-before"+std::to_string(k), keyOffset[k]);
        // HASH_MASK retrieves mask according to the number of encoding bytes of a keyword
        keyHash.push_back(b->CreateAnd(hashValue[k], lg.HASH_MASK, "keyHash"+std::to_string(k)));

        // Select the correct hashTable based on HASH_MASK or keyLength
        // for lookup or try storing the keyword
        // Every sub-length in a length group has equal sized hashTable.
        // To access the hashTable
        hashTablePtr.push_back(b->CreateInBoundsGEP(hashTableBasePtr, b->CreateMul(b->CreateSub(keyLength[k], lg.LO), lg.SUBTABLE_SIZE)));

        // A position in scalar hashTable is specific to a keyWord/phrase.
        // To get correct HashTable key location: Truncate keyHash to 8 bits HASH_MASK
        Value * tempHash = keyHash[k];
        Value * keyLookupHash = b->CreateAnd(tempHash, b->getSize((1UL << 8 /*lengthGroup.hash_bits*/) - 1UL));
        tableEntryPtr.push_back(b->CreateGEP(hashTablePtr[k], b->CreateMul(keyLookupHash, lg.HI)));
        /*b->CreateBr(maskDone);

        b->SetInsertPoint(maskDone);
        PHINode * const maskUpdate = b->CreatePHI(sizeTy, 3);
        maskUpdate->addIncoming(currentMask[k], entry);
        maskUpdate->addIncoming(sz_ZERO, keyExtractionLoop);
        currentMask[k] = maskUpdate;*/
    }

    //b->CreateBr(proceed);
    //b->SetInsertPoint(proceed);
    std::vector<Value * > entry1;
    std::vector<Value * > entry2;
    std::vector<Value * > sym1;
    std::vector<Value * > sym2;
    std::vector<Value * > tblPtr1;
    std::vector<Value * > tblPtr2;

    SmallVector<PHINode *, 64> searchMore(mNumSym);
    for (unsigned i = 0; i < mNumSym; i++) {
        PHINode * const found = b->CreatePHI(boolTy, 2);
        found->addIncoming(i1_TRUE, strideMasksReady);
        searchMore[i] = found;
    }

    Value * foundKeyLength;
    Value * foundKeyStartPos;
    Value * foundKeyMarkPos;
    Value * foundKeyHash;
    Value * entryFound = i1_FALSE;
    for(unsigned k = 0; k < mNumSym; k++) {
        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const check = b->CreateBasicBlock();
        BasicBlock * const storeEntryInfo = b->CreateBasicBlock();
        BasicBlock * const next = b->CreateBasicBlock();

        Value * const notFound = b->CreateICmpNE(entryFound, searchMore[k]);
        b->CreateLikelyCondBr(notFound, check, next);
        b->SetInsertPoint(check);
        //b->CallPrintInt("entryFound"+std::to_string(mNumSym), entryFound);
        //b->CallPrintInt("tableEntryPtr"+std::to_string(k), tableEntryPtr[k]);
        //b->CallPrintInt("keyOffset-after"+std::to_string(k), keyOffset[k]);
        tblPtr1.push_back(b->CreateBitCast(tableEntryPtr[k], lg.halfSymPtrTy));
        // loads second half bytes of the given length keyword
        tblPtr2.push_back(b->CreateBitCast(b->CreateGEP(tableEntryPtr[k], keyOffset[k]), lg.halfSymPtrTy));
        // read 16-byte (first half) of keyword?
        Value * symPtr1 = b->CreateBitCast(b->getRawInputPointer("byteData", keyStartPos[k]), lg.halfSymPtrTy);
        Value * symPtr2 = b->CreateBitCast(b->getRawInputPointer("byteData", b->CreateAdd(keyStartPos[k], keyOffset[k])), lg.halfSymPtrTy);
        //b->CallPrintInt("tblPtr1"+std::to_string(k), tblPtr1[k]);
        //b->CallPrintInt("tblPtr2"+std::to_string(k), tblPtr2[k]);
        entry1.push_back(b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr1[k]));
        entry2.push_back(b->CreateMonitoredScalarFieldLoad("hashTable", tblPtr2[k]));
        sym1.push_back(b->CreateAlignedLoad(symPtr1, 1));
        sym2.push_back(b->CreateAlignedLoad(symPtr2, 1));

        // check for hashTable keyword/phrase and current phrase equality
        Value * symIsEqEntry = b->CreateAnd(b->CreateICmpEQ(entry1[k], sym1[k]), b->CreateICmpEQ(entry2[k], sym2[k]));
        b->CreateCondBr(symIsEqEntry, storeEntryInfo, next);
        b->SetInsertPoint(storeEntryInfo);

        foundKeyLength = keyLength[k];
        foundKeyStartPos = keyStartPos[k];
        foundKeyMarkPos = keyMarkPos[k];
        foundKeyHash = keyHash[k];
        b->CreateBr(next);

        b->SetInsertPoint(next);
        PHINode * const updateEntry = b->CreatePHI(boolTy, 3);
        updateEntry->addIncoming(entryFound, entry);
        updateEntry->addIncoming(entryFound, check);
        updateEntry->addIncoming(i1_TRUE, storeEntryInfo);
        entryFound = updateEntry;

        PHINode * const nextFind = b->CreatePHI(boolTy, 3);
        nextFind->addIncoming(i1_TRUE, entry);
        nextFind->addIncoming(i1_TRUE, check);
        nextFind->addIncoming(i1_TRUE, storeEntryInfo);
        searchMore[k] = nextFind;
    }
    // check if the current keyword/phrase to be compressed overlaps with the already compressed keyword/phrases
    // Aproach:
    // Identify the keyword/phrase span with sequence of 1 from its startPos till endPos
    // Check for an overlap with any of the already compressed keyword/phrase tracked using a temporary output stream compSymSeq
    // Update compSymSeq if no overlap with current keyWord/phrase is occurring.
    /// TODO: select the longest keyword/phrase to be compressed among the overlapping keywords from q different symbol streams

    b->CreateCondBr(entryFound, checkOverlappingKey, tryStore);
    b->SetInsertPoint(checkOverlappingKey);

    Value * keyWordMaskLen = b->CreateZExt(foundKeyLength, sizeTy);
    Value * keySpan = b->CreateSub(b->CreateShl(sz_ONE, keyWordMaskLen), sz_ONE);
    // determine the aligned keyBase position
    Value * startBase = b->CreateSub(foundKeyStartPos, b->CreateURem(foundKeyStartPos, b->getSize(8))); //lowest byte boundary in keyword
    Value * markBase = b->CreateSub(foundKeyMarkPos, b->CreateURem(foundKeyMarkPos, sz_BITS)); //lowest word boundary in keyWord
    Value * keyBase = b->CreateSelect(b->CreateICmpULT(startBase, markBase), startBase, markBase);
    Value * const keyBasePtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", keyBase), sizeTy->getPointerTo());

    // keyStartPos may not be aligned for efficient access**
    Value * const keySpanBasePtr = b->CreateBitCast(b->getRawOutputPointer("compSymSeq", foundKeyStartPos), sizeTy->getPointerTo());
    Value * curCompressed = b->CreateLoad(keySpanBasePtr, 1);
    Value * toCompress = b->CreateAnd(b->CreateNot(curCompressed), keySpan);

    /// TODO: choose between tryStore and skip next mNumSym-1 phrases here??
    b->CreateCondBr(b->CreateICmpEQ(toCompress, sz_ZERO), markCompression, tryStore);
    b->SetInsertPoint(markCompression);

    Value * initialMask = b->CreateLoad(keyBasePtr, 1);
    //keylen - 2 is the # bytes to be compressed
    Value * compressMaskLength = b->CreateZExt(b->CreateSub(foundKeyLength, lg.ENC_BYTES, "compressMaskLength"), sizeTy);
    // Eg: 10000 - 1 = 01111
    Value * mask = b->CreateSub(b->CreateShl(sz_ONE, compressMaskLength), sz_ONE);
    assert(SIZE_T_BITS - 8 > 2 * lg.groupHalfLength);
    Value * bitOffset = b->CreateSub(foundKeyStartPos, keyBase);
    mask = b->CreateShl(mask, bitOffset);

    Value * updated = b->CreateAnd(initialMask, b->CreateNot(mask));
    b->CreateStore(b->CreateAnd(updated, b->CreateNot(mask)), keyBasePtr, 1);
    Value * curPos = foundKeyMarkPos;
    Value * curHash = foundKeyHash;

    // Update compSymSeq to add the span of compressed keyword
    b->CreateStore(b->CreateAnd(curCompressed, b->CreateNot(keySpan)), keySpanBasePtr, 1);
    //b->CallPrintInt("updated compSymSeq", b->CreateLoad(keySpanBasePtr, 1));
    // Write the suffixes.
    // No changes needed to handle multi byte suffix phrases
    for (unsigned i = 0; i < lg.groupInfo.encoding_bytes - 1; i++) {
        Value * ZTF_suffix = b->CreateTrunc(b->CreateAnd(curHash, lg.SUFFIX_MASK, "ZTF_suffix"), b->getInt8Ty());
        b->CreateStore(ZTF_suffix, b->getRawOutputPointer("encodedBytes", curPos));
        curPos = b->CreateSub(curPos, sz_ONE);
        curHash = b->CreateLShr(curHash, lg.SUFFIX_BITS);
    }
    Value * lgthBase = b->CreateShl(b->CreateSub(foundKeyLength, lg.LO), lg.PREFIX_LENGTH_OFFSET);
    Value * ZTF_prefix = b->CreateAdd(b->CreateAdd(lg.PREFIX_BASE, lgthBase), curHash, "ZTF_prefix");
    b->CreateStore(b->CreateTrunc(ZTF_prefix, b->getInt8Ty()), b->getRawOutputPointer("encodedBytes", curPos));

    b->CreateBr(nextKey);
    b->CreateCondBr(entryFound, nextKey, tryStore);
    b->SetInsertPoint(tryStore);
    for(unsigned k = 0; k < mNumSym; k++) {
        BasicBlock * const storeKey = b->CreateBasicBlock();

        Value * isEmptyEntry = b->CreateICmpEQ(b->CreateOr(entry1[k], entry2[k]), Constant::getNullValue(lg.halfLengthTy));
        b->CreateLikelyCondBr(isEmptyEntry, storeKey, nextKey);

        b->SetInsertPoint(storeKey);
        b->CreateMonitoredScalarFieldStore("hashTable", sym1[k], tblPtr1[k]);
        b->CreateMonitoredScalarFieldStore("hashTable", sym2[k], tblPtr2[k]);
    }

    b->CreateBr(nextKey);
    b->SetInsertPoint(nextKey);
    BasicBlock * currentBB = b->GetInsertBlock();
    for(unsigned k = 0; k < mNumSym; k++) {
        Value * dropKey = b->CreateResetLowestBit(theKeyWord[k]);
        Value * thisWordDone = b->CreateICmpEQ(dropKey, sz_ZERO);
        nextKeyMask.push_back(b->CreateSelect(thisWordDone, b->CreateResetLowestBit(keyMaskPhi[k]), keyMaskPhi[k]));
        keyMaskPhi[k]->addIncoming(nextKeyMask[k], currentBB);
        keyWordPhi[k]->addIncoming(dropKey, currentBB);
    }
    /*for(unsigned k = 0; k < mNumSym; k++) {
        b->CallPrintRegister("keyMaskPhi - updated"+std::to_string(k), keyMaskPhi[k]);
        b->CallPrintInt("keyWordPhi - updated"+std::to_string(k), keyWordPhi[k]);
    }*/
    Value * nextKeyMaskAccum = nextKeyMask[0];
    for(unsigned k = 1; k < mNumSym; k++) {
        nextKeyMaskAccum = b->CreateOr(nextKeyMaskAccum, nextKeyMask[k]);
    }
    b->CreateCondBr(b->CreateICmpNE(nextKeyMaskAccum, sz_ZERO), keyProcessingLoop, keysDone);
    b->SetInsertPoint(keysDone);

    strideNo->addIncoming(nextStrideNo, keysDone);
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), stridePrologue, stridesDone);

    b->SetInsertPoint(stridesDone);
    if (DeferredAttribute) {
        Value * processed = b->CreateSub(avail, lg.HI);
        b->setProcessedItemCount("byteData", processed);
        //b->CallPrintInt("processed", processed);
    }
    Value * produced = b->CreateSelect(b->isFinal(), avail, b->CreateSub(avail, sz_BLOCKWIDTH));
    b->setProducedItemCount("compressionMask", produced);
    // test
    b->setProducedItemCount("compSymSeq", produced);
    b->CreateCondBr(b->isFinal(), compressionMaskDone, updatePending);
    b->SetInsertPoint(updatePending);
    Value * pendingPtr = b->CreateBitCast(b->getRawOutputPointer("compressionMask", produced), bitBlockPtrTy);
    Value * lastMask = b->CreateBlockAlignedLoad(pendingPtr);
    b->setScalarField("pendingMaskInverted", b->CreateNot(lastMask));
    b->CreateBr(compressionMaskDone);
    b->SetInsertPoint(compressionMaskDone);
}