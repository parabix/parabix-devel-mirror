

#include "lz4_swizzled_match_copy_kernel.h"
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>
#include <vector>
#include <llvm/Support/raw_ostream.h>


using namespace llvm;
using namespace std;
namespace kernel {

Value *LZ4SwizzledMatchCopyKernel::advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, string inputName, Value *startPos, bool isNextOne) {
    BasicBlock* entryBlock = iBuilder->GetInsertBlock();

    Constant* SIZE_0 = iBuilder->getSize(0);
    Constant* SIZE_1 = iBuilder->getSize(1);
    Value* SIZE_64 = iBuilder->getSize(64); // maybe need to handle 32 bit machine
    Value* SIZE_INPUT_64_COUNT = iBuilder->getSize(this->getInputStreamSetBuffer(inputName)->getBufferBlocks() * iBuilder->getBitBlockWidth() / 64);

    Value* initCurrentPos = startPos;

    Value* offsetMarkerRawPtr = iBuilder->CreatePointerCast(iBuilder->getRawInputPointer(inputName, SIZE_0), iBuilder->getInt64Ty()->getPointerTo());

    BasicBlock* findNextMatchOffsetConBlock = iBuilder->CreateBasicBlock("findNextMatchOffsetConBlock");
    BasicBlock* findNextMatchOffsetBodyBlock = iBuilder->CreateBasicBlock("findNextMatchOffsetBodyBlock");

    iBuilder->CreateBr(findNextMatchOffsetConBlock);
    iBuilder->SetInsertPoint(findNextMatchOffsetConBlock);
    // Find position marker bit of next 1 bit

    PHINode* phiCurrentPos = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    phiCurrentPos->addIncoming(initCurrentPos, entryBlock);

    Value* currentPosGlobalBlockIndex = iBuilder->CreateUDiv(phiCurrentPos, SIZE_64);
    Value* currentPosLocalBlockIndex = iBuilder->CreateURem(currentPosGlobalBlockIndex, SIZE_INPUT_64_COUNT);
    Value* currentPosBlockOffset = iBuilder->CreateURem(phiCurrentPos, SIZE_64);
    Value* currentValue = iBuilder->CreateLoad(iBuilder->CreateGEP(offsetMarkerRawPtr, currentPosLocalBlockIndex));

    Value* countValue = iBuilder->CreateLShr(currentValue, currentPosBlockOffset);
    if (!isNextOne) {
        countValue = iBuilder->CreateNot(countValue);
    }
    Value* forwardZero = iBuilder->CreateCountForwardZeroes(countValue);
    Value* realForwardZero = iBuilder->CreateAdd(currentPosBlockOffset, forwardZero);

    // If targetMarker == 0, move to next block, otherwise count forward zero
    phiCurrentPos->addIncoming(iBuilder->CreateMul(SIZE_64, iBuilder->CreateAdd(currentPosGlobalBlockIndex, SIZE_1)), iBuilder->GetInsertBlock());
    iBuilder->CreateCondBr(iBuilder->CreateICmpUGE(realForwardZero, SIZE_64), findNextMatchOffsetConBlock, findNextMatchOffsetBodyBlock);

    iBuilder->SetInsertPoint(findNextMatchOffsetBodyBlock);

    Value* newPosition = iBuilder->CreateAdd(iBuilder->CreateMul(currentPosGlobalBlockIndex, SIZE_64), realForwardZero);

    return newPosition;
}

pair<Value*, Value*> LZ4SwizzledMatchCopyKernel::loadNextMatchOffset(const unique_ptr<KernelBuilder> &iBuilder) {
    Value* initCurrentPos = iBuilder->CreateAdd(iBuilder->getScalarField("currentOffsetMarkerPos"), iBuilder->getSize(1));
    Value* newPosition = this->advanceUntilNextBit(iBuilder, "MatchOffsetMarker", initCurrentPos, true);

    // Load Match Offset from newPosition
    Value* matchOffsetPtr = iBuilder->getRawInputPointer("byteStream", newPosition);
    // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
    matchOffsetPtr = iBuilder->CreatePointerCast(matchOffsetPtr, iBuilder->getInt16Ty()->getPointerTo());
    Value* matchOffset = iBuilder->CreateZExt(iBuilder->CreateLoad(matchOffsetPtr), iBuilder->getSizeTy());

    return std::make_pair(matchOffset, newPosition);
}


void LZ4SwizzledMatchCopyKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    // ---- Contant
    ConstantInt * const SIZE_4_MEGS = iBuilder->getSize(4 * 1024 * 1024);
    ConstantInt * const SIZE_0 = iBuilder->getSize(0);
    ConstantInt * const SIZE_1 = iBuilder->getSize(1);
    ConstantInt * const SIZE_64 = iBuilder->getSize(64);
    ConstantInt * const INT64_0 = iBuilder->getInt64(0);
    ConstantInt * const INT64_1 = iBuilder->getInt64(1);

    Value * BITBLOCK_0 = iBuilder->CreateBitCast(ConstantInt::get(iBuilder->getIntNTy(iBuilder->getBitBlockWidth()), 0), iBuilder->getBitBlockType());

    // ---- Type
    Type* BITBLOCK_TYPE = iBuilder->getBitBlockType();
    Type* BITBLOCK_PTR_TYPE = BITBLOCK_TYPE->getPointerTo();
    Type* I64_TY = iBuilder->getInt64Ty();
    Type* I64_PTR_TY = I64_TY->getPointerTo();

    Value * PDEP_func = Intrinsic::getDeclaration(iBuilder->getModule(), Intrinsic::x86_bmi_pdep_64); //TODO for now only consider 64 bits

    // ---- EntryBlock
    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const exitBlock = iBuilder->CreateBasicBlock("exitBlock");

    Value * const available = iBuilder->getAvailableItemCount("sourceStreamSet0");
    Value * const processed = iBuilder->getProcessedItemCount("sourceStreamSet0");

    Value * const itemsToDo = iBuilder->CreateUMin(iBuilder->CreateSub(available, processed), SIZE_4_MEGS);
    iBuilder->setTerminationSignal(iBuilder->CreateICmpULT(itemsToDo, SIZE_4_MEGS));


    Value* m0MarkerBasePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("M0Marker", SIZE_0), I64_PTR_TY); // i64*
    vector<Value*> sourceStreamBasePtrs, outputStreamBasePtrs; // <4 * i64>*
    for (int i = 0; i < mStreamSize; i++) {
        sourceStreamBasePtrs.push_back(iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr("sourceStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
        outputStreamBasePtrs.push_back(iBuilder->CreatePointerCast(iBuilder->getOutputStreamBlockPtr("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
    }


    BasicBlock * const processLoopCon = iBuilder->CreateBasicBlock("processLoopCon");
    BasicBlock * const processLoopBody = iBuilder->CreateBasicBlock("processLoopBody");
    BasicBlock * const processLoopExit = iBuilder->CreateBasicBlock("processLoopExit");

    iBuilder->CreateBr(processLoopCon);

    // ---- ProcessLoopCon
    // ProcessLoop will process one block of data each time (64bit m0, <4 * i64> input and output data)
    iBuilder->SetInsertPoint(processLoopCon);

    // carryBit === 0x1 only when the most significant bit of the target M0 block is one, which means the first position of next block need to be deposited (match copy)

    PHINode* phiCarryBit = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
    PHINode* phiCurrentPosition = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2); // 0~4mb, and all M0 related
    PHINode* phiCarryMatchOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);

    phiCarryBit->addIncoming(INT64_0, entryBlock);
    phiCurrentPosition->addIncoming(INT64_0, entryBlock);
    phiCarryMatchOffset->addIncoming(SIZE_0, entryBlock);

    iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpULT(phiCurrentPosition, itemsToDo), processLoopBody, processLoopExit);

    // ---- ProcessLoopBody
    iBuilder->SetInsertPoint(processLoopBody);

    Value* dataBlockIndex = iBuilder->CreateUDiv(phiCurrentPosition, SIZE_64);
    Value* currentInitM0 = iBuilder->CreateLoad(iBuilder->CreateGEP(m0MarkerBasePtr, dataBlockIndex));
    vector<Value*> initSourceData;
    for (int i = 0; i < mStreamSize; i++) {
        // Because of swizzled form, the sourceStream can be accessed linearly
        initSourceData.push_back(iBuilder->CreateLoad(iBuilder->CreateGEP(sourceStreamBasePtrs[i], dataBlockIndex)));
    }

    BasicBlock* carryBitProcessBlock = iBuilder->CreateBasicBlock("CarryBitProcessBlock");

    BasicBlock* matchCopyLoopCon = iBuilder->CreateBasicBlock("MatchCopyLoopCon");
    BasicBlock* matchCopyLoopBody = iBuilder->CreateBasicBlock("MatchCopyLoopBody");
    BasicBlock* matchCopyLoopExit = iBuilder->CreateBasicBlock("MatchCopyLoopExit");

    //
    // The carry bit will need to be processed specially only when
    // the most significant bit of previous block is 1 (the carry bit is 0x1) and the
    // least significant bit of current block is 0
    // e.g.
    //   Assume the most significant bit is on the right side
    //
    //                    i64_1       i64_2
    //   M0         ... 0000 0011 | 0111 0000 ...  - Carry bit need to be handle specially
    //   M0         ... 0000 0011 | 1011 0000 ...  - Carry bit will be handle in the loop of i64_2
    //   Carry Bit                  1000 0000 ...  - 0x1

    Value* needProcessCarryBit = iBuilder->CreateAnd(phiCarryBit, iBuilder->CreateNot(iBuilder->CreateAnd(currentInitM0, iBuilder->getInt64(1))));
    needProcessCarryBit = iBuilder->CreateICmpNE(needProcessCarryBit, INT64_0);

    iBuilder->CreateUnlikelyCondBr(needProcessCarryBit, carryBitProcessBlock, matchCopyLoopCon);

    // ---- CarryBitProcessBlock
    iBuilder->SetInsertPoint(carryBitProcessBlock);
    vector<Value*> initSourceDataWithCarry;
    Value* carryCopyFromPos = iBuilder->CreateSub(phiCurrentPosition, phiCarryMatchOffset);
    Value* carryCopyFromBlockIndex = iBuilder->CreateUDiv(carryCopyFromPos, SIZE_64);
    Value* carryCopyFromOffset = iBuilder->CreateURem(carryCopyFromPos, SIZE_64);
    for (int i = 0; i < mStreamSize; i++) {
        Value* v = iBuilder->CreateLoad(iBuilder->CreateGEP(outputStreamBasePtrs[i], carryCopyFromBlockIndex));
        v = iBuilder->CreateLShr(v, iBuilder->simd_fill(mPDEPWidth, carryCopyFromOffset));
        v = iBuilder->CreateAnd(v, iBuilder->simd_fill(mPDEPWidth, INT64_1));
        initSourceDataWithCarry.push_back(iBuilder->CreateOr(v, initSourceData[i]));
    }
    iBuilder->CreateBr(matchCopyLoopCon);

    // ---- MatchCopyLoopCon
    // MatchCopy Loop will handle one continuous data deposit each time
    iBuilder->SetInsertPoint(matchCopyLoopCon);

    PHINode* phiLatestMatchOffset = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    phiLatestMatchOffset->addIncoming(phiCarryMatchOffset, processLoopBody);
    phiLatestMatchOffset->addIncoming(phiCarryMatchOffset, carryBitProcessBlock);

    PHINode* phiRemainingM0Marker = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
    phiRemainingM0Marker->addIncoming(currentInitM0, processLoopBody);
    phiRemainingM0Marker->addIncoming(currentInitM0, carryBitProcessBlock);

    vector<PHINode*> outputData;
    for (int i = 0; i < mStreamSize; i++) {
        PHINode* outputValue = iBuilder->CreatePHI(iBuilder->getBitBlockType(), 3);
        outputValue->addIncoming(initSourceData[i], processLoopBody);
        outputValue->addIncoming(initSourceDataWithCarry[i], carryBitProcessBlock);
        outputData.push_back(outputValue);
    }
//    iBuilder->CreateOr()
    iBuilder->CreateLikelyCondBr(iBuilder->CreateICmpNE(phiRemainingM0Marker, INT64_0), matchCopyLoopBody, matchCopyLoopExit);

    // ---- MatchCopyLoopBody
    iBuilder->SetInsertPoint(matchCopyLoopBody);

    // Match Offset
    // M0      0111 1000  - load new match offset
    // M0      1100 0011  - use carryMatchOffset
    Value* remainM0ForwardZero = iBuilder->CreateCountForwardZeroes(phiRemainingM0Marker);

    BasicBlock* loadNextMatchOffsetBlock = iBuilder->CreateBasicBlock("loadNextMatchOffsetBlock");
    BasicBlock* doMatchCopyBlock = iBuilder->CreateBasicBlock("DoMatchCopyBlock");

    iBuilder->CreateLikelyCondBr(
            iBuilder->CreateOr(iBuilder->CreateICmpEQ(phiLatestMatchOffset, INT64_0),
                               iBuilder->CreateICmpNE(remainM0ForwardZero, INT64_0)
            ),
            loadNextMatchOffsetBlock, doMatchCopyBlock
    );

    // ---- loadNextMatchOffsetBlock
    iBuilder->SetInsertPoint(loadNextMatchOffsetBlock);
    auto matchOffsetRet = this->loadNextMatchOffset(iBuilder);
    BasicBlock* loadNextMatchOffsetExitBlock = iBuilder->GetInsertBlock();
    Value* newMatchOffset = matchOffsetRet.first;
    Value* newMatchOffsetPos = matchOffsetRet.second;
    iBuilder->setScalarField("currentOffsetMarkerPos", newMatchOffsetPos);
    iBuilder->setProcessedItemCount("MatchOffsetMarker", newMatchOffsetPos);
    iBuilder->CreateBr(doMatchCopyBlock);

    // ---- doMatchCopyBlock
    iBuilder->SetInsertPoint(doMatchCopyBlock);

    PHINode* phiTargetMatchOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    phiTargetMatchOffset->addIncoming(phiLatestMatchOffset, matchCopyLoopBody);
    phiTargetMatchOffset->addIncoming(newMatchOffset, loadNextMatchOffsetExitBlock);

    //
    // M0            0011 0010
    // boundary      0000 1000
    // nextMask      0000 0111
    // deposit       0011 1000
    // newM0         0000 0010

    Value* remainStart = iBuilder->CreateShl(INT64_1, remainM0ForwardZero);

    Value* boundaryMarker = iBuilder->CreateAnd(iBuilder->CreateAdd(phiRemainingM0Marker, remainStart), iBuilder->CreateNot(phiRemainingM0Marker));

    Value* nextMask = iBuilder->CreateSub(INT64_0, iBuilder->CreateShl(boundaryMarker, INT64_1));
    Value* depositMarker = iBuilder->CreateAnd(
            iBuilder->CreateOr(phiRemainingM0Marker, boundaryMarker),
            iBuilder->CreateNot(nextMask)
    );
    Value* newM0Marker = iBuilder->CreateAnd(phiRemainingM0Marker, nextMask);
    Value* depositMarkerPopcount = iBuilder->CreatePopcount(depositMarker);

    Value* matchCopyFromStart = iBuilder->CreateSub(iBuilder->CreateAdd(phiCurrentPosition, remainM0ForwardZero), phiTargetMatchOffset);
    Value* matchCopyFromBlockIndex = iBuilder->CreateUDiv(matchCopyFromStart, SIZE_64);

    Value* matchCopyFromOffset = iBuilder->CreateURem(matchCopyFromStart, SIZE_64);
    Value* matchCopyFromRemaining = iBuilder->CreateSub(SIZE_64, matchCopyFromOffset);
    Value* matchCopyFromNextBlockIndex = iBuilder->CreateAdd(matchCopyFromBlockIndex, iBuilder->CreateSelect(iBuilder->CreateICmpULE(depositMarkerPopcount, matchCopyFromRemaining), SIZE_0, SIZE_1));


    vector<Value*> pdepSourceData;

    for (int i = 0; i < mStreamSize; i++) {
        Value* fromPtr = iBuilder->CreateGEP(outputStreamBasePtrs[i], matchCopyFromBlockIndex);
        Value* fromBlockValue = iBuilder->CreateLoad(fromPtr);
        // when dataBlockIndex == matchCopyFromBlockIndex, we need to use current output value as input
        fromBlockValue = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(dataBlockIndex, matchCopyFromBlockIndex), outputData[i], fromBlockValue);

        Value* fromNextPtr = iBuilder->CreateGEP(outputStreamBasePtrs[i], matchCopyFromNextBlockIndex);
        Value* fromNextBlockValue = iBuilder->CreateLoad(fromNextPtr);
        fromNextBlockValue = iBuilder->CreateSelect(iBuilder->CreateICmpEQ(dataBlockIndex, matchCopyFromNextBlockIndex), outputData[i], fromNextBlockValue);


        Value * allFromValue = iBuilder->CreateOr(
                iBuilder->CreateLShr(fromBlockValue, iBuilder->simd_fill(mPDEPWidth, matchCopyFromOffset)),
                iBuilder->CreateShl(fromNextBlockValue, iBuilder->simd_fill(mPDEPWidth, matchCopyFromRemaining))
        );
        pdepSourceData.push_back(allFromValue);
    }

    BasicBlock* doubleSourceDataCon = iBuilder->CreateBasicBlock("doubleSourceDataCon");
    BasicBlock* doubleSourceDataBody = iBuilder->CreateBasicBlock("doubleSourceDataBody");
    BasicBlock* doubleSourceDataExit = iBuilder->CreateBasicBlock("doubleSourceDataExit");

    iBuilder->CreateBr(doubleSourceDataCon);

    //
    // When matchOffset < depositMarkerPopcount, we need to use log2 approach to double the source data
    // e.g.
    // Assume that match copy start position is 1, matchOffset is 1, match length is 5
    //     outputBuffer              a000 0000 0000 0000
    //     sourceDataBeforeDouble    a000 0000 0000 0000
    // At this point, only 1 bit of source data is accessable, so it will double the source data 3 times until we have
    // 1 * 2 ^ 3 = 8 bits accessable
    //     sourceDataAfterDouble     aaaa aaaa 0000 0000
    //     outputBuffer(after copy)  aaaa aa00 0000 0000
    //

    // ---- doubleSourceDataCon
    iBuilder->SetInsertPoint(doubleSourceDataCon);
    PHINode* phiSourceDataAccessable = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
    phiSourceDataAccessable->addIncoming(phiTargetMatchOffset, doMatchCopyBlock);
    vector<PHINode*> phiPdepSourceData;
    for (int i = 0; i < mStreamSize; i++) {
        PHINode* v = iBuilder->CreatePHI(iBuilder->getBitBlockType(), 2);
        v->addIncoming(pdepSourceData[i], doMatchCopyBlock);
        phiPdepSourceData.push_back(v);
    }
    iBuilder->CreateUnlikelyCondBr(iBuilder->CreateICmpULT(phiSourceDataAccessable, depositMarkerPopcount), doubleSourceDataBody, doubleSourceDataExit);

    // ---- doubleSourceDataBody
    iBuilder->SetInsertPoint(doubleSourceDataBody);
    for (int i = 0; i < mStreamSize; i++) {
        PHINode* v = phiPdepSourceData[i];
        Value* newValue = iBuilder->CreateOr(v, iBuilder->CreateShl(v, iBuilder->simd_fill(mPDEPWidth, phiSourceDataAccessable)));
        v->addIncoming(newValue, doubleSourceDataBody);
    }
    phiSourceDataAccessable->addIncoming(iBuilder->CreateShl(phiSourceDataAccessable, SIZE_1), doubleSourceDataBody);

    iBuilder->CreateBr(doubleSourceDataCon);

    // ---- doubleSourceDataExit
    iBuilder->SetInsertPoint(doubleSourceDataExit);
    // At this point, we can guarantee we have enough data for pdep
    for (int i = 0; i < mStreamSize; i++) {
        // Do Match Copy by PDEP
        Value* allFromValue = phiPdepSourceData[i];
        Value* newValue = BITBLOCK_0;
        for (uint64_t j = 0; j < 4; j++) { // For now, we assume bit block type is always <4 * i64>
            Value* source_field = iBuilder->CreateExtractElement(allFromValue, j);
            Value * PDEP_field = iBuilder->CreateCall(PDEP_func, {source_field, depositMarker});
            newValue = iBuilder->CreateInsertElement(newValue, PDEP_field, j);
        }
        PHINode* outputValue = outputData[i];
        Value* newOutputValue = iBuilder->CreateOr(outputValue, newValue);
        outputValue->addIncoming(newOutputValue, iBuilder->GetInsertBlock());
    }
    phiRemainingM0Marker->addIncoming(newM0Marker, iBuilder->GetInsertBlock());
    phiLatestMatchOffset->addIncoming(phiTargetMatchOffset, iBuilder->GetInsertBlock());

    iBuilder->CreateBr(matchCopyLoopCon);

    // ---- MatchCopyLoopExit
    iBuilder->SetInsertPoint(matchCopyLoopExit);
    for (int i = 0; i < mStreamSize; i++) {
        iBuilder->CreateStore(outputData[i], iBuilder->CreateGEP(outputStreamBasePtrs[i], dataBlockIndex));
    }
    Value* hasNewCarryBit = iBuilder->CreateAnd(currentInitM0, iBuilder->CreateShl(INT64_1, iBuilder->getInt64(63)));
    hasNewCarryBit = iBuilder->CreateICmpNE(hasNewCarryBit, INT64_0);
    Value* newCarryBit = iBuilder->CreateSelect(hasNewCarryBit, INT64_1, INT64_0);
    phiCarryBit->addIncoming(newCarryBit, iBuilder->GetInsertBlock());

    phiCarryMatchOffset->addIncoming(iBuilder->CreateSelect(hasNewCarryBit, phiLatestMatchOffset, iBuilder->getSize(0)), iBuilder->GetInsertBlock());

    phiCurrentPosition->addIncoming(iBuilder->CreateAdd(phiCurrentPosition, SIZE_64), iBuilder->GetInsertBlock());

    iBuilder->CreateBr(processLoopCon);

    // ---- ProcessLoopExit
    iBuilder->SetInsertPoint(processLoopExit);
    Value * const toProcessItemCount = iBuilder->CreateAdd(processed, itemsToDo);
    iBuilder->setProcessedItemCount("M0Marker", toProcessItemCount);
    iBuilder->CreateBr(exitBlock);

    // ---- ExitBlock
    iBuilder->SetInsertPoint(exitBlock);


}

LZ4SwizzledMatchCopyKernel::LZ4SwizzledMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, unsigned streamCount/*=4*/, unsigned streamSize/*=2*/, unsigned swizzleFactor/*=4*/, unsigned PDEP_width/*64*/)
: SegmentOrientedKernel("LZ4SwizzledMatchCopyKernel",
// Inputs
{
                                   Binding{iBuilder->getStreamSetTy(1, 1), "MatchOffsetMarker", BoundedRate(0, 1)},
                                   Binding{iBuilder->getStreamSetTy(1, 1), "M0Marker", BoundedRate(0, 1)},
                                   Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", RateEqualTo("MatchOffsetMarker")}
},
// Outputs
{},
// Arguments
{
},
{},
{
       Binding(iBuilder->getSizeTy(), "currentOffsetMarkerPos"),
//       Binding(iBuilder->getSizeTy(), "currentOffsetMarkerPos"),
})
, mSwizzleFactor(swizzleFactor)
, mPDEPWidth(PDEP_width)
, mStreamSize(streamSize)
, mStreamCount(streamCount) {

    assert((mSwizzleFactor == (iBuilder->getBitBlockWidth() / PDEP_width)) && "swizzle factor must equal bitBlockWidth / PDEP_width");
    assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");
    setStride(4 * 1024 * 1024);
    addAttribute(MustExplicitlyTerminate());

    mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet0", RateEqualTo("M0Marker"), Swizzled()});
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet0", RateEqualTo("M0Marker")});

    for (unsigned i = 1; i < streamSize; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("M0Marker"), Swizzled()});
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("M0Marker")});
    }
}

}
