
#include "lz4_bitstream_match_copy_kernel.h"

#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <toolchain/toolchain.h>
#include <vector>
#include <llvm/Support/raw_ostream.h>


using namespace llvm;
using namespace std;
namespace kernel {

    std::vector<llvm::Value *>
    LZ4BitStreamMatchCopyKernel::loadAllI64BitStreamValues(const std::unique_ptr<KernelBuilder> &b,
                                                                 llvm::Value *basePtr,
                                                                 llvm::Value *i64PackIndex) {
        // TODO use gather to improve performance
        std::vector<llvm::Value *, allocator<llvm::Value*>> v;

        // For now, assume bit block type is always <4 * i64>
        Value* bitBlockIndex = b->CreateUDiv(i64PackIndex, b->getSize(4));
        Value* extractIndex = b->CreateURem(i64PackIndex, b->getSize(4));

        for (int i = 0; i < mNumberOfStreams; i++) {
            Value* bitBlockPtr =  b->CreateGEP(basePtr, b->CreateAdd(b->CreateMul(bitBlockIndex, b->getSize(mNumberOfStreams)), b->getSize(i)));
            bitBlockPtr = b->CreatePointerCast(bitBlockPtr, b->getInt64Ty()->getPointerTo());
            v.push_back(b->CreateLoad(b->CreateGEP(bitBlockPtr, extractIndex)));
        }
        return v;
    }

    Value *LZ4BitStreamMatchCopyKernel::advanceUntilNextBit(const std::unique_ptr<KernelBuilder> &iBuilder, string inputName, Value *startPos, bool isNextOne) {
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

    pair<Value*, Value*> LZ4BitStreamMatchCopyKernel::loadNextMatchOffset(const unique_ptr<KernelBuilder> &iBuilder) {
        Value* initCurrentPos = iBuilder->CreateAdd(iBuilder->getScalarField("currentOffsetMarkerPos"), iBuilder->getSize(1));
        Value* newPosition = this->advanceUntilNextBit(iBuilder, "MatchOffsetMarker", initCurrentPos, true);

        // Load Match Offset from newPosition
        Value* matchOffsetPtr = iBuilder->getRawInputPointer("byteStream", newPosition);
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        matchOffsetPtr = iBuilder->CreatePointerCast(matchOffsetPtr, iBuilder->getInt16Ty()->getPointerTo());
        Value* matchOffset = iBuilder->CreateZExt(iBuilder->CreateLoad(matchOffsetPtr), iBuilder->getSizeTy());

        return std::make_pair(matchOffset, newPosition);
    }


    void LZ4BitStreamMatchCopyKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {
        // ---- Contant
        ConstantInt * const SIZE_4_MEGS = b->getSize(4 * 1024 * 1024);
        ConstantInt * const SIZE_0 = b->getSize(0);
        ConstantInt * const SIZE_1 = b->getSize(1);
        ConstantInt * const SIZE_64 = b->getSize(64);
        ConstantInt * const INT64_0 = b->getInt64(0);
        ConstantInt * const INT64_1 = b->getInt64(1);

        Value * BITBLOCK_0 = b->CreateBitCast(ConstantInt::get(b->getIntNTy(b->getBitBlockWidth()), 0), b->getBitBlockType());

        // ---- Type
        Type* BITBLOCK_TYPE = b->getBitBlockType();
        Type* BITBLOCK_PTR_TYPE = BITBLOCK_TYPE->getPointerTo();
        Type* I64_TY = b->getInt64Ty();
        Type* I64_PTR_TY = I64_TY->getPointerTo();

        Value * PDEP_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64); //TODO for now only consider 64 bits

        // ---- EntryBlock
        BasicBlock * const entryBlock = b->GetInsertBlock();
        BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");

        Value * const available = b->getAvailableItemCount("inputBitStream");
        Value * const processed = b->getProcessedItemCount("inputBitStream");

        Value * const itemsToDo = b->CreateUMin(b->CreateSub(available, processed), SIZE_4_MEGS);
        b->setTerminationSignal(b->CreateICmpULT(itemsToDo, SIZE_4_MEGS));


        Value* m0MarkerBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("M0Marker", SIZE_0), I64_PTR_TY); // i64*
        Value* sourceStreamPtr = b->CreatePointerCast(b->getInputStreamBlockPtr("inputBitStream", SIZE_0), BITBLOCK_PTR_TYPE);
        Value* outputStreamPtr = b->CreatePointerCast(b->getOutputStreamBlockPtr("outputBitStream", SIZE_0), BITBLOCK_PTR_TYPE);

        BasicBlock * const processLoopCon = b->CreateBasicBlock("processLoopCon");
        BasicBlock * const processLoopBody = b->CreateBasicBlock("processLoopBody");
        BasicBlock * const processLoopExit = b->CreateBasicBlock("processLoopExit");

        b->CreateBr(processLoopCon);

        // ---- ProcessLoopCon
        // ProcessLoop will process one block of data each time (64bit m0, <4 * i64> input and output data)
        b->SetInsertPoint(processLoopCon);

        // carryBit === 0x1 only when the most significant bit of the target M0 block is one, which means the first position of next block need to be deposited (match copy)

        PHINode* phiCarryBit = b->CreatePHI(b->getInt64Ty(), 2);
        PHINode* phiCurrentPosition = b->CreatePHI(b->getInt64Ty(), 2); // 0~4mb, and all M0 related
        PHINode* phiCarryMatchOffset = b->CreatePHI(b->getSizeTy(), 2);

        phiCarryBit->addIncoming(INT64_0, entryBlock);
        phiCurrentPosition->addIncoming(INT64_0, entryBlock);
        phiCarryMatchOffset->addIncoming(SIZE_0, entryBlock);

        b->CreateLikelyCondBr(b->CreateICmpULT(phiCurrentPosition, itemsToDo), processLoopBody, processLoopExit);

        // ---- ProcessLoopBody
        b->SetInsertPoint(processLoopBody);

        Value* dataBlockIndex = b->CreateUDiv(phiCurrentPosition, SIZE_64);
        Value* currentInitM0 = b->CreateLoad(b->CreateGEP(m0MarkerBasePtr, dataBlockIndex));
        vector<Value*> initSourceData = this->loadAllI64BitStreamValues(b, sourceStreamPtr, dataBlockIndex);

        Value* bitBlockIndex = b->CreateUDiv(dataBlockIndex, b->getSize(4));
        Value* extractIndex = b->CreateURem(dataBlockIndex, b->getSize(4));

        BasicBlock* carryBitProcessBlock = b->CreateBasicBlock("CarryBitProcessBlock");

        BasicBlock* matchCopyLoopCon = b->CreateBasicBlock("MatchCopyLoopCon");
        BasicBlock* matchCopyLoopBody = b->CreateBasicBlock("MatchCopyLoopBody");
        BasicBlock* matchCopyLoopExit = b->CreateBasicBlock("MatchCopyLoopExit");

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

        Value* needProcessCarryBit = b->CreateAnd(phiCarryBit, b->CreateNot(b->CreateAnd(currentInitM0, b->getInt64(1))));
        needProcessCarryBit = b->CreateICmpNE(needProcessCarryBit, INT64_0);

        b->CreateUnlikelyCondBr(needProcessCarryBit, carryBitProcessBlock, matchCopyLoopCon);

        // ---- CarryBitProcessBlock
        b->SetInsertPoint(carryBitProcessBlock);
        Value* carryCopyFromPos = b->CreateSub(phiCurrentPosition, phiCarryMatchOffset);
        Value* carryCopyFromBlockIndex = b->CreateUDiv(carryCopyFromPos, SIZE_64);
        Value* carryCopyFromOffset = b->CreateURem(carryCopyFromPos, SIZE_64);

        vector<Value*> initSourceDataWithCarry = this->loadAllI64BitStreamValues(b, outputStreamPtr,
                                                                                 carryCopyFromBlockIndex);

        for (int i = 0; i < mNumberOfStreams; i++) {
            Value* v = initSourceDataWithCarry[i];
            v = b->CreateLShr(v, carryCopyFromOffset);
            v = b->CreateAnd(v, INT64_1);
            initSourceDataWithCarry[i] = b->CreateOr(v, initSourceData[i]);
        }
        b->CreateBr(matchCopyLoopCon);

        // ---- MatchCopyLoopCon
        // MatchCopy Loop will handle one continuous data deposit each time
        b->SetInsertPoint(matchCopyLoopCon);

        PHINode* phiLatestMatchOffset = b->CreatePHI(b->getInt64Ty(), 3);
        phiLatestMatchOffset->addIncoming(phiCarryMatchOffset, processLoopBody);
        phiLatestMatchOffset->addIncoming(phiCarryMatchOffset, carryBitProcessBlock);

        PHINode* phiRemainingM0Marker = b->CreatePHI(b->getInt64Ty(), 3);
        phiRemainingM0Marker->addIncoming(currentInitM0, processLoopBody);
        phiRemainingM0Marker->addIncoming(currentInitM0, carryBitProcessBlock);

        std::vector<PHINode*> outputData;
        for (int i = 0; i < mNumberOfStreams; i++) {
            PHINode* outputValue = b->CreatePHI(b->getInt64Ty(), 3);
            outputValue->addIncoming(initSourceData[i], processLoopBody);
            outputValue->addIncoming(initSourceDataWithCarry[i], carryBitProcessBlock);
            outputData.push_back(outputValue);
        }

        b->CreateLikelyCondBr(b->CreateICmpNE(phiRemainingM0Marker, INT64_0), matchCopyLoopBody, matchCopyLoopExit);

        // ---- MatchCopyLoopBody
        b->SetInsertPoint(matchCopyLoopBody);

        // Match Offset
        // M0      0111 1000  - load new match offset
        // M0      1100 0011  - use carryMatchOffset
        Value* remainM0ForwardZero = b->CreateCountForwardZeroes(phiRemainingM0Marker);

        BasicBlock* loadNextMatchOffsetBlock = b->CreateBasicBlock("loadNextMatchOffsetBlock");
        BasicBlock* doMatchCopyBlock = b->CreateBasicBlock("DoMatchCopyBlock");

        b->CreateLikelyCondBr(
                b->CreateOr(b->CreateICmpEQ(phiLatestMatchOffset, INT64_0),
                                   b->CreateICmpNE(remainM0ForwardZero, INT64_0)
                ),
                loadNextMatchOffsetBlock, doMatchCopyBlock
        );

        // ---- loadNextMatchOffsetBlock
        b->SetInsertPoint(loadNextMatchOffsetBlock);
        auto matchOffsetRet = this->loadNextMatchOffset(b);
        BasicBlock* loadNextMatchOffsetExitBlock = b->GetInsertBlock();
        Value* newMatchOffset = matchOffsetRet.first;
        Value* newMatchOffsetPos = matchOffsetRet.second;
        b->setScalarField("currentOffsetMarkerPos", newMatchOffsetPos);
        b->setProcessedItemCount("MatchOffsetMarker", newMatchOffsetPos);
        b->CreateBr(doMatchCopyBlock);

        // ---- doMatchCopyBlock
        b->SetInsertPoint(doMatchCopyBlock);

        PHINode* phiTargetMatchOffset = b->CreatePHI(b->getSizeTy(), 2);
        phiTargetMatchOffset->addIncoming(phiLatestMatchOffset, matchCopyLoopBody);
        phiTargetMatchOffset->addIncoming(newMatchOffset, loadNextMatchOffsetExitBlock);

        //
        // M0            0011 0010
        // boundary      0000 1000
        // nextMask      0000 0111
        // deposit       0011 1000
        // newM0         0000 0010

        Value* remainStart = b->CreateShl(INT64_1, remainM0ForwardZero);

        Value* boundaryMarker = b->CreateAnd(b->CreateAdd(phiRemainingM0Marker, remainStart), b->CreateNot(phiRemainingM0Marker));

        Value* nextMask = b->CreateSub(INT64_0, b->CreateShl(boundaryMarker, INT64_1));
        Value* depositMarker = b->CreateAnd(
                b->CreateOr(phiRemainingM0Marker, boundaryMarker),
                b->CreateNot(nextMask)
        );
        Value* newM0Marker = b->CreateAnd(phiRemainingM0Marker, nextMask);
        Value* depositMarkerPopcount = b->CreatePopcount(depositMarker);

        Value* matchCopyFromStart = b->CreateSub(b->CreateAdd(phiCurrentPosition, remainM0ForwardZero), phiTargetMatchOffset);
        Value* matchCopyFromBlockIndex = b->CreateUDiv(matchCopyFromStart, SIZE_64);

        Value* matchCopyFromOffset = b->CreateURem(matchCopyFromStart, SIZE_64);
        Value* matchCopyFromRemaining = b->CreateSub(SIZE_64, matchCopyFromOffset);
        Value* matchCopyFromNextBlockIndex = b->CreateAdd(matchCopyFromBlockIndex, b->CreateSelect(b->CreateICmpULE(depositMarkerPopcount, matchCopyFromRemaining), SIZE_0, SIZE_1));


        vector<Value*> fromBlockValues = this->loadAllI64BitStreamValues(b, outputStreamPtr, matchCopyFromBlockIndex);
        vector<Value*> fromNextBlockValues = this->loadAllI64BitStreamValues(b, outputStreamPtr, matchCopyFromNextBlockIndex);

        vector<Value*> pdepSourceData;
        for (int i = 0; i < mNumberOfStreams; i++) {
            Value* fromBlockValue = fromBlockValues[i];
            // when dataBlockIndex == matchCopyFromBlockIndex, we need to use current output value as input
            fromBlockValue = b->CreateSelect(b->CreateICmpEQ(dataBlockIndex, matchCopyFromBlockIndex), outputData[i], fromBlockValue);
            Value* fromNextBlockValue = fromNextBlockValues[i];
            fromNextBlockValue = b->CreateSelect(b->CreateICmpEQ(dataBlockIndex, matchCopyFromNextBlockIndex), outputData[i], fromNextBlockValue);


            Value * allFromValue = b->CreateOr(
                    b->CreateLShr(fromBlockValue, matchCopyFromOffset),
                    b->CreateShl(fromNextBlockValue, matchCopyFromRemaining)
            );
            pdepSourceData.push_back(allFromValue);
        }


        BasicBlock* doubleSourceDataCon = b->CreateBasicBlock("doubleSourceDataCon");
        BasicBlock* doubleSourceDataBody = b->CreateBasicBlock("doubleSourceDataBody");
        BasicBlock* doubleSourceDataExit = b->CreateBasicBlock("doubleSourceDataExit");

        b->CreateBr(doubleSourceDataCon);

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
        b->SetInsertPoint(doubleSourceDataCon);
        PHINode* phiSourceDataAccessable = b->CreatePHI(b->getSizeTy(), 2);
        phiSourceDataAccessable->addIncoming(phiTargetMatchOffset, doMatchCopyBlock);
        vector<PHINode*> phiPdepSourceData;
        for (int i = 0; i < mNumberOfStreams; i++) {
            PHINode* v = b->CreatePHI(b->getInt64Ty(), 2);
            v->addIncoming(pdepSourceData[i], doMatchCopyBlock);
            phiPdepSourceData.push_back(v);
        }
        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiSourceDataAccessable, depositMarkerPopcount), doubleSourceDataBody, doubleSourceDataExit);

        // ---- doubleSourceDataBody
        b->SetInsertPoint(doubleSourceDataBody);
        for (int i = 0; i < mNumberOfStreams; i++) {
            PHINode* v = phiPdepSourceData[i];
            Value* newValue = b->CreateOr(v, b->CreateShl(v, phiSourceDataAccessable));
            v->addIncoming(newValue, doubleSourceDataBody);
        }
        phiSourceDataAccessable->addIncoming(b->CreateShl(phiSourceDataAccessable, SIZE_1), doubleSourceDataBody);

        b->CreateBr(doubleSourceDataCon);

        // ---- doubleSourceDataExit
        b->SetInsertPoint(doubleSourceDataExit);
        // At this point, we can guarantee we have enough data for pdep
        for (int i = 0; i < mNumberOfStreams; i++) {
            // Do Match Copy by PDEP
            Value* source_field = phiPdepSourceData[i];
            Value * newValue = b->CreateCall(PDEP_func, {source_field, depositMarker});
            PHINode* outputValue = outputData[i];
            Value* newOutputValue = b->CreateOr(outputValue, newValue);
            outputValue->addIncoming(newOutputValue, b->GetInsertBlock());
        }
        phiRemainingM0Marker->addIncoming(newM0Marker, b->GetInsertBlock());
        phiLatestMatchOffset->addIncoming(phiTargetMatchOffset, b->GetInsertBlock());

        b->CreateBr(matchCopyLoopCon);

        // ---- MatchCopyLoopExit
        b->SetInsertPoint(matchCopyLoopExit);
        this->storeAllI64BitStreamValues(b, outputStreamPtr, dataBlockIndex, outputData);

        Value* hasNewCarryBit = b->CreateAnd(currentInitM0, b->CreateShl(INT64_1, b->getInt64(63)));
        hasNewCarryBit = b->CreateICmpNE(hasNewCarryBit, INT64_0);
        Value* newCarryBit = b->CreateSelect(hasNewCarryBit, INT64_1, INT64_0);
        phiCarryBit->addIncoming(newCarryBit, b->GetInsertBlock());

        phiCarryMatchOffset->addIncoming(b->CreateSelect(hasNewCarryBit, phiLatestMatchOffset, b->getSize(0)), b->GetInsertBlock());

        phiCurrentPosition->addIncoming(b->CreateAdd(phiCurrentPosition, SIZE_64), b->GetInsertBlock());

        b->CreateBr(processLoopCon);

        // ---- ProcessLoopExit
        b->SetInsertPoint(processLoopExit);
        Value * const toProcessItemCount = b->CreateAdd(processed, itemsToDo);
        b->setProcessedItemCount("M0Marker", toProcessItemCount);
        b->CreateBr(exitBlock);

        // ---- ExitBlock
        b->SetInsertPoint(exitBlock);
    }

    void LZ4BitStreamMatchCopyKernel::storeAllI64BitStreamValues(const std::unique_ptr<KernelBuilder> &b,
                                                                       llvm::Value *basePtr, llvm::Value *i64PackIndex,
                                                                       const std::vector<llvm::PHINode *> &values) {
        // TODO PHINode here is a workaround
        // For now, assume bit block type is always <4 * i64>
        Value* bitBlockIndex = b->CreateUDiv(i64PackIndex, b->getSize(4));
        Value* extractIndex = b->CreateURem(i64PackIndex, b->getSize(4));

        for (int i = 0; i < mNumberOfStreams; i++) {
            Value* bitBlockPtr =  b->CreateGEP(basePtr, b->CreateAdd(b->CreateMul(bitBlockIndex, b->getSize(mNumberOfStreams)), b->getSize(i)));
            bitBlockPtr = b->CreatePointerCast(bitBlockPtr, b->getInt64Ty()->getPointerTo());
            b->CreateStore(values[i], b->CreateGEP(bitBlockPtr, extractIndex));
        }

    }

    LZ4BitStreamMatchCopyKernel::LZ4BitStreamMatchCopyKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned numberOfStreams, std::string name)
            : SegmentOrientedKernel(std::move(name),
// Inputs
                                    {
                                            Binding{iBuilder->getStreamSetTy(1, 1), "MatchOffsetMarker", BoundedRate(0, 1)},
                                            Binding{iBuilder->getStreamSetTy(1, 1), "M0Marker", BoundedRate(0, 1)},
                                            Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", RateEqualTo("MatchOffsetMarker")},
                                            Binding{iBuilder->getStreamSetTy(numberOfStreams, 1), "inputBitStream",
                                                    RateEqualTo("M0Marker")},
                                    },
// Outputs
                                    {
                                            Binding{iBuilder->getStreamSetTy(numberOfStreams, 1), "outputBitStream",
                                                    RateEqualTo("M0Marker")}
                                    },
// Arguments
                                    {
                                    },
                                    {},
                                    {
                    Binding(iBuilder->getSizeTy(), "currentOffsetMarkerPos"),
            })
            , mNumberOfStreams(numberOfStreams) {

        setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

}
