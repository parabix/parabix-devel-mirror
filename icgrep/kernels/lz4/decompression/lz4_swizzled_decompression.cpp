
#include "lz4_swizzled_decompression.h"



#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

#define PHINotChange(b, phi) phi->addIncoming(phi, b->GetInsertBlock())

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel{

    LZ4SwizzledDecompressionKernel::LZ4SwizzledDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned streamCount,
                                               unsigned streamSize, unsigned swizzleFactor,
                                               unsigned blockSize):
    LZ4SequentialDecompressionKernel(b, "LZ4SwizzledDecompressionKernel", blockSize),
    mStreamCount(streamCount),
    mStreamSize(streamSize),
    mSwizzleFactor(swizzleFactor),
    mPDEPWidth(b->getBitBlockWidth() / mSwizzleFactor)
    {
//        assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(streamCount), "sourceStreamSet0", RateEqualTo("byteStream"), {Swizzled(), AlwaysConsume()}});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(streamCount), "outputStreamSet0", BoundedRate(0, 1)});

        for (unsigned i = 1; i < streamSize; i++) {
            mStreamSetInputs.push_back(Binding{b->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0"), {Swizzled(), AlwaysConsume()}});
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("outputStreamSet0")});
        }

        this->addScalar(b->getSizeTy(), "accelerationOutputIndex");
        this->addScalar(ArrayType::get(b->getSizeTy(), 48), "literalStartArray");
        this->addScalar(ArrayType::get(b->getSizeTy(), 48), "literalLengthArray");
        this->addScalar(ArrayType::get(b->getSizeTy(), 48), "matchOffsetArray");
        this->addScalar(ArrayType::get(b->getSizeTy(), 48), "matchLengthArray");
    }

    void LZ4SwizzledDecompressionKernel::prepareAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos) {
        b->setScalarField("accelerationOutputIndex", b->getSize(0));
        /*
        std::vector<Value*> inputValuesVector = std::vector<Value*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* sourceBasePtr = b->CreatePointerCast(b->getRawInputPointer("sourceStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            Value* inputValue = b->CreateLoad(b->CreateGEP(sourceBasePtr, literalBlockIndex));
            inputValuesVector.push_back(inputValue);
        }
        */
    }

    void LZ4SwizzledDecompressionKernel::doAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                           llvm::Value *literalLength, llvm::Value* blockStart) {
//        this->handleAccelerationLiteralCopy(b, literalStart, literalLength, inputValuesVector);

        Type* sizePtrTy = b->getSizeTy()->getPointerTo();
        Value* outputIndex = b->getScalarField("accelerationOutputIndex");

        Value* literalStartArray = b->CreatePointerCast(b->getScalarFieldPtr("literalStartArray"), sizePtrTy);
        Value* literalLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("literalLengthArray"), sizePtrTy);
        b->CreateStore(literalStart, b->CreateGEP(literalStartArray, outputIndex));
        b->CreateStore(literalLength, b->CreateGEP(literalLengthArray, outputIndex));

        b->setScalarField("accelerationOutputIndex", b->CreateAdd(outputIndex, b->getSize(1)));
    }

    void LZ4SwizzledDecompressionKernel::doAccelerationMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                         llvm::Value *matchLength) {
//        this->handleMatchCopy(b, matchOffset, matchLength);

        Type* sizePtrTy = b->getSizeTy()->getPointerTo();
        Value* outputIndex = b->getScalarField("accelerationOutputIndex");
        Value* matchOffsetArray = b->CreatePointerCast(b->getScalarFieldPtr("matchOffsetArray"), sizePtrTy);
        Value* matchLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("matchLengthArray"), sizePtrTy);
        b->CreateStore(matchOffset, b->CreateGEP(matchOffsetArray, outputIndex));
        b->CreateStore(matchLength, b->CreateGEP(matchLengthArray, outputIndex));
        b->setScalarField("accelerationOutputIndex", b->CreateAdd(outputIndex, b->getSize(1)));
    }

    void LZ4SwizzledDecompressionKernel::finishAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value* beginTokenPos, llvm::Value* literalMask) {
        Value* inputStreamSize = b->getCapacity("sourceStreamSet0");
        Value* SIZE_64 = b->getSize(64);
        Value* literalBlockIndex = b->CreateUDiv(b->CreateURem(beginTokenPos, inputStreamSize), SIZE_64);

        Value* outputIndex = b->getScalarField("accelerationOutputIndex");
        Type* sizePtrTy = b->getSizeTy()->getPointerTo();
//        Value* literalStartArray = b->CreatePointerCast(b->getScalarFieldPtr("literalStartArray"), sizePtrTy);
        Value* literalLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("literalLengthArray"), sizePtrTy);
        Value* matchOffsetArray = b->CreatePointerCast(b->getScalarFieldPtr("matchOffsetArray"), sizePtrTy);
        Value* matchLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("matchLengthArray"), sizePtrTy);
        this->handleAccelerationPdepOutput(
                b,
                literalBlockIndex,
                literalMask,
                literalLengthArray,
                matchOffsetArray,
                matchLengthArray,
                outputIndex
        );

        this->handleAccelerationMatchCopyOutput(
                b,
                literalLengthArray,
                matchOffsetArray,
                matchLengthArray,
                outputIndex
        );
    }

    void LZ4SwizzledDecompressionKernel::handleAccelerationMatchCopyOutput(
            const std::unique_ptr<KernelBuilder> &b,
            llvm::Value *literalLengthArray,
            llvm::Value *matchOffsetArray,
            llvm::Value *matchLengthArray,
            llvm::Value *numOfElements
    ) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);

        Value* initOutputPos = b->getScalarField("outputPos");

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* matchCopyConBlock = b->CreateBasicBlock("matchCopyConBlock");
        BasicBlock* matchCopyBodyBlock = b->CreateBasicBlock("matchCopyBodyBlock");
        BasicBlock* matchCopyExitBlock = b->CreateBasicBlock("matchCopyExitBlock");


        b->CreateBr(matchCopyConBlock);

        // ---- matchCopyConBlock
        b->SetInsertPoint(matchCopyConBlock);

        PHINode* phiMatchCopyDataIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiMatchCopyDataIndex->addIncoming(SIZE_0, entryBlock);
        PHINode* phiMatchCopyOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiMatchCopyOutputPos->addIncoming(initOutputPos, entryBlock);

//        b->CallPrintInt("phiMatchCopyDataIndex", phiMatchCopyDataIndex);
        b->CreateCondBr(b->CreateICmpULT(phiMatchCopyDataIndex, numOfElements), matchCopyBodyBlock, matchCopyExitBlock);

        // ---- matchCopyBodyBlock
        b->SetInsertPoint(matchCopyBodyBlock);

        Value* currentLiteralLength = b->CreateLoad(b->CreateGEP(literalLengthArray, phiMatchCopyDataIndex));
        Value* currentMatchOffset = b->CreateLoad(b->CreateGEP(matchOffsetArray, phiMatchCopyDataIndex));
        Value* currentMatchLength = b->CreateLoad(b->CreateGEP(matchLengthArray, phiMatchCopyDataIndex));
        Value* matchPos = b->CreateAdd(phiMatchCopyOutputPos, currentLiteralLength);
        this->handleMatchCopy(b, matchPos, currentMatchOffset, currentMatchLength, false);

        phiMatchCopyDataIndex->addIncoming(b->CreateAdd(phiMatchCopyDataIndex, SIZE_1), b->GetInsertBlock());
        phiMatchCopyOutputPos->addIncoming(b->CreateAdd(matchPos, currentMatchLength), b->GetInsertBlock());

        b->CreateBr(matchCopyConBlock);

        // ---- matchCopyExitBlock
        b->SetInsertPoint(matchCopyExitBlock);
        b->setScalarField("outputPos", phiMatchCopyOutputPos);

    }

    void LZ4SwizzledDecompressionKernel::handleAccelerationPdepOutput(
            const std::unique_ptr<KernelBuilder> &b,
            Value *literalBlockIndex,
            Value *literalMasks,
            Value *literalLengthArray,
            Value *matchOffsetArray,
            Value *matchLengthArray,
            Value *numOfElements
    ) {
        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();
        Value* SIZE_OUTPUT_BLOCKS_COUNT = b->CreateUDiv(b->getCapacity("outputStreamSet0"), SIZE_64);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        std::vector<Value*> initExtractedLiteral = std::vector<Value*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* sourceBasePtr = b->CreatePointerCast(b->getRawInputPointer("sourceStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            Value* inputValue = b->CreateLoad(b->CreateGEP(sourceBasePtr, literalBlockIndex));
            Value* extractedInputValue = b->simd_pext(64, inputValue, b->simd_fill(64, literalMasks));
            initExtractedLiteral.push_back(extractedInputValue);
        }
        Value* initOutputPos = b->getScalarField("outputPos");
        Value* initOutputBlockIndex = b->CreateUDiv(initOutputPos, SIZE_64);
        Value* initOutputPosRem = b->CreateURem(initOutputPos, SIZE_64);
        Value* initOutputPosMask = b->CreateSub(b->CreateShl(SIZE_1, initOutputPosRem), SIZE_1);
        std::vector<Value*> initOutputValues = std::vector<Value*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            Value* outputValue = b->CreateLoad(b->CreateGEP(outputBasePtr, b->CreateURem(initOutputBlockIndex, SIZE_OUTPUT_BLOCKS_COUNT)));
            initOutputValues.push_back(b->CreateAnd(outputValue, b->simd_fill(64, initOutputPosMask)));
        }

        // PDEP part will be responsible for clearing the output space

        BasicBlock* pdepConBlock = b->CreateBasicBlock("pdepConBlock");
        BasicBlock* pdepStoreOutputBlock = b->CreateBasicBlock("pdepStoreOutputBlock");
        BasicBlock* pdepCheckAvailableDataBlock = b->CreateBasicBlock("pdepCheckAvailableDataBlock");
        BasicBlock* pdepBuildMarkerBlock = b->CreateBasicBlock("pdepBuildMarkerBlock");
        BasicBlock* pdepBuildMarkerReachNextBlock = b->CreateBasicBlock("pdepBuildMarkerReachNextBlock");
        BasicBlock* pdepBuildMarkerNotReachNextBlock = b->CreateBasicBlock("pdepBuildMarkerNotReachNextBlock");
        BasicBlock* pdepExitBlock = b->CreateBasicBlock("pdepExitBlock");

        b->CreateBr(pdepConBlock);

        // ---- pdepConBlock
        b->SetInsertPoint(pdepConBlock);
        std::vector<PHINode*> remainingExtractedLiteral = std::vector<PHINode*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* initValue = initExtractedLiteral[i];
            PHINode* n = b->CreatePHI(initValue->getType(), 4);
            n->addIncoming(initValue, entryBlock);
            remainingExtractedLiteral.push_back(n);
        }
        PHINode* phiPdepDataIndex = b->CreatePHI(b->getSizeTy(), 4);
        phiPdepDataIndex->addIncoming(SIZE_0, entryBlock);
        PHINode* phiPdepOutputPos = b->CreatePHI(b->getSizeTy(), 4);
        phiPdepOutputPos->addIncoming(initOutputPos, entryBlock);
        PHINode* phiCurrentOutputBlockIndex = b->CreatePHI(b->getSizeTy(), 4);
        phiCurrentOutputBlockIndex->addIncoming(initOutputBlockIndex, entryBlock);
        PHINode* phiPdepMarker = b->CreatePHI(b->getSizeTy(), 4);
        phiPdepMarker->addIncoming(SIZE_0, entryBlock);
        std::vector<PHINode*> phiOriginalOutputValues = std::vector<PHINode*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* initOutputValue = initOutputValues[i];
            PHINode* originalOutputValue = b->CreatePHI(initOutputValue->getType(), 4);
            originalOutputValue->addIncoming(initOutputValue, entryBlock);
            phiOriginalOutputValues.push_back(originalOutputValue);
        }
        PHINode* phiUseTemporaryLiteralLength = b->CreatePHI(b->getInt1Ty(), 4);
        phiUseTemporaryLiteralLength->addIncoming(b->getInt1(false), entryBlock);
        PHINode* phiTemporaryLiteralLength = b->CreatePHI(b->getSizeTy(), 4);
        phiTemporaryLiteralLength->addIncoming(SIZE_0, entryBlock);


        Value* pdepOutputPosBlockIndex = b->CreateUDiv(phiPdepOutputPos, SIZE_64);

        b->CreateCondBr(b->CreateICmpEQ(pdepOutputPosBlockIndex, phiCurrentOutputBlockIndex), pdepCheckAvailableDataBlock, pdepStoreOutputBlock);

        // ---- pdepStoreOutputBlock
        b->SetInsertPoint(pdepStoreOutputBlock);
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* pdepResult = b->simd_pdep(64, remainingExtractedLiteral[i], b->simd_fill(64, phiPdepMarker));
            Value* outputValue = b->CreateOr(pdepResult, phiOriginalOutputValues[i]);
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            b->CreateStore(outputValue, b->CreateGEP(outputBasePtr, b->CreateURem(phiCurrentOutputBlockIndex, SIZE_OUTPUT_BLOCKS_COUNT)));
        }
        Value* pdepMarkerPopcount = b->CreatePopcount(phiPdepMarker);
        for (unsigned i = 0; i < mStreamSize; i++) {
            remainingExtractedLiteral[i]->addIncoming(b->simd_srlv(64, remainingExtractedLiteral[i], b->simd_fill(64, pdepMarkerPopcount)), b->GetInsertBlock());
            phiOriginalOutputValues[i]->addIncoming(Constant::getNullValue(b->getBitBlockType()), b->GetInsertBlock());
        }
        PHINotChange(b, phiPdepDataIndex);
        PHINotChange(b, phiPdepOutputPos);
        PHINotChange(b, phiUseTemporaryLiteralLength);
        PHINotChange(b, phiTemporaryLiteralLength);
        phiCurrentOutputBlockIndex->addIncoming(b->CreateAdd(phiCurrentOutputBlockIndex, SIZE_1), b->GetInsertBlock());
        phiPdepMarker->addIncoming(SIZE_0, b->GetInsertBlock());
        b->CreateBr(pdepConBlock);

        // ---- pdepCheckAvailableDataBlock
        b->SetInsertPoint(pdepCheckAvailableDataBlock);
        b->CreateCondBr(b->CreateICmpULT(phiPdepDataIndex, numOfElements), pdepBuildMarkerBlock, pdepExitBlock);

        // ---- pdepBuildMarkerBlock
        b->SetInsertPoint(pdepBuildMarkerBlock);
        Value* currentLiteralLength =  b->CreateSelect(
                phiUseTemporaryLiteralLength,
                phiTemporaryLiteralLength,
                b->CreateLoad(b->CreateGEP(literalLengthArray, phiPdepDataIndex))
        );

        Value* currentMatchLength = b->CreateLoad(b->CreateGEP(matchLengthArray, phiPdepDataIndex));

        Value* pdepOutputPosRem = b->CreateURem(phiPdepOutputPos, SIZE_64);
        Value* literalEndRem = b->CreateAdd(pdepOutputPosRem, currentLiteralLength);
        Value* pdepOutputRemainingLength = b->CreateSub(SIZE_64, pdepOutputPosRem);
        Value* newPdepMarker = b->CreateOr(
                phiPdepMarker,
                b->CreateSub(
                        b->CreateSelect(b->CreateICmpUGE(literalEndRem, SIZE_64), SIZE_0, b->CreateShl(SIZE_1, literalEndRem)),
                        b->CreateShl(SIZE_1, pdepOutputPosRem)
                )
        );

        b->CreateCondBr(b->CreateICmpUGT(literalEndRem, SIZE_64), pdepBuildMarkerReachNextBlock, pdepBuildMarkerNotReachNextBlock);

        // ---- pdepBuildMarkerReachNextBlock
        b->SetInsertPoint(pdepBuildMarkerReachNextBlock);
        Value* remainingLiteralLength = b->CreateSub(currentLiteralLength, pdepOutputRemainingLength);
        for (unsigned i = 0; i < mStreamSize; i++) {
            PHINotChange(b, remainingExtractedLiteral[i]);
            PHINotChange(b, phiOriginalOutputValues[i]);
        }
        PHINotChange(b, phiPdepDataIndex);
        PHINotChange(b, phiCurrentOutputBlockIndex);
        phiTemporaryLiteralLength->addIncoming(remainingLiteralLength, b->GetInsertBlock());
        phiUseTemporaryLiteralLength->addIncoming(b->getInt1(true), b->GetInsertBlock());
        phiPdepMarker->addIncoming(newPdepMarker, b->GetInsertBlock());
        phiPdepOutputPos->addIncoming(b->CreateAdd(phiPdepOutputPos, pdepOutputRemainingLength), b->GetInsertBlock());
        b->CreateBr(pdepConBlock);

        // ---- pdepBuildMarkerNotReachNextBlock
        b->SetInsertPoint(pdepBuildMarkerNotReachNextBlock);
        for (unsigned i = 0; i < mStreamSize; i++) {
            PHINotChange(b, remainingExtractedLiteral[i]);
            PHINotChange(b, phiOriginalOutputValues[i]);
        }
        phiPdepDataIndex->addIncoming(b->CreateAdd(phiPdepDataIndex, SIZE_1), b->GetInsertBlock());
        phiPdepOutputPos->addIncoming(b->CreateAdd(phiPdepOutputPos, b->CreateAdd(currentLiteralLength, currentMatchLength)), b->GetInsertBlock());
        phiPdepMarker->addIncoming(newPdepMarker, b->GetInsertBlock());
        PHINotChange(b, phiCurrentOutputBlockIndex);
        phiTemporaryLiteralLength->addIncoming(SIZE_0, b->GetInsertBlock());
        phiUseTemporaryLiteralLength->addIncoming(b->getInt1(false), b->GetInsertBlock());

        b->CreateBr(pdepConBlock);

        // ---- pdepExitBlock
        b->SetInsertPoint(pdepExitBlock);
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* pdepResult = b->simd_pdep(64, remainingExtractedLiteral[i], b->simd_fill(64, phiPdepMarker));
            Value* outputValue = b->CreateOr(pdepResult, phiOriginalOutputValues[i]);
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            b->CreateStore(outputValue, b->CreateGEP(outputBasePtr, b->CreateURem(phiCurrentOutputBlockIndex, SIZE_OUTPUT_BLOCKS_COUNT)));
            b->CreateStore(outputValue, b->CreateGEP(outputBasePtr, b->CreateURem(phiCurrentOutputBlockIndex, SIZE_OUTPUT_BLOCKS_COUNT)));
//            b->CallPrintRegister("pdepOutputValue" + std::to_string(i), outputValue);
        }
    }

    void LZ4SwizzledDecompressionKernel::handleAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                 llvm::Value *literalLength, const std::vector<llvm::Value*>& inputLiteralValues) {
        Value* SIZE_64  = b->getSize(64);
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);

//        Value* inputStreamSize = b->getCapacity("sourceStreamSet0");
        Value* outputStreamSize = b->getCapacity("outputStreamSet0");

        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();


        Value* outputPos = b->getScalarField("outputPos");

        BasicBlock* literalCopyConBlock = b->CreateBasicBlock("literalCopyConBlock");
        BasicBlock* literalCopyBodyBlock = b->CreateBasicBlock("literalCopyBodyBlock");
        BasicBlock* literalCopyExitBlock = b->CreateBasicBlock("literalCopyExitBlock");

        vector<Value*> outputStreamBasePtrs; // <4 * i64>*
        for (unsigned i = 0; i < mStreamSize; i++) {
            outputStreamBasePtrs.push_back(b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
        }

        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyConBlock
        b->SetInsertPoint(literalCopyConBlock);

        b->CreateCondBr(b->CreateICmpNE(literalLength, b->getSize(0)), literalCopyBodyBlock, literalCopyExitBlock);

        // ---- literalCopyBodyBlock
        b->SetInsertPoint(literalCopyBodyBlock);

        Value* literalRem = b->CreateURem(literalStart, SIZE_64);

        Value* outputPosRem = b->CreateURem(outputPos, SIZE_64);
        Value* remainingOutputPos = b->CreateSub(SIZE_64, outputPosRem);
        Value* outputBlockIndex = b->CreateUDiv(b->CreateURem(outputPos, outputStreamSize), SIZE_64);

        Value* copySize1 = b->CreateUMin(literalLength, remainingOutputPos);
        Value* copySize2 = b->CreateSub(literalLength, copySize1);


        Value* l1 = b->CreateSub(SIZE_64, b->CreateAdd(literalRem, copySize1));
        Value* l1_1 = b->simd_fill(64, l1);
        Value* r1_1 = b->simd_fill(64, b->CreateAdd(l1, literalRem));
        Value* l1_2 = b->simd_fill(64, outputPosRem);
        Value* clearMask1 = b->simd_fill(64, b->CreateSub(b->CreateShl(SIZE_1, outputPosRem), SIZE_1)); // It seems that we do not need to clear it every time

        Value* literalRem2 = b->CreateAdd(literalRem, copySize1);
        Value* l2 = b->CreateSub(SIZE_64, b->CreateAdd(literalRem2, copySize2));
        Value* l2_1 = b->simd_fill(64, l2);
        Value* r2_1 = b->simd_fill(64, b->CreateAdd(l2, literalRem2));
        Value* l2_2 = b->simd_fill(64, SIZE_0);
        Value* clearMask2 = b->simd_fill(64, b->CreateSub(b->CreateShl(SIZE_1, SIZE_0), SIZE_1));


        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* inputValue = inputLiteralValues[i];

            Value* outputPtr1 = b->CreateGEP(outputStreamBasePtrs[i], outputBlockIndex);
            Value* outputValue1 = b->CreateLoad(outputPtr1);
            Value* inputValue1 = b->simd_sllv(64, inputValue, l1_1);
            inputValue1 = b->simd_srlv(64, inputValue1, r1_1);
            inputValue1 = b->simd_sllv(64, inputValue1, l1_2);
            outputValue1 = b->CreateOr(b->CreateAnd(outputValue1, clearMask1), inputValue1);
            b->CreateStore(outputValue1, outputPtr1);

            Value* outputPtr2 = b->CreateGEP(outputPtr1, SIZE_1);
            Value* outputValue2 = b->CreateLoad(outputPtr2);
            Value* oldOutputValue2 = outputValue2;
            Value* inputValue2 = b->simd_sllv(64, inputValue, l2_1);
            inputValue2 = b->simd_srlv(64, inputValue2, r2_1);
            inputValue2 = b->simd_sllv(64, inputValue2, l2_2);
            outputValue2 = b->CreateOr(b->CreateAnd(outputValue2, clearMask2), inputValue2);
            outputValue2 = b->CreateSelect(b->CreateICmpEQ(copySize2, SIZE_0), oldOutputValue2, outputValue2);
            b->CreateStore(outputValue2, outputPtr2);
        }

        b->CreateBr(literalCopyExitBlock);

        // ---- literalCopyExitBlock
        b->SetInsertPoint(literalCopyExitBlock);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }

    void LZ4SwizzledDecompressionKernel::handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* outputPos, llvm::Value* matchOffset, llvm::Value* matchLength, bool clearBuffer) {
        Value* SIZE_64  = b->getSize(64);
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);

        Value* outputStreamSize = b->getCapacity("outputStreamSet0");
        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();

        vector<Value*> outputStreamBasePtrs; // <4 * i64>*
        for (unsigned i = 0; i < mStreamSize; i++) {
            outputStreamBasePtrs.push_back(b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
        }

        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* matchCopyCon = b->CreateBasicBlock("matchCopyCon");
        BasicBlock* matchCopyBody = b->CreateBasicBlock("matchCopyBody");
        BasicBlock* matchCopyExit = b->CreateBasicBlock("matchCopyExit");
        b->CreateBr(matchCopyCon);

        // ---- matchCopyCon
        b->SetInsertPoint(matchCopyCon);
        PHINode* phiMatchLength = b->CreatePHI(b->getSizeTy(), 2);
        phiMatchLength->addIncoming(matchLength, entryBlock);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(outputPos, entryBlock);

        b->CreateCondBr(b->CreateICmpNE(phiMatchLength, b->getSize(0)), matchCopyBody, matchCopyExit);

        // ---- matchCopyBody
        b->SetInsertPoint(matchCopyBody);
//        b->CallPrintInt("phiMatchLength", phiMatchLength);

        Value* copyFromPos = b->CreateSub(phiOutputPos, matchOffset);

        Value* outputFromPosBlockIndex = b->CreateUDiv(b->CreateURem(copyFromPos, outputStreamSize), SIZE_64);
        Value* outputFromPosRem = b->CreateURem(copyFromPos, SIZE_64);

        Value* outputToPosBlockIndex = b->CreateUDiv(b->CreateURem(phiOutputPos, outputStreamSize), SIZE_64);
        Value* outputToPosRem = b->CreateURem(phiOutputPos, SIZE_64);

        Value* copySize = b->CreateSub(SIZE_64, b->CreateUMax(outputFromPosRem, outputToPosRem));
        copySize = b->CreateUMin(copySize, matchOffset);
        copySize = b->CreateUMin(copySize, phiMatchLength);

        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* inputValue = b->CreateLoad(b->CreateGEP(outputStreamBasePtrs[i], outputFromPosBlockIndex));
            Value* outputValue = b->CreateLoad(b->CreateGEP(outputStreamBasePtrs[i], outputToPosBlockIndex));

            Value* l1 = b->CreateSub(SIZE_64, b->CreateAdd(outputFromPosRem, copySize));
            inputValue = b->simd_sllv(64, inputValue, b->simd_fill(64, l1));
            inputValue = b->simd_srlv(64, inputValue, b->simd_fill(64, b->CreateAdd(l1, outputFromPosRem)));
            inputValue = b->simd_sllv(64, inputValue, b->simd_fill(64,outputToPosRem));

            if (clearBuffer) {
                Value* outputClearMask = b->simd_fill(64, b->CreateSub(b->CreateShl(SIZE_1, outputToPosRem), SIZE_1));
                outputValue = b->CreateAnd(outputValue, outputClearMask);
            }

            outputValue = b->CreateOr(outputValue, inputValue);

            b->CreateStore(outputValue, b->CreateGEP(outputStreamBasePtrs[i], outputToPosBlockIndex));

        }

        phiMatchLength->addIncoming(b->CreateSub(phiMatchLength, copySize), b->GetInsertBlock());
        phiOutputPos->addIncoming(b->CreateAdd(phiOutputPos, copySize), b->GetInsertBlock());
        b->CreateBr(matchCopyCon);

        // ---- matchCopyExit
        b->SetInsertPoint(matchCopyExit);
    }

    void LZ4SwizzledDecompressionKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) {
        b->setProducedItemCount("outputStreamSet0", produced);
    }

    void LZ4SwizzledDecompressionKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                             llvm::Value *matchLength) {
        Value* outputPos = b->getScalarField("outputPos");
        this->handleMatchCopy(b, outputPos, matchOffset, matchLength);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }

    void LZ4SwizzledDecompressionKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                             llvm::Value *literalLength, llvm::Value* blockStart) {
        Value* SIZE_64  = b->getSize(64);
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);

        Value* inputStreamSize = b->getCapacity("sourceStreamSet0");
        Value* outputStreamSize = b->getCapacity("outputStreamSet0");

        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();


        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* outputPos = b->getScalarField("outputPos");

        BasicBlock* literalCopyConBlock = b->CreateBasicBlock("literalCopyConBlock");
        BasicBlock* literalCopyBodyBlock = b->CreateBasicBlock("literalCopyBodyBlock");
        BasicBlock* literalCopyExitBlock = b->CreateBasicBlock("literalCopyExitBlock");

        vector<Value*> sourceStreamBasePtrs, outputStreamBasePtrs; // <4 * i64>*
        for (unsigned i = 0; i < mStreamSize; i++) {
            sourceStreamBasePtrs.push_back(b->CreatePointerCast(b->getRawInputPointer("sourceStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE, "aa" + std::to_string(i)));
            outputStreamBasePtrs.push_back(b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
        }

        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyConBlock
        b->SetInsertPoint(literalCopyConBlock);
        PHINode* phiLiteralStart = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralStart->addIncoming(literalStart, entryBlock);
        PHINode* phiLiteralLength = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralLength->addIncoming(literalLength, entryBlock);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(outputPos, entryBlock);

        b->CreateCondBr(b->CreateICmpNE(phiLiteralLength, b->getSize(0)), literalCopyBodyBlock, literalCopyExitBlock);

        // ---- literalCopyBodyBlock
        b->SetInsertPoint(literalCopyBodyBlock);

        Value* literalRem = b->CreateURem(phiLiteralStart, SIZE_64);
        Value* literalBlockIndex = b->CreateUDiv(b->CreateURem(phiLiteralStart, inputStreamSize), SIZE_64);

        Value* outputPosRem = b->CreateURem(phiOutputPos, SIZE_64);
        Value* outputBlockIndex = b->CreateUDiv(b->CreateURem(phiOutputPos, outputStreamSize), SIZE_64);

        Value* copySize = b->CreateSub(SIZE_64, b->CreateUMax(literalRem, outputPosRem));
        copySize = b->CreateUMin(copySize, phiLiteralLength);

        Value* l1 = b->CreateSub(SIZE_64, b->CreateAdd(literalRem, copySize));
        Value* l1_ = b->simd_fill(64, l1);
        Value* r1 = b->simd_fill(64, b->CreateAdd(l1, literalRem));
        Value* l2 = b->simd_fill(64, outputPosRem);
        Value* clearMask = b->simd_fill(64, b->CreateSub(b->CreateShl(SIZE_1, outputPosRem), SIZE_1));

        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* inputValue = b->CreateLoad(b->CreateGEP(sourceStreamBasePtrs[i], literalBlockIndex));
            Value* outputValue = b->CreateLoad(b->CreateGEP(outputStreamBasePtrs[i], outputBlockIndex));

            inputValue = b->simd_sllv(64, inputValue, l1_);
            inputValue = b->simd_srlv(64, inputValue, r1);
            inputValue = b->simd_sllv(64, inputValue, l2);
            outputValue = b->CreateOr(b->CreateAnd(outputValue, clearMask), inputValue);
            b->CreateStore(outputValue, b->CreateGEP(outputStreamBasePtrs[i], outputBlockIndex));
        }

        phiLiteralStart->addIncoming(b->CreateAdd(phiLiteralStart, copySize), b->GetInsertBlock());
        phiLiteralLength->addIncoming(b->CreateSub(phiLiteralLength, copySize), b->GetInsertBlock());
        phiOutputPos->addIncoming(b->CreateAdd(phiOutputPos, copySize), b->GetInsertBlock());
        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyExitBlock
        b->SetInsertPoint(literalCopyExitBlock);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }

}