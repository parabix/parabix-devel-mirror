
#include "lz4_swizzled_aio.h"



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

    LZ4SwizzledAioKernel::LZ4SwizzledAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned streamCount, unsigned streamSize, unsigned swizzleFactor)
            :SegmentOrientedKernel("LZ4SwizzledAioKernel",
            // Inputs
                                   {
                                           Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},
                                           Binding{b->getStreamSetTy(1, 1), "extender", RateEqualTo("byteStream")},

                                           // block data
                                           Binding{b->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1), AlwaysConsume()},
                                           Binding{b->getStreamSetTy(1, 64), "blockStart", RateEqualTo("isCompressed"), AlwaysConsume()},
                                           Binding{b->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("isCompressed"), AlwaysConsume()}

                                   },
            //Outputs
                                   {
//                                           Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)},
                                   },
            //Arguments
                                   {
                                           Binding{b->getSizeTy(), "fileSize"}
                                   },
                                   {},
            //Internal states:
                                   {
                                           Binding{b->getInt64Ty(), "tempTimes"},
                                           Binding{b->getSizeTy(), "blockDataIndex"},
                                           Binding{b->getInt64Ty(), "outputPos"},

                                           // Temp array
                                           Binding{ArrayType::get(b->getSizeTy(), 32), "literalStartArray"},
                                           Binding{ArrayType::get(b->getSizeTy(), 32), "literalLengthArray"},
                                           Binding{ArrayType::get(b->getSizeTy(), 32), "matchOffsetArray"},
                                           Binding{ArrayType::get(b->getSizeTy(), 32), "matchLengthArray"},

                                   }),
             mStreamCount(streamCount),
             mStreamSize(streamSize),
             mSwizzleFactor(swizzleFactor),
             mPDEPWidth(b->getBitBlockWidth() / mSwizzleFactor)
    {
//        assert((mPDEPWidth == 64 || mPDEPWidth == 32) && "PDEP width must be 32 or 64");

        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());

        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(streamCount), "sourceStreamSet0", RateEqualTo("byteStream"), {Swizzled(), AlwaysConsume()}});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(streamCount), "outputStreamSet0", BoundedRate(0, 1)});

        for (unsigned i = 1; i < streamSize; i++) {
            mStreamSetInputs.push_back(Binding{b->getStreamSetTy(streamCount), "sourceStreamSet" + std::to_string(i), RateEqualTo("sourceStreamSet0"), {Swizzled(), AlwaysConsume()}});
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(streamCount), "outputStreamSet" + std::to_string(i), RateEqualTo("outputStreamSet0")});
        }

    }

    void LZ4SwizzledAioKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getScalarField("blockDataIndex");

        // In MultiblockKernel, availableItemCount + processedItemCount == producedItemCount from previous kernel
        // While in SegmentOrigentedKernel, availableItemCount == producedItemCount from previous kernel
        Value * totalNumber = b->getAvailableItemCount("blockEnd");
        Value * totalExtender = b->getAvailableItemCount("extender");
        Value * totalSource = b->getAvailableItemCount("sourceStreamSet0");

        Value * blockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);

//        b->CallPrintInt("blockEnd", blockEnd);
//        b->CallPrintInt("processedSource", b->getProcessedItemCount("sourceStreamSet0"));
//        b->CallPrintInt("totalSource", totalSource);
//        b->CallPrintInt("fileSize", b->getScalarField("fileSize"));

        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        b->SetInsertPoint(blockEndConBlock);
        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);
        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
        b->CreateCondBr(b->CreateICmpULE(blockEnd, b->CreateUMin(totalExtender, totalSource)), processBlock, exitBlock);

        b->SetInsertPoint(processBlock);

        //TODO handle uncompressed block
        this->generateProcessCompressedBlock(b, blockStart, blockEnd);

        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("isCompressed", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);

        b->setProducedItemCount("outputStreamSet0", b->getScalarField("outputPos"));

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->CallPrintIntCond("time", b->getScalarField("tempTimes"), b->getTerminationSignal());
    }

    llvm::Value *LZ4SwizzledAioKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset) {
        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    void
    LZ4SwizzledAioKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                                           llvm::Value *lz4BlockEnd) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");

        BasicBlock* beforeProcessConBlock = b->GetInsertBlock();
        b->CreateBr(processCon);
        b->SetInsertPoint(processCon);


        PHINode* phiCursorValue = b->CreatePHI(b->getInt64Ty(), 2, "phiCursorValue"); // phiCursorValue should always be the position of next token except for the final sequence
        phiCursorValue->addIncoming(lz4BlockStart, beforeProcessConBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCursorValue, lz4BlockEnd), processBody, exitBlock);

        b->SetInsertPoint(processBody);

/*
        auto accelerationRet = this->generateAcceleration(b, phiCursorValue, lz4BlockEnd);

        Value* tokenMarkers = accelerationRet.first.first;

        Value* cursorBlockPosBase = b->CreateSub(phiCursorValue, b->CreateURem(phiCursorValue, b->getSize(ACCELERATION_WIDTH)));
        Value* nextTokenLocalPos = b->CreateSub(b->CreateSub(b->getSize(ACCELERATION_WIDTH), b->CreateCountReverseZeroes(tokenMarkers)), b->getSize(1));
        Value* nextTokenGlobalPos = b->CreateAdd(cursorBlockPosBase, nextTokenLocalPos);

        nextTokenGlobalPos = this->processBlockBoundary(b, nextTokenGlobalPos, lz4BlockEnd);
*/
        Value* nextTokenGlobalPos = this->processBlockBoundary(b, phiCursorValue, lz4BlockEnd);

        phiCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
    }

    std::pair<std::pair<llvm::Value *, llvm::Value *>, llvm::Value *>
    LZ4SwizzledAioKernel::generateAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                                                 llvm::Value *blockEnd) {

        Value* time1 = b->CreateReadCycleCounter();
        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* outputProducingBlock = b->CreateBasicBlock("outputProducingBlock");

        // Constant
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_64 = b->getSize(64);
        Value* SIZE_ACCELERATION_WIDTH = b->getSize(ACCELERATION_WIDTH);
        Value* inputStreamSize = b->getCapacity("sourceStreamSet0");

        Type* INT_ACCELERATION_TYPE = b->getIntNTy(ACCELERATION_WIDTH);
        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();

        Value* INT_ACCELERATION_0 = b->getIntN(ACCELERATION_WIDTH, 0);
        Value* INT_ACCELERATION_1 = b->getIntN(ACCELERATION_WIDTH, 1);

        Type* sizePtrTy = b->getSizeTy()->getPointerTo();

        Value* literalStartArray = b->CreatePointerCast(b->getScalarFieldPtr("literalStartArray"), sizePtrTy);
        Value* literalLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("literalLengthArray"), sizePtrTy);
        Value* matchOffsetArray = b->CreatePointerCast(b->getScalarFieldPtr("matchOffsetArray"), sizePtrTy);
        Value* matchLengthArray = b->CreatePointerCast(b->getScalarFieldPtr("matchLengthArray"), sizePtrTy);

        // ---- Entry Block

        Value* literalBlockIndex = b->CreateUDiv(b->CreateURem(beginTokenPos, inputStreamSize), SIZE_64);
        /*
        std::vector<Value*> inputValuesVector = std::vector<Value*>();
        for (unsigned i = 0; i < mStreamSize; i++) {
            Value* sourceBasePtr = b->CreatePointerCast(b->getRawInputPointer("sourceStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE);
            Value* inputValue = b->CreateLoad(b->CreateGEP(sourceBasePtr, literalBlockIndex));
            inputValuesVector.push_back(inputValue);
        }
        */

        Value* maskedTokenPos = b->CreateURem(beginTokenPos, b->getCapacity("extender"));
        Value* tokenPosRem = b->CreateURem(beginTokenPos, SIZE_ACCELERATION_WIDTH);
        Value* blockPosBase = b->CreateSub(beginTokenPos, tokenPosRem);

        Value* currentExtenderValue = b->CreateLoad(
                b->CreateGEP(
                        b->CreatePointerCast(b->getRawInputPointer("extender", SIZE_0), INT_ACCELERATION_TYPE->getPointerTo()),
                        b->CreateUDiv(maskedTokenPos, SIZE_ACCELERATION_WIDTH)
                )
        );

        Value* initTokenMarker = b->CreateShl(INT_ACCELERATION_1, tokenPosRem);
        BasicBlock* accelerationProcessBlock = b->CreateBasicBlock("accelerationProcessBlock");
        BasicBlock* accelerationExitBlock = b->CreateBasicBlock("accelerationExitBlock");
        b->CreateBr(accelerationProcessBlock);

        // ---- AccelerationProcessBlock
        b->SetInsertPoint(accelerationProcessBlock);
        PHINode* phiTokenMarkers = b->CreatePHI(INT_ACCELERATION_TYPE, 2);
        phiTokenMarkers->addIncoming(initTokenMarker, entryBlock);
        PHINode* phiLiteralMasks = b->CreatePHI(INT_ACCELERATION_TYPE, 2);
        phiLiteralMasks->addIncoming(INT_ACCELERATION_0, entryBlock);
        PHINode* phiMatchOffsetMarkers = b->CreatePHI(INT_ACCELERATION_TYPE, 2);
        phiMatchOffsetMarkers->addIncoming(INT_ACCELERATION_0, entryBlock);
        PHINode* phiOutputArrayPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputArrayPos->addIncoming(SIZE_0, entryBlock);


        Value* tokenReverseZeros = b->CreateCountReverseZeroes(phiTokenMarkers);
        Value* currentTokenLocalPos = b->CreateSub(b->CreateSub(SIZE_ACCELERATION_WIDTH, tokenReverseZeros), SIZE_1);
        Value* currentTokenMarker = b->CreateShl(INT_ACCELERATION_1, currentTokenLocalPos); // 1 marker is in Token Pos
        Value* currentTokenGlobalPos = b->CreateAdd(blockPosBase, currentTokenLocalPos);
        Value* tokenValue = b->CreateLoad(b->getRawInputPointer("byteStream", currentTokenGlobalPos));

        llvm::Value *literalLength, *literalLengthEndMarker;
        std::tie(literalLength, literalLengthEndMarker) =
                this->scanThruLiteralLength(b, currentTokenMarker, currentExtenderValue, tokenValue, blockPosBase,
                                            currentTokenLocalPos);
        Value* literalBeginMarker = b->CreateShl(literalLengthEndMarker, SIZE_1);
        Value* literalStartLocalPos = b->CreateCountForwardZeroes(literalBeginMarker);
        Value* literalStartGlobalPos = b->CreateAdd(blockPosBase, literalStartLocalPos);


        Value* literalEndMarker = b->CreateSelect(b->CreateICmpULT(literalLength, SIZE_ACCELERATION_WIDTH), b->CreateShl(literalBeginMarker, literalLength), INT_ACCELERATION_0);

        Value* newLiteralMask = b->CreateSub(literalEndMarker, literalBeginMarker);

        Value* newMatchOffsetStartMarker = literalEndMarker;

        Value *matchLength, *matchLengthEndMarker;
        std::tie(matchLength, matchLengthEndMarker) =
                this->scanThruMatchLength(b, b->CreateShl(newMatchOffsetStartMarker, SIZE_1), currentExtenderValue,
                                          tokenValue, blockPosBase);
        Value *newTokenMarker = b->CreateShl(matchLengthEndMarker, SIZE_1);
        Value* newTokenMarkerLocalPos = b->CreateCountForwardZeroes(newTokenMarker);

        Value* reachEnd = b->CreateOr(
                b->CreateICmpEQ(newTokenMarker, b->getSize(0)),
                b->CreateICmpUGE(b->CreateAdd(newTokenMarkerLocalPos, blockPosBase), blockEnd)
        );

        BasicBlock* dataProcessBlock = b->CreateBasicBlock("dataProcessBlock");
        b->CreateUnlikelyCondBr(reachEnd, outputProducingBlock, dataProcessBlock);

        // ---- dataProcessBlock
        b->SetInsertPoint(dataProcessBlock);

        Value* matchOffsetStartLocalPos = b->CreateCountForwardZeroes(newMatchOffsetStartMarker);
        Value* matchOffsetStartGlobalPos = b->CreateAdd(matchOffsetStartLocalPos, blockPosBase);

        Value* matchOffsetPtr = b->getRawInputPointer("byteStream", matchOffsetStartGlobalPos);
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        matchOffsetPtr = b->CreatePointerCast(matchOffsetPtr, b->getInt16Ty()->getPointerTo());
        Value* matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), b->getSizeTy());

        b->CreateStore(literalStartGlobalPos, b->CreateGEP(literalStartArray, phiOutputArrayPos));
        b->CreateStore(literalLength, b->CreateGEP(literalLengthArray, phiOutputArrayPos));
        b->CreateStore(matchOffset, b->CreateGEP(matchOffsetArray, phiOutputArrayPos));
        b->CreateStore(matchLength, b->CreateGEP(matchLengthArray, phiOutputArrayPos));

//        this->handleAccelerationLiteralCopy(b, literalStartGlobalPos, literalLength, inputValuesVector);
//        this->handleMatchCopy(b, matchOffset, matchLength);

        phiTokenMarkers->addIncoming(b->CreateOr(phiTokenMarkers, newTokenMarker), b->GetInsertBlock());
        phiLiteralMasks->addIncoming(b->CreateOr(phiLiteralMasks, newLiteralMask), b->GetInsertBlock());
        phiMatchOffsetMarkers->addIncoming(b->CreateOr(phiMatchOffsetMarkers, newMatchOffsetStartMarker), b->GetInsertBlock());
        phiOutputArrayPos->addIncoming(b->CreateAdd(phiOutputArrayPos, SIZE_1), b->GetInsertBlock());

        b->CreateBr(accelerationProcessBlock);

        // ---- outputProducingBlock

        b->SetInsertPoint(outputProducingBlock);
        Value* time2 = b->CreateReadCycleCounter();
        b->setScalarField("tempTimes", b->CreateAdd(b->getScalarField("tempTimes"), b->CreateSub(time2, time1)));


        this->handleAccelerationPdepOutput(
                b,
                literalBlockIndex,
                phiLiteralMasks,
                literalLengthArray,
                matchOffsetArray,
                matchLengthArray,
                phiOutputArrayPos
        );





        this->handleAccelerationMatchCopyOutput(
                b,
                literalBlockIndex,
                phiLiteralMasks,
                literalLengthArray,
                matchOffsetArray,
                matchLengthArray,
                phiOutputArrayPos
        );

        b->CreateBr(accelerationExitBlock);

        // ---- AccelerationExitBlock
        b->SetInsertPoint(accelerationExitBlock);
//        b->CallPrintInt("outputPos", b->getScalarField("outputPos"));
        return std::make_pair(std::make_pair(phiTokenMarkers, phiLiteralMasks), phiMatchOffsetMarkers);
    }

    void LZ4SwizzledAioKernel::handleAccelerationMatchCopyOutput(
            const std::unique_ptr<KernelBuilder> &b,
            llvm::Value *literalBlockIndex,
            llvm::Value *literalMasks,
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

    void LZ4SwizzledAioKernel::handleAccelerationPdepOutput(
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
//        b->CallPrintInt("-------", SIZE_0);
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
//            b->CallPrintRegister("pdepOutputValue" + std::to_string(i), outputValue);
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
//        b->CallPrintInt("phiPdepOutputPos", phiPdepOutputPos);
//        b->CallPrintInt("currentLiteralLength", currentLiteralLength);
//        b->CallPrintInt("phiPdepOutputPos", phiPdepOutputPos);


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

    llvm::Value *LZ4SwizzledAioKernel::processBlockBoundary(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                                                              llvm::Value *lz4BlockEnd) {
// Constant
        ConstantInt* SIZE_0 = b->getSize(0);
        ConstantInt* SIZE_1 = b->getSize(1);
        ConstantInt* BYTE_FF = b->getInt8(0xff);
        Value* BYTE_F0 = b->getInt8(0xf0);
        Value* BYTE_0F = b->getInt8(0x0f);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* tokenValue = b->CreateLoad(b->CreateGEP(bytePtrBase, beginTokenPos));

        Value* shouldExtendLiteral = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_F0), BYTE_F0);

        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);

        BasicBlock* extendLiteralCond = b->CreateBasicBlock("extendLiteralCond");
        BasicBlock* extendLiteralEnd = b->CreateBasicBlock("extendLiteralEnd");

        Value* initExtendLiteralPos = b->CreateAdd(beginTokenPos, b->getSize(1));

        b->CreateCondBr(shouldExtendLiteral, extendLiteralCond, extendLiteralEnd);

        // ---- extendLiteralCond
        b->SetInsertPoint(extendLiteralCond);
        PHINode* phiCurrentExtendLiteralPos = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentExtendLiteralPos->addIncoming(initExtendLiteralPos, entryBlock);
        PHINode* phiExtendLiteralLength = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendLiteralLength->addIncoming(SIZE_0, entryBlock);

        Value* currentLiteralLengthByte = b->CreateLoad(b->CreateGEP(bytePtrBase, phiCurrentExtendLiteralPos));
        Value* newExtendLiteralLength = b->CreateAdd(phiExtendLiteralLength, b->CreateZExt(currentLiteralLengthByte, b->getSizeTy()));

        phiCurrentExtendLiteralPos->addIncoming(b->CreateAdd(phiCurrentExtendLiteralPos, SIZE_1), b->GetInsertBlock());
        phiExtendLiteralLength->addIncoming(newExtendLiteralLength, b->GetInsertBlock());

        b->CreateCondBr(b->CreateICmpEQ(currentLiteralLengthByte, BYTE_FF), extendLiteralCond, extendLiteralEnd);

        // ---- extendLiteralEnd
        b->SetInsertPoint(extendLiteralEnd);
        PHINode* literalExtendValue = b->CreatePHI(b->getSizeTy(), 2);
        literalExtendValue->addIncoming(SIZE_0, entryBlock);
        literalExtendValue->addIncoming(newExtendLiteralLength, extendLiteralCond);
        PHINode* phiExtendLiteralEndPos = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendLiteralEndPos->addIncoming(beginTokenPos, entryBlock);
        phiExtendLiteralEndPos->addIncoming(phiCurrentExtendLiteralPos, extendLiteralCond);

        Value* literalLength = b->CreateAdd(literalExtendValue, b->CreateZExt(b->CreateLShr(tokenValue, b->getInt8(4)), b->getSizeTy()));

        Value* literalStartPos = b->CreateAdd(phiExtendLiteralEndPos, SIZE_1);
        Value* literalEndPos = b->CreateAdd(literalStartPos, literalLength);

        Value* matchOffsetBeginPos = literalEndPos;
        Value* matchOffsetNextPos = b->CreateAdd(matchOffsetBeginPos, SIZE_1);

        BasicBlock* hasMatchPartBlock = b->CreateBasicBlock("hasMatchPartBlock");
        BasicBlock* extendMatchCon = b->CreateBasicBlock("extendMatchCon");
        BasicBlock* extendMatchExit = b->CreateBasicBlock("extendMatchExit");

        // This literal copy will always cross 64 bits literal boundary
        this->handleLiteralCopy(b, literalStartPos, literalLength);
        BasicBlock* extendLiteralEndFinal = b->GetInsertBlock();

        b->CreateLikelyCondBr(b->CreateICmpULT(matchOffsetBeginPos, lz4BlockEnd), hasMatchPartBlock, exitBlock);

        // ---- hasMatchPartBlock
        b->SetInsertPoint(hasMatchPartBlock);
        Value* initExtendMatchPos = b->CreateAdd(matchOffsetBeginPos, b->getSize(2));
        b->CreateCondBr(shouldExtendMatch, extendMatchCon, extendMatchExit);

        // ---- extendMatchCon
        b->SetInsertPoint(extendMatchCon);
        PHINode* phiCurrentExtendMatchPos = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentExtendMatchPos->addIncoming(initExtendMatchPos, hasMatchPartBlock);
        PHINode* phiExtendMatchLength = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendMatchLength->addIncoming(SIZE_0, hasMatchPartBlock);

        Value* currentMatchLengthByte = b->CreateLoad(b->CreateGEP(bytePtrBase, phiCurrentExtendMatchPos));
        Value* newExtendMatchLength = b->CreateAdd(phiExtendMatchLength, b->CreateZExt(currentMatchLengthByte, b->getSizeTy()));

        phiCurrentExtendMatchPos->addIncoming(b->CreateAdd(phiCurrentExtendMatchPos, SIZE_1), b->GetInsertBlock());
        phiExtendMatchLength->addIncoming(newExtendMatchLength, b->GetInsertBlock());

        b->CreateCondBr(b->CreateICmpEQ(currentMatchLengthByte, BYTE_FF), extendMatchCon, extendMatchExit);

        // ---- extendMatchExit
        b->SetInsertPoint(extendMatchExit);
        PHINode* matchExtendValue = b->CreatePHI(b->getSizeTy(), 2);
        matchExtendValue->addIncoming(SIZE_0, hasMatchPartBlock);
        matchExtendValue->addIncoming(newExtendMatchLength, extendMatchCon);
        PHINode* phiExtendMatchEndPos = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendMatchEndPos->addIncoming(matchOffsetNextPos, hasMatchPartBlock);
        phiExtendMatchEndPos->addIncoming(phiCurrentExtendMatchPos, extendMatchCon);

        // matchLength = (size_t)token & 0xf + 4 + matchExtendValue
        Value* matchLength = b->CreateAdd(
                b->CreateAdd(matchExtendValue, b->getSize(4)),
                b->CreateZExt(b->CreateAnd(tokenValue, BYTE_0F), b->getSizeTy())
        );

        Value* matchOffsetPtr = b->getRawInputPointer("byteStream", matchOffsetBeginPos);
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        matchOffsetPtr = b->CreatePointerCast(matchOffsetPtr, b->getInt16Ty()->getPointerTo());
        Value* matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), b->getSizeTy());
        this->handleMatchCopy(b, matchOffset, matchLength);
        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiBeforeTokenPos = b->CreatePHI(b->getSizeTy(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPos, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(phiExtendMatchEndPos, extendMatchExitFinal);
        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, SIZE_1);



        return nextTokenPos;
    }

    std::pair<llvm::Value *, llvm::Value *>
    LZ4SwizzledAioKernel::scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *currentTokenMarker,
                                                  llvm::Value *currentExtenderValue, llvm::Value *tokenValue,
                                                  llvm::Value *blockPosBase, llvm::Value *currentTokenLocalPos) {
        Value* SIZE_1 = b->getSize(1);
        Value* BYTE_F0 = b->getInt8(0xf0);
        Value* shouldExtendLiteral = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_F0), BYTE_F0);

        // Extend Literal Length
        Value* literalScanThruResult = this->scanThru(b, b->CreateShl(currentTokenMarker, SIZE_1), currentExtenderValue);
        Value* literalScanThruLocalPos = b->CreateCountForwardZeroes(literalScanThruResult);
        Value* literalScanThruGlobalPos = b->CreateAdd(literalScanThruLocalPos, blockPosBase);
        Value* literalScanThruLastBit = b->CreateLoad(b->getRawInputPointer("byteStream", literalScanThruGlobalPos));
        // literalExtendResult = 255 * (literalScanThruLocalPos - currentTokenLocalPos - 1) + (size_t)literalScanThruLastBit
        Value* literalExtendResult = b->CreateAdd(
                b->CreateMul(
                        b->getSize(255),
                        b->CreateSub(
                                b->CreateSub(literalScanThruLocalPos, currentTokenLocalPos),
                                SIZE_1
                        )
                ),
                b->CreateZExt(literalScanThruLastBit, b->getSizeTy())
        );

        // literalLength = (size_t)token >> 4 + shouldExtendLiteral ? literalExtendResult : 0
        Value* literalLength = b->CreateAdd(
                b->CreateLShr(
                        b->CreateZExt(tokenValue, b->getSizeTy()),
                        b->getSize(4)
                ),
                b->CreateSelect(
                        shouldExtendLiteral,
                        literalExtendResult,
                        b->getSize(0)
                )
        );
        Value* retLiteralMarker = b->CreateSelect(shouldExtendLiteral, literalScanThruResult, currentTokenMarker);

        return std::make_pair(literalLength, retLiteralMarker);
    }

    std::pair<llvm::Value *, llvm::Value *>
    LZ4SwizzledAioKernel::scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetEndMarker,
                                                llvm::Value *currentExtenderValue, llvm::Value *tokenValue,
                                                llvm::Value *blockPosBase) {
        Value* SIZE_1 = b->getSize(1);
        Value* BYTE_0F = b->getInt8(0x0f);
        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);

        // Extend Match Length
        Value* matchScanThruResult = this->scanThru(b, b->CreateShl(matchOffsetEndMarker, SIZE_1), currentExtenderValue);
        Value* matchScanThruLocalPos = b->CreateCountForwardZeroes(matchScanThruResult);
        Value* matchScanThruGlobalPos = b->CreateAdd(matchScanThruLocalPos, blockPosBase);
        Value* matchScanThruLastBit = b->CreateLoad(b->getRawInputPointer("byteStream", matchScanThruGlobalPos));
        Value* matchOffsetEndLocalPos = b->CreateCountForwardZeroes(matchOffsetEndMarker);

        // matchExtendResult = 255 * (matchScanThruLocalPos - matchOffsetEndLocalPos - 1) + (size_t)matchScanThruLastBit
        Value* matchExtendResult = b->CreateAdd(
                b->CreateMul(
                        b->getSize(255),
                        b->CreateSub(
                                b->CreateSub(matchScanThruLocalPos, matchOffsetEndLocalPos),
                                SIZE_1
                        )
                ),
                b->CreateZExt(matchScanThruLastBit, b->getSizeTy())
        );

        // matchLength = (size_t)token & 0x0f + 4 + shouldExtendMatch ? matchExtendResult : 0
        Value* matchLength = b->CreateAdd(
                b->CreateAdd(
                        b->CreateZExt(b->CreateAnd(tokenValue, BYTE_0F), b->getSizeTy()),
                        b->getSize(4)
                ),
                b->CreateSelect(
                        shouldExtendMatch,
                        matchExtendResult,
                        b->getSize(0)
                )
        );

        Value* retMatchMarker = b->CreateSelect(shouldExtendMatch, matchScanThruResult, matchOffsetEndMarker);
        return std::make_pair(matchLength, retMatchMarker);
    }

    llvm::Value *
    LZ4SwizzledAioKernel::scanThru(const std::unique_ptr<KernelBuilder> &b, llvm::Value *from, llvm::Value *thru) {
        return b->CreateAnd(
                b->CreateAdd(from, thru),
                b->CreateNot(thru)
        );
    }

    void LZ4SwizzledAioKernel::handleAccelerationLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                 llvm::Value *literalLength, const std::vector<llvm::Value*>& inputLiteralValues) {
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

        vector<Value*> outputStreamBasePtrs; // <4 * i64>*
        for (int i = 0; i < mStreamSize; i++) {
            outputStreamBasePtrs.push_back(b->CreatePointerCast(b->getRawOutputPointer("outputStreamSet" + std::to_string(i), SIZE_0), BITBLOCK_PTR_TYPE));
        }

        b->CreateBr(literalCopyConBlock);

        // ---- literalCopyConBlock
        b->SetInsertPoint(literalCopyConBlock);

        b->CreateCondBr(b->CreateICmpNE(literalLength, b->getSize(0)), literalCopyBodyBlock, literalCopyExitBlock);

        // ---- literalCopyBodyBlock
        b->SetInsertPoint(literalCopyBodyBlock);

        Value* literalRem = b->CreateURem(literalStart, SIZE_64);
        Value* literalBlockIndex = b->CreateUDiv(b->CreateURem(literalStart, inputStreamSize), SIZE_64);

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


        for (int i = 0; i < mStreamSize; i++) {
            Value* inputValue = inputLiteralValues[i];

            Value* outputPtr1 = b->CreateGEP(outputStreamBasePtrs[i], outputBlockIndex);
            Value* outputValue1 = b->CreateLoad(outputPtr1);
            Value* inputValue1 = b->simd_sllv(64, inputValue, l1_1);
            inputValue1 = b->simd_srlv(64, inputValue1, r1_1);
            inputValue1 = b->simd_sllv(64, inputValue1, l1_2);
            outputValue1 = b->CreateOr(b->CreateAnd(outputValue1, clearMask1), inputValue1);
            b->CreateStore(outputValue1, outputPtr1);


            // TODO test whether using branch for output2 will be better
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

    void LZ4SwizzledAioKernel::handleLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                   llvm::Value *literalLength) {
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
        for (int i = 0; i < mStreamSize; i++) {
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

        for (int i = 0; i < mStreamSize; i++) {
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

    void LZ4SwizzledAioKernel::handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* outputPos, llvm::Value* matchOffset, llvm::Value* matchLength, bool clearBuffer) {
        Value* SIZE_64  = b->getSize(64);
        Value* SIZE_0 = b->getSize(0);
        Value* SIZE_1 = b->getSize(1);

        Value* outputStreamSize = b->getCapacity("outputStreamSet0");
        Type* BITBLOCK_PTR_TYPE = b->getBitBlockType()->getPointerTo();

        vector<Value*> outputStreamBasePtrs; // <4 * i64>*
        for (int i = 0; i < mStreamSize; i++) {
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

        for (int i = 0; i < mStreamSize; i++) {
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

    void LZ4SwizzledAioKernel::handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                                 llvm::Value *matchLength) {
        Value* outputPos = b->getScalarField("outputPos");
        this->handleMatchCopy(b, outputPos, matchOffset, matchLength);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }


}