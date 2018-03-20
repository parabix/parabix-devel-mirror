//
// Created by wxy325 on 2018/3/16.
//

#include "lz4_index_builder.h"


#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;
namespace kernel{
    LZ4IndexBuilderKernel::LZ4IndexBuilderKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder)
            : MultiBlockKernel("LZ4IndexBuilderKernel",
            // Inputs
                               {
                                       Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 1), "extender", RateEqualTo("byteStream")},
                                       Binding{iBuilder->getStreamSetTy(1, 1), "CC_0xFX", RateEqualTo("byteStream")},
                                       Binding{iBuilder->getStreamSetTy(1, 1), "CC_0xXF", RateEqualTo("byteStream")},

                                       // block data
                                       Binding{iBuilder->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1),
                                               ConstantStrideLengthOne()},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1),
                                               ConstantStrideLengthOne()},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", BoundedRate(0, 1),
                                               ConstantStrideLengthOne()}

                               },
            //Outputs
                               {
                                       // Uncompressed_data
                                       Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedStartPos",
                                               BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedLength",
                                               BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedOutputPos",
                                               BoundedRate(0, 1)},

                                       Binding{iBuilder->getStreamSetTy(1, 1), "e1Marker", BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1)},
                                       Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1)}
                               },
            //Arguments
                               {},
                               {},
            //Internal states:
                               {
                                       Binding{iBuilder->getSizeTy(), "blockDataIndex"},
                                       Binding{iBuilder->getInt64Ty(), "m0OutputPos"}
                               }) {
//        addAttribute(MustExplicitlyTerminate());
    }

    void LZ4IndexBuilderKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> &iBuilder, llvm::Value *const numOfStrides) {
//        iBuilder->CallPrintInt("entry", iBuilder->getSize(0));
//        iBuilder->CallPrintInt("aaa", iBuilder->getProducedItemCount("e1Marker"));

        // Clear Output Buffer
        previousE1Produced = iBuilder->getProducedItemCount("e1Marker");

        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = iBuilder->CreateBasicBlock("blockEndConBlock");
        Value* blockDataIndex = iBuilder->getScalarField("blockDataIndex");

        Value* totalNumber = iBuilder->CreateAdd(iBuilder->getAvailableItemCount("blockEnd"), iBuilder->getProcessedItemCount("blockEnd"));

//        iBuilder->CallPrintInt("blockDataIndex", blockDataIndex);
//        iBuilder->CallPrintInt("totalNumber", totalNumber);
//        iBuilder->setTerminationSignal(iBuilder->CreateICmpEQ(availableBlockEnd, iBuilder->getSize(0)));

        iBuilder->CreateCondBr(iBuilder->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        iBuilder->SetInsertPoint(blockEndConBlock);
        Value* blockEnd = this->generateLoadCircularInput(iBuilder, "blockEnd", blockDataIndex, iBuilder->getInt64Ty()->getPointerTo());


        Value* totalExtender = iBuilder->CreateAdd(iBuilder->getAvailableItemCount("extender"), iBuilder->getProcessedItemCount("extender"));
//        iBuilder->CallPrintInt("totalExtender", totalExtender);

//        iBuilder->CallPrintInt("processByteStream", iBuilder->getProcessedItemCount("byteStream"));
//        iBuilder->CallPrintInt("availableByteStream", iBuilder->getAvailableItemCount("byteStream"));


//        iBuilder->CallPrintInt("consumedExtender", iBuilder->getConsumedItemCount("extender"));
//        iBuilder->CallPrintInt("processExtender", iBuilder->getProcessedItemCount("extender"));
//        iBuilder->CallPrintInt("availableExtender", iBuilder->getAvailableItemCount("extender"));
//        iBuilder->CallPrintInt("blockDataIndex", blockDataIndex);
//        iBuilder->CallPrintInt("blockEnd", blockEnd);

        Value* blockStart = this->generateLoadCircularInput(iBuilder, "blockStart", blockDataIndex, iBuilder->getInt64Ty()->getPointerTo());

        BasicBlock* processBlock = iBuilder->CreateBasicBlock("processBlock");
//        iBuilder->CallPrintInt("----totalExtender", totalExtender);
//        iBuilder->CallPrintInt("----blockStart", blockStart);
//        iBuilder->CallPrintInt("----blockEnd", blockEnd);

        iBuilder->CreateCondBr(iBuilder->CreateICmpULE(blockEnd, totalExtender), processBlock, exitBlock);

        iBuilder->SetInsertPoint(processBlock);


        //TODO handle uncompressed block
        this->generateProcessCompressedBlock(iBuilder, blockStart, blockEnd);



        Value* newBlockDataIndex = iBuilder->CreateAdd(blockDataIndex, iBuilder->getInt64(1));
        iBuilder->setScalarField("blockDataIndex", newBlockDataIndex);
        iBuilder->setProcessedItemCount("blockEnd", newBlockDataIndex);
        iBuilder->setProcessedItemCount("blockStart", newBlockDataIndex);
        iBuilder->setProcessedItemCount("isCompressed", newBlockDataIndex);


        iBuilder->setProcessedItemCount("byteStream", blockEnd);
//        iBuilder->setProcessedItemCount("extender", blockEnd);
//        iBuilder->setProcessedItemCount("CC_0xFX", blockEnd);
//        iBuilder->setProcessedItemCount("CC_0xXF", blockEnd);

        iBuilder->CreateBr(exitBlock);

        iBuilder->SetInsertPoint(exitBlock);
    }

    Value* LZ4IndexBuilderKernel::processLiteral(const std::unique_ptr<KernelBuilder> &iBuilder, Value* token, Value* tokenPos, Value* blockEnd) {
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value* extendedLiteralValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf0)), iBuilder->getInt8(0xf0));
//        iBuilder->CallPrintInt("token", token);

        BasicBlock* extendLiteralLengthBody = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_body");
        BasicBlock* extendLiteralLengthExit = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_exit");

        iBuilder->CreateCondBr(extendedLiteralValue, extendLiteralLengthBody, extendLiteralLengthExit);

        iBuilder->SetInsertPoint(extendLiteralLengthBody);
        Value* newCursorPos = this->advanceUntilNextZero(iBuilder, "extender", iBuilder->CreateAdd(tokenPos, iBuilder->getInt64(1)), blockEnd);
        BasicBlock* advanceFinishBlock = iBuilder->GetInsertBlock();

        iBuilder->CreateBr(extendLiteralLengthExit);

        iBuilder->SetInsertPoint(extendLiteralLengthExit);

        PHINode* phiCursorPosAfterLiteral = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
        phiCursorPosAfterLiteral->addIncoming(newCursorPos, advanceFinishBlock);
        phiCursorPosAfterLiteral->addIncoming(tokenPos, entryBlock);

        Value* literalExtensionSize = iBuilder->CreateSub(phiCursorPosAfterLiteral, tokenPos);
//        iBuilder->CallPrintInt("literalExtensionSize", literalExtensionSize);
        Value* finalLengthByte = this->generateLoadSourceInputByte(iBuilder, phiCursorPosAfterLiteral);
        finalLengthByte = iBuilder->CreateZExt(finalLengthByte, iBuilder->getInt64Ty());
        Value* literalLengthExtendValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpUGT(literalExtensionSize, iBuilder->getSize(0)),
                iBuilder->CreateAdd(
                        iBuilder->CreateMul(
                                iBuilder->CreateSub(literalExtensionSize, iBuilder->getSize(1)),
                                iBuilder->getSize(255)
                        ),
                        finalLengthByte
                ),
                iBuilder->getSize(0)
        );
        literalLengthExtendValue = iBuilder->CreateZExt(literalLengthExtendValue, iBuilder->getInt64Ty());
        Value* literalLengthBase = iBuilder->CreateLShr(iBuilder->CreateZExt(token, iBuilder->getInt64Ty()), iBuilder->getInt64(4));
        Value* literalLength = iBuilder->CreateAdd(literalLengthBase, literalLengthExtendValue);

        Value* offsetPos = iBuilder->CreateAdd(
                iBuilder->CreateAdd(
                        phiCursorPosAfterLiteral,
                        literalLength),
                iBuilder->getSize(1));
//        iBuilder->CallPrintInt("offsetPos", offsetPos);
        this->markCircularOutputBitstream(iBuilder, "e1Marker", iBuilder->getProducedItemCount("e1Marker"), iBuilder->CreateAdd(phiCursorPosAfterLiteral, iBuilder->getSize(1)), false);
        this->markCircularOutputBitstream(iBuilder, "e1Marker", iBuilder->CreateAdd(phiCursorPosAfterLiteral, iBuilder->getSize(1)), offsetPos, true);
        this->increaseScalarField(iBuilder, "m0OutputPos", literalLength); //TODO m0OutputPos may be removed from scalar fields
        return offsetPos;
    }

    Value* LZ4IndexBuilderKernel::processMatch(const std::unique_ptr<KernelBuilder> &iBuilder, Value* offsetPos, Value* token, Value* blockEnd) {
        Constant* INT64_ONE = iBuilder->getInt64(1);

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value* matchLengthStartPos = iBuilder->CreateAdd(offsetPos, INT64_ONE);
        Value* extendedMatchValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf)), iBuilder->getInt8(0xf));

        BasicBlock* extendMatchBodyBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_body");
        BasicBlock* extendMatchExitBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_exit");

        iBuilder->CreateCondBr(extendedMatchValue, extendMatchBodyBlock, extendMatchExitBlock);

        iBuilder->SetInsertPoint(extendMatchBodyBlock);

        //ExtendMatchBodyBlock
        Value* newCursorPos = this->advanceUntilNextZero(iBuilder, "extender", iBuilder->CreateAdd(matchLengthStartPos, INT64_ONE), blockEnd);
        BasicBlock* advanceFinishBlock = iBuilder->GetInsertBlock();

        // ----May be in a different segment now
        iBuilder->CreateBr(extendMatchExitBlock);

        //ExtendMatchExitBlock
        iBuilder->SetInsertPoint(extendMatchExitBlock);
        PHINode* phiCursorPosAfterMatch = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
        phiCursorPosAfterMatch->addIncoming(newCursorPos, advanceFinishBlock);
        phiCursorPosAfterMatch->addIncoming(matchLengthStartPos, entryBlock);

        Value* oldMatchExtensionSize = iBuilder->CreateSub(phiCursorPosAfterMatch, matchLengthStartPos);
//        iBuilder->CallPrintInt("totalExtender", iBuilder->CreateAdd(iBuilder->getAvailableItemCount("extender"), iBuilder->getProcessedItemCount("extender")));
//        iBuilder->CallPrintInt("aaa", oldMatchExtensionSize);

        extendedMatchValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf)), iBuilder->getInt8(0xf));
        Value* matchExtensionSize = iBuilder->CreateSelect(
                iBuilder->CreateICmpEQ(extendedMatchValue, iBuilder->getInt1(true)),
                oldMatchExtensionSize,
                iBuilder->getSize(0)
        );
        Value* matchLengthBase = iBuilder->CreateZExt(iBuilder->CreateAnd(token, iBuilder->getInt8(0x0f)), iBuilder->getInt64Ty());
        Value* matchLength = iBuilder->CreateAdd(matchLengthBase, iBuilder->getInt64(4));


        Value* extensionLastBitPos = iBuilder->CreateAdd(offsetPos, iBuilder->getSize(1));
        extensionLastBitPos = iBuilder->CreateAdd(extensionLastBitPos, matchExtensionSize);

        Value* extensionLastBitValue = this->generateLoadSourceInputByte(iBuilder, extensionLastBitPos);
        extensionLastBitValue = iBuilder->CreateZExt(extensionLastBitValue, iBuilder->getSizeTy());


        Value* matchLengthAddValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpUGT(matchExtensionSize, iBuilder->getSize(0)),
                iBuilder->CreateAdd(
                        iBuilder->CreateMul(
                                iBuilder->CreateSub(matchExtensionSize, iBuilder->getSize(1)),
                                iBuilder->getSize(255)
                        ),
                        extensionLastBitValue
                )
                ,
                iBuilder->getSize(0)
        );
        matchLengthAddValue = iBuilder->CreateZExt(matchLengthAddValue, iBuilder->getInt64Ty());

        matchLength = iBuilder->CreateAdd(matchLength, matchLengthAddValue);

        Value* outputPos = iBuilder->getScalarField("m0OutputPos");

        Value* outputEndPos = iBuilder->CreateSub(
                iBuilder->CreateAdd(outputPos, matchLength),
                iBuilder->getInt64(1)
        );

        Value* matchOffset = iBuilder->CreateAdd(
                iBuilder->CreateZExt(this->generateLoadSourceInputByte(iBuilder, offsetPos), iBuilder->getSizeTy()),
                iBuilder->CreateShl(iBuilder->CreateZExt(this->generateLoadSourceInputByte(iBuilder, iBuilder->CreateAdd(offsetPos, iBuilder->getSize(1))), iBuilder->getSizeTy()), iBuilder->getSize(8))
        );
//        iBuilder->CallPrintInt("matchOffset", matchOffset);
        this->generateStoreCircularOutput(iBuilder, "m0Start", iBuilder->getInt64Ty()->getPointerTo(), outputPos);
//    iBuilder->CallPrintInt("m0Start", outputPos);
        this->generateStoreCircularOutput(iBuilder, "m0End", iBuilder->getInt64Ty()->getPointerTo(), outputEndPos);
//    iBuilder->CallPrintInt("m0End", outputEndPos);
        this->generateStoreCircularOutput(iBuilder, "matchOffset", iBuilder->getInt64Ty()->getPointerTo(), matchOffset);
//    iBuilder->CallPrintInt("matchOffset", matchOffset);
        this->increaseScalarField(iBuilder, "m0OutputPos", matchLength);
        return iBuilder->CreateAdd(phiCursorPosAfterMatch, INT64_ONE);
    }


    void LZ4IndexBuilderKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &iBuilder, Value* blockStart, Value* blockEnd) {
        // Constant
        Constant* INT64_ONE = iBuilder->getInt64(1);

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();
        //TODO use memset to clear output buffer
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = iBuilder->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = iBuilder->CreateBasicBlock("processCompressedBodyBlock");

        iBuilder->CreateBr(processCon);
        iBuilder->SetInsertPoint(processCon);

        PHINode* phiCursorValue = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3); // phiCursorValue should always be the position of next token except for the final sequence
        phiCursorValue->addIncoming(blockStart, entryBlock);

        iBuilder->CreateCondBr(iBuilder->CreateICmpULT(phiCursorValue, blockEnd), processBody, exitBlock);

        // Process Body
        iBuilder->SetInsertPoint(processBody);

        //TODO add acceleration here
        Value* token = this->generateLoadSourceInputByte(iBuilder, phiCursorValue);
//        iBuilder->CallPrintInt("tokenPos", phiCursorValue);
//        iBuilder->CallPrintInt("token", token);

        // Process Literal
        BasicBlock* processLiteralBlock = iBuilder->CreateBasicBlock("processLiteralBlock");
        iBuilder->CreateBr(processLiteralBlock);
        iBuilder->SetInsertPoint(processLiteralBlock);

        Value* offsetPos = this->processLiteral(iBuilder, token, phiCursorValue, blockEnd);
//        iBuilder->CallPrintInt("offsetPos", offsetPos);
        // Process Match
        BasicBlock* handleM0BodyBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_handle_m0_body");
        BasicBlock* handleM0ElseBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_handle_m0_else");

        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(offsetPos, blockEnd),
                handleM0BodyBlock,
                handleM0ElseBlock
        );

        // HandleM0Body
        iBuilder->SetInsertPoint(handleM0BodyBlock);
        Value* nextTokenPos = this->processMatch(iBuilder, offsetPos, token, blockEnd);
//        iBuilder->CallPrintInt("nextTokenPos", nextTokenPos);
        phiCursorValue->addIncoming(nextTokenPos, iBuilder->GetInsertBlock());

        iBuilder->CreateBr(processCon);


        // HandleM0Else
        iBuilder->SetInsertPoint(handleM0ElseBlock);

        phiCursorValue->addIncoming(offsetPos, handleM0ElseBlock);
        // Store final M0 pos to make sure the bit stream will be long enough
        Value* finalM0OutputPos = iBuilder->getScalarField("m0OutputPos");
//        iBuilder->CallPrintInt("finalM0OutputPos", finalM0OutputPos);
        this->generateStoreCircularOutput(iBuilder, "m0Start", iBuilder->getInt64Ty()->getPointerTo(), finalM0OutputPos);
        this->generateStoreCircularOutput(iBuilder, "m0End", iBuilder->getInt64Ty()->getPointerTo(), finalM0OutputPos);
        this->generateStoreCircularOutput(iBuilder, "matchOffset", iBuilder->getInt64Ty()->getPointerTo(), iBuilder->getInt64(0));

        iBuilder->CreateBr(processCon);


        iBuilder->SetInsertPoint(exitBlock);
    }

    Value *LZ4IndexBuilderKernel::advanceUntilNextZero(const unique_ptr<KernelBuilder> &iBuilder, string inputName, Value* startPos, Value* maxPos) {
        return advanceUntilNextValue(iBuilder, inputName, startPos, true, maxPos);
    }

    Value *LZ4IndexBuilderKernel::advanceUntilNextOne(const unique_ptr<KernelBuilder> &iBuilder, string inputName, Value* startPos, Value* maxPos) {
        return advanceUntilNextValue(iBuilder, inputName, startPos, false, maxPos);
    }

    Value *LZ4IndexBuilderKernel::advanceUntilNextValue(const unique_ptr<KernelBuilder> &iBuilder, string inputName, Value* startPos, bool isNextZero, Value* maxPos) {
        unsigned int bitBlockWidth = iBuilder->getBitBlockWidth();
        Constant* INT64_BIT_BLOCK_WIDTH = iBuilder->getInt64(bitBlockWidth);
        Constant* SIZE_ZERO = iBuilder->getSize(0);
        Type* bitBlockType = iBuilder->getBitBlockType();
        Type* bitBlockWidthIntTy = iBuilder->getIntNTy(bitBlockWidth);


        Value* baseOffset = iBuilder->getProcessedItemCount(inputName);
        baseOffset = iBuilder->CreateSub(baseOffset, iBuilder->CreateURem(baseOffset, INT64_BIT_BLOCK_WIDTH));

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();


        Value* inputBasePtr = iBuilder->CreatePointerCast(iBuilder->getInputStreamBlockPtr(inputName, SIZE_ZERO), bitBlockType->getPointerTo());

        BasicBlock* advanceConBlock = iBuilder->CreateBasicBlock("advanceConBlock");
        BasicBlock* advanceBodyBlock = iBuilder->CreateBasicBlock("advanceBodyBlock");
        BasicBlock* advanceExitBlock = iBuilder->CreateBasicBlock("advanceExitBlock");

        iBuilder->CreateBr(advanceConBlock);
        // TODO special handling for the first advance may have better performance
        iBuilder->SetInsertPoint(advanceConBlock);

        PHINode* phiCurrentPos = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
        phiCurrentPos->addIncoming(startPos, entryBlock);
        PHINode* phiIsFinish = iBuilder->CreatePHI(iBuilder->getInt1Ty(), 2);
        phiIsFinish->addIncoming(iBuilder->getInt1(false), entryBlock);
        iBuilder->CreateCondBr(iBuilder->CreateNot(phiIsFinish), advanceBodyBlock, advanceExitBlock);

        iBuilder->SetInsertPoint(advanceBodyBlock);


//        iBuilder->CallPrintInt("phiCurrentPos", phiCurrentPos);
//        iBuilder->CallPrintInt("baseOffset", baseOffset);
        Value* currentPosBitBlockIndex = iBuilder->CreateUDiv(iBuilder->CreateSub(phiCurrentPos, baseOffset), INT64_BIT_BLOCK_WIDTH);
//        iBuilder->CallPrintInt("currentPosBitBlockIndex", currentPosBitBlockIndex);
        Value* currentPosBitBlockOffset = iBuilder->CreateURem(phiCurrentPos, INT64_BIT_BLOCK_WIDTH);

        Value* ptr = iBuilder->CreateGEP(inputBasePtr, iBuilder->CreateTruncOrBitCast(currentPosBitBlockIndex, iBuilder->getSizeTy()));
//        iBuilder->CallPrintInt("ptr", ptr);
//        iBuilder->CallPrintInt("blockBasePtr", iBuilder->getInputStreamBlockPtr(inputName, SIZE_ZERO));
        Value* currentBitValue = iBuilder->CreateBitCast(iBuilder->CreateLoad(ptr), bitBlockWidthIntTy);
//        iBuilder->CallPrintRegister("ptrValue", currentBitValue);

        currentBitValue = iBuilder->CreateLShr(currentBitValue, iBuilder->CreateZExt(currentPosBitBlockOffset, bitBlockWidthIntTy));
        if (isNextZero) {
            currentBitValue = iBuilder->CreateNot(currentBitValue);
        }
        Value* forwardZeroCount = iBuilder->CreateTrunc(iBuilder->CreateCountForwardZeroes(currentBitValue), iBuilder->getInt64Ty());
        Value* newOffset = iBuilder->CreateAdd(currentPosBitBlockOffset, forwardZeroCount);
        newOffset = iBuilder->CreateUMin(newOffset, INT64_BIT_BLOCK_WIDTH);

        Value* actualAdvanceValue = iBuilder->CreateSub(newOffset, currentPosBitBlockOffset);
        Value* newPos = iBuilder->CreateAdd(phiCurrentPos, actualAdvanceValue);
        if (maxPos) {
            newPos = iBuilder->CreateUMin(maxPos, newPos);
            actualAdvanceValue = iBuilder->CreateSub(newPos, phiCurrentPos);
            newOffset = iBuilder->CreateAdd(currentPosBitBlockOffset, actualAdvanceValue);
        }

        phiIsFinish->addIncoming(iBuilder->CreateNot(iBuilder->CreateICmpEQ(newOffset, INT64_BIT_BLOCK_WIDTH)), iBuilder->GetInsertBlock());
        phiCurrentPos->addIncoming(newPos, iBuilder->GetInsertBlock());
        iBuilder->CreateBr(advanceConBlock);

        iBuilder->SetInsertPoint(advanceExitBlock);
        return phiCurrentPos;
    }

    Value *
    LZ4IndexBuilderKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName,
                                                Value *offset, Type *pointerType) {
        size_t inputSize = this->getInputStreamSetBuffer(inputBufferName)->getBufferBlocks() * iBuilder->getStride();
        Value *offsetMask = iBuilder->getSize(inputSize - 1);
        Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value *inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }

    Value *LZ4IndexBuilderKernel::generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, Value *offset) {
        Value *blockStartPtr = iBuilder->CreatePointerCast(
                iBuilder->getRawInputPointer("byteStream", iBuilder->getInt32(0)),
                iBuilder->getInt8PtrTy()
        );
        Value *ptr = iBuilder->CreateGEP(blockStartPtr, offset);

        return iBuilder->CreateLoad(ptr);
    }

    void LZ4IndexBuilderKernel::increaseScalarField(const unique_ptr<KernelBuilder> &iBuilder, const string &fieldName, Value *value) {
        Value *fieldValue = iBuilder->getScalarField(fieldName);
        fieldValue = iBuilder->CreateAdd(fieldValue, value);
        iBuilder->setScalarField(fieldName, fieldValue);
    }

    size_t LZ4IndexBuilderKernel::getOutputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        return this->getOutputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    // Assume we have enough output buffer
    llvm::BasicBlock *LZ4IndexBuilderKernel::markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                    const std::string &bitstreamName,
                                                                    llvm::Value *start, llvm::Value *end, bool isOne,
                                                                    bool setProduced) {
        Value* originalEnd = end;
        Value* baseOffset = iBuilder->CreateSub(previousE1Produced, iBuilder->CreateURem(previousE1Produced, iBuilder->getInt64(iBuilder->getBitBlockWidth())));;
//        iBuilder->CallPrintInt("baseOffset", baseOffset);
//        iBuilder->CallPrintInt("start", start);
//        iBuilder->CallPrintInt("end", end);
        start = iBuilder->CreateSub(start, baseOffset);
        end = iBuilder->CreateSub(end, baseOffset);
        //TODO possible bug here
        BasicBlock *entryBlock = iBuilder->GetInsertBlock();



        Value *outputBasePtr = iBuilder->getOutputStreamBlockPtr(bitstreamName, iBuilder->getSize(0));
//        iBuilder->CallPrintInt("outputBasePtr", outputBasePtr);
//        iBuilder->CallPrintInt("a", iBuilder->getRawOutputPointer(bitstreamName, iBuilder->getSize(0)));

        outputBasePtr = iBuilder->CreatePointerCast(outputBasePtr, iBuilder->getInt64Ty()->getPointerTo());

//        size_t outputBufferSize = this->getOutputBufferSize(iBuilder, bitstreamName);
//        Value *outputMask = iBuilder->getSize(outputBufferSize / 64 - 1);

        BasicBlock *conBlock = iBuilder->CreateBasicBlock("mark_bit_one_con");
        BasicBlock *bodyBlock = iBuilder->CreateBasicBlock("mark_bit_one_body");
        BasicBlock *exitBlock = iBuilder->CreateBasicBlock("mark_bit_one_exit");

        Value *startOffset = iBuilder->CreateLShr(start, iBuilder->getSize(std::log2(64)), "startOffset");

        iBuilder->CreateBr(conBlock);

        // Con
        iBuilder->SetInsertPoint(conBlock);


        PHINode *curOffset = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2);
        curOffset->addIncoming(startOffset, entryBlock);
//        iBuilder->CallPrintInt("curOffset", curOffset);
//        iBuilder->CallPrintInt("end", end);

        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(iBuilder->CreateShl(curOffset, std::log2(64)), end),
                bodyBlock,
                exitBlock
        );

        // Body
        iBuilder->SetInsertPoint(bodyBlock);
        Value *maskedOffset = curOffset;

        Value *outputLowestBitValue = iBuilder->CreateSelect(
                iBuilder->CreateICmpULE(
                        iBuilder->CreateShl(curOffset, std::log2(64)),
                        start
                ),
                iBuilder->CreateShl(iBuilder->getSize(1), iBuilder->CreateAnd(start, iBuilder->getSize(64 - 1))),
                iBuilder->getSize(1)
        );

        Value *hasNotReachEnd = iBuilder->CreateICmpULE(
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );
        Value *producedItemsCount = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->CreateShl(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), std::log2(64)),
                end
        );
        producedItemsCount = iBuilder->CreateAdd(producedItemsCount, baseOffset);

        Value *outputHighestBitValue = iBuilder->CreateSelect(
                hasNotReachEnd,
                iBuilder->getSize(0),
                iBuilder->CreateShl(
                        iBuilder->getSize(1),
                        iBuilder->CreateAnd(end, iBuilder->getSize(64 - 1))
                )
        );


        Value *bitMask = iBuilder->CreateSub(
                outputHighestBitValue,
                outputLowestBitValue
        );

        if (!isOne) {
            bitMask = iBuilder->CreateNot(bitMask);
        }

        Value *targetPtr = iBuilder->CreateGEP(outputBasePtr, maskedOffset);
//        iBuilder->CallPrintInt("maskedOffset", maskedOffset);
        Value *oldValue = iBuilder->CreateLoad(targetPtr);
        Value *newValue = NULL;
        if (isOne) {
            newValue = iBuilder->CreateOr(oldValue, bitMask);
        } else {
            newValue = iBuilder->CreateAnd(oldValue, bitMask);
        }
        iBuilder->CreateStore(
                newValue,
                targetPtr
        );
//        iBuilder->CallPrintInt("targetPtr", targetPtr);
        if (setProduced) {
            iBuilder->setProducedItemCount(bitstreamName, producedItemsCount);
        }

        curOffset->addIncoming(iBuilder->CreateAdd(curOffset, iBuilder->getSize(1)), bodyBlock);
        iBuilder->CreateBr(conBlock);

        // Exit
        iBuilder->SetInsertPoint(exitBlock);
        return exitBlock;
    }


    void
    LZ4IndexBuilderKernel::generateStoreCircularOutput(const unique_ptr<KernelBuilder> &iBuilder, string outputBufferName,
                                                  Type *pointerType, Value *value) {
        //TODO possible bug here
        Value *offset = iBuilder->getProducedItemCount(outputBufferName);

        size_t inputSize = this->getOutputBufferSize(iBuilder, outputBufferName);
        Value *offsetMask = iBuilder->getSize(inputSize - 1);
        Value *maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value *outputBufferPtr = iBuilder->getRawOutputPointer(outputBufferName, iBuilder->getSize(0));

        outputBufferPtr = iBuilder->CreatePointerCast(outputBufferPtr, pointerType);
        iBuilder->CreateStore(value, iBuilder->CreateGEP(outputBufferPtr, maskedOffset));

        offset = iBuilder->CreateAdd(offset, iBuilder->getSize(1));
        iBuilder->setProducedItemCount(outputBufferName, offset);
    }
}