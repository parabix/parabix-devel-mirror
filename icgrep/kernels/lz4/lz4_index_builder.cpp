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
    : SegmentOrientedKernel("LZ4IndexBuilderKernel",
    // Inputs
    {
           Binding{iBuilder->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},
           Binding{iBuilder->getStreamSetTy(1, 1), "extender", RateEqualTo("byteStream")},

           // block data
           Binding{iBuilder->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1), AlwaysConsume()},
           Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", RateEqualTo("isCompressed"), AlwaysConsume()},
           Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("isCompressed"), AlwaysConsume()}

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

           Binding{iBuilder->getStreamSetTy(1, 1), "deletionMarker", BoundedRate(0, 1)},
           Binding{iBuilder->getStreamSetTy(1, 1), "M0Marker", BoundedRate(0, 1)},
           Binding{iBuilder->getStreamSetTy(1, 1), "M0CountMarker", BoundedRate(0, 1)},
           Binding{iBuilder->getStreamSetTy(1, 1), "MatchOffsetMarker", RateEqualTo("byteStream")}
    },
    //Arguments
    {
           Binding{iBuilder->getSizeTy(), "fileSize"}
    },
    {},
    //Internal states:
    {
           Binding{iBuilder->getSizeTy(), "blockDataIndex"},
           Binding{iBuilder->getInt64Ty(), "m0OutputPos"},
           Binding{iBuilder->getInt64Ty(), "compressedSpaceClearPos"}
    }) {
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZ4IndexBuilderKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
//        iBuilder->CallPrintInt("IndexBuilder:entry", iBuilder->getSize(0));

        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = iBuilder->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = iBuilder->getScalarField("blockDataIndex");

        // In MultiblockKernel, availableItemCount + processedItemCount == producedItemCount from previous kernel
        // While in SegmentOrigentedKernel, availableItemCount == producedItemCount from previous kernel
        Value * totalNumber = iBuilder->getAvailableItemCount("blockEnd");
        Value * totalExtender = iBuilder->getAvailableItemCount("extender");

        Value * blockEnd = this->generateLoadInt64NumberInput(iBuilder, "blockEnd", blockDataIndex);

        iBuilder->CreateCondBr(iBuilder->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        iBuilder->SetInsertPoint(blockEndConBlock);
        Value * blockStart = this->generateLoadInt64NumberInput(iBuilder, "blockStart", blockDataIndex);
        BasicBlock * processBlock = iBuilder->CreateBasicBlock("processBlock");
        iBuilder->CreateCondBr(iBuilder->CreateICmpULE(blockEnd, totalExtender), processBlock, exitBlock);

        iBuilder->SetInsertPoint(processBlock);

        //TODO handle uncompressed block

        this->generateProcessCompressedBlock(iBuilder, blockStart, blockEnd);

        Value * newBlockDataIndex = iBuilder->CreateAdd(blockDataIndex, iBuilder->getInt64(1));
        iBuilder->setScalarField("blockDataIndex", newBlockDataIndex);
        iBuilder->setProcessedItemCount("isCompressed", newBlockDataIndex);
//        iBuilder->setProcessedItemCount("blockEnd", newBlockDataIndex);
//        iBuilder->setProcessedItemCount("blockStart", newBlockDataIndex);

        iBuilder->setProcessedItemCount("byteStream", blockEnd);
        iBuilder->CreateBr(exitBlock);

        iBuilder->SetInsertPoint(exitBlock);
    }

    Value* LZ4IndexBuilderKernel::processLiteral(const std::unique_ptr<KernelBuilder> &iBuilder, Value* token, Value* tokenPos, Value* blockEnd) {
//        iBuilder->CallPrintInt("blockEnd", blockEnd);
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value * extendedLiteralValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf0)), iBuilder->getInt8(0xf0));

        BasicBlock* extendLiteralLengthCon = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_con");
        BasicBlock* extendLiteralLengthBody = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_body");
        BasicBlock* extendLiteralLengthExit = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_exit");

        iBuilder->CreateCondBr(extendedLiteralValue, extendLiteralLengthCon, extendLiteralLengthExit);

        iBuilder->SetInsertPoint(extendLiteralLengthCon);

        Value * const nextTokenPos = iBuilder->CreateAdd(tokenPos, iBuilder->getInt64(1));
        Value * const nextToken = iBuilder->CreateLoad(iBuilder->getRawInputPointer("byteStream", nextTokenPos));
        Value * const isExitToken = iBuilder->CreateICmpNE(nextToken, iBuilder->getInt8(0xff));
        iBuilder->CreateLikelyCondBr(isExitToken, extendLiteralLengthExit, extendLiteralLengthBody);


        iBuilder->SetInsertPoint(extendLiteralLengthBody);
        Value* newCursorPos2 = this->advanceUntilNextZero(iBuilder, "extender", iBuilder->CreateAdd(tokenPos, iBuilder->getInt64(1)), blockEnd);
        BasicBlock* advanceFinishBlock = iBuilder->GetInsertBlock();


        iBuilder->CreateBr(extendLiteralLengthExit);

        iBuilder->SetInsertPoint(extendLiteralLengthExit);
        PHINode* phiCursorPosAfterLiteral = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 3);
        phiCursorPosAfterLiteral->addIncoming(nextTokenPos, extendLiteralLengthCon);
        phiCursorPosAfterLiteral->addIncoming(newCursorPos2, advanceFinishBlock);
        phiCursorPosAfterLiteral->addIncoming(tokenPos, entryBlock);

        Value * literalExtensionSize = iBuilder->CreateSub(phiCursorPosAfterLiteral, tokenPos);
        Value * finalLengthByte = this->generateLoadSourceInputByte(iBuilder, phiCursorPosAfterLiteral);
        finalLengthByte = iBuilder->CreateZExt(finalLengthByte, iBuilder->getInt64Ty());
        Value * literalLengthExtendValue = iBuilder->CreateSelect(
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

        this->setCircularOutputBitstream(iBuilder, "deletionMarker", iBuilder->getProducedItemCount("deletionMarker"), iBuilder->CreateAdd(phiCursorPosAfterLiteral, iBuilder->getSize(1)));

        iBuilder->setProducedItemCount("deletionMarker", offsetPos);
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


        iBuilder->setProducedItemCount("M0CountMarker", iBuilder->CreateAdd(iBuilder->getProducedItemCount("M0CountMarker"), iBuilder->getSize(1)));
        this->markCircularOutputBitstream(iBuilder, "MatchOffsetMarker", offsetPos);
        this->increaseScalarField(iBuilder, "m0OutputPos", matchLength);
        this->setCircularOutputBitstream(iBuilder, "M0Marker", outputPos, outputEndPos);

        return iBuilder->CreateAdd(phiCursorPosAfterMatch, INT64_ONE);
    }

    void LZ4IndexBuilderKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &iBuilder, Value* blockStart, Value* blockEnd) {
        // Constant
        Value* clearPos = iBuilder->getScalarField("compressedSpaceClearPos");
        // We can not only clear [blockStart, blockEnd), since there are 4 bytes between blockEnd and nextBlockStart
        this->clearCircularOutputBitstream(iBuilder, "deletionMarker", clearPos, blockEnd);
        this->clearCircularOutputBitstream(iBuilder, "MatchOffsetMarker", clearPos, blockEnd);
        iBuilder->setScalarField("compressedSpaceClearPos", blockEnd);

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value* m0OutputBlockPtr = iBuilder->getOutputStreamBlockPtr("M0Marker", iBuilder->getSize(0));
        iBuilder->CreateMemSet(m0OutputBlockPtr, iBuilder->getInt8(0), 4 * 1024 * 1024 / 8, true);


        Value* isTerminal = iBuilder->CreateICmpEQ(blockEnd, iBuilder->getScalarField("fileSize"));
        iBuilder->setTerminationSignal(isTerminal);

        //TODO use memset to clear output buffer for extract marker

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
        // Process Literal
        BasicBlock* processLiteralBlock = iBuilder->CreateBasicBlock("processLiteralBlock");
        iBuilder->CreateBr(processLiteralBlock);
        iBuilder->SetInsertPoint(processLiteralBlock);

        Value* offsetPos = this->processLiteral(iBuilder, token, phiCursorValue, blockEnd);
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
        phiCursorValue->addIncoming(nextTokenPos, iBuilder->GetInsertBlock());

        iBuilder->CreateBr(processCon);


        // HandleM0Else
        iBuilder->SetInsertPoint(handleM0ElseBlock);

        phiCursorValue->addIncoming(offsetPos, handleM0ElseBlock);
        // Store final M0 pos to make sure the bit stream will be long enough
        Value* finalM0OutputPos = iBuilder->getScalarField("m0OutputPos");
        iBuilder->setProducedItemCount("M0Marker", finalM0OutputPos);
        // finalM0OutputPos should always be 4MB * n except for the final block

        iBuilder->CreateBr(processCon);


        iBuilder->SetInsertPoint(exitBlock);
    }

    Value * LZ4IndexBuilderKernel::advanceUntilNextZero(const unique_ptr<KernelBuilder> &iBuilder, string inputName, Value * startPos, Value * maxPos) {

        Constant* SIZE_64 = iBuilder->getSize(64);

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

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

        Value * currentBlockGlobalPos = iBuilder->CreateUDiv(phiCurrentPos, SIZE_64);
        Value * currentBlockLocalPos = iBuilder->CreateURem(currentBlockGlobalPos, iBuilder->getSize(this->getAnyStreamSetBuffer(inputName)->getBufferBlocks() * iBuilder->getBitBlockWidth() / 64));
        Value * currentPosBitBlockOffset = iBuilder->CreateURem(phiCurrentPos, SIZE_64);

        Value * ptr = iBuilder->CreatePointerCast(iBuilder->getRawInputPointer(inputName, iBuilder->getSize(0)), iBuilder->getInt64Ty()->getPointerTo());
        Value * currentBitValue = iBuilder->CreateLoad(iBuilder->CreateGEP(ptr, currentBlockLocalPos));

        currentBitValue = iBuilder->CreateLShr(currentBitValue, currentPosBitBlockOffset);
        currentBitValue = iBuilder->CreateNot(currentBitValue);

        Value * forwardZeroCount = iBuilder->CreateTrunc(iBuilder->CreateCountForwardZeroes(currentBitValue), iBuilder->getInt64Ty());
        Value * newOffset = iBuilder->CreateAdd(currentPosBitBlockOffset, forwardZeroCount);
        newOffset = iBuilder->CreateUMin(newOffset, iBuilder->getSize(64));

        Value * actualAdvanceValue = iBuilder->CreateSub(newOffset, currentPosBitBlockOffset);
        Value * newPos = iBuilder->CreateAdd(phiCurrentPos, actualAdvanceValue);
        if (maxPos) {
            newPos = iBuilder->CreateUMin(maxPos, newPos);
            actualAdvanceValue = iBuilder->CreateSub(newPos, phiCurrentPos);
            newOffset = iBuilder->CreateAdd(currentPosBitBlockOffset, actualAdvanceValue);
        }

        phiIsFinish->addIncoming(iBuilder->CreateICmpNE(newOffset, iBuilder->getSize(64)), iBuilder->GetInsertBlock());
        phiCurrentPos->addIncoming(newPos, iBuilder->GetInsertBlock());
        iBuilder->CreateBr(advanceConBlock);

        iBuilder->SetInsertPoint(advanceExitBlock);
        return phiCurrentPos;
    }

    Value * LZ4IndexBuilderKernel::generateLoadInt64NumberInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value * globalOffset) {
        Constant* SIZE_STRIDE_SIZE = iBuilder->getSize(getStride());
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, ConstantExpr::getNeg(SIZE_STRIDE_SIZE));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    Value *LZ4IndexBuilderKernel::generateLoadSourceInputByte(const std::unique_ptr<KernelBuilder> &iBuilder, Value * offset) {
        Value * ptr = iBuilder->getRawInputPointer("byteStream", offset);
        return iBuilder->CreateLoad(ptr);
    }

    void LZ4IndexBuilderKernel::increaseScalarField(const unique_ptr<KernelBuilder> &iBuilder, const string &fieldName, Value *value) {
        Value *fieldValue = iBuilder->getScalarField(fieldName);
        fieldValue = iBuilder->CreateAdd(fieldValue, value);
        iBuilder->setScalarField(fieldName, fieldValue);
    }

    void LZ4IndexBuilderKernel::generateStoreNumberOutput(const unique_ptr<KernelBuilder> &iBuilder,
                                                          const string & outputBufferName,
                                                          Value * value) {

        Value * outputOffset = iBuilder->getProducedItemCount(outputBufferName);
        Value * outputRawPtr = iBuilder->getRawOutputPointer(outputBufferName, outputOffset);
        iBuilder->CreateStore(value, outputRawPtr);
        iBuilder->setProducedItemCount(outputBufferName, iBuilder->CreateAdd(outputOffset, iBuilder->getSize(1)));
    }


    void LZ4IndexBuilderKernel::clearCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                             const std::string &bitstreamName,
                                                             llvm::Value *start, llvm::Value *end) {
        //TODO currently we assume that start/end pos is not in the same byte because of the requirement of the LZ4 format
        Value* SIZE_0 = iBuilder->getSize(0);
        Value* SIZE_8 = iBuilder->getSize(8);
        Value* INT8_0 = iBuilder->getInt8(0);
        Type* INT8_PTR_TY = iBuilder->getInt8PtrTy();

        Value* outputBufferBytes = iBuilder->CreateUDiv(iBuilder->getSize(this->getAnyStreamSetBuffer(bitstreamName)->getBufferBlocks() * iBuilder->getBitBlockWidth()), SIZE_8);
        Value* rawOutputPtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer(bitstreamName, SIZE_0), INT8_PTR_TY);

        Value* startRemain = iBuilder->CreateURem(start, SIZE_8);
        Value* startBytePos = iBuilder->CreateUDiv(start, SIZE_8);
        Value* endRemain = iBuilder->CreateURem(end, SIZE_8);
        Value* endBytePos = iBuilder->CreateUDiv(end, SIZE_8);

        BasicBlock* startByteCpyBlock = iBuilder->CreateBasicBlock("startByteCpyBlock");
        BasicBlock* endByteCpyConBlock = iBuilder->CreateBasicBlock("endByteCpyConBlock");
        BasicBlock* endByteCpyBlock = iBuilder->CreateBasicBlock("endByteCpyBlock");
        BasicBlock* memsetBlock = iBuilder->CreateBasicBlock("memsetBlock");

        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(startRemain, SIZE_0), startByteCpyBlock, endByteCpyConBlock);

        // Clear highest {startShiftAmount} bits
        iBuilder->SetInsertPoint(startByteCpyBlock);
        Value* startPtr = iBuilder->CreateGEP(rawOutputPtr, iBuilder->CreateURem(startBytePos, outputBufferBytes));
        Value* startValue = iBuilder->CreateLoad(startPtr);

        Value* startShiftAmount = iBuilder->CreateSub(SIZE_8, startRemain);
        startShiftAmount = iBuilder->CreateZExtOrTrunc(startShiftAmount, startValue->getType());
        startValue = iBuilder->CreateLShr(iBuilder->CreateShl(startValue, startShiftAmount), startShiftAmount);

        iBuilder->CreateStore(startValue, startPtr);
        iBuilder->CreateBr(endByteCpyConBlock);

        iBuilder->SetInsertPoint(endByteCpyConBlock);
        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(endBytePos, SIZE_0), endByteCpyBlock, memsetBlock);

        // Clear lowest {endRemain} bits
        iBuilder->SetInsertPoint(endByteCpyBlock);
        Value* endPtr = iBuilder->CreateGEP(rawOutputPtr, iBuilder->CreateURem(endBytePos, outputBufferBytes));
        Value* endValue = iBuilder->CreateLoad(endPtr);
        endRemain = iBuilder->CreateZExtOrTrunc(endRemain, endValue->getType());
        endValue = iBuilder->CreateShl(iBuilder->CreateLShr(endValue, endRemain), endRemain);
        iBuilder->CreateStore(endValue, endPtr);
        iBuilder->CreateBr(memsetBlock);

        iBuilder->SetInsertPoint(memsetBlock);
        Value* memsetStartByte = iBuilder->CreateUDivCeil(start, SIZE_8);
        Value* memsetEndByte = endBytePos;

        Value* memsetSize = iBuilder->CreateSub(memsetEndByte, memsetStartByte);

        memsetSize = iBuilder->CreateUMin(memsetSize, outputBufferBytes);
        // We always assume that  (memsetEndByte - memsetStartByte) < outputBufferBytes

        Value* memsetStartByteRem = iBuilder->CreateURem(memsetStartByte, outputBufferBytes);

        Value* memsetSize1 = iBuilder->CreateUMin(iBuilder->CreateSub(outputBufferBytes, memsetStartByteRem), memsetSize);
        Value* memsetSize2 = iBuilder->CreateSub(memsetSize, memsetSize1);

        iBuilder->CreateMemSet(iBuilder->CreateGEP(rawOutputPtr, memsetStartByteRem), INT8_0, memsetSize1, true);
        iBuilder->CreateMemSet(rawOutputPtr, INT8_0, memsetSize2, true);
    }

    void LZ4IndexBuilderKernel::setCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                             const std::string &bitstreamName,
                                                             llvm::Value *start, llvm::Value *end) {
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exitBlock");

        Value* SIZE_0 = iBuilder->getSize(0);
        Value* SIZE_1 = iBuilder->getSize(1);
        Value* SIZE_8 = iBuilder->getSize(8);
//        Value* INT8_0 = iBuilder->getInt8(0);
//        Value* INT8_1 = iBuilder->getInt8(1);
        Type* INT8_PTR_TY = iBuilder->getInt8PtrTy();

        Value* outputBufferBytes = iBuilder->getSize(this->getAnyStreamSetBuffer(bitstreamName)->getBufferBlocks() * iBuilder->getBitBlockWidth() / 8);
        Value* rawOutputPtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer(bitstreamName, SIZE_0), INT8_PTR_TY);

        Value* startRemain = iBuilder->CreateURem(start, SIZE_8);
        Value* startBytePos = iBuilder->CreateUDiv(start, SIZE_8);
        Value* endRemain = iBuilder->CreateURem(end, SIZE_8);
        Value* endBytePos = iBuilder->CreateUDiv(end, SIZE_8);
        Value* startShiftAmount = iBuilder->CreateSub(SIZE_8, startRemain);

        BasicBlock* shortSetBlock = iBuilder->CreateBasicBlock("shortSetBlock");
        BasicBlock* longSetBlock = iBuilder->CreateBasicBlock("longSetBlock");

//        iBuilder->CreateBr(startByteCpyBlock);
        iBuilder->CreateCondBr(iBuilder->CreateICmpEQ(startBytePos, endBytePos), shortSetBlock, longSetBlock);

        // When startPos and endPos are in the same byte
        iBuilder->SetInsertPoint(shortSetBlock);
        Value* targetPtr = iBuilder->CreateGEP(rawOutputPtr, iBuilder->CreateURem(startBytePos, outputBufferBytes));
        Value* targetValue = iBuilder->CreateLoad(targetPtr);
        Value* rangeMask = iBuilder->CreateSub(iBuilder->CreateShl(SIZE_1, endRemain), iBuilder->CreateShl(SIZE_1, startRemain));
        rangeMask = iBuilder->CreateZExtOrTrunc(rangeMask, targetValue->getType());
        targetValue = iBuilder->CreateOr(rangeMask, targetValue);

//        targetValue = iBuilder->CreateNot(iBuilder->CreateLShr(iBuilder->CreateShl(iBuilder->CreateNot(targetValue), startShiftAmount), startShiftAmount));
//        targetValue = iBuilder->CreateShl(iBuilder->CreateLShr(targetValue, endRemain), endRemain);
        iBuilder->CreateStore(targetValue, targetPtr);
        iBuilder->CreateBr(exitBlock);

        iBuilder->SetInsertPoint(longSetBlock);

        BasicBlock* startByteCpyBlock = iBuilder->CreateBasicBlock("startByteCpyBlock");
        BasicBlock* endByteCpyConBlock = iBuilder->CreateBasicBlock("endByteCpyConBlock");
        BasicBlock* endByteCpyBlock = iBuilder->CreateBasicBlock("endByteCpyBlock");
        BasicBlock* memsetBlock = iBuilder->CreateBasicBlock("memsetBlock");

        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(startRemain, SIZE_0), startByteCpyBlock, endByteCpyConBlock);
        // Clear highest {startShiftAmount} bits
        iBuilder->SetInsertPoint(startByteCpyBlock);
        Value* startPtr = iBuilder->CreateGEP(rawOutputPtr, iBuilder->CreateURem(startBytePos, outputBufferBytes));
        Value* startValue = iBuilder->CreateLoad(startPtr);

        Value* startShiftAmount2 = iBuilder->CreateZExtOrTrunc(startShiftAmount, startValue->getType());
        startValue = iBuilder->CreateNot(iBuilder->CreateLShr(iBuilder->CreateShl(iBuilder->CreateNot(startValue), startShiftAmount2), startShiftAmount2));

        iBuilder->CreateStore(startValue, startPtr);
        iBuilder->CreateBr(endByteCpyConBlock);

        iBuilder->SetInsertPoint(endByteCpyConBlock);
        iBuilder->CreateCondBr(iBuilder->CreateICmpNE(endBytePos, SIZE_0), endByteCpyBlock, memsetBlock);

        // Clear lowest {endRemain} bits
        iBuilder->SetInsertPoint(endByteCpyBlock);
        Value* endPtr = iBuilder->CreateGEP(rawOutputPtr, iBuilder->CreateURem(endBytePos, outputBufferBytes));
        Value* endValue = iBuilder->CreateLoad(endPtr);
        Value* endRemain2 = iBuilder->CreateZExtOrTrunc(endRemain, endValue->getType());
        endValue = iBuilder->CreateNot(iBuilder->CreateShl(iBuilder->CreateLShr(iBuilder->CreateNot(endValue), endRemain2), endRemain2));
        iBuilder->CreateStore(endValue, endPtr);
        iBuilder->CreateBr(memsetBlock);

        iBuilder->SetInsertPoint(memsetBlock);
        Value* memsetStartByte = iBuilder->CreateUDivCeil(start, SIZE_8);
        Value* memsetEndByte = endBytePos;

        Value* memsetSize = iBuilder->CreateSub(memsetEndByte, memsetStartByte);

        memsetSize = iBuilder->CreateUMin(memsetSize, outputBufferBytes);

        // We always assume that  (memsetEndByte - memsetStartByte) < outputBufferBytes

        Value* memsetStartByteRem = iBuilder->CreateURem(memsetStartByte, outputBufferBytes);

        Value* memsetSize1 = iBuilder->CreateUMin(iBuilder->CreateSub(outputBufferBytes, memsetStartByteRem), memsetSize);
        Value* memsetSize2 = iBuilder->CreateSub(memsetSize, memsetSize1);

        iBuilder->CreateMemSet(iBuilder->CreateGEP(rawOutputPtr, memsetStartByteRem), iBuilder->getInt8(0xff), memsetSize1, true);
        iBuilder->CreateMemSet(rawOutputPtr, iBuilder->getInt8(0xff), memsetSize2, true);
        iBuilder->CreateBr(exitBlock);

        iBuilder->SetInsertPoint(exitBlock);
    }

    void LZ4IndexBuilderKernel::markCircularOutputBitstream(const std::unique_ptr<KernelBuilder> &iBuilder, const string &bitstreamName, Value *pos) {
        Value* SIZE_0 = iBuilder->getSize(0);
        Value* SIZE_8 = iBuilder->getSize(8);
        Value* INT8_1 = iBuilder->getInt8(1);
        Type* bytePtrType = iBuilder->getInt8PtrTy();

        Value* outputBufferBytes = iBuilder->getSize(this->getOutputStreamSetBuffer(bitstreamName)->getBufferBlocks() * iBuilder->getBitBlockWidth() / 8);

        Value* bytePos = iBuilder->CreateUDiv(pos, SIZE_8);
        bytePos = iBuilder->CreateURem(bytePos, outputBufferBytes);
        Value* byteOffset = iBuilder->CreateTrunc(iBuilder->CreateURem(pos, SIZE_8), iBuilder->getInt8Ty());

        Value* outputRawPtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer(bitstreamName, SIZE_0), bytePtrType);
        Value* outputTargetPtr = iBuilder->CreateGEP(outputRawPtr, bytePos);

        Value* targetValue = iBuilder->CreateLoad(outputTargetPtr);
        targetValue = iBuilder->CreateOr(targetValue, iBuilder->CreateShl(INT8_1, byteOffset));
        iBuilder->CreateStore(targetValue, outputTargetPtr);
    }

}
