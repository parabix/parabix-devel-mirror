
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

    // TODO IndexBuilderKernel is responsible to clear the output buffer for final produced block
    // e.g. when produce item count is 0x120, IndexBuilderKernel needs to set 0x121 ~ 0x200 to 0

    LZ4IndexBuilderKernel::LZ4IndexBuilderKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
    : SegmentOrientedKernel("LZ4IndexBuilderKernel",
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
           // Uncompressed_data
           Binding{b->getStreamSetTy(1, 64), "uncompressedStartPos",
                   BoundedRate(0, 1)},
           Binding{b->getStreamSetTy(1, 64), "uncompressedLength",
                   BoundedRate(0, 1)},
           Binding{b->getStreamSetTy(1, 64), "uncompressedOutputPos",
                   BoundedRate(0, 1)},

           Binding{b->getStreamSetTy(1, 1), "deletionMarker", BoundedRate(0, 1)},
           Binding{b->getStreamSetTy(1, 1), "M0Marker", BoundedRate(0, 1)},
           Binding{b->getStreamSetTy(1, 1), "MatchOffsetMarker", RateEqualTo("byteStream")}
    },
    //Arguments
    {
           Binding{b->getSizeTy(), "fileSize"}
    },
    {},
    //Internal states:
    {
           Binding{b->getSizeTy(), "blockDataIndex"},
           Binding{b->getInt64Ty(), "m0OutputPos"},

           // For MatchOffset Output
           Binding{b->getIntNTy(64), "pendingMatchOffsetMarkerBits"},
           Binding{b->getInt64Ty(), "pendingMarchOffsetMarkerIndex"},

           // For deletionMarker output
           Binding{b->getIntNTy(64), "pendingDeletionMarkerStartBits"},
           Binding{b->getIntNTy(64), "pendingDeletionMarkerEndBits"},
           Binding{b->getIntNTy(64), "pendingDeletionMarkerCarryBit"},
           Binding{b->getInt64Ty(), "pendingDeletionMarkerIndex"},

           // For M0 Output
           Binding{b->getIntNTy(64), "pendingM0StartBits"},
           Binding{b->getIntNTy(64), "pendingM0EndBits"},
           Binding{b->getIntNTy(64), "pendingM0CarryBit"},
           Binding{b->getInt64Ty(), "pendingM0Index"},


    }) {
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZ4IndexBuilderKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getScalarField("blockDataIndex");

        // In MultiblockKernel, availableItemCount + processedItemCount == producedItemCount from previous kernel
        // While in SegmentOrigentedKernel, availableItemCount == producedItemCount from previous kernel
        Value * totalNumber = b->getAvailableItemCount("blockEnd");
        Value * totalExtender = b->getAvailableItemCount("extender");

        Value * blockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);

        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        b->SetInsertPoint(blockEndConBlock);
        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);
        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
        b->CreateCondBr(b->CreateICmpULE(blockEnd, totalExtender), processBlock, exitBlock);

        b->SetInsertPoint(processBlock);

        //TODO handle uncompressed block

        this->generateProcessCompressedBlock(b, blockStart, blockEnd);
        this->storePendingM0(b);
        this->storePendingDeletionMarker(b);
        this->storePendingMatchOffsetMarker(b);
        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("isCompressed", newBlockDataIndex);

        b->setProcessedItemCount("byteStream", blockEnd);
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
    }

    Value* LZ4IndexBuilderKernel::processLiteral(const std::unique_ptr<KernelBuilder> &b, Value* token, Value* tokenPos, Value* blockEnd) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value * extendedLiteralValue = b->CreateICmpEQ(b->CreateAnd(token, b->getInt8(0xf0)), b->getInt8(0xf0));

        BasicBlock* extendLiteralLengthCon = b->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_con");
        BasicBlock* extendLiteralLengthBody = b->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_body");
        BasicBlock* extendLiteralLengthExit = b->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_exit");

        b->CreateCondBr(extendedLiteralValue, extendLiteralLengthCon, extendLiteralLengthExit);

        b->SetInsertPoint(extendLiteralLengthCon);

        Value * const nextTokenPos = b->CreateAdd(tokenPos, b->getInt64(1));
        Value * const nextToken = b->CreateLoad(b->getRawInputPointer("byteStream", nextTokenPos));
        Value * const isExitToken = b->CreateICmpNE(nextToken, b->getInt8(0xff));
        b->CreateLikelyCondBr(isExitToken, extendLiteralLengthExit, extendLiteralLengthBody);


        b->SetInsertPoint(extendLiteralLengthBody);
        Value* newCursorPos2 = this->advanceUntilNextZero(b, "extender", b->CreateAdd(tokenPos, b->getInt64(1)), blockEnd);
        BasicBlock* advanceFinishBlock = b->GetInsertBlock();


        b->CreateBr(extendLiteralLengthExit);

        b->SetInsertPoint(extendLiteralLengthExit);
        PHINode* phiCursorPosAfterLiteral = b->CreatePHI(b->getInt64Ty(), 3);
        phiCursorPosAfterLiteral->addIncoming(nextTokenPos, extendLiteralLengthCon);
        phiCursorPosAfterLiteral->addIncoming(newCursorPos2, advanceFinishBlock);
        phiCursorPosAfterLiteral->addIncoming(tokenPos, entryBlock);

        Value * literalExtensionSize = b->CreateSub(phiCursorPosAfterLiteral, tokenPos);
        Value * finalLengthByte = this->generateLoadSourceInputByte(b, phiCursorPosAfterLiteral);
        finalLengthByte = b->CreateZExt(finalLengthByte, b->getInt64Ty());
        Value * literalLengthExtendValue = b->CreateSelect(
                b->CreateICmpUGT(literalExtensionSize, b->getSize(0)),
                b->CreateAdd(
                        b->CreateMul(
                                b->CreateSub(literalExtensionSize, b->getSize(1)),
                                b->getSize(255)
                        ),
                        finalLengthByte
                ),
                b->getSize(0)
        );
        literalLengthExtendValue = b->CreateZExt(literalLengthExtendValue, b->getInt64Ty());
        Value* literalLengthBase = b->CreateLShr(b->CreateZExt(token, b->getInt64Ty()), b->getInt64(4));
        Value* literalLength = b->CreateAdd(literalLengthBase, literalLengthExtendValue);

        Value* offsetPos = b->CreateAdd(
                b->CreateAdd(
                        phiCursorPosAfterLiteral,
                        literalLength),
                b->getSize(1));

        this->appendDeletionMarkerOutput(b, b->getProducedItemCount("deletionMarker"), b->CreateAdd(phiCursorPosAfterLiteral, b->getSize(1)));

        b->setProducedItemCount("deletionMarker", offsetPos);
        this->increaseScalarField(b, "m0OutputPos", literalLength); //TODO m0OutputPos may be removed from scalar fields
        return offsetPos;
    }

    Value* LZ4IndexBuilderKernel::processMatch(const std::unique_ptr<KernelBuilder> &iBuilder, Value* offsetPos, Value* token, Value* blockEnd) {
        Constant* INT64_ONE = iBuilder->getInt64(1);

        BasicBlock* entryBlock = iBuilder->GetInsertBlock();

        Value* extendMatchStartPos = iBuilder->CreateAdd(offsetPos, INT64_ONE);
        Value* extendedMatchValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf)), iBuilder->getInt8(0xf));

        BasicBlock* extendMatchBodyBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_body");
        BasicBlock* extendMatchExitBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_exit");

        iBuilder->CreateCondBr(extendedMatchValue, extendMatchBodyBlock, extendMatchExitBlock);

        iBuilder->SetInsertPoint(extendMatchBodyBlock);

        //ExtendMatchBodyBlock
        Value* newCursorPos = this->advanceUntilNextZero(iBuilder, "extender", iBuilder->CreateAdd(extendMatchStartPos, INT64_ONE), blockEnd);
        BasicBlock* advanceFinishBlock = iBuilder->GetInsertBlock();

        iBuilder->CreateBr(extendMatchExitBlock);

        //ExtendMatchExitBlock
        iBuilder->SetInsertPoint(extendMatchExitBlock);
        PHINode* phiCursorPosAfterMatch = iBuilder->CreatePHI(iBuilder->getInt64Ty(), 2);
        phiCursorPosAfterMatch->addIncoming(newCursorPos, advanceFinishBlock);
        phiCursorPosAfterMatch->addIncoming(extendMatchStartPos, entryBlock);

        Value* oldMatchExtensionSize = iBuilder->CreateSub(phiCursorPosAfterMatch, extendMatchStartPos);
        Value* matchExtensionSize = iBuilder->CreateSelect(extendedMatchValue, oldMatchExtensionSize, iBuilder->getSize(0));
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



        this->appendMatchOffsetMarkerOutput(iBuilder, offsetPos);
        this->increaseScalarField(iBuilder, "m0OutputPos", matchLength);
        this->appendM0Output(iBuilder, outputPos, outputEndPos);

        return iBuilder->CreateAdd(phiCursorPosAfterMatch, INT64_ONE);
    }

    void LZ4IndexBuilderKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &iBuilder, Value* blockStart, Value* blockEnd) {
        BasicBlock* entryBlock = iBuilder->GetInsertBlock();


        Value* isTerminal = iBuilder->CreateICmpEQ(blockEnd, iBuilder->getScalarField("fileSize"));
        iBuilder->setTerminationSignal(isTerminal);

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
//        Constant* SIZE_STRIDE_SIZE = iBuilder->getSize(getStride());
        Constant* SIZE_STRIDE_SIZE = iBuilder->getSize(this->getInputStreamSetBuffer(inputBufferName)->getBufferBlocks() * iBuilder->getBitBlockWidth());
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

    void LZ4IndexBuilderKernel::appendM0Output(const std::unique_ptr<KernelBuilder> &b, llvm::Value *start, llvm::Value *end) {
        // ---- Entry
        // Constant

        int fw = 64;
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_256 = b->getSize(fw);
        Value* INT256_0 = b->getIntN(fw, 0);
        Value* INT256_1 = b->getIntN(fw, 1);

        Value* startBlockIndex = b->CreateUDiv(start, SIZE_256);
        Value* startOffset = b->CreateZExt(b->CreateURem(start, SIZE_256), b->getIntNTy(fw));
        Value* endBlockIndex = b->CreateUDiv(end, SIZE_256);
        Value* endOffset = b->CreateZExt(b->CreateURem(end, SIZE_256), b->getIntNTy(fw));


        BasicBlock* appendM0Con = b->CreateBasicBlock("appendM0Con");
        BasicBlock* appendM0Body = b->CreateBasicBlock("appendM0Body");
        BasicBlock* appendM0Exit = b->CreateBasicBlock("appendM0Exit");

        Value* pendingM0Index = b->getScalarField("pendingM0Index");
        Value* pendingM0StartBits = b->getScalarField("pendingM0StartBits");
        Value* pendingM0EndBits = b->getScalarField("pendingM0EndBits");
        Value* pendingM0CarryBit = b->getScalarField("pendingM0CarryBit");

        b->CreateBr(appendM0Con);

        // ---- AppendM0Con
        b->SetInsertPoint(appendM0Con);
        PHINode* phiCurrentIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentIndex->addIncoming(pendingM0Index, entryBlock);
        PHINode* phiStartBits = b->CreatePHI(b->getIntNTy(fw), 2);
        phiStartBits->addIncoming(pendingM0StartBits, entryBlock);
        PHINode* phiEndBits = b->CreatePHI(b->getIntNTy(fw), 2);
        phiEndBits->addIncoming(pendingM0EndBits, entryBlock);
        PHINode* phiCarryBit = b->CreatePHI(b->getIntNTy(fw), 2);
        phiCarryBit->addIncoming(pendingM0CarryBit, entryBlock);


        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiCurrentIndex, endBlockIndex), appendM0Body, appendM0Exit);
        // ---- AppendM0Body
        b->SetInsertPoint(appendM0Body);
        Value* actualStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT256_1, startOffset)), phiStartBits);
        Value* outputValue = b->CreateSub(b->CreateSub(phiEndBits, actualStartBits), phiCarryBit);
        Value* newCarryBit = b->CreateZExt(b->CreateICmpUGT(b->CreateAdd(actualStartBits, phiCarryBit), phiEndBits), b->getIntNTy(fw));

        this->storeM0(b, phiCurrentIndex, outputValue);

        phiCurrentIndex->addIncoming(b->CreateAdd(phiCurrentIndex, SIZE_1), b->GetInsertBlock());
        phiStartBits->addIncoming(INT256_0, b->GetInsertBlock());
        phiEndBits->addIncoming(INT256_0, b->GetInsertBlock());
        phiCarryBit->addIncoming(newCarryBit, b->GetInsertBlock());

        b->CreateBr(appendM0Con);

        // ---- AppendM0Exit
        b->SetInsertPoint(appendM0Exit);
        Value* finalStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT256_1, startOffset)), phiStartBits);
        Value* finalEndBits = b->CreateOr(phiEndBits, b->CreateShl(INT256_1, endOffset));
        b->setScalarField("pendingM0Index", phiCurrentIndex);
        b->setScalarField("pendingM0StartBits", finalStartBits);
        b->setScalarField("pendingM0EndBits", finalEndBits);
        b->setScalarField("pendingM0CarryBit", phiCarryBit);
    }

    void LZ4IndexBuilderKernel::storeM0(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockIndex, llvm::Value* value) {
        int fw = 64;
        Value* m0BufferBlocks = b->getSize(this->getOutputStreamSetBuffer("M0Marker")->getBufferBlocks() * b->getBitBlockWidth() / fw);
        Value* indexRem = b->CreateURem(blockIndex, m0BufferBlocks);
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("M0Marker", b->getSize(0)), b->getIntNTy(fw)->getPointerTo());
        b->CreateStore(value, b->CreateGEP(outputBasePtr, indexRem));
    }

    void LZ4IndexBuilderKernel::storePendingM0(const std::unique_ptr<KernelBuilder> &b) {
        Value* outputValue = b->CreateSub(
                b->CreateSub(
                        b->getScalarField("pendingM0EndBits"),
                        b->getScalarField("pendingM0StartBits")
                ),
                b->getScalarField("pendingM0CarryBit")
        );
        this->storeM0(b, b->getScalarField("pendingM0Index"), outputValue);
    }

    void LZ4IndexBuilderKernel::appendDeletionMarkerOutput(const std::unique_ptr<KernelBuilder> &b,
                                                           llvm::Value *start, llvm::Value *end) {
        // ---- Entry
        // Constant

        int fw = 64;
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_256 = b->getSize(fw);
        Value* INT256_0 = b->getIntN(fw, 0);
        Value* INT256_1 = b->getIntN(fw, 1);

        Value* startBlockIndex = b->CreateUDiv(start, SIZE_256);
        Value* startOffset = b->CreateZExt(b->CreateURem(start, SIZE_256), b->getIntNTy(fw));
        Value* endBlockIndex = b->CreateUDiv(end, SIZE_256);
        Value* endOffset = b->CreateZExt(b->CreateURem(end, SIZE_256), b->getIntNTy(fw));


        BasicBlock* appendDeletionMarkerCon = b->CreateBasicBlock("appendDeletionMarkerCon");
        BasicBlock* appendDeletionMarkerBody = b->CreateBasicBlock("appendDeletionMarkerBody");
        BasicBlock* appendDeletionMarkerExit = b->CreateBasicBlock("appendDeletionMarkerExit");

        Value* pendingDeletionMarkerIndex = b->getScalarField("pendingDeletionMarkerIndex");
        Value* pendingDeletionMarkerStartBits = b->getScalarField("pendingDeletionMarkerStartBits");
        Value* pendingDeletionMarkerEndBits = b->getScalarField("pendingDeletionMarkerEndBits");
        Value* pendingDeletionMarkerCarryBit = b->getScalarField("pendingDeletionMarkerCarryBit");

        b->CreateBr(appendDeletionMarkerCon);

        // ---- AppendM0Con
        b->SetInsertPoint(appendDeletionMarkerCon);
        PHINode* phiCurrentIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentIndex->addIncoming(pendingDeletionMarkerIndex, entryBlock);
        PHINode* phiStartBits = b->CreatePHI(b->getIntNTy(fw), 2);
        phiStartBits->addIncoming(pendingDeletionMarkerStartBits, entryBlock);
        PHINode* phiEndBits = b->CreatePHI(b->getIntNTy(fw), 2);
        phiEndBits->addIncoming(pendingDeletionMarkerEndBits, entryBlock);
        PHINode* phiCarryBit = b->CreatePHI(b->getIntNTy(fw), 2);
        phiCarryBit->addIncoming(pendingDeletionMarkerCarryBit, entryBlock);


        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiCurrentIndex, endBlockIndex), appendDeletionMarkerBody, appendDeletionMarkerExit);
        // ---- AppendM0Body
        b->SetInsertPoint(appendDeletionMarkerBody);
        Value* actualStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT256_1, startOffset)), phiStartBits);
        Value* outputValue = b->CreateSub(b->CreateSub(phiEndBits, actualStartBits), phiCarryBit);
        Value* newCarryBit = b->CreateZExt(b->CreateICmpUGT(b->CreateAdd(actualStartBits, phiCarryBit), phiEndBits), b->getIntNTy(fw));

        this->storeDeletionMarker(b, phiCurrentIndex, outputValue);

        phiCurrentIndex->addIncoming(b->CreateAdd(phiCurrentIndex, SIZE_1), b->GetInsertBlock());
        phiStartBits->addIncoming(INT256_0, b->GetInsertBlock());
        phiEndBits->addIncoming(INT256_0, b->GetInsertBlock());
        phiCarryBit->addIncoming(newCarryBit, b->GetInsertBlock());

        b->CreateBr(appendDeletionMarkerCon);

        // ---- AppendM0Exit
        b->SetInsertPoint(appendDeletionMarkerExit);
        Value* finalStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT256_1, startOffset)), phiStartBits);
        Value* finalEndBits = b->CreateOr(phiEndBits, b->CreateShl(INT256_1, endOffset));
        b->setScalarField("pendingDeletionMarkerIndex", phiCurrentIndex);
        b->setScalarField("pendingDeletionMarkerStartBits", finalStartBits);
        b->setScalarField("pendingDeletionMarkerEndBits", finalEndBits);
        b->setScalarField("pendingDeletionMarkerCarryBit", phiCarryBit);
    }

    void
    LZ4IndexBuilderKernel::storeDeletionMarker(const std::unique_ptr<KernelBuilder> &b, llvm::Value *blockIndex,
                                               llvm::Value *value) {
        int fw = 64;
        Value* m0BufferBlocks = b->getSize(this->getOutputStreamSetBuffer("deletionMarker")->getBufferBlocks() * b->getBitBlockWidth() / fw);
        Value* indexRem = b->CreateURem(blockIndex, m0BufferBlocks);

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("deletionMarker", b->getSize(0)), b->getIntNTy(fw)->getPointerTo());
        b->CreateStore(value, b->CreateGEP(outputBasePtr, indexRem));
    }

    void LZ4IndexBuilderKernel::storePendingDeletionMarker(const std::unique_ptr<KernelBuilder> &b) {
        Value* outputValue = b->CreateSub(
                b->CreateSub(
                        b->getScalarField("pendingDeletionMarkerEndBits"),
                        b->getScalarField("pendingDeletionMarkerStartBits")
                ),
                b->getScalarField("pendingDeletionMarkerCarryBit")
        );
        this->storeDeletionMarker(b, b->getScalarField("pendingDeletionMarkerIndex"), outputValue);
    }

    void LZ4IndexBuilderKernel::appendMatchOffsetMarkerOutput(const std::unique_ptr<KernelBuilder> &b,
                                                              llvm::Value *position) {
        // ---- Entry
        // Constant
        int fw = 64;
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_256 = b->getSize(fw);
        Value* INT256_0 = b->getIntN(fw, 0);
        Value* INT256_1 = b->getIntN(fw, 1);

        Value* endBlockIndex = b->CreateUDiv(position, SIZE_256);
        Value* endOffset = b->CreateZExt(b->CreateURem(position, SIZE_256), b->getIntNTy(fw));

        BasicBlock* appendMatchOffsetMarkerCon = b->CreateBasicBlock("appendMatchOffsetMarkerCon");
        BasicBlock* appendMatchOffsetMarkerBody = b->CreateBasicBlock("appendMatchOffsetMarkerBody");
        BasicBlock* appendMatchOffsetMarkerExit = b->CreateBasicBlock("appendMatchOffsetMarkerExit");

        Value* pendingMatchOffsetMarkerIndex = b->getScalarField("pendingMarchOffsetMarkerIndex");
        Value* pendingMatchOffsetMarkerEndBits = b->getScalarField("pendingMatchOffsetMarkerBits");

        b->CreateBr(appendMatchOffsetMarkerCon);

        // ---- AppendM0Con
        b->SetInsertPoint(appendMatchOffsetMarkerCon);
        PHINode* phiCurrentIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentIndex->addIncoming(pendingMatchOffsetMarkerIndex, entryBlock);
        PHINode* phiEndBits = b->CreatePHI(b->getIntNTy(fw), 2);
        phiEndBits->addIncoming(pendingMatchOffsetMarkerEndBits, entryBlock);

        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiCurrentIndex, endBlockIndex), appendMatchOffsetMarkerBody, appendMatchOffsetMarkerExit);
        // ---- AppendM0Body
        b->SetInsertPoint(appendMatchOffsetMarkerBody);
        this->storeMatchOffsetMarker(b, phiCurrentIndex, phiEndBits);
        phiCurrentIndex->addIncoming(b->CreateAdd(phiCurrentIndex, SIZE_1), b->GetInsertBlock());
        phiEndBits->addIncoming(INT256_0, b->GetInsertBlock());

        b->CreateBr(appendMatchOffsetMarkerCon);

        // ---- AppendM0Exit
        b->SetInsertPoint(appendMatchOffsetMarkerExit);
        Value* finalEndBits = b->CreateOr(phiEndBits, b->CreateShl(INT256_1, endOffset));
        b->setScalarField("pendingMarchOffsetMarkerIndex", phiCurrentIndex);
        b->setScalarField("pendingMatchOffsetMarkerBits", finalEndBits);
    }

    void LZ4IndexBuilderKernel::storeMatchOffsetMarker(const std::unique_ptr<KernelBuilder> &b,
                                                       llvm::Value *blockIndex, llvm::Value *value) {
        int fw = 64;
        Value* m0BufferBlocks = b->getSize(this->getOutputStreamSetBuffer("MatchOffsetMarker")->getBufferBlocks() * b->getBitBlockWidth() / fw);
        Value* indexRem = b->CreateURem(blockIndex, m0BufferBlocks);

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("MatchOffsetMarker", b->getSize(0)), b->getIntNTy(fw)->getPointerTo());
        b->CreateStore(value, b->CreateGEP(outputBasePtr, indexRem));
    }

    void LZ4IndexBuilderKernel::storePendingMatchOffsetMarker(const std::unique_ptr<KernelBuilder> &b) {
        this->storeMatchOffsetMarker(
                b,
                b->getScalarField("pendingMarchOffsetMarkerIndex"),
                b->getScalarField("pendingMatchOffsetMarkerBits")
        );
    }
}