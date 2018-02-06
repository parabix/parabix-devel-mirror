
#include "lz4_extract_e_m0.h"
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <iostream>

//#define APPLY_64PACK_ACCELERATION
#define ACCELERATION_LOOP_COUNT (20)

using namespace llvm;
using namespace kernel;
using namespace std;

void LZ4ExtractEM0Kernel::generateDoSequentialSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
    BasicBlock* entryBlock = iBuilder->GetInsertBlock();

    BasicBlock* blockDataLoopCon = iBuilder->CreateBasicBlock("block_data_loop_con");
    BasicBlock* blockDataLoopProcess = iBuilder->CreateBasicBlock("block_data_loop_process");
    BasicBlock* blockDataLoopCompressed = iBuilder->CreateBasicBlock("block_data_loop_compressed");
    BasicBlock* blockDataLoopUncompressed = iBuilder->CreateBasicBlock("block_data_loop_uncompressed");

    BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exit_block");

    iBuilder->CreateBr(blockDataLoopCon);

    // blockDataLoopCon
    iBuilder->SetInsertPoint(blockDataLoopCon);
    Value* blockDataIndex = iBuilder->getScalarField("blockDataIndex");
    Value* availableBlockData = iBuilder->getAvailableItemCount("blockStart");
    iBuilder->CreateCondBr(iBuilder->CreateICmpULT(blockDataIndex, availableBlockData), blockDataLoopProcess, exitBlock);

    // blockDataLoopProcess
    iBuilder->SetInsertPoint(blockDataLoopProcess);
    Value* isCompressed = this->generateLoadCircularInput(iBuilder, "isCompressed", blockDataIndex, iBuilder->getInt1Ty()->getPointerTo());
    iBuilder->CreateCondBr(isCompressed, blockDataLoopCompressed, blockDataLoopUncompressed);

    // blockDataLoop Compressed
    iBuilder->SetInsertPoint(blockDataLoopCompressed);
    this->generateHandleCompressedBlock(iBuilder);


    this->generateIncreaseBlockDataIndex(iBuilder);
    iBuilder->CreateBr(blockDataLoopCon);


    // blockDataLoop Uncompressed
    iBuilder->SetInsertPoint(blockDataLoopUncompressed);
    //handle uncompressed block
    this->generateRecordUncompressedBlock(iBuilder);
//    iBuilder->setProducedItemCount("e1", this->loadCurrentBlockData(iBuilder, "blockEnd"));
    this->generateIncreaseBlockDataIndex(iBuilder);
    iBuilder->CreateBr(blockDataLoopCon);

    //Exit
    iBuilder->SetInsertPoint(exitBlock);

}

BasicBlock* LZ4ExtractEM0Kernel::generateHandleCompressedBlock(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    BasicBlock* entryBlock = iBuilder->GetInsertBlock();
    BasicBlock* exitBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_exit");

    Value* blockStart = this->loadCurrentBlockData(iBuilder, "blockStart");

    this->advanceCursorUntilPos(iBuilder, "extender", iBuilder->CreateZExtOrTrunc(blockStart, iBuilder->getSizeTy()));

    BasicBlock* compressedBlockLoopCon = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_con");
    BasicBlock* compressedBlockLoopBody = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_body");
    BasicBlock* compressedBlockLoopFinal = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_final");

    iBuilder->CreateBr(compressedBlockLoopCon);

    // compressedBlockLoopCon
    iBuilder->SetInsertPoint(compressedBlockLoopCon);
    Value* cursorValue = this->getCursorValue(iBuilder, "extender");
    Value* blockEndPos = this->loadCurrentBlockData(iBuilder, "blockEnd");
    iBuilder->CreateCondBr(iBuilder->CreateICmpULT(cursorValue, blockEndPos), compressedBlockLoopBody, exitBlock);

    // body
    iBuilder->SetInsertPoint(compressedBlockLoopBody);

#ifdef APPLY_64PACK_ACCELERATION
    BasicBlock* accelerationEndBlock = iBuilder->CreateBasicBlock("acceleration_end_block");
    BasicBlock* accelerationFinishBlock = iBuilder->CreateBasicBlock("acceleration_finish_block");

    iBuilder->SetInsertPoint(accelerationEndBlock);

    PHINode* phiTokenMarkers = iBuilder->CreatePHI(iBuilder->getInt64Ty(), ACCELERATION_LOOP_COUNT + 1);
    PHINode* phiE1FinalValue = iBuilder->CreatePHI(iBuilder->getInt64Ty(), ACCELERATION_LOOP_COUNT + 1);
    Value* extenderOffset = this->getCursorValue(iBuilder, "extender");
    Value* packBaseOffset = this->offsetToPackBaseOffset(iBuilder, extenderOffset);

    //    Value* tempTokenMarkers = iBuilder->getScalarField("temp_TokenMarkers");
    Value *tokenPackPos = iBuilder->CreateSub(
            iBuilder->CreateSub(
                    iBuilder->getSize(64),
                    iBuilder->CreateCountReverseZeroes(phiTokenMarkers)
            ),
            iBuilder->getSize(1)
    );

    Value *tokenActualPos = iBuilder->CreateAdd(packBaseOffset, tokenPackPos);

    //TODO output here
    {
        Value* targetPackIndex = iBuilder->CreateSub(iBuilder->getSize(64), iBuilder->CreateCountReverseZeroes(phiE1FinalValue));
        Value* targetActualIndex = iBuilder->CreateAdd(targetPackIndex, packBaseOffset);
        Value* preActualIndex = iBuilder->getProducedItemCount("e1Marker");
        targetActualIndex = iBuilder->CreateSelect(
                iBuilder->CreateICmpUGE(targetActualIndex, preActualIndex),
                targetActualIndex,
                preActualIndex
        );
        this->markCircularOutputBitstream(iBuilder, "e1Marker", preActualIndex, tokenActualPos, false, false);

        Value* extenderOffset = this->getCursorValue(iBuilder, "extender");
        Value* targetOutputIndex = iBuilder->CreateLShr(extenderOffset, iBuilder->getSize(std::log2(64)));
        size_t packNum = this->getOutputBufferSize(iBuilder, "e1Marker") / 64;
        Value* maskedPackIndex = iBuilder->CreateAnd(targetOutputIndex, iBuilder->getSize(packNum - 1));
        Value* accelerationOutputPtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("e1Marker", iBuilder->getSize(0)), iBuilder->getInt64Ty()->getPointerTo());
        accelerationOutputPtr = iBuilder->CreateGEP(accelerationOutputPtr, maskedPackIndex);

//        iBuilder->CallPrintInt("expected", iBuilder->CreateLoad(accelerationOutputPtr));

        Value* v = iBuilder->CreateOr(iBuilder->CreateLoad(accelerationOutputPtr), phiE1FinalValue);
        // TODO phiE1FinalValue
        iBuilder->CreateStore(v, accelerationOutputPtr);
//        iBuilder->CallPrintInt("actual", iBuilder->CreateLoad(accelerationOutputPtr));

        iBuilder->setProducedItemCount("e1Marker", targetActualIndex);
    }



    this->advanceCursorUntilPos(iBuilder, "extender", tokenActualPos);
    iBuilder->CreateBr(accelerationFinishBlock);


    iBuilder->SetInsertPoint(compressedBlockLoopBody);
    //------------------------------------ 64 Pack Acceleration Start
    this->waitCursorUntilInputAvailable(iBuilder, "extender", "extender");

    Value* currentBlockEnd = this->loadCurrentBlockData(iBuilder, "blockEnd");

    extenderOffset = this->getCursorValue(iBuilder, "extender");
    Value* targetOutputIndex = iBuilder->CreateLShr(extenderOffset, iBuilder->getSize(std::log2(64)));
    size_t packNum = this->getOutputBufferSize(iBuilder, "e1Marker") / 64;
    Value* maskedPackIndex = iBuilder->CreateAnd(targetOutputIndex, iBuilder->getSize(packNum - 1));
    Value* accelerationOutputPtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("e1Marker", iBuilder->getSize(0)), iBuilder->getInt64Ty()->getPointerTo());
    accelerationOutputPtr = iBuilder->CreateGEP(accelerationOutputPtr, maskedPackIndex);



//    Value* accelerationPackIndex = this->offsetToPackIndex(iBuilder, extenderOffset);
//    Value* maskedAccelerationPackIndex = this->generateLoadCircularInputPack()


    packBaseOffset = this->offsetToPackBaseOffset(iBuilder, extenderOffset);
    Value* packOffset = this->offsetToPackOffset(iBuilder, extenderOffset);

    Value* extenderPackData = this->generateLoadCircularInputPack(iBuilder, "extender", extenderOffset);
    Value* CC_0xFXPackData = this->generateLoadCircularInputPack(iBuilder, "CC_0xFX", extenderOffset);
    Value* CC_0xXFPackData = this->generateLoadCircularInputPack(iBuilder, "CC_0xXF", extenderOffset);

    // extend_match_only_byte = CC_0xFX &~ CC_0xXF
    Value* extendMatchOnlyBytePack = iBuilder->CreateAnd(
            CC_0xXFPackData,
            iBuilder->CreateNot(
                    CC_0xFXPackData
            )
    );

    Value* tokenMarkers = iBuilder->CreateShl(iBuilder->getInt64(1), packOffset);
//    Value* outputInitValue = iBuilder->CreateLoad(accelerationOutputPtr);
    Value* outputInitValue = iBuilder->getInt64(0);


//    iBuilder->setScalarField("temp_TokenMarkers", tokenMarkers);

    for (int i = 0; i < ACCELERATION_LOOP_COUNT; i++) {
        Value *tokenPackPos = iBuilder->CreateSub(
                iBuilder->CreateSub(
                        iBuilder->getSize(64),
                        iBuilder->CreateCountReverseZeroes(tokenMarkers)
                ),
                iBuilder->getSize(1)
        );
        Value *tokenActualPos = iBuilder->CreateAdd(packBaseOffset, tokenPackPos);

        Value *tokenValue = iBuilder->CreateZExt(
                this->generateLoadSourceInputByte(iBuilder, "byteStream", tokenActualPos),
                iBuilder->getSizeTy()
        );

        Value *literalLengthBase =
                iBuilder->CreateLShr(
                        tokenValue,
                        iBuilder->getSize(4)
                );

        Value* matchLengthBase = iBuilder->CreateAnd(
                tokenValue,
                iBuilder->getSize(0x0f)
        );
        matchLengthBase = iBuilder->CreateAdd(matchLengthBase, iBuilder->getSize(0x4));

        Value *tokenMarker = iBuilder->CreateShl(iBuilder->getInt64(1), tokenPackPos);
        Value * notExtendLiteralMarker = iBuilder->CreateAnd(
                tokenMarker,
                iBuilder->CreateNot(CC_0xFXPackData)
        );
        ////
        Value* expectedNotExtendNextTokenMarker = iBuilder->CreateShl(
                notExtendLiteralMarker,
                iBuilder->CreateAdd(literalLengthBase, iBuilder->getSize(3))
        );  // If not extend literal or match, next token pos will be here, 1 (token) + literalLengthBase + 2 (match offset)


        Value *matchExtendOnlyMarker = iBuilder->CreateAnd(
                tokenMarker,
                extendMatchOnlyBytePack
        );

        Value *extenderMarker = iBuilder->CreateShl(
                matchExtendOnlyMarker,
                iBuilder->CreateAdd(literalLengthBase, iBuilder->getSize(2))
        );  // Match offset

        //ScanThru
        ////
        Value *expectedNextTokenMarker = iBuilder->CreateAnd(
                iBuilder->CreateAdd(
                        extenderMarker,
                        iBuilder->CreateOr(
                                extenderMarker,
                                extenderPackData
                        )
                ),
                iBuilder->CreateNot(extenderPackData)
        );
        expectedNextTokenMarker = iBuilder->CreateShl(
                expectedNextTokenMarker,
                iBuilder->getSize(1)
        );

        Value* needExtendMatch = iBuilder->CreateNot(iBuilder->CreateICmpEQ(matchExtendOnlyMarker, iBuilder->getSize(0)));

        expectedNextTokenMarker = iBuilder->CreateSelect(
                needExtendMatch,
                expectedNextTokenMarker,
                expectedNotExtendNextTokenMarker
        );

        Value *expectedNextTokenPos = iBuilder->CreateAdd(
                packBaseOffset,
                iBuilder->CreateCountForwardZeroes(expectedNextTokenMarker)
        );
        Value *reachBlockEnd = iBuilder->CreateICmpUGE(
                expectedNextTokenPos,
                currentBlockEnd
        );

        expectedNextTokenMarker = iBuilder->CreateSelect(
                reachBlockEnd,
                iBuilder->getInt64(0),
                expectedNextTokenMarker
        );

        tokenMarkers = iBuilder->CreateOr(tokenMarkers, expectedNextTokenMarker, "tokenMarkers");
//        iBuilder->setScalarField("temp_TokenMarkers", tokenMarkers);

        Value* matchOffsetActualPos = iBuilder->CreateAdd(
                tokenActualPos,
                iBuilder->CreateAdd(
                        iBuilder->getSize(1),
                        literalLengthBase
                )
        );

        Value* matchOffsetValue = iBuilder->CreateAdd(
                iBuilder->CreateZExt(
                        this->generateLoadSourceInputByte(iBuilder, "byteStream", matchOffsetActualPos),
                        iBuilder->getSizeTy()
                ),
                iBuilder->CreateShl(
                        iBuilder->CreateZExt(this->generateLoadSourceInputByte(
                                iBuilder,
                                "byteStream",
                                iBuilder->CreateAdd(matchOffsetActualPos , iBuilder->getSize(1))),
                                             iBuilder->getSizeTy()
                        ),
                        iBuilder->getSize(8)
                )
        );


        Value* shouldEndAcceleration = iBuilder->CreateOr(
                iBuilder->CreateICmpEQ(expectedNextTokenMarker, iBuilder->getSize(0)),
                reachBlockEnd
        );


        BasicBlock *pack64AcceMatchExtendOnlyExitBlock = iBuilder->CreateBasicBlock(
                "pack64AcceMatchExtendOnlyExitBlock");

        phiTokenMarkers->addIncoming(tokenMarkers, iBuilder->GetInsertBlock());
        phiE1FinalValue->addIncoming(outputInitValue, iBuilder->GetInsertBlock());

        iBuilder->CreateUnlikelyCondBr(
                shouldEndAcceleration,
                accelerationEndBlock,
                pack64AcceMatchExtendOnlyExitBlock
        );

        iBuilder->SetInsertPoint(pack64AcceMatchExtendOnlyExitBlock);

        // ------------------
        // Yellow Logic Start
        Value *nextTokenPos = iBuilder->CreateAdd(
                packBaseOffset,
                iBuilder->CreateCountForwardZeroes(expectedNextTokenMarker)
        );

        Value *matchExtendLastBitPos = iBuilder->CreateSub(
                nextTokenPos,
                iBuilder->getSize(1)
        );

        Value *matchExtendLastBitValue = this->generateLoadSourceInputByte(iBuilder, "byteStream",
                                                                           matchExtendLastBitPos);
        matchExtendLastBitValue = iBuilder->CreateZExt(matchExtendLastBitValue, iBuilder->getSizeTy());
        Value *matchLength = iBuilder->CreateAdd(
                matchExtendLastBitValue,
                matchLengthBase
        );

        Value *matchExtendLength = iBuilder->CreateSub(
                iBuilder->CreateCountForwardZeroes(expectedNextTokenMarker),
                iBuilder->CreateCountForwardZeroes(extenderMarker)
        );

        matchLength = iBuilder->CreateSelect(
                needExtendMatch,
                iBuilder->CreateAdd(
                        matchLength,
                        iBuilder->CreateMul(
                                iBuilder->CreateSub(matchExtendLength, iBuilder->getSize(2)),
                                iBuilder->getSize(0xff)
                        )
                ),
                matchLengthBase
        );

        Value* oldM0OutputPos = iBuilder->getScalarField("m0OutputPos");


        // Mark E1
        Value *expectedNewOffsetPos = iBuilder->CreateAdd(
                tokenActualPos,
                iBuilder->CreateAdd(
                        literalLengthBase,
                        iBuilder->getSize(1)
                )
        );
        Value *newOffsetPos = expectedNewOffsetPos;
        iBuilder->setScalarField("offsetPos", newOffsetPos);

        // Yellow Logic End
        // ------------------------------
        // Blue Logic Start

        // e1 start:tokenActualPos, e1 end:expectedNewOffsetPos,
//        this->markCircularOutputBitstream(iBuilder, "e1Marker", iBuilder->getProducedItemCount("e1Marker"), tokenActualPos, false);
//        this->markCircularOutputBitstream(iBuilder, "e1Marker", tokenActualPos, expectedNewOffsetPos, true);


        Value* newMask = iBuilder->CreateSub(
                iBuilder->CreateShl(iBuilder->getInt64(1), iBuilder->CreateSub(expectedNewOffsetPos, packBaseOffset)),
                iBuilder->CreateShl(iBuilder->getInt64(1), iBuilder->CreateAdd(iBuilder->CreateSub(tokenActualPos, packBaseOffset), iBuilder->getSize(1)))
        );

        outputInitValue = iBuilder->CreateOr(outputInitValue, newMask);

        Value* basePtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("e1Marker", iBuilder->getSize(0)), iBuilder->getInt64Ty()->getPointerTo());


        Value* m0OutputPos = iBuilder->CreateAdd(oldM0OutputPos, literalLengthBase);

        // Mark M0
        Value *outputEndPos = iBuilder->CreateSub(
                iBuilder->CreateAdd(m0OutputPos, matchLength),
                iBuilder->getInt64(1)
        );

        this->generateStoreCircularOutput(iBuilder, "m0Start", iBuilder->getInt64Ty()->getPointerTo(), m0OutputPos);
//        iBuilder->CallPrintInt("m0Start", m0OutputPos);
        this->generateStoreCircularOutput(iBuilder, "m0End", iBuilder->getInt64Ty()->getPointerTo(), outputEndPos);
//        iBuilder->CallPrintInt("m0End", outputEndPos);
        this->generateStoreCircularOutput(iBuilder, "matchOffset", iBuilder->getInt64Ty()->getPointerTo(), matchOffsetValue);
//        iBuilder->CallPrintInt("matchOffset1", matchOffsetValue);

        m0OutputPos = iBuilder->CreateAdd(m0OutputPos, matchLength);
        iBuilder->setScalarField("m0OutputPos", m0OutputPos);

        // Blue Logic End
        // ---------------------------
    }
    phiTokenMarkers->addIncoming(tokenMarkers, iBuilder->GetInsertBlock());
    phiE1FinalValue->addIncoming(outputInitValue, iBuilder->GetInsertBlock());

    //------------------------------------ 64 Pack Acceleration End
    // Config Extender Cursor after Acceleration
    iBuilder->CreateBr(accelerationEndBlock);

    iBuilder->SetInsertPoint(accelerationFinishBlock);

    //------------------------------------- Finish
#endif

//    iBuilder->CallPrintInt("tokenPos", this->getCursorValue(iBuilder, "extender"));
    Value* token = this->generateLoadSourceInputByte(iBuilder, "byteStream", this->getCursorValue(iBuilder, "extender"));
//    iBuilder->CallPrintInt("token", token);
//    iBuilder->CallPrintInt("tokenPos", this->getCursorValue(iBuilder, "extender"));

//    iBuilder->CreateAssert(iBuilder->CreateICmpULT(this->getCursorValue(iBuilder, "extender"), iBuilder->getSize(0xcb32a)), "ee");
    iBuilder->setScalarField("token", token);

    Value* extendedLiteralValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf0)), iBuilder->getInt8(0xf0));

    // TokenPos can not be refered by next few statements since they may be in different segment
    iBuilder->setScalarField("tokenPos", this->getCursorValue(iBuilder, "extender"));

    BasicBlock* extendLiteralLengthBody = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_body");
    BasicBlock* extendLiteralLengthExit = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_extend_literal_length_exit");

    iBuilder->CreateCondBr(extendedLiteralValue, extendLiteralLengthBody, extendLiteralLengthExit);

    iBuilder->SetInsertPoint(extendLiteralLengthBody);

    this->advanceCursor(iBuilder, "extender", iBuilder->getSize(1));

    this->advanceCursorUntilNextZero(iBuilder, "extender", "extender", this->loadCurrentBlockData(iBuilder, "blockEnd"));

    iBuilder->CreateBr(extendLiteralLengthExit);

    iBuilder->SetInsertPoint(extendLiteralLengthExit);
    // ----May be in a different segment now
    Value* literalLengthEndPos = this->getCursorValue(iBuilder, "extender");
    Value* literalExtensionSize = iBuilder->CreateSub(literalLengthEndPos, iBuilder->getScalarField("tokenPos"));
    Value* finalLengthByte = this->generateLoadSourceInputByte(iBuilder, "byteStream", this->getCursorValue(iBuilder, "extender"));

    finalLengthByte = iBuilder->CreateZExt(finalLengthByte, iBuilder->getSizeTy());
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
    token = iBuilder->getScalarField("token");
    Value* literalLengthBase = iBuilder->CreateLShr(iBuilder->CreateZExt(token, iBuilder->getInt64Ty()), iBuilder->getInt64(4));
    Value* literalLength = iBuilder->CreateAdd(literalLengthBase, literalLengthExtendValue);


    Value* previousOffsetPos = iBuilder->getScalarField("offsetPos");

    Value* offsetPos = iBuilder->CreateAdd(
            iBuilder->CreateAdd(
                    literalLengthEndPos,
                    literalLength),
            iBuilder->getSize(1));
    iBuilder->setScalarField("offsetPos", offsetPos);
//    iBuilder->CallPrintInt("literalStart", iBuilder->CreateAdd(literalLengthEndPos, iBuilder->getSize(1)));
//    iBuilder->CallPrintInt("literalLength", literalLength);
    this->markCircularOutputBitstream(iBuilder, "e1Marker", iBuilder->getProducedItemCount("e1Marker"), iBuilder->CreateAdd(literalLengthEndPos, iBuilder->getSize(1)), false);
    this->markCircularOutputBitstream(iBuilder, "e1Marker", iBuilder->CreateAdd(literalLengthEndPos, iBuilder->getSize(1)), offsetPos, true);

    Value* basePtr = iBuilder->CreatePointerCast(iBuilder->getRawOutputPointer("e1Marker", iBuilder->getSize(0)), iBuilder->getInt64Ty()->getPointerTo());

    this->increaseScalarField(iBuilder, "m0OutputPos", literalLength);


    BasicBlock* handleM0BodyBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_handle_m0_body");
    BasicBlock* handleM0ElseBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_handle_m0_else");

    iBuilder->CreateCondBr(
            iBuilder->CreateICmpULT(offsetPos, this->loadCurrentBlockData(iBuilder, "blockEnd")),
            handleM0BodyBlock,
            handleM0ElseBlock
    );
    // HandleM0Body

    iBuilder->SetInsertPoint(handleM0BodyBlock);

    Value* matchLengthStartPos = iBuilder->CreateAdd(iBuilder->getScalarField("offsetPos"), iBuilder->getSize(1));
    iBuilder->setScalarField("matchLengthStartPos", matchLengthStartPos);
    this->advanceCursorUntilPos(iBuilder, "extender", matchLengthStartPos);


    token = iBuilder->getScalarField("token");

    Value* extendedMatchValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf)), iBuilder->getInt8(0xf), "extendedMatchValue");

    BasicBlock* extendMatchBodyBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_body");
    BasicBlock* extendMatchExitBlock = iBuilder->CreateBasicBlock("block_data_loop_handle_compressed_block_loop_extend_match_exit");

    iBuilder->CreateCondBr(extendedMatchValue, extendMatchBodyBlock, extendMatchExitBlock);

    iBuilder->SetInsertPoint(extendMatchBodyBlock);

    //ExtendMatchBodyBlock
    this->advanceCursor(iBuilder, "extender", iBuilder->getSize(1));
    this->advanceCursorUntilNextZero(iBuilder, "extender", "extender", this->loadCurrentBlockData(iBuilder, "blockEnd"));

    // ----May be in a different segment now
    iBuilder->CreateBr(extendMatchExitBlock);

    //ExtendMatchExitBlock
    iBuilder->SetInsertPoint(extendMatchExitBlock);
    matchLengthStartPos = iBuilder->getScalarField("matchLengthStartPos");
    Value* oldMatchExtensionSize = iBuilder->CreateSub(this->getCursorValue(iBuilder, "extender"), matchLengthStartPos);

    token = iBuilder->getScalarField("token");
    extendedMatchValue = iBuilder->CreateICmpEQ(iBuilder->CreateAnd(token, iBuilder->getInt8(0xf)), iBuilder->getInt8(0xf));

    Value* matchExtensionSize = iBuilder->CreateSelect(
            iBuilder->CreateICmpEQ(extendedMatchValue, iBuilder->getInt1(true)),
            oldMatchExtensionSize,
            iBuilder->getSize(0)
    );


    Value* matchLengthBase = iBuilder->CreateZExt(iBuilder->CreateAnd(token, iBuilder->getInt8(0x0f)), iBuilder->getInt64Ty());
    Value* matchLength = iBuilder->CreateAdd(matchLengthBase, iBuilder->getInt64(4));


    Value* extensionLastBitPos = iBuilder->CreateAdd(iBuilder->getScalarField("offsetPos"), iBuilder->getSize(1));
    extensionLastBitPos = iBuilder->CreateAdd(extensionLastBitPos, matchExtensionSize);
    Value* extensionLastBitValue = this->generateLoadSourceInputByte(iBuilder, "byteStream", extensionLastBitPos);
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
            iBuilder->CreateZExt(
                    this->generateLoadSourceInputByte(iBuilder, "byteStream", iBuilder->getScalarField("offsetPos")),
                    iBuilder->getSizeTy()
            ),
            iBuilder->CreateShl(
                    iBuilder->CreateZExt(this->generateLoadSourceInputByte(
                            iBuilder,
                            "byteStream",
                            iBuilder->CreateAdd(iBuilder->getScalarField("offsetPos"), iBuilder->getSize(1))),
                                         iBuilder->getSizeTy()
                    ),
                    iBuilder->getSize(8)
            )
    );
//    iBuilder->CallPrintInt("matchOffset", matchOffset);


    this->generateStoreCircularOutput(iBuilder, "m0Start", iBuilder->getInt64Ty()->getPointerTo(), outputPos);
//    iBuilder->CallPrintInt("m0Start", outputPos);
    this->generateStoreCircularOutput(iBuilder, "m0End", iBuilder->getInt64Ty()->getPointerTo(), outputEndPos);
//    iBuilder->CallPrintInt("m0End", outputEndPos);
    this->generateStoreCircularOutput(iBuilder, "matchOffset", iBuilder->getInt64Ty()->getPointerTo(), matchOffset);
//    iBuilder->CallPrintInt("matchOffset", matchOffset);


    this->increaseScalarField(iBuilder, "m0OutputPos", matchLength);
    this->advanceCursor(iBuilder, "extender", iBuilder->getSize(1));

    iBuilder->CreateBr(compressedBlockLoopFinal);

    // HandleM0Else
    iBuilder->SetInsertPoint(handleM0ElseBlock);
    this->advanceCursorUntilPos(iBuilder, "extender", iBuilder->getScalarField("offsetPos"));
    iBuilder->CreateBr(compressedBlockLoopFinal);

    // final
    iBuilder->SetInsertPoint(compressedBlockLoopFinal);
    iBuilder->CreateBr(compressedBlockLoopCon);

    // Exit
    iBuilder->SetInsertPoint(exitBlock);

    return exitBlock;
}

void LZ4ExtractEM0Kernel::generateRecordUncompressedBlock(const unique_ptr<kernel::KernelBuilder> & iBuilder) {
    Value* blockStart = this->loadCurrentBlockData(iBuilder, "blockStart");
    Value* blockEnd = this->loadCurrentBlockData(iBuilder, "blockEnd");
    Value* length = iBuilder->CreateSub(blockEnd, blockStart);
    Value* outputPos = iBuilder->getScalarField("m0OutputPos");
    this->increaseScalarField(iBuilder, "m0OutputPos", length);

    // Store Uncompressed Data
    this->generateStoreCircularOutput(iBuilder, "uncompressedStartPos", iBuilder->getInt64Ty()->getPointerTo(), blockStart);
    this->generateStoreCircularOutput(iBuilder, "uncompressedLength", iBuilder->getInt64Ty()->getPointerTo(), length);
    this->generateStoreCircularOutput(iBuilder, "uncompressedOutputPos", iBuilder->getInt64Ty()->getPointerTo(), outputPos);
}

void LZ4ExtractEM0Kernel::generateIncreaseBlockDataIndex(const unique_ptr<kernel::KernelBuilder> & iBuilder) {
    this->increaseScalarField(iBuilder, "blockDataIndex", iBuilder->getSize(1));
}

Value* LZ4ExtractEM0Kernel::loadCurrentBlockData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const std::string& name) {
    Value* blockDataIndex = iBuilder->getScalarField("blockDataIndex");
    return this->generateLoadCircularInput(iBuilder, name, blockDataIndex, iBuilder->getInt64Ty()->getPointerTo());
}

LZ4ExtractEM0Kernel::LZ4ExtractEM0Kernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder, const std::map<std::string, size_t>& inputIndexMap):
        SequentialKernel(
                iBuilder,
                "lz4_extract_e_m0_kernel",
                {//Inputs
                        Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "extender"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "CC_0xFX"},
                        Binding{iBuilder->getStreamSetTy(1, 1), "CC_0xXF"},

                        // block data
                        Binding{iBuilder->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "blockEnd", BoundedRate(0, 1)}
                },
                {//Outputs
                        // Uncompressed_data
                        Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedStartPos", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedLength", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedOutputPos", BoundedRate(0, 1)},

                        Binding{iBuilder->getStreamSetTy(1, 1), "e1Marker", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "m0Start", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "m0End", BoundedRate(0, 1)},
                        Binding{iBuilder->getStreamSetTy(1, 64), "matchOffset", BoundedRate(0, 1)}
                },
                {//Arguments
                },
                {},
                {//Internal States
                        Binding{iBuilder->getSizeTy(), "blockDataIndex"},
                        Binding{iBuilder->getInt64Ty(), "m0OutputPos"},
                        Binding{iBuilder->getSizeTy(), "tokenPos"},
                        Binding{iBuilder->getInt8Ty(), "token"},
                        Binding{iBuilder->getSizeTy(), "matchLengthStartPos"},
                        Binding{iBuilder->getSizeTy(), "offsetPos"}
//                        Binding{iBuilder->getInt64Ty(), "temp_TokenMarkers"}
                }
        ) {
    this->initBufferCursor(iBuilder, {"extender"});
    this->configIndexBits(iBuilder, inputIndexMap);
//    this->configOutputBufferToBeClear({{"byteStream", "e1Marker"}});
//    setNoTerminateAttribute(true);
}