//
// Created by wxy325 on 2018/6/22.
//

#include "lz4_sequential_aio_base.h"
#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>


using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{
    LZ4SequentialAioBaseKernel::LZ4SequentialAioBaseKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::string&& kernelName, unsigned blockSize)
            :SegmentOrientedKernel(std::move(kernelName),
            // Inputs
                                   {
                    Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},
//                    Binding{b->getStreamSetTy(1, 1), "extender", RateEqualTo("byteStream")},

                    // block data
                    Binding{b->getStreamSetTy(1, 1), "isCompressed", BoundedRate(0, 1), AlwaysConsume()},
                    Binding{b->getStreamSetTy(1, 64), "blockStart", RateEqualTo("isCompressed"), AlwaysConsume()},
                    Binding{b->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("isCompressed"), AlwaysConsume()}

            },
            //Outputs
                                   {

                                   },
            //Arguments
                                   {
                                           Binding{b->getSizeTy(), "fileSize"}
                                   },
                                   {},
            //Internal states:
                                   {
                                           Binding{b->getSizeTy(), "blockDataIndex"},
                                           Binding{b->getInt64Ty(), "outputPos"},


                                   }){
        this->setStride(blockSize);
        addAttribute(MustExplicitlyTerminate());
    }

    // ---- Kernel Methods
    void LZ4SequentialAioBaseKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getScalarField("blockDataIndex");

        // In MultiblockKernel, availableItemCount + processedItemCount == producedItemCount from previous kernel
        // While in SegmentOrigentedKernel, availableItemCount == producedItemCount from previous kernel
        Value * totalNumber = b->getAvailableItemCount("blockEnd");
//        Value * totalExtender = b->getAvailableItemCount("extender");

        Value * blockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);

        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        b->SetInsertPoint(blockEndConBlock);
        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);
        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
//        b->CreateCondBr(b->CreateICmpULE(blockEnd, totalExtender), processBlock, exitBlock);
        b->CreateBr(processBlock);

        b->SetInsertPoint(processBlock);

        //TODO handle uncompressed block
        this->processCompressedLz4Block(b, blockStart, blockEnd);
        this->storePendingOutput(b);

//        this->storePendingM0(b);
//        this->storePendingLiteralMask(b);
//        this->storePendingMatchOffsetMarker(b);
        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("isCompressed", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);
        this->setProducedOutputItemCount(b, b->getScalarField("outputPos"));
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
    }


    // ---- LZ4 Format Parsing
    void
    LZ4SequentialAioBaseKernel::processCompressedLz4Block(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                                llvm::Value *lz4BlockEnd) {
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
        auto accelerationRet = this->doAcceleration(b, phiCursorValue, lz4BlockEnd);
        Value* tokenMarkers = accelerationRet.first.first;

        Value* cursorBlockPosBase = b->CreateSub(phiCursorValue, b->CreateURem(phiCursorValue, b->getSize(ACCELERATION_WIDTH)));
        Value* nextTokenLocalPos = b->CreateSub(b->CreateSub(b->getSize(ACCELERATION_WIDTH), b->CreateCountReverseZeroes(tokenMarkers)), b->getSize(1));
        Value* nextTokenGlobalPos = b->CreateAdd(cursorBlockPosBase, nextTokenLocalPos);

        nextTokenGlobalPos = this->processLz4Sequence(b, nextTokenGlobalPos, lz4BlockEnd);
        */
        Value* nextTokenGlobalPos = this->processLz4Sequence(b, phiCursorValue, lz4BlockEnd);
        phiCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
    }

    std::pair<std::pair<llvm::Value *, llvm::Value *>, llvm::Value *>
    LZ4SequentialAioBaseKernel::doAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                                     llvm::Value *blockEnd) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        // Constant
        Value* SIZE_ACCELERATION_WIDTH = b->getSize(ACCELERATION_WIDTH);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_0 = b->getSize(0);

        Type* INT_ACCELERATION_TYPE = b->getIntNTy(ACCELERATION_WIDTH);
        Value* INT_ACCELERATION_0 = b->getIntN(ACCELERATION_WIDTH, 0);
        Value* INT_ACCELERATION_1 = b->getIntN(ACCELERATION_WIDTH, 1);

        // ---- Entry Block
        this->prepareAcceleration(b, beginTokenPos);
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

        Value* tokenReverseZeros = b->CreateCountReverseZeroes(phiTokenMarkers);
        Value* currentTokenLocalPos = b->CreateSub(b->CreateSub(SIZE_ACCELERATION_WIDTH, tokenReverseZeros), SIZE_1);
        Value* currentTokenMarker = b->CreateShl(INT_ACCELERATION_1, currentTokenLocalPos); // 1 marker is in Token Pos
        Value* currentTokenGlobalPos = b->CreateAdd(blockPosBase, currentTokenLocalPos);
        Value* tokenValue = b->CreateLoad(b->getRawInputPointer("byteStream", currentTokenGlobalPos));

        llvm::Value *literalLength, *literalLengthEndMarker;
        std::tie(literalLength, literalLengthEndMarker) =
//                this->scanThruLiteralLength(b, currentTokenMarker, currentExtenderValue, tokenValue, blockPosBase,
//                                            currentTokenLocalPos);
                this->noExtensionLiteralLength(b, currentTokenMarker, currentExtenderValue, tokenValue, blockPosBase,
                                               currentTokenLocalPos);
        Value* literalBeginMarker = b->CreateShl(literalLengthEndMarker, SIZE_1);
        Value* literalStartLocalPos = b->CreateCountForwardZeroes(literalBeginMarker);
        Value* literalStartGlobalPos = b->CreateAdd(blockPosBase, literalStartLocalPos);


        Value* literalEndMarker = b->CreateSelect(b->CreateICmpULT(literalLength, SIZE_ACCELERATION_WIDTH), b->CreateShl(literalBeginMarker, literalLength), INT_ACCELERATION_0);

        Value* newLiteralMask = b->CreateSub(literalEndMarker, literalBeginMarker);

        Value* newMatchOffsetStartMarker = literalEndMarker;

        Value *matchLength, *matchLengthEndMarker;
//        std::tie(matchLength, matchLengthEndMarker) =
//                this->scanThruMatchLength(b, b->CreateShl(newMatchOffsetStartMarker, SIZE_1), currentExtenderValue,
//                                          tokenValue, blockPosBase);

        std::tie(matchLength, matchLengthEndMarker) =
                this->noExtensionMatchLength(b, b->CreateShl(newMatchOffsetStartMarker, SIZE_1), currentExtenderValue,
                                             tokenValue, blockPosBase);

        Value *newTokenMarker = b->CreateShl(matchLengthEndMarker, SIZE_1);
        Value* newTokenMarkerLocalPos = b->CreateCountForwardZeroes(newTokenMarker);

        Value* reachEnd = b->CreateOr(
                b->CreateICmpEQ(newTokenMarker, b->getSize(0)),
                b->CreateICmpUGE(b->CreateAdd(newTokenMarkerLocalPos, blockPosBase), blockEnd)
        );

        BasicBlock* dataProcessBlock = b->CreateBasicBlock("dataProcessBlock");
        b->CreateUnlikelyCondBr(reachEnd, accelerationExitBlock, dataProcessBlock);

        // ---- dataProcessBlock
        b->SetInsertPoint(dataProcessBlock);

        Value* matchOffsetStartLocalPos = b->CreateCountForwardZeroes(newMatchOffsetStartMarker);
        Value* matchOffsetStartGlobalPos = b->CreateAdd(matchOffsetStartLocalPos, blockPosBase);

        Value* matchOffsetPtr = b->getRawInputPointer("byteStream", matchOffsetStartGlobalPos);
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        matchOffsetPtr = b->CreatePointerCast(matchOffsetPtr, b->getInt16Ty()->getPointerTo());
        Value* matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), b->getSizeTy());

        // TODO all of the literal data here will always be in the same 64-bit literal block, it may be better if we provide
        //      this information to the literal copy method, especially when we are working with swizzled form
        this->doAccelerationLiteralCopy(b, literalStartGlobalPos, literalLength);
        this->doAccelerationMatchCopy(b, matchOffset, matchLength);

        phiTokenMarkers->addIncoming(b->CreateOr(phiTokenMarkers, newTokenMarker), b->GetInsertBlock());
        phiLiteralMasks->addIncoming(b->CreateOr(phiLiteralMasks, newLiteralMask), b->GetInsertBlock());
        phiMatchOffsetMarkers->addIncoming(b->CreateOr(phiMatchOffsetMarkers, newMatchOffsetStartMarker), b->GetInsertBlock());

        b->CreateBr(accelerationProcessBlock);

        // ---- AccelerationExitBlock
        b->SetInsertPoint(accelerationExitBlock);

        this->finishAcceleration(b, beginTokenPos, phiLiteralMasks);

        return std::make_pair(std::make_pair(phiTokenMarkers, phiLiteralMasks), phiMatchOffsetMarkers);
    }

    llvm::Value *LZ4SequentialAioBaseKernel::processLz4Sequence(const std::unique_ptr<KernelBuilder> &b,
                                                      llvm::Value *beginTokenPos,
                                                      llvm::Value *lz4BlockEnd) {
        // Constant
        ConstantInt* SIZE_0 = b->getSize(0);
        ConstantInt* SIZE_1 = b->getSize(1);
        ConstantInt* BYTE_FF = b->getInt8(0xff);
        Value* BYTE_F0 = b->getInt8(0xf0);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* tokenValue = b->CreateLoad(b->CreateGEP(bytePtrBase, beginTokenPos));

        Value* shouldExtendLiteral = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_F0), BYTE_F0);

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

        // This literal copy will always cross 64 bits literal boundary
        this->doLiteralCopy(b, literalStartPos, literalLength);
        BasicBlock* extendLiteralEndFinal = b->GetInsertBlock();

        b->CreateLikelyCondBr(b->CreateICmpULT(matchOffsetBeginPos, lz4BlockEnd), hasMatchPartBlock, exitBlock);

        // ---- hasMatchPartBlock
        b->SetInsertPoint(hasMatchPartBlock);

        llvm::Value *matchLength, *matchEndPos;
        std::tie(matchEndPos, matchLength) = this->parseMatchInfo(b, matchOffsetBeginPos, tokenValue);

        Value* matchOffsetPtr = b->getRawInputPointer("byteStream", matchOffsetBeginPos);
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        matchOffsetPtr = b->CreatePointerCast(matchOffsetPtr, b->getInt16Ty()->getPointerTo());
        Value* matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), b->getSizeTy());
        this->doMatchCopy(b, matchOffset, matchLength);
        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiBeforeTokenPos = b->CreatePHI(b->getSizeTy(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPos, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(matchEndPos, extendMatchExitFinal);
        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, SIZE_1);

        return nextTokenPos;
    }

    std::pair<llvm::Value*, llvm::Value*> LZ4SequentialAioBaseKernel::parseMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetBeginPos, llvm::Value* tokenValue) {

        ConstantInt* SIZE_0 = b->getSize(0);
        ConstantInt* SIZE_1 = b->getSize(1);
        ConstantInt* BYTE_FF = b->getInt8(0xff);
        Value* BYTE_0F = b->getInt8(0x0f);
        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* matchOffsetNextPos = b->CreateAdd(matchOffsetBeginPos, SIZE_1);

        BasicBlock* extendMatchCon = b->CreateBasicBlock("extendMatchCon");
        BasicBlock* extendMatchExit = b->CreateBasicBlock("extendMatchExit");



        // ---- entryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);
        Value* initExtendMatchPos = b->CreateAdd(matchOffsetBeginPos, b->getSize(2));
        b->CreateCondBr(shouldExtendMatch, extendMatchCon, extendMatchExit);

        // ---- extendMatchCon
        b->SetInsertPoint(extendMatchCon);

        PHINode* phiCurrentExtendMatchPos = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentExtendMatchPos->addIncoming(initExtendMatchPos, entryBlock);
        PHINode* phiExtendMatchLength = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendMatchLength->addIncoming(SIZE_0, entryBlock);

        Value* currentMatchLengthByte = b->CreateLoad(b->CreateGEP(bytePtrBase, phiCurrentExtendMatchPos));
        Value* newExtendMatchLength = b->CreateAdd(phiExtendMatchLength, b->CreateZExt(currentMatchLengthByte, b->getSizeTy()));

        phiCurrentExtendMatchPos->addIncoming(b->CreateAdd(phiCurrentExtendMatchPos, SIZE_1), b->GetInsertBlock());
        phiExtendMatchLength->addIncoming(newExtendMatchLength, b->GetInsertBlock());

        b->CreateCondBr(b->CreateICmpEQ(currentMatchLengthByte, BYTE_FF), extendMatchCon, extendMatchExit);

        // ---- extendMatchExit
        b->SetInsertPoint(extendMatchExit);
        PHINode* matchExtendValue = b->CreatePHI(b->getSizeTy(), 2);
        matchExtendValue->addIncoming(SIZE_0, entryBlock);
        matchExtendValue->addIncoming(newExtendMatchLength, extendMatchCon);
        PHINode* phiExtendMatchEndPos = b->CreatePHI(b->getSizeTy(), 2);
        phiExtendMatchEndPos->addIncoming(matchOffsetNextPos, entryBlock);
        phiExtendMatchEndPos->addIncoming(phiCurrentExtendMatchPos, extendMatchCon);

        // matchLength = (size_t)token & 0xf + 4 + matchExtendValue
        Value* matchLength = b->CreateAdd(
                b->CreateAdd(matchExtendValue, b->getSize(4)),
                b->CreateZExt(b->CreateAnd(tokenValue, BYTE_0F), b->getSizeTy())
        );
        return std::make_pair(phiExtendMatchEndPos, matchLength);
    };

    std::pair<llvm::Value*, llvm::Value*> LZ4SequentialAioBaseKernel::parseMatchInfo2(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetBeginPos, llvm::Value* tokenValue) {

        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* BYTE_0F = b->getInt8(0x0f);
        Value* SIZE_1 = b->getSize(1);

        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* initExtendMatchPos = b->CreateAdd(matchOffsetBeginPos, b->getSize(2));
        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);


        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");


        Value* baseMatchLength = b->CreateAdd(b->CreateZExt(b->CreateAnd(tokenValue, BYTE_0F), b->getSizeTy()), b->getSize(4));
        Value* matchOffsetNextPos = b->CreateAdd(matchOffsetBeginPos, SIZE_1);

        b->CreateLikelyCondBr(shouldExtendMatch, bodyBlock, exitBlock);

        b->SetInsertPoint(bodyBlock);


        Value *currentMatchLengthValues = b->CreateLoad(b->CreatePointerCast(b->CreateGEP(bytePtrBase, initExtendMatchPos), b->getIntNTy(64)->getPointerTo()));
        Value* forwardZeros = b->CreateCountForwardZeroes(b->CreateNot(currentMatchLengthValues));
        Value* i64ForwardZeros = b->CreateTrunc(forwardZeros, b->getInt64Ty());
        Value* extensionLength = b->CreateUDiv(i64ForwardZeros, b->getInt64(8));
        Value* a = b->CreateTrunc(b->CreateLShr(currentMatchLengthValues, b->CreateZExt(b->CreateMul(extensionLength, b->getInt64(8)), b->getIntNTy(64))), b->getInt64Ty());
        a = b->CreateAnd(a, b->getInt64(0xff));

        Value* extensionValue = b->CreateAdd(b->CreateMul(extensionLength, b->getInt64(0xff)), a);

        extensionValue = b->CreateSelect(shouldExtendMatch, extensionValue, b->getInt64(0));

        Value* matchLength2 = b->CreateAdd(
                baseMatchLength,
                extensionValue
        );
        Value* newPos = b->CreateAdd(matchOffsetNextPos, b->CreateSelect(shouldExtendMatch, b->CreateAdd(extensionLength, b->getInt64(1)), b->getInt64(0)));
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);

        PHINode* phiNewPos = b->CreatePHI(b->getSizeTy(), 2);
        phiNewPos->addIncoming(matchOffsetNextPos, entryBlock);
        phiNewPos->addIncoming(newPos, bodyBlock);

        PHINode* phiMatchLength = b->CreatePHI(b->getSizeTy(), 2);
        phiMatchLength->addIncoming(baseMatchLength, entryBlock);
        phiMatchLength->addIncoming(matchLength2, bodyBlock);

        return std::make_pair(phiNewPos, phiMatchLength);
    }


    std::pair<llvm::Value *, llvm::Value *> LZ4SequentialAioBaseKernel::noExtensionLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                                             llvm::Value *currentTokenMarker,
                                                                                             llvm::Value *currentExtenderValue,
                                                                                             llvm::Value *tokenValue,
                                                                                             llvm::Value *blockPosBase,
                                                                                             llvm::Value *currentTokenLocalPos
    ) {
        Value* BYTE_F0 = b->getInt8(0xf0);
        Value* shouldExtendLiteral = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_F0), BYTE_F0);

        Value* literalLength =
                b->CreateLShr(
                        b->CreateZExt(tokenValue, b->getSizeTy()),
                        b->getSize(4)
                );

        Value* retLiteralMarker = b->CreateSelect(shouldExtendLiteral, b->getInt64(0), currentTokenMarker);

        return std::make_pair(literalLength, retLiteralMarker);
    };

    std::pair<llvm::Value *, llvm::Value *>
    LZ4SequentialAioBaseKernel::scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *currentTokenMarker,
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

    inline std::pair<llvm::Value *, llvm::Value *> LZ4SequentialAioBaseKernel::noExtensionMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                                                  llvm::Value *matchOffsetEndMarker,
                                                                                                  llvm::Value *currentExtenderValue,
                                                                                                  llvm::Value *tokenValue,
                                                                                                  llvm::Value *blockPosBase
    ) {
        Value* BYTE_0F = b->getInt8(0x0f);
        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);

        // Extend Match Length
        Value* matchLength =
                b->CreateAdd(
                        b->CreateZExt(b->CreateAnd(tokenValue, BYTE_0F), b->getSizeTy()),
                        b->getSize(4)
                );

        Value* retMatchMarker = b->CreateSelect(shouldExtendMatch, b->getInt64(0), matchOffsetEndMarker);
        return std::make_pair(matchLength, retMatchMarker);
    };

    std::pair<llvm::Value *, llvm::Value *>
    LZ4SequentialAioBaseKernel::scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetEndMarker,
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

    // ---- Basic Function
    llvm::Value *LZ4SequentialAioBaseKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset) {
        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    llvm::Value *
    LZ4SequentialAioBaseKernel::scanThru(const std::unique_ptr<KernelBuilder> &b, llvm::Value *from, llvm::Value *thru) {
        return b->CreateAnd(
                b->CreateAdd(from, thru),
                b->CreateNot(thru)
        );
    }
}