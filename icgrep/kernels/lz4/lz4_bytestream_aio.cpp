
#include "lz4_bytestream_aio.h"


#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel{

    LZ4ByteStreamAioKernel::LZ4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
            :SegmentOrientedKernel("LZ4ByteStreamAioKernel",
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
                                           Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)},
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


            }){
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZ4ByteStreamAioKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
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
        // TODO store pending value
//        this->storePendingM0(b);
//        this->storePendingLiteralMask(b);
//        this->storePendingMatchOffsetMarker(b);
        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("isCompressed", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);
        b->setProducedItemCount("outputStream", b->getScalarField("outputPos"));
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
//        b->CallPrintIntCond("time", b->getScalarField("tempTimes"), b->getTerminationSignal());
    }

    llvm::Value *LZ4ByteStreamAioKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                            std::string inputBufferName, llvm::Value *globalOffset) {

        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    void
    LZ4ByteStreamAioKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                                 llvm::Value *lz4BlockEnd) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");


        // TODO
        // We store all of the pending data before the parabixBlock of lz4BlockStart to make sure the current pending
        // block will be the same as the block we process during the acceleration
//        this->storePendingLiteralMasksUntilPos(b, lz4BlockStart);
        // TODO handle matchOffsetMarker, and maybe we also need to handle M0Marker here



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
    LZ4ByteStreamAioKernel::generateAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
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
        this->handleLiteralCopy(b, literalStartGlobalPos, literalLength);
        this->handleMatchCopy(b, matchOffset, matchLength);

        phiTokenMarkers->addIncoming(b->CreateOr(phiTokenMarkers, newTokenMarker), b->GetInsertBlock());
        phiLiteralMasks->addIncoming(b->CreateOr(phiLiteralMasks, newLiteralMask), b->GetInsertBlock());
        phiMatchOffsetMarkers->addIncoming(b->CreateOr(phiMatchOffsetMarkers, newMatchOffsetStartMarker), b->GetInsertBlock());

        b->CreateBr(accelerationProcessBlock);

        // ---- AccelerationExitBlock
        b->SetInsertPoint(accelerationExitBlock);

        return std::make_pair(std::make_pair(phiTokenMarkers, phiLiteralMasks), phiMatchOffsetMarkers);
    }

    llvm::Value *LZ4ByteStreamAioKernel::processBlockBoundary(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
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

    std::pair<llvm::Value *, llvm::Value *> LZ4ByteStreamAioKernel::noExtensionLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                                            llvm::Value *currentTokenMarker,
                                                                            llvm::Value *currentExtenderValue,
                                                                            llvm::Value *tokenValue,
                                                                            llvm::Value *blockPosBase,
                                                                            llvm::Value *currentTokenLocalPos
    ) {
        Value* SIZE_1 = b->getSize(1);
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
    LZ4ByteStreamAioKernel::scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *currentTokenMarker,
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

    inline std::pair<llvm::Value *, llvm::Value *> LZ4ByteStreamAioKernel::noExtensionMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                                          llvm::Value *matchOffsetEndMarker,
                                                                          llvm::Value *currentExtenderValue,
                                                                          llvm::Value *tokenValue,
                                                                          llvm::Value *blockPosBase
    ) {
        Value* SIZE_1 = b->getSize(1);
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
    LZ4ByteStreamAioKernel::scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetEndMarker,
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
    LZ4ByteStreamAioKernel::scanThru(const std::unique_ptr<KernelBuilder> &b, llvm::Value *from, llvm::Value *thru) {
        return b->CreateAnd(
                b->CreateAdd(from, thru),
                b->CreateNot(thru)
        );
    }

    void LZ4ByteStreamAioKernel::handleLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                         llvm::Value *literalLength) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        Value* inputBytePtr = b->getRawInputPointer("byteStream", literalStart);
        Value* inputPtr = b->CreatePointerCast(inputBytePtr, INT_FW_PTR);

        Value* outputPos = b->getScalarField("outputPos");
        Value* outputBufferSize = b->getCapacity("outputStream");
        Value* outputPtr = b->getRawOutputPointer("outputStream", b->CreateURem(outputPos, outputBufferSize));
        outputPtr = b->CreatePointerCast(outputPtr, INT_FW_PTR);

        // We can always assume that we have enough output buffer based on our output buffer allocation strategy (except in extract only case)

        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");
        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiOutputPtr = b->CreatePHI(outputPtr->getType(), 2);
        phiOutputPtr->addIncoming(outputPtr, entryBlock);
        PHINode* phiInputPtr = b->CreatePHI(inputPtr->getType(), 2);
        phiInputPtr->addIncoming(inputPtr, entryBlock);
        PHINode* phiCopiedLength = b->CreatePHI(literalLength->getType(), 2);
        phiCopiedLength->addIncoming(b->getSize(0), entryBlock);
        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, literalLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);
        // Always copy fw bits to improve performance
        // TODO sometime it will crash because of overflow copy in the end of the buffer, need to add 4 bytes of
        //      extra buffer in order to make sure it does not crash.
        b->CreateStore(b->CreateLoad(phiInputPtr), phiOutputPtr);

        phiInputPtr->addIncoming(b->CreateGEP(phiInputPtr, b->getSize(1)), b->GetInsertBlock());
        phiOutputPtr->addIncoming(b->CreateGEP(phiOutputPtr, b->getSize(1)), b->GetInsertBlock());
        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, b->getSize(fw / 8)), b->GetInsertBlock());
        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }

    void LZ4ByteStreamAioKernel::handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                       llvm::Value *matchLength) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* outputPos = b->getScalarField("outputPos");
        Value* outputBufferSize = b->getCapacity("outputStream");

        Value* copyToPtr = b->getRawOutputPointer("outputStream", b->CreateURem(outputPos, outputBufferSize));
        Value* copyFromPtr = b->getRawOutputPointer("outputStream", b->CreateURem(b->CreateSub(outputPos, matchOffset), outputBufferSize));

        BasicBlock* matchCopyCon = b->CreateBasicBlock("matchCopyCon");
        BasicBlock* matchCopyBody = b->CreateBasicBlock("matchCopyBody");
        BasicBlock* matchCopyExit = b->CreateBasicBlock("matchCopyExit");

        b->CreateBr(matchCopyCon);

        // ---- matchCopyCon
        b->SetInsertPoint(matchCopyCon);
        PHINode* phiFromPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiFromPtr->addIncoming(copyFromPtr, entryBlock);
        PHINode* phiToPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiToPtr->addIncoming(copyToPtr, entryBlock);
        PHINode* phiCopiedSize = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedSize->addIncoming(b->getSize(0), entryBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCopiedSize, matchLength), matchCopyBody, matchCopyExit);

        // ---- matchCopyBody
        b->SetInsertPoint(matchCopyBody);
        b->CreateStore(
                b->CreateLoad(b->CreatePointerCast(phiFromPtr, INT_FW_PTR)),
        b->CreatePointerCast(phiToPtr, INT_FW_PTR)
        );

        Value* copySize = b->CreateUMin(matchOffset, b->getSize(fw / 8));
        phiFromPtr->addIncoming(b->CreateGEP(phiFromPtr, copySize), b->GetInsertBlock());
        phiToPtr->addIncoming(b->CreateGEP(phiToPtr, copySize), b->GetInsertBlock());
        phiCopiedSize->addIncoming(b->CreateAdd(phiCopiedSize, copySize), b->GetInsertBlock());
        b->CreateBr(matchCopyCon);

        // ---- matchCopyExit
        b->SetInsertPoint(matchCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }


}