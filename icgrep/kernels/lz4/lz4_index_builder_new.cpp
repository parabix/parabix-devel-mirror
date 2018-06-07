
#include "lz4_index_builder_new.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{


    LZ4IndexBuilderNewKernel::LZ4IndexBuilderNewKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
    :SegmentOrientedKernel("LZ4IndexBuilderNewKernel",
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

                                   Binding{b->getStreamSetTy(1, 1), "deletionMarker", RateEqualTo("byteStream")},
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
                                Binding{b->getInt64Ty(), "tempTimes"},
                                   Binding{b->getSizeTy(), "blockDataIndex"},
//                                   Binding{b->getInt64Ty(), "m0OutputPos"},
//
//                                   // For MatchOffset Output
//                                   Binding{b->getIntNTy(64), "pendingMatchOffsetMarkerBits"},
//                                   Binding{b->getInt64Ty(), "pendingMarchOffsetMarkerIndex"},
//
//                                   // For deletionMarker output
                                    //pendingDeletionMarker = (EndBits - StartBits - CarryBit) | FullBits
                                   Binding{b->getIntNTy(64), "pendingLiteralMarkerStartBits"},
                                   Binding{b->getIntNTy(64), "pendingLiteralMarkerEndBits"},
                                   Binding{b->getIntNTy(64), "pendingLiteralMarkerCarryBit"},
                                   Binding{b->getIntNTy(64), "pendingLiteralMarkerFullBits"},
                                   Binding{b->getInt64Ty(), "pendingLiteralMarkerIndex"},

//                                   // For M0 Output
//                                   Binding{b->getIntNTy(64), "pendingM0StartBits"},
//                                   Binding{b->getIntNTy(64), "pendingM0EndBits"},
//                                   Binding{b->getIntNTy(64), "pendingM0CarryBit"},
//                                   Binding{b->getInt64Ty(), "pendingM0Index"},


                           }){
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZ4IndexBuilderNewKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
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
        this->storePendingLiteralMask(b);
//        this->storePendingMatchOffsetMarker(b);
        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("isCompressed", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->CallPrintIntCond("time", b->getScalarField("tempTimes"), b->getTerminationSignal());
    }

    llvm::Value *LZ4IndexBuilderNewKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                        std::string inputBufferName,
                                                                        llvm::Value *globalOffset) {
        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    void LZ4IndexBuilderNewKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b,
                                                                  llvm::Value *lz4BlockStart, llvm::Value *lz4BlockEnd) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");

        // We store all of the pending data before the parabixBlock of lz4BlockStart to make sure the current pending
        // block will be the same as the block we process during the acceleration
        this->storePendingLiteralMasksUntilPos(b, lz4BlockStart);
        // TODO handle matchOffsetMarker, and maybe we also need to handle M0Marker here


        BasicBlock* beforeProcessConBlock = b->GetInsertBlock();
        b->CreateBr(processCon);
        b->SetInsertPoint(processCon);

        PHINode* phiCursorValue = b->CreatePHI(b->getInt64Ty(), 2, "phiCursorValue"); // phiCursorValue should always be the position of next token except for the final sequence
        phiCursorValue->addIncoming(lz4BlockStart, beforeProcessConBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCursorValue, lz4BlockEnd), processBody, exitBlock);

        b->SetInsertPoint(processBody);

        Value* time1 = b->CreateReadCycleCounter();
        auto accelerationRet = this->generateAcceleration(b, phiCursorValue, lz4BlockEnd);
        Value* time2 = b->CreateReadCycleCounter();
        b->setScalarField("tempTimes", b->CreateAdd(b->getScalarField("tempTimes"), b->CreateSub(time2, time1)));


        Value* tokenMarkers = accelerationRet.first.first;
        Value* literalMasks = accelerationRet.first.second;
//        b->CallPrintInt("phiCursorValue", phiCursorValue);
//        b->CallPrintInt("literalMasks", literalMasks);
//        Value* matchOffsetMarkers = accelerationRet.second;

        // The pending data block index (for compression space) will be the same as the block index of current acceleration block
        b->setScalarField("pendingLiteralMarkerFullBits", b->CreateOr(literalMasks, b->getScalarField("pendingLiteralMarkerFullBits")));


        Value* cursorBlockPosBase = b->CreateSub(phiCursorValue, b->CreateURem(phiCursorValue, b->getSize(ACCELERATION_WIDTH)));
        Value* nextTokenLocalPos = b->CreateSub(b->CreateSub(b->getSize(ACCELERATION_WIDTH), b->CreateCountReverseZeroes(tokenMarkers)), b->getSize(1));
        Value* nextTokenGlobalPos = b->CreateAdd(cursorBlockPosBase, nextTokenLocalPos);

        nextTokenGlobalPos = this->processBlockBoundary(b, nextTokenGlobalPos, lz4BlockEnd);

        phiCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
    }

    llvm::Value* LZ4IndexBuilderNewKernel::processBlockBoundary(const std::unique_ptr<KernelBuilder> &b,
                              llvm::Value *beginTokenPos, llvm::Value* lz4BlockEnd) {
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
        // TODO mark MatchOffset : matchOffsetBeginPos, and matchLength: matchLength
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiBeforeTokenPos = b->CreatePHI(b->getSizeTy(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPos, extendLiteralEnd);
        phiBeforeTokenPos->addIncoming(phiExtendMatchEndPos, extendMatchExit);
        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, SIZE_1);

        this->appendBoundaryLiteralMaskOutput(b, literalStartPos, literalEndPos, nextTokenPos);

        return nextTokenPos;
    }

    std::pair<std::pair<llvm::Value*, llvm::Value*>, llvm::Value*> LZ4IndexBuilderNewKernel::generateAcceleration(const std::unique_ptr<KernelBuilder> &b,
                                                        llvm::Value *beginTokenPos, llvm::Value* blockEnd) {
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

//        Value* extenderPtrBase = b->CreatePointerCast(b->getRawInputPointer("extender", SIZE_0), INT_ACCELERATION_TYPE->getPointerTo());
//        Value* actualPtr = b->CreateGEP(
//                b->CreatePointerCast(b->getRawInputPointer("extender", SIZE_0), INT_ACCELERATION_TYPE->getPointerTo()),
//                b->CreateUDiv(maskedTokenPos, SIZE_ACCELERATION_WIDTH)
//        );
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
                this->scanThruLiteralLength(b, currentTokenMarker, currentExtenderValue, tokenValue, blockPosBase,
                                            currentTokenLocalPos);
        Value* literalBeginMarker = b->CreateShl(literalLengthEndMarker, SIZE_1);
        Value* literalEndMarker = b->CreateSelect(b->CreateICmpULT(literalLength, SIZE_ACCELERATION_WIDTH), b->CreateShl(literalBeginMarker, literalLength), INT_ACCELERATION_0);

        Value* newLiteralMask = b->CreateSub(literalEndMarker, literalBeginMarker);

        Value* newMatchOffsetStartMarker = literalEndMarker; // TODO It is possible that we reach block end here in final block

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

        phiTokenMarkers->addIncoming(b->CreateOr(phiTokenMarkers, newTokenMarker), b->GetInsertBlock());
        phiLiteralMasks->addIncoming(b->CreateOr(phiLiteralMasks, newLiteralMask), b->GetInsertBlock());
        phiMatchOffsetMarkers->addIncoming(b->CreateOr(phiMatchOffsetMarkers, newMatchOffsetStartMarker), b->GetInsertBlock());

        b->CreateCondBr(reachEnd, accelerationExitBlock, accelerationProcessBlock);

        // ---- AccelerationExitBlock
        b->SetInsertPoint(accelerationExitBlock);
        // TODO handle m0 output

        return std::make_pair(std::make_pair(phiTokenMarkers, phiLiteralMasks), phiMatchOffsetMarkers);
    }

    std::pair<llvm::Value *, llvm::Value *>
    LZ4IndexBuilderNewKernel::scanThruLiteralLength(const std::unique_ptr<KernelBuilder> &b,
                                                    llvm::Value *currentTokenMarker,
                                                    llvm::Value *currentExtenderValue,
                                                    llvm::Value *tokenValue,
                                                    llvm::Value *blockPosBase,
                                                    llvm::Value *currentTokenLocalPos
    ) {
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
    LZ4IndexBuilderNewKernel::scanThruMatchLength(const std::unique_ptr<KernelBuilder> &b,
                                                    llvm::Value *matchOffsetEndMarker,
                                                    llvm::Value *currentExtenderValue,
                                                    llvm::Value *tokenValue,
                                                    llvm::Value *blockPosBase
    ){
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
    };

    llvm::Value *LZ4IndexBuilderNewKernel::scanThru(const std::unique_ptr<KernelBuilder> &b, llvm::Value *from,
                                                    llvm::Value *thru) {
        return b->CreateAnd(
                b->CreateAdd(from, thru),
                b->CreateNot(thru)
        );
    }


    void LZ4IndexBuilderNewKernel::appendBoundaryLiteralMaskOutput(const std::unique_ptr<KernelBuilder> &b,
                                                                   llvm::Value *globalLiteralStartPos,
                                                                   llvm::Value *globalLiteralEndPos,
                                                                   llvm::Value *outputUntilPos) {
        // block index of globalStartPos will always equal to pending block index,
        // and outputUntilPos will be equal to nextTokenPos,
        // after this method, the pendingIndex will always be equal to the block index
        // of next acceleration

        // ---- Entry
        // Constant
        unsigned fw = 64;
        Type* INT_FW_TY = b->getIntNTy(fw);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_FW = b->getSize(fw);
        Value* INT_FW_0 = b->getIntN(fw, 0);
        Value* INT_FW_1 = b->getIntN(fw, 1);



        Value* startBlockIndex = b->CreateUDiv(globalLiteralStartPos, SIZE_FW);
        Value* startOffset = b->CreateZExt(b->CreateURem(globalLiteralStartPos, SIZE_FW), b->getIntNTy(fw));
        Value* endBlockIndex = b->CreateUDiv(globalLiteralEndPos, SIZE_FW);
        Value* endOffset = b->CreateZExt(b->CreateURem(globalLiteralEndPos, SIZE_FW), b->getIntNTy(fw));
        Value* finishBlockIndex = b->CreateUDiv(outputUntilPos, SIZE_FW);


        BasicBlock* appendLiteralMaskCon = b->CreateBasicBlock("appendLiteralMaskCon");
        BasicBlock* appendLiteralMaskBody = b->CreateBasicBlock("appendLiteralMaskBody");
        BasicBlock* appendLiteralMaskExit = b->CreateBasicBlock("appendLiteralMaskExit1");

        Value* pendingLiteralMarkerIndex = b->getScalarField("pendingLiteralMarkerIndex");
        Value* pendingLiteralMarkerStartBits = b->getScalarField("pendingLiteralMarkerStartBits");
        Value* pendingLiteralMarkerEndBits = b->getScalarField("pendingLiteralMarkerEndBits");
        Value* pendingLiteralMarkerCarryBit = b->getScalarField("pendingLiteralMarkerCarryBit");
        Value* pendingLiteralMarkerFullBits = b->getScalarField("pendingLiteralMarkerFullBits");

        BasicBlock* beforeCondBlock = b->GetInsertBlock();
        b->CreateBr(appendLiteralMaskCon);

        // ---- appendLiteralMaskCon
        b->SetInsertPoint(appendLiteralMaskCon);
        PHINode* phiCurrentIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentIndex->addIncoming(pendingLiteralMarkerIndex, beforeCondBlock);
        PHINode* phiStartBits = b->CreatePHI(INT_FW_TY, 2);
        phiStartBits->addIncoming(pendingLiteralMarkerStartBits, beforeCondBlock);
        PHINode* phiEndBits = b->CreatePHI(INT_FW_TY, 2);
        phiEndBits->addIncoming(pendingLiteralMarkerEndBits, beforeCondBlock);
        PHINode* phiCarryBit = b->CreatePHI(INT_FW_TY, 2);
        phiCarryBit->addIncoming(pendingLiteralMarkerCarryBit, beforeCondBlock);
        PHINode* phiFullBits = b->CreatePHI(INT_FW_TY, 2);
        phiFullBits->addIncoming(pendingLiteralMarkerFullBits, beforeCondBlock);

        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiCurrentIndex, finishBlockIndex), appendLiteralMaskBody, appendLiteralMaskExit);

        // ---- appendLiteralMaskBody
        b->SetInsertPoint(appendLiteralMaskBody);
        Value* actualStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT_FW_1, startOffset)), phiStartBits);
        Value* actualEndBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, endBlockIndex), b->CreateOr(phiEndBits, b->CreateShl(INT_FW_1, endOffset)), phiEndBits);
        Value* outputValue = b->CreateOr(b->CreateSub(b->CreateSub(actualEndBits, actualStartBits), phiCarryBit), phiFullBits);
        Value* newCarryBit = b->CreateZExt(b->CreateICmpUGT(b->CreateAdd(actualStartBits, phiCarryBit), actualEndBits), b->getIntNTy(fw));
        this->storeLiteralMask(b, phiCurrentIndex, outputValue);

        phiCurrentIndex->addIncoming(b->CreateAdd(phiCurrentIndex, SIZE_1), b->GetInsertBlock());
        phiStartBits->addIncoming(INT_FW_0, b->GetInsertBlock());
        phiEndBits->addIncoming(INT_FW_0, b->GetInsertBlock());
        phiFullBits->addIncoming(INT_FW_0, b->GetInsertBlock());
        phiCarryBit->addIncoming(newCarryBit, b->GetInsertBlock());

        b->CreateBr(appendLiteralMaskCon);

        // ---- appendLiteralMaskExit
        b->SetInsertPoint(appendLiteralMaskExit);
        Value* finalStartBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, startBlockIndex), b->CreateOr(phiStartBits, b->CreateShl(INT_FW_1, startOffset)), phiStartBits);
        Value* finalEndBits = b->CreateSelect(b->CreateICmpEQ(phiCurrentIndex, endBlockIndex), b->CreateOr(phiEndBits, b->CreateShl(INT_FW_1, endOffset)), phiEndBits);
        b->setScalarField("pendingLiteralMarkerIndex", phiCurrentIndex);
        b->setScalarField("pendingLiteralMarkerStartBits", finalStartBits);
        b->setScalarField("pendingLiteralMarkerEndBits", finalEndBits);
        b->setScalarField("pendingLiteralMarkerFullBits", phiFullBits);
        b->setScalarField("pendingLiteralMarkerCarryBit", phiCarryBit);
    }

    void
    LZ4IndexBuilderNewKernel::storeLiteralMask(const std::unique_ptr<KernelBuilder> &b, llvm::Value *blockIndex,
                                               llvm::Value *value) {
        unsigned fw = 64;
        Value* m0BufferBlocks = b->CreateUDiv(b->getCapacity("deletionMarker"), b->getSize(fw));
        Value* indexRem = b->CreateURem(blockIndex, m0BufferBlocks);

        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("deletionMarker", b->getSize(0)), b->getIntNTy(fw)->getPointerTo());
        b->CreateStore(value, b->CreateGEP(outputBasePtr, indexRem));

    }

    void LZ4IndexBuilderNewKernel::storePendingLiteralMask(const std::unique_ptr<KernelBuilder> &b) {
        Value* outputValue = b->CreateSub(
                b->CreateSub(
                        b->getScalarField("pendingLiteralMarkerEndBits"),
                        b->getScalarField("pendingLiteralMarkerStartBits")
                ),
                b->getScalarField("pendingLiteralMarkerCarryBit")
        );
        this->storeLiteralMask(b, b->getScalarField("pendingLiteralMarkerIndex"), outputValue);
    }

    void LZ4IndexBuilderNewKernel::storePendingLiteralMasksUntilPos(const std::unique_ptr<KernelBuilder> &b,
                                                                    llvm::Value *targetGlobalPos) {
        unsigned fw = 64;
        Type* INT_FW_TY = b->getIntNTy(fw);
        Value* SIZE_1 = b->getSize(1);
        Value* SIZE_FW = b->getSize(fw);
        Value* INT_FW_0 = b->getIntN(fw, 0);


        Value* targetBlockIndex = b->CreateUDiv(targetGlobalPos, SIZE_FW);
//        Value* targetOffset = b->CreateZExt(b->CreateURem(targetPos, SIZE_FW), INT_FW_TY);

        BasicBlock* appendLiteralMaskCon = b->CreateBasicBlock("appendLiteralMaskCon2");
        BasicBlock* appendLiteralMaskBody = b->CreateBasicBlock("appendLiteralMaskBody2");
        BasicBlock* appendLiteralMaskExit = b->CreateBasicBlock("appendLiteralMaskExit2");

        Value* pendingLiteralMarkerIndex = b->getScalarField("pendingLiteralMarkerIndex");
        Value* pendingLiteralMarkerStartBits = b->getScalarField("pendingLiteralMarkerStartBits");
        Value* pendingLiteralMarkerEndBits = b->getScalarField("pendingLiteralMarkerEndBits");
        Value* pendingLiteralMarkerCarryBit = b->getScalarField("pendingLiteralMarkerCarryBit");
        Value* pendingLiteralMarkerFullBits = b->getScalarField("pendingLiteralMarkerFullBits");


        BasicBlock* entryBlock = b->GetInsertBlock();

        b->CreateBr(appendLiteralMaskCon);

        // ---- appendLiteralMaskCon
        b->SetInsertPoint(appendLiteralMaskCon);
        PHINode* phiCurrentIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentIndex->addIncoming(pendingLiteralMarkerIndex, entryBlock);
        PHINode* phiStartBits = b->CreatePHI(INT_FW_TY, 2);
        phiStartBits->addIncoming(pendingLiteralMarkerStartBits, entryBlock);
        PHINode* phiEndBits = b->CreatePHI(INT_FW_TY, 2);
        phiEndBits->addIncoming(pendingLiteralMarkerEndBits, entryBlock);
        PHINode* phiCarryBit = b->CreatePHI(INT_FW_TY, 2);
        phiCarryBit->addIncoming(pendingLiteralMarkerCarryBit, entryBlock);
        PHINode* phiFullBits = b->CreatePHI(INT_FW_TY, 2);
        phiFullBits->addIncoming(pendingLiteralMarkerFullBits, entryBlock);

        b->CreateUnlikelyCondBr(b->CreateICmpULT(phiCurrentIndex, targetBlockIndex), appendLiteralMaskBody, appendLiteralMaskExit);

        // ---- appendLiteralMaskBody
        b->SetInsertPoint(appendLiteralMaskBody);
        Value* outputValue = b->CreateOr(b->CreateSub(b->CreateSub(phiEndBits, phiStartBits), phiCarryBit), phiFullBits);
        Value* newCarryBit = b->CreateZExt(b->CreateICmpUGT(b->CreateAdd(phiStartBits, phiCarryBit), phiEndBits), b->getIntNTy(fw));
        this->storeLiteralMask(b, phiCurrentIndex, outputValue);

        phiCurrentIndex->addIncoming(b->CreateAdd(phiCurrentIndex, SIZE_1), b->GetInsertBlock());
        phiStartBits->addIncoming(INT_FW_0, b->GetInsertBlock());
        phiEndBits->addIncoming(INT_FW_0, b->GetInsertBlock());
        phiCarryBit->addIncoming(newCarryBit, b->GetInsertBlock());
        phiFullBits->addIncoming(INT_FW_0, b->GetInsertBlock());

        b->CreateBr(appendLiteralMaskCon);

        // ---- appendLiteralMaskExit
        b->SetInsertPoint(appendLiteralMaskExit);
        b->setScalarField("pendingLiteralMarkerIndex", phiCurrentIndex);
        b->setScalarField("pendingLiteralMarkerStartBits", phiStartBits);
        b->setScalarField("pendingLiteralMarkerEndBits", phiEndBits);
        b->setScalarField("pendingLiteralMarkerCarryBit", phiCarryBit);
        b->setScalarField("pendingLiteralMarkerFullBits", phiFullBits);

    }
}