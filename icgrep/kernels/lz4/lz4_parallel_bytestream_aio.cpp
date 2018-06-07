//
// Created by wxy325 on 2018/5/31.
//

#include "lz4_parallel_bytestream_aio.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

#define SIMD_WIDTH (64)


namespace kernel{

    LZ4ParallelByteStreamAioKernel::LZ4ParallelByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
            :SegmentOrientedKernel("LZ4ParallelByteStreamAioKernel",
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
                                           Binding{b->getIntNTy(SIMD_WIDTH), "outputPos"},

                                   }){
        this->setStride(4 * 1024 * 1024 * 4);
        addAttribute(MustExplicitlyTerminate());
    }

    Value* LZ4ParallelByteStreamAioKernel::generateSimdAcceleration(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPosVec,
                                  llvm::Value *blockEndVec) {
        /*
        // TODO incomplete

        // Constant
        Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256); // TODO find ret <4 * i32> version
//        Function *gatherFunc2 = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q);

        // ---- Entry Block
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* maskedTokenPosVec = b->CreateAnd(beginTokenPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(b->getCapacity("extender")))));
        Value* maskBlockIndexVec = b->CreateLShr(beginTokenPosVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, std::log2(64))));
        Value* maskBlockOffsetVec = b->CreateAnd(beginTokenPosVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 63)));
        Value* extenderValueVec = b->CreateCall(
                gatherFunc,
                {
                        UndefValue::get(b->getBitBlockType()),
                        b->CreatePointerCast(b->getRawInputPointer("extender", b->getSize(0)), b->getInt8PtrTy()),
                        b->CreateTrunc(maskBlockIndexVec, VectorType::get(b->getInt32Ty(), 4)),
                        Constant::getAllOnesValue(b->getBitBlockType()),
                        b->getInt8(8)
                }
        );

        Value* extenderValueVec2 = b->CreateCall(
                gatherFunc,
                {
                        UndefValue::get(VectorType::get(b->getInt8Ty(), 4)),
                        b->CreatePointerCast(b->getRawInputPointer("extender", b->getSize(0)), b->getInt8PtrTy()),
                        b->CreateTrunc(maskBlockIndexVec, VectorType::get(b->getInt32Ty(), 4)),
                        Constant::getAllOnesValue(b->getBitBlockType()),
                        b->getInt8(8)
                }
        );

        Value* initTokeMarkersVec = b->CreateShl(b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 1)), maskBlockOffsetVec);

//        b->CallPrintRegister("a", beginTokenPosVec);
//        b->CallPrintRegister("maskedTokenPosVec", maskedTokenPosVec);
//        b->CallPrintRegister("maskBlockIndexVec", maskBlockIndexVec);
//        b->CallPrintRegister("gather_result", extenderValueVec);


        BasicBlock* accelerationProcessBlock = b->CreateBasicBlock("accelerationProcessBlock");
        BasicBlock* accelerationExitBlock = b->CreateBasicBlock("accelerationExitBlock");
        b->CreateBr(accelerationProcessBlock);

        // ---- AccelerationProcessBlock
        b->SetInsertPoint(accelerationProcessBlock);
        PHINode* phiTokenMarkersVec = b->CreatePHI(initTokeMarkersVec->getType(), 2);
        phiTokenMarkersVec->addIncoming(initTokeMarkersVec, entryBlock);



*/
        return beginTokenPosVec;    //TODO

    }


    std::pair<Value *, Value *> LZ4ParallelByteStreamAioKernel::simdProcessBlockBoundary(
            const std::unique_ptr<KernelBuilder> &b, Value *beginTokenPosVec, Value *lz4BlockEndVec, Value* initOutputPosVec
    ) {
        Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256); // Maybe it will be better to use <4 * i32> version

        // Constant
        Value* BIT_BLOCK_0 = ConstantVector::getNullValue(b->getBitBlockType());
        Value* BIT_BLOCK_1 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0x1));
        Value* BIT_BLOCK_F0 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xf0));
        Value* BIT_BLOCK_0F = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0x0f));
        Value* BIT_BLOCK_FF = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xff));
        Value* BIT_BLOCK_FFFF = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xffff));
        Type* INT_BIT_BLOCK_TY = b->getIntNTy(b->getBitBlockWidth());
        Constant* INT_BIT_BLOCK_TY_0 = b->getIntN(b->getBitBlockWidth(), 0);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* notFinishBlocksVec = b->CreateICmpULT(beginTokenPosVec, lz4BlockEndVec);
        Value* notFinishBitBlock = b->CreateZExt(notFinishBlocksVec, b->getBitBlockType());
        Value* notFinishMask = b->CreateNeg(notFinishBitBlock);


        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* firstTokenPos = b->CreateExtractElement(beginTokenPosVec, (uint64_t)0);
        Value* bytePtrBase = b->CreateGEP(byteRawInputPtr, firstTokenPos);



        Value* tokenValuesVec = this->simdFetchByteData(b, byteRawInputPtr, beginTokenPosVec, notFinishMask);

        Value* shouldExtendLiteralVec = b->CreateICmpEQ(b->CreateAnd(BIT_BLOCK_F0, tokenValuesVec), BIT_BLOCK_F0);
        Value* shouldExtendLiteralBitBlockVec = b->CreateZExt(shouldExtendLiteralVec, b->getBitBlockType());
        Value* shouldExtendLiteral = b->CreateICmpNE(b->CreateBitCast(shouldExtendLiteralBitBlockVec, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        Value* shouldExtendMatchVec = b->CreateICmpEQ(b->CreateAnd(BIT_BLOCK_0F, tokenValuesVec), BIT_BLOCK_0F);
        Value* shouldExtendMatchBitBlockVec = b->CreateZExt(shouldExtendMatchVec, b->getBitBlockType());
        Value* shouldExtendMatch = b->CreateICmpNE(b->CreateBitCast(shouldExtendMatchBitBlockVec, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        Value* initExtendLiteralPos = b->CreateAdd(beginTokenPosVec, shouldExtendLiteralBitBlockVec);


        BasicBlock* extendLiteralCond = b->CreateBasicBlock("extendLiteralCond");
        BasicBlock* extendLiteralEnd = b->CreateBasicBlock("extendLiteralEnd");

        b->CreateCondBr(shouldExtendLiteral, extendLiteralCond, extendLiteralEnd);


        // ---- extendLiteralCond
        b->SetInsertPoint(extendLiteralCond);
        PHINode* phiCurrentExtendLiteralPosVec = b->CreatePHI(initExtendLiteralPos->getType(), 2);
        phiCurrentExtendLiteralPosVec->addIncoming(initExtendLiteralPos, entryBlock);

        PHINode* phiExtendLiteralLengthVec = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendLiteralLengthVec->addIncoming(BIT_BLOCK_0, entryBlock);

        PHINode* phiShouldExtendLiteralBitBlockVec = b->CreatePHI(shouldExtendLiteralBitBlockVec->getType(), 2);
        phiShouldExtendLiteralBitBlockVec->addIncoming(shouldExtendLiteralBitBlockVec, entryBlock);
        Value* shouldExtendLiteralGatherMask = b->CreateNeg(phiShouldExtendLiteralBitBlockVec);
        shouldExtendLiteralGatherMask = b->CreateAnd(shouldExtendLiteralGatherMask, notFinishMask);
//        b->CallPrintInt("a", b->getSize(0));
        // TODO maybe we can load i64 once and then consume 8 times
        Value* currentLiteralLengthVec = this->simdFetchByteData(b, byteRawInputPtr, phiCurrentExtendLiteralPosVec, shouldExtendLiteralGatherMask);

        Value* newExtendLiteralLengthVec = b->CreateAdd(phiExtendLiteralLengthVec, currentLiteralLengthVec);

        Value* shouldContinueExtendVec = b->CreateICmpEQ(currentLiteralLengthVec, BIT_BLOCK_FF);
        Value* shouldContinueExtendVecBitBlock = b->CreateZExt(shouldContinueExtendVec, b->getBitBlockType());

        Value* newExtendLiteralPosVec = b->CreateAdd(phiCurrentExtendLiteralPosVec, b->CreateAnd(shouldExtendLiteralBitBlockVec, shouldContinueExtendVecBitBlock));


        phiCurrentExtendLiteralPosVec->addIncoming(newExtendLiteralPosVec, b->GetInsertBlock());
        phiExtendLiteralLengthVec->addIncoming(newExtendLiteralLengthVec, b->GetInsertBlock());


        phiShouldExtendLiteralBitBlockVec->addIncoming(shouldContinueExtendVecBitBlock, b->GetInsertBlock());
        Value* shouldContinueExtendLiteral = b->CreateICmpNE(b->CreateBitCast(shouldContinueExtendVecBitBlock, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        b->CreateCondBr(shouldContinueExtendLiteral, extendLiteralCond, extendLiteralEnd);

        // ---- extendLiteralEnd
        b->SetInsertPoint(extendLiteralEnd);
        PHINode* literalExtendValueVec = b->CreatePHI(b->getBitBlockType(), 2);
        literalExtendValueVec->addIncoming(BIT_BLOCK_0, entryBlock);
        literalExtendValueVec->addIncoming(newExtendLiteralLengthVec, extendLiteralCond);

        PHINode* phiExtendLiteralEndPos = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendLiteralEndPos->addIncoming(beginTokenPosVec, entryBlock);
        phiExtendLiteralEndPos->addIncoming(phiCurrentExtendLiteralPosVec, extendLiteralCond);


        Value* literalLengthVec = b->simd_add(SIMD_WIDTH, literalExtendValueVec, b->simd_srlv(SIMD_WIDTH, tokenValuesVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4))));
//        Value* literalLengthVec = b->CreateAdd(literalExtendValueVec, b->CreateLShr(tokenValuesVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4))));

        Value* literalStartPosVec = b->CreateAdd(phiExtendLiteralEndPos, BIT_BLOCK_1);
        Value* literalEndPosVec = b->CreateAdd(literalStartPosVec, literalLengthVec);


        this->handleSimdLiteralCopy(b, literalStartPosVec, literalLengthVec, initOutputPosVec);
        Value* outputPosAfterLiteralCpy = b->CreateAdd(initOutputPosVec, literalLengthVec);


        Value* matchOffsetBeginPosVec = literalEndPosVec;

        Value* matchOffsetNextPosVec = b->CreateAdd(matchOffsetBeginPosVec, BIT_BLOCK_1);


        BasicBlock* hasMatchPartBlock = b->CreateBasicBlock("hasMatchPartBlock");
        BasicBlock* extendMatchCon = b->CreateBasicBlock("extendMatchCon");
        BasicBlock* extendMatchExit = b->CreateBasicBlock("extendMatchExit");


        BasicBlock* extendLiteralEndFinal = b->GetInsertBlock();

        Value* hasMatchPartVec = b->CreateICmpULT(matchOffsetBeginPosVec, lz4BlockEndVec);
        Value* hasMatchPartBitBlock = b->CreateZExt(hasMatchPartVec, b->getBitBlockType());
        Value* hasMatchPartMask = b->CreateNeg(hasMatchPartBitBlock);

        b->CreateLikelyCondBr(b->CreateICmpNE(b->CreateBitCast(hasMatchPartBitBlock, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0), hasMatchPartBlock, exitBlock);

        // ---- hasMatchPartBlock
        b->SetInsertPoint(hasMatchPartBlock);
        Value* initExtendMatchPosVec = b->CreateAdd(matchOffsetNextPosVec, shouldExtendMatchBitBlockVec);
//        b->CallPrintRegister("initExtendMatchPosVec", initExtendMatchPosVec);
        b->CreateCondBr(shouldExtendMatch, extendMatchCon, extendMatchExit);

        // ---- extendMatchCon
        b->SetInsertPoint(extendMatchCon);
        PHINode* phiCurrentExtendMatchPosVec = b->CreatePHI(initExtendMatchPosVec->getType(), 2);
        phiCurrentExtendMatchPosVec->addIncoming(initExtendMatchPosVec, hasMatchPartBlock);
        PHINode* phiExtendMatchLengthVec = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendMatchLengthVec->addIncoming(BIT_BLOCK_0, hasMatchPartBlock);

        PHINode* phiShouldExtendMatchBitBlockVec = b->CreatePHI(shouldExtendMatchBitBlockVec->getType(), 2);
        phiShouldExtendMatchBitBlockVec->addIncoming(shouldExtendMatchBitBlockVec, hasMatchPartBlock);
        Value* shouldExtendMatchGatherMask = b->CreateNeg(phiShouldExtendMatchBitBlockVec);
        shouldExtendMatchGatherMask = b->CreateAnd(shouldExtendMatchGatherMask, notFinishMask);
        // TODO maybe we can load i64 once and then consume 8 times

        Value* currentMatchLengthVec = this->simdFetchByteData(b, byteRawInputPtr, phiCurrentExtendMatchPosVec, shouldExtendMatchGatherMask);

        Value* newExtendMatchLengthVec = b->CreateAdd(phiExtendMatchLengthVec, currentMatchLengthVec);


        Value* shouldContinueExtendMatchVec = b->CreateICmpEQ(currentMatchLengthVec, BIT_BLOCK_FF);
        Value* shouldContinueExtendMatchVecBitBlock = b->CreateZExt(shouldContinueExtendMatchVec, b->getBitBlockType());

        Value* newExtendMatchPosVec = b->CreateAdd(phiCurrentExtendMatchPosVec, b->CreateAnd(shouldExtendMatchBitBlockVec, shouldContinueExtendMatchVecBitBlock));


        phiCurrentExtendMatchPosVec->addIncoming(newExtendMatchPosVec, b->GetInsertBlock());
        phiExtendMatchLengthVec->addIncoming(newExtendMatchLengthVec, b->GetInsertBlock());


        phiShouldExtendMatchBitBlockVec->addIncoming(shouldContinueExtendMatchVecBitBlock, b->GetInsertBlock());
        Value* shouldContinueExtendMatch = b->CreateICmpNE(b->CreateBitCast(shouldContinueExtendMatchVecBitBlock, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        b->CreateCondBr(shouldContinueExtendMatch, extendMatchCon, extendMatchExit);


        // ---- extendMatchExit
        b->SetInsertPoint(extendMatchExit);

        PHINode* matchExtendValueVec = b->CreatePHI(newExtendMatchLengthVec->getType(), 2);
        matchExtendValueVec->addIncoming(BIT_BLOCK_0, hasMatchPartBlock);
        matchExtendValueVec->addIncoming(newExtendMatchLengthVec, extendMatchCon);
        PHINode* phiExtendMatchEndPos = b->CreatePHI(matchOffsetNextPosVec->getType(), 2);
        phiExtendMatchEndPos->addIncoming(matchOffsetNextPosVec, hasMatchPartBlock);
        phiExtendMatchEndPos->addIncoming(phiCurrentExtendMatchPosVec, extendMatchCon);


        // matchLength = (size_t)token & 0xf + 4 + matchExtendValue
        Value* matchLength = b->CreateAdd(
                b->CreateAdd(matchExtendValueVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4))),
                b->CreateAnd(tokenValuesVec, BIT_BLOCK_0F)
        );
        matchLength = b->CreateAnd(matchLength, hasMatchPartMask);


        Value* matchOffsetVec =  b->CreateCall(
                gatherFunc,
                {
                        UndefValue::get(b->getBitBlockType()),
                        bytePtrBase,
                        b->CreateTrunc(b->CreateSub(matchOffsetBeginPosVec, b->simd_fill(SIMD_WIDTH, firstTokenPos)), VectorType::get(b->getInt32Ty(), 4)),
                        hasMatchPartMask,
                        b->getInt8(1)
                }
        );
        matchOffsetVec = b->CreateAnd(matchOffsetVec, notFinishMask);
        matchOffsetVec = b->CreateAnd(matchOffsetVec, BIT_BLOCK_FFFF);

//        Value* matchOffsetVec = this->simdFetchByteData(b, byteRawInputPtr, matchOffsetBeginPosVec, b->CreateAnd(hasMatchPartMask, notFinishMask));



        this->handleSimdMatchCopy(b, matchOffsetVec, matchLength, outputPosAfterLiteralCpy);

        Value* outputPosAfterMatchCpy = b->CreateAdd(outputPosAfterLiteralCpy, matchLength);

        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();

        b->CreateBr(exitBlock);
        // ---- exitBlock

        b->SetInsertPoint(exitBlock);

        PHINode* phiBeforeTokenPos = b->CreatePHI(matchOffsetNextPosVec->getType(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPosVec, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(phiExtendMatchEndPos, extendMatchExitFinal);

        PHINode* phiNewOutputPos = b->CreatePHI(outputPosAfterLiteralCpy->getType(), 2);
        phiNewOutputPos->addIncoming(outputPosAfterLiteralCpy, extendLiteralEndFinal);
        phiNewOutputPos->addIncoming(outputPosAfterMatchCpy, extendMatchExitFinal);
//        b->CallPrintRegister("phiBeforeTokenPos", phiBeforeTokenPos);
        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, BIT_BLOCK_1);
//        b->CallPrintRegister("nextTokenPos", nextTokenPos);
        return std::make_pair(nextTokenPos, phiNewOutputPos);
    }

    void LZ4ParallelByteStreamAioKernel::generateSimdDecompression(const std::unique_ptr<KernelBuilder> &b, Value* blockDataIndex) {
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("simdDecompressionExitBlock");
        BasicBlock* processCon = b->CreateBasicBlock("simdDecompressionProcessConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("simdDecompressionProcessBodyBlock");

        // ---- entryBlock
        Value* blockStartVec = this->generateLoadSimdInt64NumberInput(b, "blockStart", blockDataIndex);
        Value* blockEndVec = this->generateLoadSimdInt64NumberInput(b, "blockEnd", blockDataIndex);

        Value* outputPos = b->getProducedItemCount("outputStream");
        Value* initOutputPosVec = b->simd_fill(SIMD_WIDTH, outputPos);
        initOutputPosVec = b->CreateAdd(
                initOutputPosVec,
                ConstantVector::get({
                                            b->getIntN(SIMD_WIDTH, 0),
                                            b->getIntN(SIMD_WIDTH, 1 * 4 * 1024 * 1024),
                                            b->getIntN(SIMD_WIDTH, 2 * 4 * 1024 * 1024),
                                            b->getIntN(SIMD_WIDTH, 3 * 4 * 1024 * 1024)
                                    }));

        // TODO handle uncompression blocks

        b->CreateBr(processCon);

        // ---- processCon
        b->SetInsertPoint(processCon);
        PHINode* phiCursorVec = b->CreatePHI(blockStartVec->getType(), 2);
        phiCursorVec->addIncoming(blockStartVec, entryBlock);

        PHINode* phiOutputPosVec = b->CreatePHI(initOutputPosVec->getType(), 2);
        phiOutputPosVec->addIncoming(initOutputPosVec, entryBlock);


        Value* hasRemaining = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        hasRemaining = b->CreateICmpNE(b->CreateBitCast(hasRemaining, b->getIntNTy(256)), Constant::getNullValue(b->getIntNTy(256)));

        b->CreateCondBr(hasRemaining, processBody, exitBlock);

        // ---- processBody
        b->SetInsertPoint(processBody);
//        Value* newCursorVec = this->generateSimdAcceleration(b, phiCursorVec, blockEndVec);
        auto ret = this->simdProcessBlockBoundary(b, phiCursorVec, blockEndVec, phiOutputPosVec);;
        Value* newCursorVec = ret.first;
        Value* newOutputPosVec = ret.second;
//        b->CallPrintInt("newOutputPosVec", b->CreateExtractElement(newOutputPosVec, (uint64_t)0));

        phiCursorVec->addIncoming(newCursorVec, b->GetInsertBlock());
        phiOutputPosVec->addIncoming(newOutputPosVec, b->GetInsertBlock());

        b->CreateBr(processCon);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);

        uint64_t lastVecIndex = b->getBitBlockWidth() / SIMD_WIDTH - 1;
        Value* lastBlockEnd = b->CreateExtractElement(blockEndVec, lastVecIndex);
        b->setProcessedItemCount("byteStream", lastBlockEnd);

        Value* lastOutputPos = b->CreateExtractElement(phiOutputPosVec, lastVecIndex);
        b->setProducedItemCount("outputStream", lastOutputPos);
//        b->CallPrintRegister("phiOutputPosVec", phiOutputPosVec);

    }

    void LZ4ParallelByteStreamAioKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        // Constant
        Value* SIZE_1 = b->getSize(1);

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value * blockDataIndex = b->getProcessedItemCount("isCompressed");
        Value * totalNumber = b->getAvailableItemCount("isCompressed");
//        b->CallPrintInt("totalNumber", totalNumber);
        Value * fileSize = b->getScalarField("fileSize");
        Value * availableNumber = b->CreateSub(totalNumber, blockDataIndex);
        Value * lastBlockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", b->CreateSub(totalNumber, SIZE_1));
        Value * isTerminal = b->CreateAnd(b->CreateICmpEQ(fileSize, lastBlockEnd), b->CreateICmpULE(availableNumber, b->getSize(b->getBitBlockWidth() / SIMD_WIDTH)));
        b->setTerminationSignal(isTerminal);

        BasicBlock* simdProcessEntryBlock = b->CreateBasicBlock("simdProcessEntryBlock");
        BasicBlock* notEnoughSimdDataBlock = b->CreateBasicBlock("notEnoughSimdDataBlock");

        b->CreateLikelyCondBr(b->CreateICmpUGE(availableNumber, b->getSize(b->getBitBlockWidth() / SIMD_WIDTH)), simdProcessEntryBlock, notEnoughSimdDataBlock);

        // ---- simdProcessEntryBlock
        b->SetInsertPoint(simdProcessEntryBlock);
        this->generateSimdDecompression(b, blockDataIndex);
        Value* newProcessedDataIndex = b->CreateAdd(blockDataIndex, b->getSize(b->getBitBlockWidth() / SIMD_WIDTH));
        b->setProcessedItemCount("isCompressed", newProcessedDataIndex);

        b->CreateBr(exitBlock);

        // ---- notEnoughSimdDataBlock
        b->SetInsertPoint(notEnoughSimdDataBlock);

        BasicBlock* notSimdProcessBlock = b->CreateBasicBlock("notSimdProcessBlock");
        b->CreateUnlikelyCondBr(isTerminal, notSimdProcessBlock, exitBlock);

        // ---- notSimdProcessBlock
        b->SetInsertPoint(notSimdProcessBlock);
        // Use loop to process the remaining block in sequential approach (the number of the remaining block should be less than (b->getBitBlockWidth() / SIMD_WIDTH))
        this->generateSequentialDecompression(b, blockDataIndex, totalNumber);
        b->CreateBr(exitBlock);


        // ---- ExitBlock
        b->SetInsertPoint(exitBlock);
    }

    void LZ4ParallelByteStreamAioKernel::generateSequentialDecompression(const std::unique_ptr<KernelBuilder> &b, llvm::Value* startBlockDataIndex, llvm::Value* endBlockDataIndex) {

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* SequentialConBlock = b->CreateBasicBlock("SequentialConBlock");
        BasicBlock* SequentialBodyBlock = b->CreateBasicBlock("SequentialBodyBlock");
        BasicBlock* SequentialExitBlock = b->CreateBasicBlock("SequentialExitBlock");


        Value* initOutputPos = b->getProducedItemCount("outputStream");

        b->CreateBr(SequentialConBlock);

        // ---- SequentialConBlock
        b->SetInsertPoint(SequentialConBlock);
        PHINode* phiCurrentBlockDataIndex = b->CreatePHI(b->getSizeTy(), 2);
        phiCurrentBlockDataIndex->addIncoming(startBlockDataIndex, entryBlock);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(initOutputPos, entryBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCurrentBlockDataIndex, endBlockDataIndex), SequentialBodyBlock, SequentialExitBlock);

        // ---- SequentialBodyBlock
        b->SetInsertPoint(SequentialBodyBlock);
        Value* lz4BlockStart = this->generateLoadInt64NumberInput(b, "blockStart", phiCurrentBlockDataIndex);
        Value* lz4BlockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", phiCurrentBlockDataIndex);
        Value* newOutputPos = this->generateProcessCompressedBlock(b, lz4BlockStart, lz4BlockEnd, phiOutputPos);

        b->setProcessedItemCount("byteStream", lz4BlockEnd);
        b->setProducedItemCount("outputStream", newOutputPos);

        phiCurrentBlockDataIndex->addIncoming(b->CreateAdd(phiCurrentBlockDataIndex, b->getSize(1)), b->GetInsertBlock());
        phiOutputPos->addIncoming(newOutputPos, b->GetInsertBlock());

        b->CreateBr(SequentialConBlock);

        // ---- SequentialExitBlock
        b->SetInsertPoint(SequentialExitBlock);
        b->setProcessedItemCount("isCompressed", endBlockDataIndex);

    }

    llvm::Value *
    LZ4ParallelByteStreamAioKernel::generateLoadSimdInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder, std::string inputBufferName,
                                     llvm::Value *globalOffset) {
        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        valuePtr = iBuilder->CreatePointerCast(valuePtr, iBuilder->getBitBlockType()->getPointerTo());
        return iBuilder->CreateLoad(valuePtr);
    }

    llvm::Value *LZ4ParallelByteStreamAioKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset) {

        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    llvm::Value*
    LZ4ParallelByteStreamAioKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value *lz4BlockStart,
                                                           llvm::Value *lz4BlockEnd, llvm::Value* initOutputPos) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");


        BasicBlock* beforeProcessConBlock = b->GetInsertBlock();
        b->CreateBr(processCon);
        b->SetInsertPoint(processCon);
        PHINode* phiOutputPos = b->CreatePHI(initOutputPos->getType(), 2);
        phiOutputPos->addIncoming(initOutputPos, entryBlock);
        PHINode* phiCursorValue = b->CreatePHI(b->getInt64Ty(), 2); // phiCursorValue should always be the position of next token except for the final sequence
        phiCursorValue->addIncoming(lz4BlockStart, beforeProcessConBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCursorValue, lz4BlockEnd), processBody, exitBlock);

        b->SetInsertPoint(processBody);


        auto ret = this->processBlockBoundary(b, phiCursorValue, lz4BlockEnd, phiOutputPos);
        Value* nextTokenGlobalPos = ret.first;
        Value* nextOutputPos = ret.second;

        phiOutputPos->addIncoming(nextOutputPos, b->GetInsertBlock());
        phiCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
        return phiOutputPos;
    }


    std::pair<llvm::Value *, llvm::Value *> LZ4ParallelByteStreamAioKernel::processBlockBoundary(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                                                              llvm::Value *lz4BlockEnd, llvm::Value* initOutputPos) {
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
        this->handleLiteralCopy(b, literalStartPos, literalLength, initOutputPos);

        Value* outputPosAfterLiteralCpy = b->CreateAdd(literalLength, initOutputPos);

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
        this->handleMatchCopy(b, matchOffset, matchLength, outputPosAfterLiteralCpy);
        Value* outputPosAfterMatchCpy = b->CreateAdd(outputPosAfterLiteralCpy, matchLength);
        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiBeforeTokenPos = b->CreatePHI(b->getSizeTy(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPos, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(phiExtendMatchEndPos, extendMatchExitFinal);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(outputPosAfterLiteralCpy, extendLiteralEndFinal);
        phiOutputPos->addIncoming(outputPosAfterMatchCpy, extendMatchExitFinal);

        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, SIZE_1);

        return std::make_pair(nextTokenPos, phiOutputPos);
    }


    void LZ4ParallelByteStreamAioKernel::handleSimdMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec) {
        // TODO use memcpy first
//        Value* l = b->CreateExtractElement(matchLengthVec, (uint64_t)0);
//        Value* shouldPrint = b->CreateICmpNE(l, b->getSize(0));
//        b->CallPrintIntCond("matchOffset", b->CreateExtractElement(matchOffsetVec, (uint64_t)0), shouldPrint);
//        b->CallPrintIntCond("matchLength", l, shouldPrint);
        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());

        for (uint64_t i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            BasicBlock* matchCopyConBlock = b->CreateBasicBlock("matchCopyConBlock" + std::to_string(i));
            BasicBlock* matchCopyBodyBlock = b->CreateBasicBlock("matchCopyBodyBlock" + std::to_string(i));
            BasicBlock* matchCopyExitBlock = b->CreateBasicBlock("matchCopyExitBlock" + std::to_string(i));

            BasicBlock* beforeConBlock = b->GetInsertBlock();

            Value* matchOffset = b->CreateExtractElement(matchOffsetVec, i);
            Value* initMatchLength = b->CreateExtractElement(matchLengthVec, i);
            Value* initOutputPos = b->CreateExtractElement(outputPosVec, i);
//            b->CreateLikelyCondBr(b->CreateICmpNE(matchOffset, b->getSize(0)), matchCopyConBlock, matchCopyExitBlock);
            b->CreateBr(matchCopyConBlock);

            // ---- matchCopyConBlock
            b->SetInsertPoint(matchCopyConBlock);
            PHINode* phiMatchLength = b->CreatePHI(initMatchLength->getType(), 2);
            phiMatchLength->addIncoming(initMatchLength, beforeConBlock);
            PHINode* phiOutputPos = b->CreatePHI(initOutputPos->getType(), 2);
            phiOutputPos->addIncoming(initOutputPos, beforeConBlock);
//            b->CallPrintInt("phiMatchLength", phiMatchLength);
//            b->CallPrintInt("matchOffset", matchOffset);

            b->CreateCondBr(b->CreateICmpUGT(phiMatchLength, b->getSize(0)), matchCopyBodyBlock, matchCopyExitBlock);

            // ---- matchCopyBodyBlock
            b->SetInsertPoint(matchCopyBodyBlock);
            Value* copySize = b->CreateUMin(phiMatchLength, matchOffset);
            Value* copyFromPos = b->CreateSub(phiOutputPos, matchOffset);

            b->CreateMemCpy(
                    b->CreateGEP(outputBasePtr, b->CreateURem(phiOutputPos, outputCapacity)),
                    b->CreateGEP(outputBasePtr, b->CreateURem(copyFromPos, outputCapacity)),
                    copySize,
                    1
            );

            phiMatchLength->addIncoming(b->CreateSub(phiMatchLength, copySize), b->GetInsertBlock());
            phiOutputPos->addIncoming(b->CreateAdd(phiOutputPos, copySize), b->GetInsertBlock());

            b->CreateBr(matchCopyConBlock);

            // ---- matchCopyExitBlock
            b->SetInsertPoint(matchCopyExitBlock);
        }
    }

    void LZ4ParallelByteStreamAioKernel::handleSimdLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStartVec, llvm::Value* literalLengthVec, llvm::Value* outputPosVec) {
        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputPosRemVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->simd_not(b->CreateNeg(outputCapacity))));
        // TODO use memcpy first

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());

        for (uint64_t i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            Value* literalStart = b->CreateExtractElement(literalStartVec, i);
            Value* literalLength = b->CreateExtractElement(literalLengthVec, i);
            Value* outputPosRem = b->CreateExtractElement(outputPosRemVec, i);;
            b->CreateMemCpy(
                    b->CreateGEP(outputBasePtr, outputPosRem),
                    b->CreateGEP(inputBasePtr, literalStart),
                    literalLength,
                    1
            );
        }

    }

    void LZ4ParallelByteStreamAioKernel::handleLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                                   llvm::Value *literalLength, llvm::Value* outputPos) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        Value* inputBytePtr = b->getRawInputPointer("byteStream", literalStart);
        Value* inputPtr = b->CreatePointerCast(inputBytePtr, INT_FW_PTR);

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
        b->CreateStore(b->CreateLoad(phiInputPtr), phiOutputPtr);

        phiInputPtr->addIncoming(b->CreateGEP(phiInputPtr, b->getSize(1)), b->GetInsertBlock());
        phiOutputPtr->addIncoming(b->CreateGEP(phiOutputPtr, b->getSize(1)), b->GetInsertBlock());
        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, b->getSize(fw / 8)), b->GetInsertBlock());
        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }

    void LZ4ParallelByteStreamAioKernel::handleMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                                 llvm::Value *matchLength, llvm::Value* outputPos) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        BasicBlock* entryBlock = b->GetInsertBlock();

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

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchByteData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask) {
        return this->simdFetchByteDataByGather(b, basePtr, offsetVec, mask);
//        return this->simdFetchByteDataByLoop(b, basePtr, offsetVec, mask);
    }

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchByteDataByGather(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask) {
        Value* BIT_BLOCK_FF = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xff));
//        Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256);
        Function *gatherFunc2 = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_d);

        Value* firstOffset = b->CreateExtractElement(offsetVec, (uint64_t)0);

        Type* i32BitBlockTy = VectorType::get(b->getInt32Ty(), 4);

//        Value* tokenValuesVec =  b->CreateCall(
//                gatherFunc,
//                {
//                        UndefValue::get(b->getBitBlockType()),
//                        b->CreateGEP(basePtr, firstOffset),
//                        b->CreateTrunc(b->CreateSub(offsetVec, b->simd_fill(SIMD_WIDTH, firstOffset)), VectorType::get(b->getInt32Ty(), 4)),
//                        mask,
//                        b->getInt8(1)
//                }
//        );

        ////
        Value* tokenValuesVec =  b->CreateCall(
                gatherFunc2,
                {
                        UndefValue::get(i32BitBlockTy),
                        b->CreateGEP(basePtr, firstOffset),
                        b->CreateTrunc(b->CreateSub(offsetVec, b->simd_fill(SIMD_WIDTH, firstOffset)), VectorType::get(b->getInt32Ty(), 4)),
                        b->CreateTrunc(mask, i32BitBlockTy),
                        b->getInt8(1)
                }
        );
        tokenValuesVec = b->CreateZExt(tokenValuesVec, b->getBitBlockType());
        /////
        
        tokenValuesVec = b->CreateAnd(tokenValuesVec, mask);
        tokenValuesVec = b->CreateAnd(tokenValuesVec, BIT_BLOCK_FF);
        return tokenValuesVec;
    }

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchByteDataByLoop(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* maskVec) {
        Value* retVec = ConstantVector::getNullValue(b->getBitBlockType());

        for (uint64_t i = 0; i < 4; i++){ //TODO 4 here is a hardcode for AVX2, it may need to be changed to (BitBlockWidth / 64)
            Value* mask = b->CreateExtractElement(maskVec, i);
            Value* shouldLoad = b->CreateICmpNE(mask, b->getInt64(0));
            Value* loadPtr = b->CreateSelect(shouldLoad, b->CreateGEP(basePtr, b->CreateExtractElement(offsetVec, i)), basePtr);
            Value* loadValue = b->CreateZExt(b->CreateLoad(loadPtr), b->getInt64Ty());

            Value* finalValue = b->CreateSelect(shouldLoad, loadValue, b->getInt64(0));
            retVec = b->CreateInsertElement(retVec, finalValue, i);
        }

        return retVec;
    }

}