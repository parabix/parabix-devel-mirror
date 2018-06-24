
#include "lz4_parallel_bytestream_aio.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>
#include <IR_Gen/idisa_target.h>

using namespace llvm;
using namespace kernel;
using namespace std;


#define SIMD_WIDTH (64)


namespace kernel{

    LZ4ParallelByteStreamAioKernel::LZ4ParallelByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned lz4BlockSize, bool enableGather, bool enableScatter, int minParallelLevel)
            :SegmentOrientedKernel("LZ4ParallelByteStreamAioKernel",
            // Inputs
                                   {
                    Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},

                    // block data
                    Binding{b->getStreamSetTy(1, 8), "isCompressed", BoundedRate(0, 1), AlwaysConsume()},
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
                                           Binding{b->getBitBlockType(), "placeholder"},

                                   }),
             mLz4BlockSize(lz4BlockSize),
             mEnableGather(enableGather),
             mEnableScatter(enableScatter),
             mMininumParallelLevel(minParallelLevel)
    {
        this->setStride(mLz4BlockSize * b->getBitBlockWidth() / SIMD_WIDTH);
        addAttribute(MustExplicitlyTerminate());
//        this->initParallelLevelMeasurement(b);
    }

    // Kernel Methods
    void LZ4ParallelByteStreamAioKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        this->initializeConstantsAndTypes(b);

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
        this->generateSimdDecompression2(b, blockDataIndex);
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

        BasicBlock* printBlock = b->CreateBasicBlock("printBlock");
        BasicBlock* realExitBlock = b->CreateBasicBlock("realExitBlock");
        b->CreateCondBr(b->getTerminationSignal(), printBlock, realExitBlock);

        b->SetInsertPoint(printBlock);
//        this->printParallelLevelResult(b);
//        b->CallPrintInt("tempTimes", b->getScalarField("tempTimes"));
        b->CreateBr(realExitBlock);

        b->SetInsertPoint(realExitBlock);
    }

    // ---- SIMD Processing


    // ---- Sequential Processing






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

    void LZ4ParallelByteStreamAioKernel::initializeConstantsAndTypes(const std::unique_ptr<KernelBuilder> &b) {
        // Constants
        BIT_BLOCK_0 = ConstantVector::getNullValue(b->getBitBlockType());
        BIT_BLOCK_1 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0x1));
        BIT_BLOCK_F0 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xf0));
        BIT_BLOCK_0F = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0x0f));
        BIT_BLOCK_FF = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xff));
        BIT_BLOCK_FFFF = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xffff));
        INT_BIT_BLOCK_TY_0 = b->getIntN(b->getBitBlockWidth(), 0);
        SIZE_0 = b->getSize(0);
        SIZE_1 = b->getSize(1);
        BYTE_FF = b->getInt8(0xff);
        BYTE_F0 = b->getInt8(0xf0);
        BYTE_0F = b->getInt8(0x0f);


        // Types
        INT_BIT_BLOCK_TY = b->getIntNTy(b->getBitBlockWidth());
        BIT_BLOCK_TY = b->getBitBlockType();

    }

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::parallelParseLiteralInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *tokenPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {

        Value* shouldExtendLiteralBitBlockVec = b->simd_eq(SIMD_WIDTH, b->simd_and(BIT_BLOCK_F0, tokenValuesVec), BIT_BLOCK_F0);
        Value* extendLiteralParallelLevel = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, shouldExtendLiteralBitBlockVec));
        BasicBlock* simdBlock = b->CreateBasicBlock("simdBlock");
        BasicBlock* loopBlock = b->CreateBasicBlock("loopBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateUnlikelyCondBr(b->CreateICmpUGE(extendLiteralParallelLevel, b->getInt32(mMininumParallelLevel)), simdBlock, loopBlock);

        b->SetInsertPoint(simdBlock);
        auto ret1 = this->simdParseLiteralInfo(b, tokenPosVec, tokenValuesVec, notFinishMask);
        BasicBlock* simdBlockFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        b->SetInsertPoint(loopBlock);
        auto ret2 = this->parseMultipleLiteralInfoByLoop(b, tokenPosVec, tokenValuesVec, notFinishMask);
        BasicBlock* loopBlockFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        PHINode* phiLiteralStartVec = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiLiteralStartVec->addIncoming(ret1.first, simdBlockFinal);
        phiLiteralStartVec->addIncoming(ret2.first, loopBlockFinal);

        PHINode* phiLiteralLengthVec = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiLiteralLengthVec->addIncoming(ret1.second, simdBlockFinal);
        phiLiteralLengthVec->addIncoming(ret2.second, loopBlockFinal);

        return std::make_pair(phiLiteralStartVec, phiLiteralLengthVec);
    };

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::parseMultipleLiteralInfoByLoop(const std::unique_ptr<KernelBuilder> &b, llvm::Value *tokenPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* initLiteralLengthVec = b->simd_srlv(SIMD_WIDTH, tokenValuesVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4)));
        Value* initLiteralStartVec = b->simd_add(SIMD_WIDTH, tokenPosVec, BIT_BLOCK_1);

        Value* shouldExtendLiteralBitBlockVec = b->simd_and(b->simd_eq(SIMD_WIDTH, initLiteralLengthVec, BIT_BLOCK_0F), notFinishMask);
        Value* extendLiteralSignMask = b->hsimd_signmask(SIMD_WIDTH, shouldExtendLiteralBitBlockVec);

        BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateBr(conBlock);

        // ---- ConBlock
        b->SetInsertPoint(conBlock);
        PHINode* phiSignMask = b->CreatePHI(extendLiteralSignMask->getType(), 2);
        phiSignMask->addIncoming(extendLiteralSignMask, entryBlock);
        PHINode* phiLiteralLengthVec = b->CreatePHI(initLiteralLengthVec->getType(), 2);
        phiLiteralLengthVec->addIncoming(initLiteralLengthVec, entryBlock);
        PHINode* phiLiteralStartVec = b->CreatePHI(initLiteralStartVec->getType(), 2);
        phiLiteralStartVec->addIncoming(initLiteralStartVec, entryBlock);

//        b->CallPrintInt("phiSignMask", phiSignMask);
        b->CreateCondBr(b->CreateICmpNE(phiSignMask, b->getInt32(0)), bodyBlock, exitBlock);

        // ---- BodyBlock
        b->SetInsertPoint(bodyBlock);
        Value* targetIndex = b->CreateCountForwardZeroes(phiSignMask);
        Value* targetTokenPos = b->CreateExtractElement(tokenPosVec, targetIndex);
        Value* targetTokenValue = b->CreateTrunc(b->CreateExtractElement(tokenValuesVec, targetIndex), b->getInt8Ty());

        Value *newLiteralStart = nullptr, *newLiteralLength = nullptr;
        std::tie(newLiteralStart, newLiteralLength) = this->parseLiteralInfo(b, targetTokenPos, targetTokenValue);
        phiLiteralLengthVec->addIncoming(b->CreateInsertElement(phiLiteralLengthVec, newLiteralLength, targetIndex), b->GetInsertBlock());
        phiLiteralStartVec->addIncoming(b->CreateInsertElement(phiLiteralStartVec, newLiteralStart, targetIndex), b->GetInsertBlock());
        phiSignMask->addIncoming(b->CreateSub(phiSignMask, b->CreateShl(b->getInt32(1), targetIndex)), b->GetInsertBlock());
        b->CreateBr(conBlock);

        // ---- ExitBlock
        b->SetInsertPoint(exitBlock);

        return std::make_pair(phiLiteralStartVec, b->simd_and(phiLiteralLengthVec, notFinishMask));
    };

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::simdParseLiteralInfo(const std::unique_ptr<KernelBuilder> &b, Value *tokenPosVec, Value* tokenValuesVec, Value* notFinishMask) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());


        Value* shouldExtendLiteralBitBlockVec = b->CreateZExt(b->CreateICmpEQ(b->CreateAnd(BIT_BLOCK_F0, tokenValuesVec), BIT_BLOCK_F0), b->getBitBlockType());
        Value* shouldExtendLiteral = b->CreateICmpNE(b->CreateBitCast(shouldExtendLiteralBitBlockVec, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        Value* initExtendLiteralPos = b->simd_add(SIMD_WIDTH, tokenPosVec, shouldExtendLiteralBitBlockVec);


        BasicBlock* extendLiteralCond = b->CreateBasicBlock("extendLiteralCond");
        BasicBlock* extendLiteralEnd = b->CreateBasicBlock("extendLiteralEnd");

        b->CreateUnlikelyCondBr(shouldExtendLiteral, extendLiteralCond, extendLiteralEnd); //extend literal will not happen quite often

        // ---- extendLiteralCond
        b->SetInsertPoint(extendLiteralCond);
        PHINode* phiCurrentExtendLiteralPosVec = b->CreatePHI(initExtendLiteralPos->getType(), 2);
        phiCurrentExtendLiteralPosVec->addIncoming(initExtendLiteralPos, entryBlock);

        PHINode* phiExtendLiteralLengthVec = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiExtendLiteralLengthVec->addIncoming(BIT_BLOCK_0, entryBlock);

        PHINode* phiShouldExtendLiteralBitBlockVec = b->CreatePHI(shouldExtendLiteralBitBlockVec->getType(), 2);
        phiShouldExtendLiteralBitBlockVec->addIncoming(shouldExtendLiteralBitBlockVec, entryBlock);
        Value* shouldExtendLiteralGatherMask = b->CreateNeg(phiShouldExtendLiteralBitBlockVec);
        shouldExtendLiteralGatherMask = b->simd_and(shouldExtendLiteralGatherMask, notFinishMask);

        // TODO maybe we can load i64 once and then consume 8 times
//        b->CallPrintRegister("shouldExtendLiteralGatherMask", shouldExtendLiteralGatherMask);
//        b->CallPrintInt("popcount", b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, shouldExtendLiteralGatherMask)));

        Value* currentLiteralLengthVec = this->simdFetchByteData(b, byteRawInputPtr, phiCurrentExtendLiteralPosVec, shouldExtendLiteralGatherMask);

        Value* newExtendLiteralLengthVec = b->simd_add(SIMD_WIDTH, phiExtendLiteralLengthVec, currentLiteralLengthVec);
        Value* shouldContinueExtendVecBitBlock = b->CreateZExt(b->CreateICmpEQ(currentLiteralLengthVec, BIT_BLOCK_FF), BIT_BLOCK_TY);
        Value* newExtendLiteralPosVec = b->CreateAdd(phiCurrentExtendLiteralPosVec, b->CreateAnd(shouldExtendLiteralBitBlockVec, shouldContinueExtendVecBitBlock));

        phiCurrentExtendLiteralPosVec->addIncoming(newExtendLiteralPosVec, b->GetInsertBlock());
        phiExtendLiteralLengthVec->addIncoming(newExtendLiteralLengthVec, b->GetInsertBlock());
        phiShouldExtendLiteralBitBlockVec->addIncoming(shouldContinueExtendVecBitBlock, b->GetInsertBlock());

        Value* shouldContinueExtendLiteral = b->CreateICmpNE(b->CreateBitCast(shouldContinueExtendVecBitBlock, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        b->CreateUnlikelyCondBr(shouldContinueExtendLiteral, extendLiteralCond, extendLiteralEnd);  //extend literal will not happen quite often

        // ---- extendLiteralEnd
        b->SetInsertPoint(extendLiteralEnd);
        PHINode* literalExtendValueVec = b->CreatePHI(b->getBitBlockType(), 2);
        literalExtendValueVec->addIncoming(BIT_BLOCK_0, entryBlock);
        literalExtendValueVec->addIncoming(newExtendLiteralLengthVec, extendLiteralCond);

        PHINode* phiExtendLiteralEndPos = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendLiteralEndPos->addIncoming(tokenPosVec, entryBlock);
        phiExtendLiteralEndPos->addIncoming(phiCurrentExtendLiteralPosVec, extendLiteralCond);


        Value* literalLengthVec = b->simd_add(SIMD_WIDTH, literalExtendValueVec, b->simd_srlv(SIMD_WIDTH, tokenValuesVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4))));
        Value* literalStartPosVec = b->simd_add(SIMD_WIDTH, phiExtendLiteralEndPos, BIT_BLOCK_1);


        return std::make_pair(literalStartPosVec, literalLengthVec);

    }

    // TODO for now, this method only handle fast step (extension length <= 8)
    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::simdParseLiteralInfo2(const std::unique_ptr<KernelBuilder> &b, llvm::Value *tokenPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {
        // ---- entryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* baseLiteralLength = b->simd_srlv(SIMD_WIDTH, tokenValuesVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4)));

        Value* shouldExtendLiteralGatherMask = b->simd_eq(SIMD_WIDTH, baseLiteralLength, BIT_BLOCK_0F);
        shouldExtendLiteralGatherMask = b->simd_and(shouldExtendLiteralGatherMask, notFinishMask);
        Value* shouldExtension = b->CreateICmpNE(b->hsimd_signmask(SIMD_WIDTH, shouldExtendLiteralGatherMask), b->getInt32(0));

        BasicBlock* extensionBlock = b->CreateBasicBlock("extensionBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* notExtensionLiteralStart = b->simd_add(SIMD_WIDTH, tokenPosVec, BIT_BLOCK_1);

        b->CreateUnlikelyCondBr(shouldExtension, extensionBlock, exitBlock);

        // ---- extensionBlock
        b->SetInsertPoint(extensionBlock);
        Value* shouldExtendLiteralBitBlockVec = b->CreateZExt(b->CreateICmpEQ(baseLiteralLength, BIT_BLOCK_0F), BIT_BLOCK_TY);

        Value* initExtendLiteralPos = b->simd_add(SIMD_WIDTH, tokenPosVec, shouldExtendLiteralBitBlockVec);
        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", SIZE_0), b->getInt8PtrTy());

        Value* i64ExtensionValue = this->simdFetchI64DataByGather(b, byteRawInputPtr, initExtendLiteralPos, shouldExtendLiteralGatherMask);

        Value* forwardZeros = b->simd_cttz(SIMD_WIDTH, b->simd_not(i64ExtensionValue));
        Value* BIT_BLOCK_LOG2_8 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, std::log2(8)));
        Value* extensionLength = b->simd_srlv(SIMD_WIDTH, forwardZeros, BIT_BLOCK_LOG2_8);
        Value* shiftedExtensionValue = b->simd_srlv(SIMD_WIDTH, i64ExtensionValue, b->simd_sllv(SIMD_WIDTH, extensionLength, BIT_BLOCK_LOG2_8));
        shiftedExtensionValue = b->simd_and(shiftedExtensionValue, BIT_BLOCK_FF);
        Value* i64ExtensionLength = b->simd_add(SIMD_WIDTH, b->simd_mult(SIMD_WIDTH, extensionLength, BIT_BLOCK_FF), shiftedExtensionValue);
        Value* newLiteralLength = b->simd_add(SIMD_WIDTH, b->simd_and(i64ExtensionLength, shouldExtendLiteralGatherMask), baseLiteralLength);

        Value* newLiteralStartPos = b->simd_add(
                SIMD_WIDTH,
                b->simd_add(SIMD_WIDTH, notExtensionLiteralStart, shouldExtendLiteralBitBlockVec),
                b->simd_and(extensionLength, shouldExtendLiteralGatherMask)
        );

        BasicBlock* extensionFinalBlock = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiLiteralStartPos = b->CreatePHI(b->getBitBlockType(), 2);
        phiLiteralStartPos->addIncoming(notExtensionLiteralStart, entryBlock);
        phiLiteralStartPos->addIncoming(newLiteralStartPos, extensionFinalBlock);

        PHINode* phiLiteralLength = b->CreatePHI(b->getBitBlockType(), 2);
        phiLiteralLength->addIncoming(baseLiteralLength, entryBlock);
        phiLiteralLength->addIncoming(newLiteralLength, extensionFinalBlock);

        return std::make_pair(phiLiteralStartPos, phiLiteralLength);
    };

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::parseMultipleMatchInfoByLoop(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* initMatchLengthVec = b->simd_add(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_0F), b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4)));
        Value* initMatchEndPosVec = b->simd_add(SIMD_WIDTH, matchOffsetPosVec, BIT_BLOCK_1);

        Value* shouldExtendLiteralBitBlockVec = b->simd_and(b->simd_eq(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_0F), BIT_BLOCK_0F), notFinishMask);
        Value* extendMatchSignMask = b->hsimd_signmask(SIMD_WIDTH, shouldExtendLiteralBitBlockVec);

        BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateBr(conBlock);

        b->SetInsertPoint(conBlock);
        PHINode* phiSignMask = b->CreatePHI(extendMatchSignMask->getType(), 2);
        phiSignMask->addIncoming(extendMatchSignMask, entryBlock);
        PHINode* phiMatchEndPosVec = b->CreatePHI(initMatchEndPosVec->getType(), 2);
        phiMatchEndPosVec->addIncoming(initMatchEndPosVec, entryBlock);
        PHINode* phiMatchLengthVec = b->CreatePHI(initMatchLengthVec->getType(), 2);
        phiMatchLengthVec->addIncoming(initMatchLengthVec, entryBlock);

        b->CreateCondBr(b->CreateICmpNE(phiSignMask, b->getInt32(0)), bodyBlock, exitBlock);

        // ---- BodyBlock
        b->SetInsertPoint(bodyBlock);
        Value* targetIndex = b->CreateCountForwardZeroes(phiSignMask);
        Value* targetMatchOffsetPos = b->CreateExtractElement(matchOffsetPosVec, targetIndex);
        Value* targetTokenValue = b->CreateTrunc(b->CreateExtractElement(tokenValuesVec, targetIndex), b->getInt8Ty());

        Value *newMatchEndPos = nullptr, *newMatchLength = nullptr;
        std::tie(newMatchEndPos, newMatchLength) = this->parseMatchInfo(b, targetMatchOffsetPos, targetTokenValue);
        phiMatchEndPosVec->addIncoming(b->CreateInsertElement(phiMatchEndPosVec, newMatchEndPos, targetIndex), b->GetInsertBlock());
        phiMatchLengthVec->addIncoming(b->CreateInsertElement(phiMatchLengthVec, newMatchLength, targetIndex), b->GetInsertBlock());
        phiSignMask->addIncoming(b->CreateSub(phiSignMask, b->CreateShl(b->getInt32(1), targetIndex)), b->GetInsertBlock());
        b->CreateBr(conBlock);

        // ---- ExitBlock
        b->SetInsertPoint(exitBlock);

        return std::make_pair(phiMatchEndPosVec, b->simd_and(phiMatchLengthVec, notFinishMask));
    }

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::parallelParseMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {

        Value* shouldExtendMatchBitBlockVec = b->simd_eq(SIMD_WIDTH, b->simd_and(BIT_BLOCK_0F, tokenValuesVec), BIT_BLOCK_0F);
        Value* extendMatchParallelLevel = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, shouldExtendMatchBitBlockVec));
        BasicBlock* simdBlock = b->CreateBasicBlock("simdBlock");
        BasicBlock* loopBlock = b->CreateBasicBlock("loopBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateUnlikelyCondBr(b->CreateICmpUGE(extendMatchParallelLevel, b->getInt32(mMininumParallelLevel)), simdBlock, loopBlock);

        b->SetInsertPoint(simdBlock);
        auto ret1 = this->simdParseMatchInfo(b, matchOffsetPosVec, tokenValuesVec, notFinishMask);
        BasicBlock* simdBlockFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        b->SetInsertPoint(loopBlock);
        auto ret2 = this->parseMultipleMatchInfoByLoop(b, matchOffsetPosVec, tokenValuesVec, notFinishMask);
        BasicBlock* loopBlockFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        PHINode* phiFirst = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiFirst->addIncoming(ret1.first, simdBlockFinal);
        phiFirst->addIncoming(ret2.first, loopBlockFinal);

        PHINode* phiSecond = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiSecond->addIncoming(ret1.second, simdBlockFinal);
        phiSecond->addIncoming(ret2.second, loopBlockFinal);

        return std::make_pair(phiFirst, phiSecond);

    }

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::simdParseMatchInfo2(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetPosVec, llvm::Value* tokenValuesVec, llvm::Value* notFinishMask) {
        // ---- entryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* matchOffsetNextPosVec = b->simd_add(SIMD_WIDTH, matchOffsetPosVec, BIT_BLOCK_1);

        Value* baseMatchLength = b->simd_and(tokenValuesVec, BIT_BLOCK_0F);

        Value* shouldExtendMatchGatherMask = b->simd_eq(SIMD_WIDTH, baseMatchLength, BIT_BLOCK_0F);
        shouldExtendMatchGatherMask = b->simd_and(shouldExtendMatchGatherMask, notFinishMask);
        Value* shouldExtendMatchBitBlockVec = b->CreateZExt(b->CreateICmpEQ(baseMatchLength, BIT_BLOCK_0F), BIT_BLOCK_TY);

        Value* shouldExtension = b->CreateICmpNE(b->hsimd_signmask(SIMD_WIDTH, shouldExtendMatchGatherMask), b->getInt32(0));

        Value* initExtendMatchPosVec = b->simd_add(SIMD_WIDTH, matchOffsetNextPosVec, shouldExtendMatchBitBlockVec);

        baseMatchLength = b->simd_add(SIMD_WIDTH, baseMatchLength, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4)));


        BasicBlock* extensionBlock = b->CreateBasicBlock("extensionBlock");
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        b->CreateLikelyCondBr(shouldExtension, extensionBlock, exitBlock);

        // ---- extensionBlock
        b->SetInsertPoint(extensionBlock);

        PHINode* phiExtendMatchPos = b->CreatePHI(initExtendMatchPosVec->getType(), 2);
        phiExtendMatchPos->addIncoming(initExtendMatchPosVec, entryBlock);
        PHINode* phiExtendMatchLength = b->CreatePHI(BIT_BLOCK_TY, 2);
        phiExtendMatchLength->addIncoming(BIT_BLOCK_0, entryBlock);
        PHINode* phiGatherMask = b->CreatePHI(shouldExtendMatchGatherMask->getType(), 2);
        phiGatherMask->addIncoming(shouldExtendMatchGatherMask, entryBlock);


        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", SIZE_0), b->getInt8PtrTy());
        Value* i64ExtensionValue = this->simdFetchI64DataByGather(b, byteRawInputPtr, phiExtendMatchPos, phiGatherMask);

        Value* forwardZeros = b->simd_cttz(SIMD_WIDTH, b->simd_not(i64ExtensionValue));
        Value* BIT_BLOCK_LOG2_8 = b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, std::log2(8)));
        Value* extensionLength = b->simd_srlv(SIMD_WIDTH, forwardZeros, BIT_BLOCK_LOG2_8);

        Value* shiftedExtensionValue = b->simd_srlv(SIMD_WIDTH, i64ExtensionValue, b->simd_sllv(SIMD_WIDTH, extensionLength, BIT_BLOCK_LOG2_8));
        shiftedExtensionValue = b->simd_and(shiftedExtensionValue, BIT_BLOCK_FF);

        Value* i64ExtensionLength = b->simd_add(SIMD_WIDTH, b->simd_mult(SIMD_WIDTH, extensionLength, BIT_BLOCK_FF), shiftedExtensionValue);

        Value* newExtendMatchLength = b->simd_add(SIMD_WIDTH, phiExtendMatchLength, b->simd_and(i64ExtensionLength, phiGatherMask));
        Value* newExtendMatchPos = b->simd_add(SIMD_WIDTH, phiExtendMatchPos, b->simd_and(extensionLength, phiGatherMask));


        Value* newMatchLength = b->simd_add(SIMD_WIDTH, newExtendMatchLength, baseMatchLength);
        Value* newMatchEndPos = newExtendMatchPos;

        Value* shouldContinueExtendMask = b->simd_eq(SIMD_WIDTH, extensionLength, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 8)));
        Value* shouldContinue = b->CreateICmpNE(b->CreateBitCast(shouldContinueExtendMask, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        BasicBlock* extensionFinalBlock = b->GetInsertBlock();


        phiExtendMatchLength->addIncoming(newExtendMatchLength, b->GetInsertBlock());
        phiExtendMatchPos->addIncoming(newExtendMatchPos, b->GetInsertBlock());
        phiGatherMask->addIncoming(shouldContinueExtendMask, b->GetInsertBlock());

        b->CreateUnlikelyCondBr(shouldContinue, extensionBlock, exitBlock);

//        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);


        PHINode* phiExtendMatchEndPos = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendMatchEndPos->addIncoming(matchOffsetNextPosVec, entryBlock);
        phiExtendMatchEndPos->addIncoming(newMatchEndPos, extensionFinalBlock);

        PHINode* phiMatchLength = b->CreatePHI(b->getBitBlockType(), 2);
        phiMatchLength->addIncoming(baseMatchLength, entryBlock);
        phiMatchLength->addIncoming(newMatchLength, extensionFinalBlock);


        return std::make_pair(phiExtendMatchEndPos, b->simd_and(phiMatchLength, notFinishMask));

    }

    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::simdParseMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetPosVec, llvm::Value* tokenValuesVec, llvm::Value* hasMatchPartMask) {
        // ---- entryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* matchOffsetNextPosVec = b->simd_add(SIMD_WIDTH, matchOffsetPosVec, BIT_BLOCK_1);
        Value* shouldExtendMatchBitBlockVec = b->CreateZExt(b->CreateICmpEQ(b->CreateAnd(BIT_BLOCK_0F, tokenValuesVec), BIT_BLOCK_0F), b->getBitBlockType());
        Value* shouldExtendMatch = b->CreateICmpNE(b->CreateBitCast(shouldExtendMatchBitBlockVec, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        BasicBlock* extendMatchCon = b->CreateBasicBlock("extendMatchCon");
        BasicBlock* extendMatchExit = b->CreateBasicBlock("extendMatchExit");

        Value* initExtendMatchPosVec = b->simd_add(SIMD_WIDTH, matchOffsetNextPosVec, shouldExtendMatchBitBlockVec);
        b->CreateLikelyCondBr(shouldExtendMatch, extendMatchCon, extendMatchExit); // Match Extension will happen quite often

        // ---- extendMatchCon
        b->SetInsertPoint(extendMatchCon);
        PHINode* phiCurrentExtendMatchPosVec = b->CreatePHI(initExtendMatchPosVec->getType(), 2);
        phiCurrentExtendMatchPosVec->addIncoming(initExtendMatchPosVec, entryBlock);
        PHINode* phiExtendMatchLengthVec = b->CreatePHI(b->getBitBlockType(), 2);
        phiExtendMatchLengthVec->addIncoming(BIT_BLOCK_0, entryBlock);

        PHINode* phiShouldExtendMatchBitBlockVec = b->CreatePHI(shouldExtendMatchBitBlockVec->getType(), 2);
        phiShouldExtendMatchBitBlockVec->addIncoming(shouldExtendMatchBitBlockVec, entryBlock);
        Value* shouldExtendMatchGatherMask = b->CreateNeg(phiShouldExtendMatchBitBlockVec);
        shouldExtendMatchGatherMask = b->simd_and(shouldExtendMatchGatherMask, hasMatchPartMask);
//        shouldExtendMatchGatherMask = b->simd_and(shouldExtendMatchGatherMask, notFinishMask);

        // TODO maybe we can load i64 once and then consume 8 times
//        this->recordParallelLevel(b, b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, shouldExtendMatchGatherMask)));
        Value* currentMatchLengthVec = this->simdFetchByteData(b, byteRawInputPtr, phiCurrentExtendMatchPosVec, shouldExtendMatchGatherMask);

        Value* newExtendMatchLengthVec = b->simd_add(SIMD_WIDTH, phiExtendMatchLengthVec, currentMatchLengthVec);

        Value* shouldContinueExtendMatchVecBitBlock = b->CreateZExt(b->CreateICmpEQ(currentMatchLengthVec, BIT_BLOCK_FF), b->getBitBlockType());
        Value* newExtendMatchPosVec = b->simd_add(SIMD_WIDTH, phiCurrentExtendMatchPosVec, b->simd_and(shouldExtendMatchBitBlockVec, shouldContinueExtendMatchVecBitBlock));

        phiCurrentExtendMatchPosVec->addIncoming(newExtendMatchPosVec, b->GetInsertBlock());
        phiExtendMatchLengthVec->addIncoming(newExtendMatchLengthVec, b->GetInsertBlock());

        phiShouldExtendMatchBitBlockVec->addIncoming(shouldContinueExtendMatchVecBitBlock, b->GetInsertBlock());
        Value* shouldContinueExtendMatch = b->CreateICmpNE(b->CreateBitCast(shouldContinueExtendMatchVecBitBlock, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0);

        b->CreateUnlikelyCondBr(shouldContinueExtendMatch, extendMatchCon, extendMatchExit); // Usually it will be enough for only 1 match extension


        // ---- extendMatchExit
        b->SetInsertPoint(extendMatchExit);

        PHINode* matchExtendValueVec = b->CreatePHI(newExtendMatchLengthVec->getType(), 2);
        matchExtendValueVec->addIncoming(BIT_BLOCK_0, entryBlock);
        matchExtendValueVec->addIncoming(newExtendMatchLengthVec, extendMatchCon);
        PHINode* phiExtendMatchEndPos = b->CreatePHI(matchOffsetNextPosVec->getType(), 2);
        phiExtendMatchEndPos->addIncoming(matchOffsetNextPosVec, entryBlock);
        phiExtendMatchEndPos->addIncoming(phiCurrentExtendMatchPosVec, extendMatchCon);


        // matchLength = (size_t)token & 0xf + 4 + matchExtendValue
        Value* matchLength = b->simd_add(
                SIMD_WIDTH,
                b->simd_add(SIMD_WIDTH, matchExtendValueVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 4))),
                b->simd_and(tokenValuesVec, BIT_BLOCK_0F)
        );
        matchLength = b->simd_and(matchLength, hasMatchPartMask);

        return std::make_pair(phiExtendMatchEndPos, matchLength);
    };


    std::pair<llvm::Value*, llvm::Value*> LZ4ParallelByteStreamAioKernel::simdProcessBlockBoundaryByLoop(const std::unique_ptr<KernelBuilder> &b,
                                                                         llvm::Value *tokenPosVec, llvm::Value *lz4BlockEnd, llvm::Value* outputPosVec) {
        Value* retNextTokenPos = ConstantVector::getNullValue(b->getBitBlockType());
        Value* retNewOutputPos = ConstantVector::getNullValue(b->getBitBlockType());

        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            auto ret = this->processBlockBoundary(
                    b,
                    b->CreateExtractElement(tokenPosVec, i),
                    b->CreateExtractElement(lz4BlockEnd, i),
                    b->CreateExtractElement(outputPosVec, i)
            );
            retNextTokenPos = b->CreateInsertElement(retNextTokenPos, ret.first, i);
            retNewOutputPos = b->CreateInsertElement(retNewOutputPos, ret.second, i);
        }
        return std::make_pair(retNextTokenPos, retNewOutputPos);
    };

    std::pair<Value *, Value *> LZ4ParallelByteStreamAioKernel::simdProcessBlockBoundary(
            const std::unique_ptr<KernelBuilder> &b, Value *tokenPosVec, Value *lz4BlockEndVec, Value* initOutputPosVec
    ) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* notFinishMask = b->simd_ult(SIMD_WIDTH, tokenPosVec, lz4BlockEndVec);

        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* tokenValuesVec = this->simdFetchByteData(b, byteRawInputPtr, tokenPosVec, notFinishMask);

        Value *literalStartPosVec = nullptr, *literalLengthVec = nullptr;


//        std::tie(literalStartPosVec, literalLengthVec) = this->simdParseLiteralInfo(b, tokenPosVec, tokenValuesVec, notFinishMask);
        std::tie(literalStartPosVec, literalLengthVec) = this->simdParseLiteralInfo2(b, tokenPosVec, tokenValuesVec, notFinishMask);

        Value* literalEndPosVec = b->simd_add(SIMD_WIDTH, literalStartPosVec, literalLengthVec);


        this->handleSimdLiteralCopy(b, literalStartPosVec, literalLengthVec, initOutputPosVec);


        Value* outputPosAfterLiteralCpy = b->simd_add(SIMD_WIDTH, initOutputPosVec, literalLengthVec);


        Value* matchOffsetBeginPosVec = literalEndPosVec;

        Value* matchOffsetNextPosVec = b->simd_add(SIMD_WIDTH, matchOffsetBeginPosVec, BIT_BLOCK_1);


        BasicBlock* hasMatchPartBlock = b->CreateBasicBlock("hasMatchPartBlock");


        BasicBlock* extendLiteralEndFinal = b->GetInsertBlock();

        Value* hasMatchPartMask = b->simd_ult(SIMD_WIDTH, matchOffsetBeginPosVec, lz4BlockEndVec);
        b->CreateLikelyCondBr(b->CreateICmpNE(b->CreateBitCast(hasMatchPartMask, INT_BIT_BLOCK_TY), INT_BIT_BLOCK_TY_0), hasMatchPartBlock, exitBlock);

        // ---- hasMatchPartBlock
        b->SetInsertPoint(hasMatchPartBlock);

        Value *matchLength = nullptr, *extendMatchEndPos = nullptr;

//        std::tie(extendMatchEndPos, matchLength) = this->simdParseMatchInfo(b, matchOffsetBeginPosVec, tokenValuesVec, hasMatchPartMask);
        std::tie(extendMatchEndPos, matchLength) = this->simdParseMatchInfo2(b, matchOffsetBeginPosVec, tokenValuesVec, hasMatchPartMask);

        Value* matchOffsetVec = this->simdFetchData(
                b,
                byteRawInputPtr,
                matchOffsetBeginPosVec,
                hasMatchPartMask
        );
        matchOffsetVec = b->simd_and(matchOffsetVec, BIT_BLOCK_FFFF);

//        Value* t1 = b->CreateReadCycleCounter();
        this->handleSimdMatchCopy(b, matchOffsetVec, matchLength, outputPosAfterLiteralCpy);
//        Value* t2 = b->CreateReadCycleCounter();
//        b->setScalarField("tempTimes", b->CreateAdd(b->getScalarField("tempTimes"), b->CreateSub(t2 ,t1)));

        Value* outputPosAfterMatchCpy = b->simd_add(SIMD_WIDTH, outputPosAfterLiteralCpy, matchLength);

        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();

        b->CreateBr(exitBlock);
        // ---- exitBlock

        b->SetInsertPoint(exitBlock);

        PHINode* phiBeforeTokenPos = b->CreatePHI(matchOffsetNextPosVec->getType(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPosVec, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(extendMatchEndPos, extendMatchExitFinal);

        PHINode* phiNewOutputPos = b->CreatePHI(outputPosAfterLiteralCpy->getType(), 2);
        phiNewOutputPos->addIncoming(outputPosAfterLiteralCpy, extendLiteralEndFinal);
        phiNewOutputPos->addIncoming(outputPosAfterMatchCpy, extendMatchExitFinal);
        Value* nextTokenPos = b->simd_add(SIMD_WIDTH, phiBeforeTokenPos, BIT_BLOCK_1);
        return std::make_pair(nextTokenPos, phiNewOutputPos);
    }

    void LZ4ParallelByteStreamAioKernel::generateSimdDecompression1(const std::unique_ptr<KernelBuilder> &b, Value* blockDataIndex) {
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("simdDecompressionExitBlock");
        BasicBlock* processCon = b->CreateBasicBlock("simdDecompressionProcessConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("simdDecompressionProcessBodyBlock");
        BasicBlock* sequentialProcessBlock = b->CreateBasicBlock("simdDecompressionSequentialProcessBlock");


        // ---- entryBlock
        Value* blockStartVec = this->generateLoadSimdInt64NumberInput(b, "blockStart", blockDataIndex);
        Value* blockEndVec = this->generateLoadSimdInt64NumberInput(b, "blockEnd", blockDataIndex);

        Value* outputPos = b->getProducedItemCount("outputStream");
        Value* initOutputPosVec = b->simd_fill(SIMD_WIDTH, outputPos);
        std::vector<Constant*> initOutputOffset;
        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            initOutputOffset.push_back(b->getIntN(SIMD_WIDTH, i * mLz4BlockSize));
        }

        initOutputPosVec = b->simd_add(SIMD_WIDTH, initOutputPosVec, ConstantVector::get(initOutputOffset));

        // TODO handle uncompression blocks

        b->CreateBr(processCon);

        // ---- processCon
        b->SetInsertPoint(processCon);
        PHINode* phiCursorVec = b->CreatePHI(blockStartVec->getType(), 3);
        phiCursorVec->addIncoming(blockStartVec, entryBlock);

        PHINode* phiOutputPosVec = b->CreatePHI(initOutputPosVec->getType(), 3);
        phiOutputPosVec->addIncoming(initOutputPosVec, entryBlock);


        Value* hasRemaining = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        Value* signMask = b->hsimd_signmask(SIMD_WIDTH, hasRemaining);
        Value* maskPopcount = b->CreatePopcount(signMask);
        Value* shouldSimdProcess = b->CreateICmpUGE(maskPopcount, b->getInt32(mMininumParallelLevel));

        hasRemaining = b->CreateICmpNE(b->CreateBitCast(hasRemaining, b->getIntNTy(b->getBitBlockWidth())), Constant::getNullValue(b->getIntNTy(b->getBitBlockWidth())));

        b->CreateLikelyCondBr(shouldSimdProcess, processBody, sequentialProcessBlock);

        // ---- processBody
        b->SetInsertPoint(processBody);

        Value* notFinishMask = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* tokenValuesVec = this->simdFetchByteData(b, byteRawInputPtr, phiCursorVec, notFinishMask);

        Value* literalN = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, b->simd_eq(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_F0), BIT_BLOCK_F0)));
        Value* matchN = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, b->simd_eq(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_0F), BIT_BLOCK_0F)));

//        Value* shouldSimdProcess2 = b->CreateAnd(
//                b->CreateOr(b->CreateICmpEQ(literalN, b->getInt32(0)), b->CreateICmpUGE(literalN, b->getInt32(mMininumParallelLevel))),
//                b->CreateOr(b->CreateICmpEQ(matchN, b->getInt32(0)), b->CreateICmpUGE(matchN, b->getInt32(mMininumParallelLevel)))
//        );

        Value* shouldSimdProcess2 = b->CreateAnd(b->CreateICmpEQ(literalN, b->getInt32(0)), b->CreateICmpUGE(matchN, b->getInt32(mMininumParallelLevel)));

        BasicBlock* loopProcessBody = b->CreateBasicBlock("loopProcessBody");
        BasicBlock* simdProcessBody = b->CreateBasicBlock("simdProcessBody");


        b->CreateCondBr(shouldSimdProcess2, simdProcessBody, loopProcessBody);

        // ---- simdProcessBody
        {
            b->SetInsertPoint(simdProcessBody);
            Value *newCursorVec = nullptr, *newOutputPosVec = nullptr;
//            std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundary(b, phiCursorVec, blockEndVec, phiOutputPosVec);
            std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundaryByLoop(b, phiCursorVec, blockEndVec, phiOutputPosVec);

            phiCursorVec->addIncoming(newCursorVec, b->GetInsertBlock());
            phiOutputPosVec->addIncoming(newOutputPosVec, b->GetInsertBlock());

            b->CreateBr(processCon);
        }

        // ---- loopProcessBody
        {
            b->SetInsertPoint(loopProcessBody);
            Value *newCursorVec = nullptr, *newOutputPosVec = nullptr;
            std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundaryByLoop(b, phiCursorVec, blockEndVec, phiOutputPosVec);

            phiCursorVec->addIncoming(newCursorVec, b->GetInsertBlock());
            phiOutputPosVec->addIncoming(newOutputPosVec, b->GetInsertBlock());

            b->CreateBr(processCon);
        }


        // ---- sequentialProcessBlock
        b->SetInsertPoint(sequentialProcessBlock);

        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {

            BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
            BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
            BasicBlock* innerExitBlock = b->CreateBasicBlock("exitBlock");

            b->CreateBr(conBlock);

            // ---- conBlock
            b->SetInsertPoint(conBlock);
            Value* shouldProcessSequentially = b->CreateICmpNE(b->CreateAnd(b->CreateLShr(signMask, b->getInt32(i)), b->getInt32(1)), b->getInt32(0));
            Value* originalOutputPos = b->CreateExtractElement(phiOutputPosVec, i);

            b->CreateCondBr(shouldProcessSequentially, bodyBlock, innerExitBlock);

            // ---- bodyBlock
            b->SetInsertPoint(bodyBlock);

            // TODO need to handle blockEnd here when doing overwriting memcpy
            Value* newOutputPos = this->generateProcessCompressedBlock(
                    b,
                    b->CreateExtractElement(phiCursorVec, i),
                    b->CreateExtractElement(blockEndVec, i),
                    originalOutputPos
            );

            if (i == b->getBitBlockWidth() / SIMD_WIDTH - 1) {
                b->setProducedItemCount("outputStream", newOutputPos);
            }

            b->CreateBr(innerExitBlock);

            // ---- exitBlock
            b->SetInsertPoint(innerExitBlock);
        }


        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);

        uint64_t lastVecIndex = b->getBitBlockWidth() / SIMD_WIDTH - 1;
        Value* lastBlockEnd = b->CreateExtractElement(blockEndVec, lastVecIndex);
        b->setProcessedItemCount("byteStream", lastBlockEnd);
        Value* lastOutputPos = b->CreateExtractElement(phiOutputPosVec, lastVecIndex);
        b->setProducedItemCount("outputStream", b->CreateUMax(b->getProducedItemCount("outputStream"), lastOutputPos));
//        b->CallPrintInt("produced", b->getProducedItemCount("outputStream"));

    }

    void LZ4ParallelByteStreamAioKernel::generateSimdDecompression2(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockDataIndex) {
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("simdDecompressionExitBlock");
        BasicBlock* processCon = b->CreateBasicBlock("simdDecompressionProcessConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("simdDecompressionProcessBodyBlock");
        BasicBlock* sequentialProcessBlock = b->CreateBasicBlock("simdDecompressionSequentialProcessBlock");

        // ---- entryBlock
        Value* blockStartVec = this->generateLoadSimdInt64NumberInput(b, "blockStart", blockDataIndex);
        Value* blockEndVec = this->generateLoadSimdInt64NumberInput(b, "blockEnd", blockDataIndex);

        Value* outputPos = b->getProducedItemCount("outputStream");
        Value* initOutputPosVec = b->simd_fill(SIMD_WIDTH, outputPos);
        std::vector<Constant*> initOutputOffset;
        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            initOutputOffset.push_back(b->getIntN(SIMD_WIDTH, i * 4 * 1024 * 1024));
        }

        initOutputPosVec = b->simd_add(SIMD_WIDTH, initOutputPosVec, ConstantVector::get(initOutputOffset));

        // TODO handle uncompression blocks

        b->CreateBr(processCon);

        // ---- processCon
        b->SetInsertPoint(processCon);
        PHINode* phiCursorVec = b->CreatePHI(blockStartVec->getType(), 2);
        phiCursorVec->addIncoming(blockStartVec, entryBlock);

        PHINode* phiOutputPosVec = b->CreatePHI(initOutputPosVec->getType(), 2);
        phiOutputPosVec->addIncoming(initOutputPosVec, entryBlock);

        Value* hasRemaining = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        Value* signMask = b->hsimd_signmask(SIMD_WIDTH, hasRemaining);
        Value* maskPopcount = b->CreatePopcount(signMask);
        Value* shouldSimdProcess = b->CreateICmpUGE(maskPopcount, b->getInt32(mMininumParallelLevel));
        b->CreateLikelyCondBr(shouldSimdProcess, processBody, sequentialProcessBlock);

        // ---- processBody
        b->SetInsertPoint(processBody);
//        Value* newCursorVec = this->generateSimdAcceleration(b, phiCursorVec, blockEndVec);

        Value *newCursorVec = nullptr, *newOutputPosVec = nullptr;
        std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundary(b, phiCursorVec, blockEndVec, phiOutputPosVec);
//        std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundaryByLoop(b, phiCursorVec, blockEndVec, phiOutputPosVec);

        phiCursorVec->addIncoming(newCursorVec, b->GetInsertBlock());
        phiOutputPosVec->addIncoming(newOutputPosVec, b->GetInsertBlock());

        b->CreateBr(processCon);

        // ---- sequentialProcessBlock
        b->SetInsertPoint(sequentialProcessBlock);

        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {

            BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
            BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
            BasicBlock* innerExitBlock = b->CreateBasicBlock("exitBlock");

            b->CreateBr(conBlock);

            // ---- conBlock
            b->SetInsertPoint(conBlock);
            Value* shouldProcessSequentially = b->CreateICmpNE(b->CreateAnd(b->CreateLShr(signMask, b->getInt32(i)), b->getInt32(1)), b->getInt32(0));
            Value* originalOutputPos = b->CreateExtractElement(phiOutputPosVec, i);

            b->CreateCondBr(shouldProcessSequentially, bodyBlock, innerExitBlock);

            // ---- bodyBlock
            b->SetInsertPoint(bodyBlock);

            // TODO need to handle blockEnd here when doing overwriting memcpy
            Value* newOutputPos = this->generateProcessCompressedBlock(
                    b,
                    b->CreateExtractElement(phiCursorVec, i),
                    b->CreateExtractElement(blockEndVec, i),
                    originalOutputPos
            );

            if (i == b->getBitBlockWidth() / SIMD_WIDTH - 1) {
                b->setProducedItemCount("outputStream", newOutputPos);
            }

            b->CreateBr(innerExitBlock);

            // ---- exitBlock
            b->SetInsertPoint(innerExitBlock);
        }


        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);

        uint64_t lastVecIndex = b->getBitBlockWidth() / SIMD_WIDTH - 1;
        Value* lastBlockEnd = b->CreateExtractElement(blockEndVec, lastVecIndex);
        b->setProcessedItemCount("byteStream", lastBlockEnd);
        Value* lastOutputPos = b->CreateExtractElement(phiOutputPosVec, lastVecIndex);
        b->setProducedItemCount("outputStream", b->CreateUMax(b->getProducedItemCount("outputStream"), lastOutputPos));
//        b->CallPrintInt("produced", b->getProducedItemCount("outputStream"));
    }


    void LZ4ParallelByteStreamAioKernel::generateSimdDecompression(const std::unique_ptr<KernelBuilder> &b, Value* blockDataIndex) {
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("simdDecompressionExitBlock");
        BasicBlock* processCon = b->CreateBasicBlock("simdDecompressionProcessConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("simdDecompressionProcessBodyBlock");
        BasicBlock* sequentialProcessBlock = b->CreateBasicBlock("simdDecompressionSequentialProcessBlock");


        // ---- entryBlock
        Value* blockStartVec = this->generateLoadSimdInt64NumberInput(b, "blockStart", blockDataIndex);
        Value* blockEndVec = this->generateLoadSimdInt64NumberInput(b, "blockEnd", blockDataIndex);

        Value* outputPos = b->getProducedItemCount("outputStream");
        Value* initOutputPosVec = b->simd_fill(SIMD_WIDTH, outputPos);
        std::vector<Constant*> initOutputOffset;
        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            initOutputOffset.push_back(b->getIntN(SIMD_WIDTH, i * 4 * 1024 * 1024));
        }

        initOutputPosVec = b->simd_add(SIMD_WIDTH, initOutputPosVec, ConstantVector::get(initOutputOffset));

        // TODO handle uncompression blocks

        b->CreateBr(processCon);

        // ---- processCon
        b->SetInsertPoint(processCon);
        PHINode* phiCursorVec = b->CreatePHI(blockStartVec->getType(), 2);
        phiCursorVec->addIncoming(blockStartVec, entryBlock);

        PHINode* phiOutputPosVec = b->CreatePHI(initOutputPosVec->getType(), 2);
        phiOutputPosVec->addIncoming(initOutputPosVec, entryBlock);


        Value* hasRemaining = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        Value* signMask = b->hsimd_signmask(SIMD_WIDTH, hasRemaining);
        Value* maskPopcount = b->CreatePopcount(signMask);


        hasRemaining = b->CreateICmpNE(b->CreateBitCast(hasRemaining, b->getIntNTy(b->getBitBlockWidth())), Constant::getNullValue(b->getIntNTy(b->getBitBlockWidth())));

        Value* notFinishMask = b->simd_ult(SIMD_WIDTH, phiCursorVec, blockEndVec);
        Value* byteRawInputPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* tokenValuesVec = this->simdFetchByteData(b, byteRawInputPtr, phiCursorVec, notFinishMask);

        Value* literalN = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, b->simd_eq(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_F0), BIT_BLOCK_F0)));
        Value* matchN = b->CreatePopcount(b->hsimd_signmask(SIMD_WIDTH, b->simd_eq(SIMD_WIDTH, b->simd_and(tokenValuesVec, BIT_BLOCK_0F), BIT_BLOCK_0F)));

        Value* shouldSimdProcess = b->CreateAnd(
                b->CreateOr(b->CreateICmpEQ(literalN, b->getInt32(0)), b->CreateICmpUGE(literalN, b->getInt32(mMininumParallelLevel))),
                b->CreateOr(b->CreateICmpEQ(matchN, b->getInt32(0)), b->CreateICmpUGE(matchN, b->getInt32(mMininumParallelLevel)))
        );
        this->recordParallelLevel(b, b->CreateZExt(shouldSimdProcess, b->getInt32Ty()));
//        shouldSimdProcess = b->getInt1(false);


        b->CreateLikelyCondBr(shouldSimdProcess, processBody, sequentialProcessBlock);

        // ---- processBody
        b->SetInsertPoint(processBody);
//        Value* newCursorVec = this->generateSimdAcceleration(b, phiCursorVec, blockEndVec);

        Value *newCursorVec = nullptr, *newOutputPosVec = nullptr;
        std::tie(newCursorVec, newOutputPosVec) = this->simdProcessBlockBoundary(b, phiCursorVec, blockEndVec, phiOutputPosVec);

        phiCursorVec->addIncoming(newCursorVec, b->GetInsertBlock());
        phiOutputPosVec->addIncoming(newOutputPosVec, b->GetInsertBlock());

        b->CreateBr(processCon);

        // ---- sequentialProcessBlock
        b->SetInsertPoint(sequentialProcessBlock);

        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {

            BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
            BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");
            BasicBlock* innerExitBlock = b->CreateBasicBlock("exitBlock");

            b->CreateBr(conBlock);

            // ---- conBlock
            b->SetInsertPoint(conBlock);
            Value* shouldProcessSequentially = b->CreateICmpNE(b->CreateAnd(b->CreateLShr(signMask, b->getInt32(i)), b->getInt32(1)), b->getInt32(0));
            Value* originalOutputPos = b->CreateExtractElement(phiOutputPosVec, i);

            b->CreateCondBr(shouldProcessSequentially, bodyBlock, innerExitBlock);

            // ---- bodyBlock
            b->SetInsertPoint(bodyBlock);

            // TODO need to handle blockEnd here when doing overwriting memcpy
            Value* newOutputPos = this->generateProcessCompressedBlock(
                    b,
                    b->CreateExtractElement(phiCursorVec, i),
                    b->CreateExtractElement(blockEndVec, i),
                    originalOutputPos
            );

            if (i == b->getBitBlockWidth() / SIMD_WIDTH - 1) {
                b->setProducedItemCount("outputStream", newOutputPos);
            }

            b->CreateBr(innerExitBlock);

            // ---- exitBlock
            b->SetInsertPoint(innerExitBlock);
        }


        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);

        uint64_t lastVecIndex = b->getBitBlockWidth() / SIMD_WIDTH - 1;
        Value* lastBlockEnd = b->CreateExtractElement(blockEndVec, lastVecIndex);
        b->setProcessedItemCount("byteStream", lastBlockEnd);
        Value* lastOutputPos = b->CreateExtractElement(phiOutputPosVec, lastVecIndex);
        b->setProducedItemCount("outputStream", b->CreateUMax(b->getProducedItemCount("outputStream"), lastOutputPos));
//        b->CallPrintInt("produced", b->getProducedItemCount("outputStream"));
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

    pair<Value*, Value*> LZ4ParallelByteStreamAioKernel::parseLiteralInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *tokenPos, llvm::Value* tokenValue) {
        BasicBlock* entryBlock = b->GetInsertBlock();

        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* shouldExtendLiteral = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_F0), BYTE_F0);
        BasicBlock* extendLiteralCond = b->CreateBasicBlock("extendLiteralCond");
        BasicBlock* extendLiteralEnd = b->CreateBasicBlock("extendLiteralEnd");

        Value* initExtendLiteralPos = b->CreateAdd(tokenPos, b->getSize(1));

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
        phiExtendLiteralEndPos->addIncoming(tokenPos, entryBlock);
        phiExtendLiteralEndPos->addIncoming(phiCurrentExtendLiteralPos, extendLiteralCond);

        Value* literalLength = b->CreateAdd(literalExtendValue, b->CreateZExt(b->CreateLShr(tokenValue, b->getInt8(4)), b->getSizeTy()));

        Value* literalStartPos = b->CreateAdd(phiExtendLiteralEndPos, SIZE_1);
        return std::make_pair(literalStartPos, literalLength);
    }

    std::pair<llvm::Value *, llvm::Value *> LZ4ParallelByteStreamAioKernel::parseMatchInfo(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffsetPos, llvm::Value* tokenValue) {
        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());

        Value* shouldExtendMatch = b->CreateICmpEQ(b->CreateAnd(tokenValue, BYTE_0F), BYTE_0F);
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* extendMatchCon = b->CreateBasicBlock("extendMatchCon");
        BasicBlock* extendMatchExit = b->CreateBasicBlock("extendMatchExit");

        Value* matchOffsetNextPos = b->CreateAdd(matchOffsetPos, SIZE_1);
        Value* initExtendMatchPos = b->CreateAdd(matchOffsetPos, b->getSize(2));

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

    pair<Value*, Value*> LZ4ParallelByteStreamAioKernel::processBlockBoundary(const std::unique_ptr<KernelBuilder> &b, llvm::Value *beginTokenPos,
                                                              llvm::Value *lz4BlockEnd, llvm::Value* initOutputPos) {
        // ---- EntryBlock
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        Value* bytePtrBase = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* tokenValue = b->CreateLoad(b->CreateGEP(bytePtrBase, beginTokenPos));


        // Literal
        Value *literalStartPos = nullptr, *literalLength = nullptr;
        std::tie(literalStartPos, literalLength) = this->parseLiteralInfo(b, beginTokenPos, tokenValue);
        this->handleLiteralCopy(b, literalStartPos, literalLength, initOutputPos);
        Value* literalEndPos = b->CreateAdd(literalStartPos, literalLength);
        Value* outputPosAfterLiteralCpy = b->CreateAdd(literalLength, initOutputPos);


        Value* matchOffsetBeginPos = literalEndPos;
        Value* matchOffsetNextPos = b->CreateAdd(matchOffsetBeginPos, SIZE_1);
        BasicBlock* hasMatchPartBlock = b->CreateBasicBlock("hasMatchPartBlock");
        BasicBlock* extendLiteralEndFinal = b->GetInsertBlock();
        b->CreateLikelyCondBr(b->CreateICmpULT(matchOffsetBeginPos, lz4BlockEnd), hasMatchPartBlock, exitBlock);


        // ---- hasMatchPartBlock
        b->SetInsertPoint(hasMatchPartBlock);
        // Match
        // For now, it is safe to cast matchOffset pointer into i16 since the input byte stream is always linear available
        Value* matchOffsetPtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", matchOffsetBeginPos), b->getInt16Ty()->getPointerTo());
        Value* matchOffset = b->CreateZExt(b->CreateLoad(matchOffsetPtr), b->getSizeTy());

        Value *matchLengthEndPos = nullptr, *matchLength = nullptr;
        std::tie(matchLengthEndPos, matchLength) = this->parseMatchInfo(b, matchOffsetBeginPos, tokenValue);
        this->handleMatchCopy(b, matchOffset, matchLength, outputPosAfterLiteralCpy);
        Value* outputPosAfterMatchCpy = b->CreateAdd(outputPosAfterLiteralCpy, matchLength);
        BasicBlock* extendMatchExitFinal = b->GetInsertBlock();
        b->CreateBr(exitBlock);


        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiBeforeTokenPos = b->CreatePHI(b->getSizeTy(), 2);
        phiBeforeTokenPos->addIncoming(matchOffsetNextPos, extendLiteralEndFinal);
        phiBeforeTokenPos->addIncoming(matchLengthEndPos, extendMatchExitFinal);
        PHINode* phiOutputPos = b->CreatePHI(b->getSizeTy(), 2);
        phiOutputPos->addIncoming(outputPosAfterLiteralCpy, extendLiteralEndFinal);
        phiOutputPos->addIncoming(outputPosAfterMatchCpy, extendMatchExitFinal);

        Value* nextTokenPos = b->CreateAdd(phiBeforeTokenPos, SIZE_1);

        return std::make_pair(nextTokenPos, phiOutputPos);
    }

    void LZ4ParallelByteStreamAioKernel::generateSimdMatchCopyByScatter(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* i64MatchCopyBlock = b->CreateBasicBlock("i64MatchCopyBlock");
        BasicBlock* i8MatchCopyBlock = b->CreateBasicBlock("i8MatchCopyBlock");

        llvm::Value* initCopiedLength = ConstantVector::getNullValue(matchLengthVec->getType());

//        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());

        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputPosRemVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(outputCapacity))));
        Value* outputPosRemBlockSizeVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(b->getIntN(SIMD_WIDTH, mLz4BlockSize)))));
        Value* remainingBlockSizeVec = b->simd_sub(SIMD_WIDTH, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, mLz4BlockSize)), outputPosRemBlockSizeVec);

        Value* possibleExceedBuffer = b->simd_uge(SIMD_WIDTH, b->simd_add(SIMD_WIDTH, matchLengthVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 8))), remainingBlockSizeVec);
        // true ffff, false 0000

        // TODO handle matchLengthVec == 0

        b->CreateUnlikelyCondBr(
                b->CreateICmpNE(b->CreateBitCast(possibleExceedBuffer, b->getIntNTy(b->getBitBlockWidth())), b->getIntN(b->getBitBlockWidth(), 0)),
                i8MatchCopyBlock,
                i64MatchCopyBlock
        );

        // ---- i8MatchCopyBlock
        b->SetInsertPoint(i8MatchCopyBlock);
        this->generateSimdSequentialMatchCopy(b, matchOffsetVec, matchLengthVec, outputPosVec);
        b->CreateBr(exitBlock);

        // ---- i64MatchCopyBlock
        b->SetInsertPoint(i64MatchCopyBlock);

        BasicBlock* i64MatchCopyConBlock = b->CreateBasicBlock("i64MatchCopyConBlock");
        BasicBlock* i64MatchCopyBodyBlock = b->CreateBasicBlock("i64MatchCopyBodyBlock");

        b->CreateBr(i64MatchCopyConBlock);

        // ---- i64MatchCopyConBlock
        b->SetInsertPoint(i64MatchCopyConBlock);
        PHINode* phiCopiedLength = b->CreatePHI(initCopiedLength->getType(), 2);
        phiCopiedLength->addIncoming(initCopiedLength, i64MatchCopyBlock);

        Value* shouldCopiedBitBlock = b->simd_ult(SIMD_WIDTH, phiCopiedLength, matchLengthVec);
//        b->CallPrintRegister("phiCopiedLength", phiCopiedLength);
//        b->CallPrintRegister("literalLengthVec", literalLengthVec);
//        b->CallPrintRegister("shouldCopiedBitBlock", shouldCopiedBitBlock);
        Value* shouldCopiedI1 = b->CreateICmpNE(
                b->CreateBitCast(shouldCopiedBitBlock, b->getIntNTy(b->getBitBlockWidth())),
                b->getIntN(b->getBitBlockWidth(), 0)
        );


        b->CreateCondBr(shouldCopiedI1, i64MatchCopyBodyBlock, exitBlock);

        // ---- i64MatchCopyBodyBlock
        b->SetInsertPoint(i64MatchCopyBodyBlock);
        Value* currentOutputPosVec = b->simd_add(SIMD_WIDTH, outputPosRemVec, phiCopiedLength);
        Value* copyFromPosVec = b->simd_sub(SIMD_WIDTH, currentOutputPosVec, matchOffsetVec);

        Value* literalData = this->simdFetchI64DataByGather(b, outputBasePtr, copyFromPosVec, shouldCopiedBitBlock);

        this->simdPutData(
                b,
                outputBasePtr,
                currentOutputPosVec,
                literalData,
                shouldCopiedBitBlock
        );
        phiCopiedLength->addIncoming(
                b->simd_add(
                        SIMD_WIDTH,
                        phiCopiedLength,
                        b->simd_and(
                                b->simd_umin(SIMD_WIDTH, matchOffsetVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 8))),
                                shouldCopiedBitBlock
                        )

                ),
                b->GetInsertBlock()
        );

        b->CreateBr(i64MatchCopyConBlock);
        b->SetInsertPoint(exitBlock);

    }

    void LZ4ParallelByteStreamAioKernel::generateSimdSequentialMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec) {
        // Constant
        Constant * SIZE_8 = b->getSize(8);
        // Type
        PointerType* i64PtrTy = b->getInt64Ty()->getPointerTo();
        PointerType* i8PtrTy = b->getInt8PtrTy();

        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputPosRemVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(outputCapacity))));
        Value* outputPosRemBlockSizeVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(b->getIntN(SIMD_WIDTH, mLz4BlockSize)))));
        Value* remainingBlockSizeVec = b->simd_sub(SIMD_WIDTH, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, mLz4BlockSize)), outputPosRemBlockSizeVec);
        Value* copyFromPosRemVec = b->simd_sub(SIMD_WIDTH, outputPosRemVec, matchOffsetVec);


        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());


        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {

            BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
            BasicBlock* i64MatchCopyBlock = b->CreateBasicBlock("i64LiteralCopyBlock");
            BasicBlock* i8MatchCopyBlock = b->CreateBasicBlock("i8LiteralCopyBlock");

            // ---- entryBlock
            Value* matchOffset = b->CreateExtractElement(matchOffsetVec, i);
            Value* matchLength = b->CreateExtractElement(matchLengthVec, i);
            Value* outputPosRem = b->CreateExtractElement(outputPosRemVec, i);
            Value* remainingBlockSize = b->CreateExtractElement(remainingBlockSizeVec, i);
            Value* outputFromRem = b->CreateExtractElement(copyFromPosRemVec, i);

            Value* inputInitPtr = b->CreateGEP(outputBasePtr, outputFromRem);
            Value* outputInitPtr = b->CreateGEP(outputBasePtr, outputPosRem);

            b->CreateLikelyCondBr(b->CreateICmpUGE(b->CreateSub(remainingBlockSize, matchLength), SIZE_8), i64MatchCopyBlock, i8MatchCopyBlock);

            //// i64 Match Copy
            // ---- i64MatchCopyBlock
            b->SetInsertPoint(i64MatchCopyBlock);

            this->generateOverwritingMemcpy(b, inputInitPtr, outputInitPtr, matchLength, i64PtrTy, b->CreateUMin(matchOffset, SIZE_8));
            b->CreateBr(exitBlock);

            //// i8 Match Copy
            // ---- i8MatchCopyBlock
            b->SetInsertPoint(i8MatchCopyBlock);
            this->generateOverwritingMemcpy(b, inputInitPtr, outputInitPtr, matchLength, i8PtrTy, 1);
            b->CreateBr(exitBlock);

            // ---- exitBlock
            b->SetInsertPoint(exitBlock);

        }

    }

    void LZ4ParallelByteStreamAioKernel::handleSimdMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* matchOffsetVec, llvm::Value* matchLengthVec, llvm::Value* outputPosVec) {
        if (AVX512BW_available() && mEnableScatter) {
            this->generateSimdMatchCopyByScatter(b, matchOffsetVec, matchLengthVec, outputPosVec);
        } else {
            this->generateSimdSequentialMatchCopy(b, matchOffsetVec, matchLengthVec, outputPosVec);
        }
    }

    void LZ4ParallelByteStreamAioKernel::handleSimdLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalStartVec, llvm::Value* literalLengthVec, llvm::Value* outputPosVec) {
//        this->recordParallelLevel(b, b->hsimd_signmask(SIMD_WIDTH, b->simd_gt(SIMD_WIDTH, literalLengthVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 32)))));
        if (AVX512BW_available() && mEnableScatter) {
            this->generateSimdLiteralCopyByScatter(b, literalStartVec, literalLengthVec, outputPosVec);
        } else {
            this->generateSimdSequentialLiteralCopy(b, literalStartVec, literalLengthVec, outputPosVec);
        }
    }

    void LZ4ParallelByteStreamAioKernel::generateSimdSequentialLiteralCopy(const std::unique_ptr<KernelBuilder> &b,
                                                                           llvm::Value *literalStartVec,
                                                                           llvm::Value *literalLengthVec,
                                                                           llvm::Value *outputPosVec) {
        // Constant
        Constant * SIZE_8 = b->getSize(8);
        // Type
        PointerType* i64PtrTy = b->getInt64Ty()->getPointerTo();
        PointerType* i8PtrTy = b->getInt8PtrTy();

        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputPosRemVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(outputCapacity))));
        Value* outputPosRemBlockSizeVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(b->getIntN(SIMD_WIDTH, mLz4BlockSize)))));
        Value* remainingBlockSizeVec = b->simd_sub(SIMD_WIDTH, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, mLz4BlockSize)), outputPosRemBlockSizeVec);

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());


        for (unsigned i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
            BasicBlock* i64LiteralCopyBlock = b->CreateBasicBlock("i64LiteralCopyBlock");
            BasicBlock* i8LiteralCopyBlock = b->CreateBasicBlock("i8LiteralCopyBlock");

            // ---- entryBlock
            Value* literalStart = b->CreateExtractElement(literalStartVec, i);
            Value* literalLength = b->CreateExtractElement(literalLengthVec, i);
            Value* outputPosRem = b->CreateExtractElement(outputPosRemVec, i);
            Value* remainingBlockSize = b->CreateExtractElement(remainingBlockSizeVec, i);

            Value* inputInitPtr = b->CreateGEP(inputBasePtr, literalStart);
            Value* outputInitPtr = b->CreateGEP(outputBasePtr, outputPosRem);

            b->CreateLikelyCondBr(b->CreateICmpUGE(b->CreateSub(remainingBlockSize, literalLength), SIZE_8), i64LiteralCopyBlock, i8LiteralCopyBlock);

            //// i64 Literal Copy
            // ---- i64LiteralCopyBlock
            b->SetInsertPoint(i64LiteralCopyBlock);
            this->generateOverwritingMemcpy(b, inputInitPtr, outputInitPtr, literalLength, i64PtrTy, 8);
            b->CreateBr(exitBlock);

            //// i8 Literal Copy
            // ---- i8LiteralCopyBlock
            b->SetInsertPoint(i8LiteralCopyBlock);
            this->generateOverwritingMemcpy(b, inputInitPtr, outputInitPtr, literalLength, i8PtrTy, 1);
            b->CreateBr(exitBlock);

            // ---- exitBlock
            b->SetInsertPoint(exitBlock);
        }
    }

    void LZ4ParallelByteStreamAioKernel::generateOverwritingMemcpy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *inputBasePtr,
                                   llvm::Value *outputBasePtr, llvm::Value *copyBytes, llvm::PointerType *targetPtrTy,
                                   llvm::Value* stepSize) {
//        unsigned targetPtrBitWidth = targetPtrTy->getElementType()->getIntegerBitWidth();

        Constant * SIZE_0 = b->getSize(0);
//        Constant * SIZE_1 = b->getSize(1);
        PointerType* i8PtrTy = b->getInt8PtrTy();

        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");

        Value* inputInitPtr = b->CreatePointerCast(inputBasePtr, targetPtrTy);
        Value* outputInitPtr = b->CreatePointerCast(outputBasePtr, targetPtrTy);

        b->CreateBr(conBlock);
        // ---- conBlock
        b->SetInsertPoint(conBlock);

        PHINode *phiCopiedSize = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedSize->addIncoming(SIZE_0, entryBlock);
        PHINode *phiInputPtr = b->CreatePHI(targetPtrTy, 2);
        phiInputPtr->addIncoming(inputInitPtr, entryBlock);
        PHINode *phiOutputPtr = b->CreatePHI(targetPtrTy, 2);
        phiOutputPtr->addIncoming(outputInitPtr, entryBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCopiedSize, copyBytes), bodyBlock, exitBlock);

        // ---- bodyBlock
        b->SetInsertPoint(bodyBlock);
        b->CreateStore(b->CreateLoad(phiInputPtr), phiOutputPtr);

        phiCopiedSize->addIncoming(b->CreateAdd(phiCopiedSize, stepSize), b->GetInsertBlock());

        Value *newInputPtr = nullptr, *newOutputPtr = nullptr;

        newInputPtr = b->CreatePointerCast(
                b->CreateGEP(b->CreatePointerCast(phiInputPtr, i8PtrTy), stepSize),
                targetPtrTy
        );

        newOutputPtr = b->CreatePointerCast(
                b->CreateGEP(b->CreatePointerCast(phiOutputPtr, i8PtrTy), stepSize),
                targetPtrTy
        );

        phiInputPtr->addIncoming(newInputPtr, b->GetInsertBlock());
        phiOutputPtr->addIncoming(newOutputPtr, b->GetInsertBlock());
        b->CreateBr(conBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
    }

    void LZ4ParallelByteStreamAioKernel::generateOverwritingMemcpy(const std::unique_ptr<KernelBuilder> &b,
                                                                   llvm::Value *inputBasePtr,
                                                                   llvm::Value *outputBasePtr,
                                                                   llvm::Value *copyBytes, llvm::PointerType *targetPtrTy,
                                                                   size_t stepSize) {
        unsigned targetPtrBitWidth = targetPtrTy->getElementType()->getIntegerBitWidth();

        Constant * SIZE_0 = b->getSize(0);
        Constant * SIZE_1 = b->getSize(1);
        PointerType* i8PtrTy = b->getInt8PtrTy();

        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* conBlock = b->CreateBasicBlock("conBlock");
        BasicBlock* bodyBlock = b->CreateBasicBlock("bodyBlock");

        Value* inputInitPtr = b->CreatePointerCast(inputBasePtr, targetPtrTy);
        Value* outputInitPtr = b->CreatePointerCast(outputBasePtr, targetPtrTy);

        b->CreateBr(conBlock);
        // ---- conBlock
        b->SetInsertPoint(conBlock);

        PHINode *phiCopiedSize = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedSize->addIncoming(SIZE_0, entryBlock);
        PHINode *phiInputPtr = b->CreatePHI(targetPtrTy, 2);
        phiInputPtr->addIncoming(inputInitPtr, entryBlock);
        PHINode *phiOutputPtr = b->CreatePHI(targetPtrTy, 2);
        phiOutputPtr->addIncoming(outputInitPtr, entryBlock);

        b->CreateCondBr(b->CreateICmpULT(phiCopiedSize, copyBytes), bodyBlock, exitBlock); //TODO 2 cond brs (one likely br here, the other unlikely br at the end of body) may be better

        // ---- bodyBlock
        b->SetInsertPoint(bodyBlock);
        b->CreateStore(b->CreateLoad(phiInputPtr), phiOutputPtr);

        phiCopiedSize->addIncoming(b->CreateAdd(phiCopiedSize, b->getSize(stepSize)), b->GetInsertBlock());

        Value *newInputPtr = nullptr, *newOutputPtr = nullptr;

        if (targetPtrBitWidth / 8 == stepSize) {
            newInputPtr = b->CreateGEP(phiInputPtr, SIZE_1);
            newOutputPtr = b->CreateGEP(phiOutputPtr, SIZE_1);
        } else {
            newInputPtr = b->CreatePointerCast(
                    b->CreateGEP(b->CreatePointerCast(phiInputPtr, i8PtrTy), b->getSize(stepSize)),
                    targetPtrTy
            );

            newOutputPtr = b->CreatePointerCast(
                    b->CreateGEP(b->CreatePointerCast(phiOutputPtr, i8PtrTy), b->getSize(stepSize)),
                    targetPtrTy
            );

        }

        phiInputPtr->addIncoming(newInputPtr, b->GetInsertBlock());
        phiOutputPtr->addIncoming(newOutputPtr, b->GetInsertBlock());
        b->CreateBr(conBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
    }

    void LZ4ParallelByteStreamAioKernel::generateSimdLiteralCopyByScatter(const std::unique_ptr<KernelBuilder> &b,
                                                                          llvm::Value *literalStartVec,
                                                                          llvm::Value *literalLengthVec,
                                                                          llvm::Value *outputPosVec) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* i64LiteralCopyBlock = b->CreateBasicBlock("i64LiteralCopyBlock");
        BasicBlock* i8LiteralCopyBlock = b->CreateBasicBlock("i8LiteralCopyBlock");

        llvm::Value* initCopiedLength = ConstantVector::getNullValue(literalLengthVec->getType());

        Value* inputBasePtr = b->CreatePointerCast(b->getRawInputPointer("byteStream", b->getSize(0)), b->getInt8PtrTy());
        Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream", b->getSize(0)), b->getInt8PtrTy());

        Value* outputCapacity = b->getCapacity("outputStream");
        Value* outputPosRemVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(outputCapacity))));
        Value* outputPosRemBlockSizeVec = b->simd_and(outputPosVec, b->simd_fill(SIMD_WIDTH, b->CreateNot(b->CreateNeg(b->getIntN(SIMD_WIDTH, mLz4BlockSize)))));
        Value* remainingBlockSizeVec = b->simd_sub(SIMD_WIDTH, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, mLz4BlockSize)), outputPosRemBlockSizeVec);

        Value* possibleExceedBuffer = b->simd_uge(SIMD_WIDTH, b->simd_add(SIMD_WIDTH, literalLengthVec, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 8))), remainingBlockSizeVec);
        // true ffff, false 0000

        // TODO handle literalLengthVec == 0

        b->CreateUnlikelyCondBr(
                b->CreateICmpNE(b->CreateBitCast(possibleExceedBuffer, b->getIntNTy(b->getBitBlockWidth())), b->getIntN(b->getBitBlockWidth(), 0)),
                i8LiteralCopyBlock,
                i64LiteralCopyBlock
        );

        // ---- i8LiteralCopyBlock
        b->SetInsertPoint(i8LiteralCopyBlock);
        this->generateSimdSequentialLiteralCopy(b, literalStartVec, literalLengthVec, outputPosVec);
        b->CreateBr(exitBlock);

        // ---- i64LiteralCopyBlock
        b->SetInsertPoint(i64LiteralCopyBlock);

        BasicBlock* i64LiteralCopyConBlock = b->CreateBasicBlock("i64LiteralCopyConBlock");
        BasicBlock* i64LiteralCopyBodyBlock = b->CreateBasicBlock("i64LiteralCopyBodyBlock");


        b->CreateBr(i64LiteralCopyConBlock);

        // ---- i64LiteralCopyConBlock
        b->SetInsertPoint(i64LiteralCopyConBlock);
        PHINode* phiCopiedLength = b->CreatePHI(initCopiedLength->getType(), 2);
        phiCopiedLength->addIncoming(initCopiedLength, i64LiteralCopyBlock);

        Value* shouldCopiedBitBlock = b->simd_ult(SIMD_WIDTH, phiCopiedLength, literalLengthVec);
//        b->CallPrintRegister("phiCopiedLength", phiCopiedLength);
//        b->CallPrintRegister("literalLengthVec", literalLengthVec);
//        b->CallPrintRegister("shouldCopiedBitBlock", shouldCopiedBitBlock);
        Value* shouldCopiedI1 = b->CreateICmpNE(
                b->CreateBitCast(shouldCopiedBitBlock, b->getIntNTy(b->getBitBlockWidth())),
                b->getIntN(b->getBitBlockWidth(), 0)
        );


        b->CreateCondBr(shouldCopiedI1, i64LiteralCopyBodyBlock, exitBlock);


        // ---- i64LiteralCopyBodyBlock
        b->SetInsertPoint(i64LiteralCopyBodyBlock);
        Value* literalData = this->simdFetchI64DataByGather(b, inputBasePtr, b->simd_add(SIMD_WIDTH, literalStartVec, phiCopiedLength), shouldCopiedBitBlock);

        this->simdPutData(
                b,
                outputBasePtr,
                b->simd_add(SIMD_WIDTH, outputPosRemVec, phiCopiedLength),
                literalData,
                shouldCopiedBitBlock
        );
//        b->CallPrintRegister("phiCopiedLength", phiCopiedLength);
        phiCopiedLength->addIncoming(
                b->simd_add(
                        SIMD_WIDTH,
                        phiCopiedLength,
                        b->simd_and(
                                b->simd_fill(
                                        SIMD_WIDTH,
                                        b->getIntN(SIMD_WIDTH, 8)
                                ),
                                shouldCopiedBitBlock
                        )

                ),
                b->GetInsertBlock()
        );

        b->CreateBr(i64LiteralCopyConBlock);

        b->SetInsertPoint(exitBlock);
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

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask) {
//        return this->simdFetchDataByLoop(b, basePtr, offsetVec, mask);
        if (AVX2_available() && mEnableGather) {
            return this->simdFetchI32DataByGather(b, basePtr, offsetVec, mask);
        } else {
            return this->simdFetchDataByLoop(b, basePtr, offsetVec, mask);
        }
    }



    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchByteData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec, llvm::Value* mask) {
        if (AVX2_available() && mEnableGather) {
            return b->simd_and(this->simdFetchI32DataByGather(b, basePtr, offsetVec, mask), b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0xff)));
        } else {
            return this->simdFetchDataByLoop(b, basePtr, offsetVec, mask, 8);
        }
    }

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchI32DataByGather(const std::unique_ptr<KernelBuilder> &b,
                                                                          llvm::Value *basePtr, llvm::Value *offsetVec,
                                                                          llvm::Value *mask) {

        Function *gatherFunc2 = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_d);
        Function *gatherFunc3 = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_d_256);

        Function *gatherFunc = AVX512BW_available() ? gatherFunc3 : gatherFunc2;

        Type* i32BitBlockTy = VectorType::get(b->getInt32Ty(), b->getBitBlockWidth() / SIMD_WIDTH);


        Value* tokenValuesVec =  b->CreateCall(
                gatherFunc,
                {
                        UndefValue::get(i32BitBlockTy),
                        basePtr,
                        b->CreateTrunc(offsetVec, i32BitBlockTy),
                        b->CreateTrunc(mask, i32BitBlockTy),
                        b->getInt8(1)
                }
        );
        tokenValuesVec = b->CreateZExt(tokenValuesVec, b->getBitBlockType());
        tokenValuesVec = b->CreateAnd(tokenValuesVec, mask);

        return tokenValuesVec;
    }

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchI64DataByGather(const std::unique_ptr<KernelBuilder> &b,
                                                                          llvm::Value *basePtr, llvm::Value *offsetVec,
                                                                          llvm::Value *mask) {
        Type* i32BitBlockTy = VectorType::get(b->getInt32Ty(), b->getBitBlockWidth() / SIMD_WIDTH);
        if (AVX512BW_available()) {
            // AVX512 gather use i8 mask
            //declare <8 x int> @llvm.x86.avx512.gather.dpq.512(<8 x i64>, i8*, <8 x i32>, i8, i32) #1
            Function *gatherFunc512 = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx512_gather_dpq_512);
            return b->CreateCall(
                    gatherFunc512,
                    {
                            UndefValue::get(b->getBitBlockType()),
                            basePtr,
                            b->CreateTrunc(offsetVec, i32BitBlockTy),
                            b->CreateTruncOrBitCast(b->hsimd_signmask(SIMD_WIDTH, mask), b->getInt8Ty()),
                            b->getInt32(1)
                    });
            // return result & i512Mask ?
        } else {
            // AVX2 gather use i256 mask
            Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256);
            Value* tokenValuesVec =  b->CreateCall(
                    gatherFunc,
                    {
                            UndefValue::get(b->getBitBlockType()),
                            basePtr,
                            b->CreateTrunc(offsetVec, VectorType::get(b->getInt32Ty(), 4)),
                            mask,
                            b->getInt8(1)
                    }
            );
            tokenValuesVec = b->CreateAnd(tokenValuesVec, mask);
            return tokenValuesVec;
        }
    }

    llvm::Value* LZ4ParallelByteStreamAioKernel::simdFetchDataByLoop(const std::unique_ptr<KernelBuilder> &b,
                                                                     llvm::Value *basePtr, llvm::Value *offsetVec,
                                                                     llvm::Value *maskVec, unsigned resultBitWidth) {
        Type* INT_TY = b->getIntNTy(resultBitWidth);
        Type* INT_PTR_TY = INT_TY->getPointerTo();

        Value* placeHolderPtr = b->CreatePointerCast(b->getScalarFieldPtr("placeholder"), INT_PTR_TY);
        Value* retVec = ConstantVector::getNullValue(b->getBitBlockType());

        for (uint64_t i = 0; i < b->getBitBlockWidth() / SIMD_WIDTH; i++){
            Value* mask = b->CreateExtractElement(maskVec, i);
            Value* shouldLoad = b->CreateICmpNE(mask, b->getInt64(0));
            Value* loadPtr = b->CreateSelect(shouldLoad, b->CreatePointerCast(b->CreateGEP(basePtr, b->CreateExtractElement(offsetVec, i)), INT_PTR_TY), placeHolderPtr);
//            Value* loadPtr = b->CreateGEP(basePtr, b->CreateExtractElement(offsetVec, i));
            Value* loadValue = b->CreateZExt(b->CreateLoad(loadPtr), b->getInt64Ty());
            Value* finalValue = b->CreateSelect(shouldLoad, loadValue, b->getInt64(0));
            retVec = b->CreateInsertElement(retVec, finalValue, i);
        }

        return retVec;
    }

    void LZ4ParallelByteStreamAioKernel::simdPutData(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec,llvm::Value* values, llvm::Value* mask) {
        if (AVX512BW_available()) {
            this->simdPutDataByScatter(b, basePtr, offsetVec, values, mask);
        } else {
            this->simdPutDataByLoop(b, basePtr, offsetVec, values, mask);
        }


    }

    void LZ4ParallelByteStreamAioKernel::simdPutDataByLoop(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec,llvm::Value* values, llvm::Value* mask) {

        Value* shouldStoreVec = b->CreateICmpNE(mask, b->simd_fill(SIMD_WIDTH, b->getIntN(SIMD_WIDTH, 0)));

        for (unsigned i = 0 ; i < b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            BasicBlock* conBlock = b->CreateBasicBlock("simdPutDataByLoopCon");
            BasicBlock* bodyBlock = b->CreateBasicBlock("simdPutDataByLoopBody");
            BasicBlock* exitBlock = b->CreateBasicBlock("simdPutDataByLoopExit");

            b->CreateBr(conBlock);

            // ---- ConBlock
            b->SetInsertPoint(conBlock);
            Value* shouldStore = b->CreateExtractElement(shouldStoreVec, i);
            b->CreateCondBr(shouldStore, bodyBlock, exitBlock);

            // ---- BodyBlock
            b->SetInsertPoint(bodyBlock);
            b->CreateStore(
                    b->CreateExtractElement(values, i),
                    b->CreatePointerCast(b->CreateGEP(basePtr, b->CreateExtractElement(offsetVec, i)), b->getIntNTy(SIMD_WIDTH)->getPointerTo())
            );

            b->CreateBr(exitBlock);

            // ---- ExitBlock
            b->SetInsertPoint(exitBlock);

        }
    }
    void LZ4ParallelByteStreamAioKernel::simdPutDataByScatter(const std::unique_ptr<KernelBuilder> &b, llvm::Value* basePtr, llvm::Value* offsetVec,llvm::Value* values, llvm::Value* mask) {
        Function *scatterFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx512_scatter_dpq_512);
        //declare void @llvm.x86.avx512.scatter.dpq.512(i8*, i8, <8 x i32>, <8 x i64>, i32)

        Value* i8Mask = b->CreateTruncOrBitCast(b->hsimd_signmask(SIMD_WIDTH, mask), b->getIntNTy(b->getBitBlockWidth() / SIMD_WIDTH));

        b->CreateCall(scatterFunc, {
                basePtr,
                i8Mask,
                b->CreateTrunc(offsetVec, VectorType::get(b->getInt32Ty(), b->getBitBlockWidth() / SIMD_WIDTH)),
                values,
                b->getInt32(1)
        });

    }


    void LZ4ParallelByteStreamAioKernel::initParallelLevelMeasurement(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i <= b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            addScalar(b->getSizeTy(), "ParallelLevel" + std::to_string(i));
        }
    }
    void LZ4ParallelByteStreamAioKernel::recordParallelLevel(const std::unique_ptr<KernelBuilder> &b, llvm::Value* v) {
        for (unsigned i = 0; i <= b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            auto name = "ParallelLevel" + std::to_string(i);
            Value* addValue = b->CreateZExt(b->CreateICmpEQ(v, b->getInt32(i)), b->getSizeTy());
            b->setScalarField(name, b->CreateAdd(b->getScalarField(name), addValue));
        }
    }
    void LZ4ParallelByteStreamAioKernel::printParallelLevelResult(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i <= b->getBitBlockWidth() / SIMD_WIDTH; i++) {
            auto name = "ParallelLevel" + std::to_string(i);
            b->CallPrintInt(name, b->getScalarField(name));
        }
    }

}