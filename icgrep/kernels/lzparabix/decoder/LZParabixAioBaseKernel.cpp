
#include "LZParabixAioBaseKernel.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{
    LZParabixAioBaseKernel::LZParabixAioBaseKernel(const std::unique_ptr<kernel::KernelBuilder> &b, std::string &&name)
            : SegmentOrientedKernel(std::move(name),
            // Inputs
                                    {
                                            Binding{b->getStreamSetTy(1, 8), "byteStream", BoundedRate(0, 1)},

                                            // block data
                                            Binding{b->getStreamSetTy(1, 64), "blockStart", BoundedRate(0, 1),
                                                    AlwaysConsume()},
                                            Binding{b->getStreamSetTy(1, 64), "blockEnd", RateEqualTo("blockStart"),
                                                    AlwaysConsume()}
                                    },
            //Outputs
                                    {},
            //Arguments
                                    {
                                            Binding{b->getSizeTy(), "fileSize"}
                                    },
                                    {},
            //Internal states:
                                    {
                                            Binding{b->getSizeTy(), "blockDataIndex"},
                                            Binding{b->getInt64Ty(), "outputPos"}

                                    }) {
        this->setStride(4 * 1024 * 1024);
        addAttribute(MustExplicitlyTerminate());
    }

    void LZParabixAioBaseKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        this->initDoSegmentMethod(b);


        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");
        BasicBlock* blockEndConBlock = b->CreateBasicBlock("blockEndConBlock");

        Value * blockDataIndex = b->getScalarField("blockDataIndex");
        Value * totalNumber = b->getAvailableItemCount("blockEnd");

        Value * blockEnd = this->generateLoadInt64NumberInput(b, "blockEnd", blockDataIndex);

        b->CreateCondBr(b->CreateICmpULT(blockDataIndex, totalNumber), blockEndConBlock, exitBlock);

        b->SetInsertPoint(blockEndConBlock);

        Value * blockStart = this->generateLoadInt64NumberInput(b, "blockStart", blockDataIndex);

        Value* literalLengthPtr = b->getRawInputPointer("byteStream", blockStart);
        literalLengthPtr = b->CreatePointerCast(literalLengthPtr, b->getInt32Ty()->getPointerTo());

        Value* totalLiteralLength = b->CreateZExtOrBitCast(b->CreateLoad(literalLengthPtr), b->getSizeTy());

        BasicBlock * processBlock = b->CreateBasicBlock("processBlock");
        b->CreateCondBr(this->isAllItemAvailable(b, totalLiteralLength), processBlock, exitBlock);

        b->SetInsertPoint(processBlock);

        this->generateProcessCompressedBlock(b, blockStart, blockEnd);

        Value * newBlockDataIndex = b->CreateAdd(blockDataIndex, b->getInt64(1));
        b->setScalarField("blockDataIndex", newBlockDataIndex);
        b->setProcessedItemCount("blockStart", newBlockDataIndex);
        b->setProcessedItemCount("byteStream", blockEnd);

        this->setProducedOutputItemCount(b, b->getScalarField("outputPos"));
        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
    }


    llvm::Value *LZParabixAioBaseKernel::generateLoadInt64NumberInput(const std::unique_ptr<KernelBuilder> &iBuilder,
                                                                      std::string inputBufferName, llvm::Value *globalOffset) {

        Value * capacity = iBuilder->getCapacity(inputBufferName);
        Value * processed = iBuilder->getProcessedItemCount(inputBufferName);
        processed = iBuilder->CreateAnd(processed, iBuilder->CreateNeg(capacity));
        Value * offset = iBuilder->CreateSub(globalOffset, processed);
        Value * valuePtr = iBuilder->getRawInputPointer(inputBufferName, offset);
        return iBuilder->CreateLoad(valuePtr);
    }

    void LZParabixAioBaseKernel::generateProcessCompressedBlock(const std::unique_ptr<KernelBuilder> &b,
                                                            llvm::Value *lz4BlockStart, llvm::Value *lz4BlockEnd) {
        Value* literalLengthPtr = b->getRawInputPointer("byteStream", lz4BlockStart);
        literalLengthPtr = b->CreatePointerCast(literalLengthPtr, b->getInt32Ty()->getPointerTo());
        Value* totalLiteralLength = b->CreateZExtOrBitCast(b->CreateLoad(literalLengthPtr), b->getSizeTy());

        Value* literalStartPos = this->getProcessedInputItemCount(b);

        Value* tokenStartPos = b->CreateAdd(b->CreateAdd(lz4BlockStart, b->getSize(4)), totalLiteralLength);

        Value* isTerminal = b->CreateICmpEQ(lz4BlockEnd, b->getScalarField("fileSize"));
        b->setTerminationSignal(isTerminal);

        BasicBlock* exitBlock = b->CreateBasicBlock("processCompressedExitBlock");

        BasicBlock* processCon = b->CreateBasicBlock("processCompressedConBlock");
        BasicBlock* processBody = b->CreateBasicBlock("processCompressedBodyBlock");

        BasicBlock* beforeProcessConBlock = b->GetInsertBlock();
        b->CreateBr(processCon);
        b->SetInsertPoint(processCon);

        PHINode* phiLiteralCursorValue = b->CreatePHI(b->getInt64Ty(), 2);
        phiLiteralCursorValue->addIncoming(literalStartPos, beforeProcessConBlock);

        PHINode* phiTokenCursorValue = b->CreatePHI(b->getInt64Ty(), 2); // phiCursorValue should always be the position of next token except for the final sequence
        phiTokenCursorValue->addIncoming(tokenStartPos, beforeProcessConBlock);

        b->CreateCondBr(b->CreateICmpULT(phiTokenCursorValue, lz4BlockEnd), processBody, exitBlock);

        b->SetInsertPoint(processBody);

        auto ret = this->processSequence(b, phiLiteralCursorValue, phiTokenCursorValue, lz4BlockEnd);

        Value* nextLiteralGlobalPos = ret.first;
        Value* nextTokenGlobalPos = ret.second;

        phiLiteralCursorValue->addIncoming(nextLiteralGlobalPos, b->GetInsertBlock());
        phiTokenCursorValue->addIncoming(nextTokenGlobalPos, b->GetInsertBlock());
        b->CreateBr(processCon);

        b->SetInsertPoint(exitBlock);
        this->setProcessedInputItemCount(b, b->CreateAdd(literalStartPos, totalLiteralLength));
    }

    std::pair<llvm::Value *, llvm::Value *>
    LZParabixAioBaseKernel::processSequence(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalCursorPos,
                                        llvm::Value *tokenCursorPos,
                                        llvm::Value *lz4BlockEnd) {

        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* sequenceBasePtr = b->getRawInputPointer("byteStream", tokenCursorPos);
        Value* sequenceToken = b->CreateLoad(sequenceBasePtr);

        Value* highestTokenBit = b->CreateAnd(sequenceToken, b->getInt8((uint8_t)1 << 7));
        Value* isLiteral = b->CreateICmpNE(highestTokenBit, b->getInt8(0));
        Value* tokenNumValue = b->CreateZExt(b->CreateSub(sequenceToken, highestTokenBit), b->getSizeTy());


        BasicBlock* literalProcessBlock = b->CreateBasicBlock("literalProcessBlock");
        BasicBlock* matchProcessBlock = b->CreateBasicBlock("matchProcessBlock");
        b->CreateCondBr(isLiteral, literalProcessBlock, matchProcessBlock);

        // ---- literalProcessBlock
        b->SetInsertPoint(literalProcessBlock);
        this->doLiteralCopy(b, literalCursorPos, tokenNumValue);
        Value* newTokenCursorPosAfterLiteral = b->CreateAdd(tokenCursorPos, b->getSize(1));
        Value* newLiteralCursorPosAfterLiteral = b->CreateAdd(literalCursorPos, tokenNumValue);


        BasicBlock* literalProcessFinalBlock = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- matchProcessBlock
        b->SetInsertPoint(matchProcessBlock);
        Value* tokenCursorNextPos = b->CreateAdd(tokenCursorPos, b->getSize(1));
        Value* matchIndexBytes = this->processMatch(b, tokenCursorNextPos, tokenNumValue, sequenceBasePtr);
        Value* newTokenCursorPosAfterMatch = b->CreateAdd(tokenCursorNextPos, matchIndexBytes);

        BasicBlock* matchProcessFinalBlock = b->GetInsertBlock();
        b->CreateBr(exitBlock);

        // ---- exitBlock
        b->SetInsertPoint(exitBlock);
        PHINode* phiTokenCursorValue = b->CreatePHI(b->getSizeTy(), 2);
        phiTokenCursorValue->addIncoming(newTokenCursorPosAfterLiteral, literalProcessFinalBlock);
        phiTokenCursorValue->addIncoming(newTokenCursorPosAfterMatch, matchProcessFinalBlock);

        PHINode* phiLiteralCursorValue = b->CreatePHI(b->getSizeTy(), 2);
        phiLiteralCursorValue->addIncoming(newLiteralCursorPosAfterLiteral, literalProcessFinalBlock);
        phiLiteralCursorValue->addIncoming(literalCursorPos, matchProcessFinalBlock);

        return std::make_pair(phiLiteralCursorValue, phiTokenCursorValue);
    }

    llvm::Value* LZParabixAioBaseKernel::processMatch(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* sequenceBasePtr) {
        Function* pdep = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64);


        Value* pdepMask = b->getInt64(0x0101010101010101);

        Value* oneBitIndex = b->CreateZExt(b->CreateLoad(b->CreateGEP(sequenceBasePtr, b->getSize(1))), b->getInt64Ty());
        Value* zeroBitIndex = b->CreateZExt(b->CreateLoad(b->CreateGEP(sequenceBasePtr, b->getSize(2))), b->getInt64Ty());
        Value* matchMaskPtr = b->CreatePointerCast(b->CreateGEP(sequenceBasePtr, b->getSize(3)), b->getInt64Ty()->getPointerTo());
        Value* matchMask = b->CreateLoad(matchMaskPtr);

        Value* fullOneBitIndex = b->CreateCall(pdep, {oneBitIndex, pdepMask});
        Value* fullZeroBitIndex = b->CreateCall(pdep, {zeroBitIndex, pdepMask});


        Value* remainingMask = b->CreateNot(b->CreateMul(b->CreateOr(fullOneBitIndex, fullZeroBitIndex), b->getInt64(0xff)));
        Value* fullMatchMask = b->CreateOr(
                b->CreateMul(fullOneBitIndex, b->getInt64(0xff)),
                b->CreateCall(pdep, {matchMask, remainingMask})
        );

        this->doMatchCopy(b, cursorPos, matchOffset, fullMatchMask);

        return b->CreateAdd(b->CreateSub(b->getSize(8), b->CreatePopcount(b->CreateOr(oneBitIndex, zeroBitIndex))), b->getSize(2));
    }

    llvm::Value* LZParabixAioBaseKernel::isAllItemAvailable(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalLength) {
        return b->getInt1(true);
    }


}