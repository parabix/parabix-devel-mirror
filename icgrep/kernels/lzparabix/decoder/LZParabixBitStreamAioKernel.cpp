
#include "LZParabixBitStreamAioKernel.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel{

    LZParabixBitStreamAioKernel::LZParabixBitStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                             vector<unsigned int, allocator<unsigned int>> numsOfBitStreams)
            : LZParabixAioBaseKernel(b, "LZParabixBitStreamAioKernel"), mNumsOfBitStreams(numsOfBitStreams) {

        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[0], 1), "inputBitStream0", BoundedRate(0, 1), AlwaysConsume()});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[0], 1), "outputStream0", BoundedRate(0, 1)});

        for (unsigned i = 1; i < numsOfBitStreams.size(); i++) {
            mStreamSetInputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[i], 1), "inputBitStream" + std::to_string(i), RateEqualTo("inputBitStream0"), AlwaysConsume()});
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numsOfBitStreams[i], 1), "outputStream" + std::to_string(i), RateEqualTo("outputStream0")});
        }

        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            this->addScalar(b->getInt64Ty()->getPointerTo(), "currentOutputPtr_" + std::to_string(i));
        }
    }

    void LZParabixBitStreamAioKernel::initDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* ptr = b->CreatePointerCast(b->getOutputStreamBlockPtr("outputStream" + std::to_string(i), b->getSize(0)), b->getInt64Ty()->getPointerTo());
            b->setScalarField("currentOutputPtr_" + std::to_string(i), ptr);

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                b->CreateStore(b->getInt64(0), b->CreateGEP(ptr, b->getInt32(j * 4)));
            }
        }
    }

    llvm::Value* LZParabixBitStreamAioKernel::isAllItemAvailable(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalLength) {
        Value* literalStartPos = b->getProcessedItemCount("inputBitStream0");
        Value* literalEndPos = b->CreateAdd(literalStartPos, literalLength);

        Value* allItemAvailable = b->getInt1(true);
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            allItemAvailable = b->CreateAnd(allItemAvailable, b->CreateICmpULE(literalEndPos, b->getAvailableItemCount("inputBitStream" + std::to_string(i))));
        }
        return allItemAvailable;
    }

    void LZParabixBitStreamAioKernel::appendBitStreamOutput(const std::unique_ptr<KernelBuilder> &b, std::vector<llvm::Value*>& extractedValues, llvm::Value* valueLength) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));

        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* outputPtr = b->getScalarField("currentOutputPtr_" + std::to_string(i));
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(outputPtr, b->getSize(j * 4));
                Value* newValue = b->CreateOr(b->CreateLoad(ptr), b->CreateShl(extractedValues[iStreamIndex], oldOutputPosRem64));
                b->CreateStore(newValue, ptr);
                ++iStreamIndex;
            }
        }

        BasicBlock* exceedOutputBlock =b->CreateBasicBlock("exceedOutputBlock");
        b->CreateCondBr(b->CreateICmpULT(b->CreateAdd(oldOutputPosRem64, valueLength), b->getSize(64)), exitBlock, exceedOutputBlock);

        // ---- storeOutputBlock
        b->SetInsertPoint(exceedOutputBlock);

        Value* shiftAmount = b->CreateSub(b->getSize(0x40), oldOutputPosRem64);
        Value* fullyShift = b->CreateICmpEQ(shiftAmount, b->getSize(0x40));

        Value* exceedBlock = b->CreateICmpUGE(b->CreateAdd(b->CreateURem(oldOutputPos, b->getSize(b->getBitBlockWidth())), valueLength), b->getSize(b->getBitBlockWidth()));
        iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* oldOutputPtr = b->getScalarField("currentOutputPtr_" + std::to_string(i));
            Value* distance = b->CreateSelect(exceedBlock, b->getSize(1 + (mNumsOfBitStreams[i] - 1) * b->getBitBlockWidth() / 64), b->getSize(1));
            Value* newOutputPtr = b->CreateGEP(oldOutputPtr, distance);
            b->setScalarField("currentOutputPtr_" + std::to_string(i), newOutputPtr);
            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* newValue = b->CreateSelect(fullyShift, b->getInt64(0), b->CreateLShr(extractedValues[iStreamIndex], shiftAmount));
                Value* ptr = b->CreateGEP(newOutputPtr, b->getSize(j * 4));
                b->CreateStore(newValue, ptr);
                ++iStreamIndex;
            }
        }

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setScalarField("outputPos", b->CreateAdd(oldOutputPos, valueLength));
    }


    void LZParabixBitStreamAioKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos,
                                               llvm::Value *literalLength) {
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* remCursorPos = b->CreateURem(cursorPos, b->getCapacity("inputBitStream0"));

        BasicBlock* processLiteralConBlock = b->CreateBasicBlock("processLiteralConBlock");
        BasicBlock* processLiteralBodyBlock = b->CreateBasicBlock("processLiteralBodyBlock");
        BasicBlock* processLiteralExitBlock = b->CreateBasicBlock("processLiteralExitBlock");

        b->CreateBr(processLiteralConBlock);

        // ---- processLiteralConBlock
        b->SetInsertPoint(processLiteralConBlock);
        PHINode* phiRemCursorPos = b->CreatePHI(b->getSizeTy(), 2);
        phiRemCursorPos->addIncoming(remCursorPos, entryBlock);
        PHINode* phiRemainingLiteralLength = b->CreatePHI(b->getSizeTy(), 2);
        phiRemainingLiteralLength->addIncoming(literalLength, entryBlock);

        b->CreateCondBr(b->CreateICmpUGT(phiRemainingLiteralLength, b->getSize(0)), processLiteralBodyBlock, processLiteralExitBlock);

        // ---- processLiteralBodyBlock
        b->SetInsertPoint(processLiteralBodyBlock);

        Value* targetLiteralLength = b->CreateSub(b->getSize(64), b->CreateURem(phiRemCursorPos, b->getSize(64)));
        targetLiteralLength = b->CreateUMin(phiRemainingLiteralLength, targetLiteralLength);


        Value* cursorBlockIndex = b->CreateUDiv(phiRemCursorPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorBlockRem = b->CreateURem(phiRemCursorPos, b->getSize(b->getBitBlockWidth()));
        Value* cursorI64BlockIndex = b->CreateUDiv(cursorBlockRem, b->getSize(64));
        Value* cursorI64BlockRem = b->CreateURem(cursorBlockRem, b->getSize(64));
        Value* literalMask = b->CreateSub(
                b->CreateSelect(b->CreateICmpEQ(targetLiteralLength, b->getInt64(0x40)), b->getInt64(0), b->CreateShl(b->getInt64(1), targetLiteralLength)),
                b->getInt64(1)
        );

        std::vector<llvm::Value*> extractValues;

        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawInputPointer("inputBitStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* targetBlockBasePtr = b->CreatePointerCast(b->CreateGEP(bitStreamBasePtr, b->CreateMul(cursorBlockIndex, b->getSize(mNumsOfBitStreams[i]))), b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(targetBlockBasePtr, b->CreateAdd(cursorI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* extractV = b->CreateLShr(b->CreateLoad(ptr), cursorI64BlockRem);
                extractV = b->CreateAnd(extractV, literalMask);
                extractValues.push_back(extractV);
            }
        }

        this->appendBitStreamOutput(b, extractValues, targetLiteralLength);

        phiRemCursorPos->addIncoming(b->CreateURem(b->CreateAdd(phiRemCursorPos, targetLiteralLength), b->getCapacity("inputBitStream0")), b->GetInsertBlock());
        phiRemainingLiteralLength->addIncoming(b->CreateSub(phiRemainingLiteralLength, targetLiteralLength), b->GetInsertBlock());

        b->CreateBr(processLiteralConBlock);

        // ---- processLiteralExitBlock
        b->SetInsertPoint(processLiteralExitBlock);
    }

    void LZParabixBitStreamAioKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value* cursorPos, llvm::Value* matchOffset, llvm::Value* matchMask) {
        Constant * PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
        Value* remCursorPos = b->CreateURem(cursorPos, b->getCapacity("outputStream0"));
        Value* cursorPosI64BlockIndex = b->CreateUDiv(remCursorPos, b->getSize(64));

        Value* remOutputPos = b->CreateURem(b->getScalarField("outputPos"), b->getCapacity("outputStream0"));
        Value* outputPosI64BlockIndex = b->CreateUDiv(remOutputPos, b->getSize(64));

        Value* matchCopyFromI64BlockIndex = b->CreateSub(outputPosI64BlockIndex, matchOffset);

        Value* matchCopyFromBitBlockIndex = b->CreateUDiv(matchCopyFromI64BlockIndex, b->getSize(4));
        matchCopyFromI64BlockIndex = b->CreateURem(matchCopyFromI64BlockIndex, b->getSize(4));

        std::vector<llvm::Value*> extractValues;
        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumsOfBitStreams.size(); i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputStream" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* outputBlockBasePtr = b->CreateGEP(outputBasePtr, b->CreateMul(matchCopyFromBitBlockIndex, b->getSize(mNumsOfBitStreams[i])));
            outputBlockBasePtr = b->CreatePointerCast(outputBlockBasePtr, b->getInt64Ty()->getPointerTo());

            for (unsigned j = 0; j < mNumsOfBitStreams[i]; j++) {
                Value* ptr = b->CreateGEP(outputBlockBasePtr, b->CreateAdd(matchCopyFromI64BlockIndex, b->getSize(j * (b->getBitBlockWidth() / 64))));
                Value* value = b->CreateLoad(ptr);
                Value * extractV = b->CreateCall(PEXT_func, {value, matchMask});
                extractValues.push_back(extractV);
                ++iStreamIndex;
            }
        }

        this->appendBitStreamOutput(b, extractValues, b->CreatePopcount(matchMask));
    }

    void
    LZParabixBitStreamAioKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value *c) {
        b->setProducedItemCount("outputStream0", c);
    }


    llvm::Value* LZParabixBitStreamAioKernel::getProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b) {
        return b->getProcessedItemCount("inputBitStream0");
    }
    void LZParabixBitStreamAioKernel::setProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) {
        b->setProcessedItemCount("inputBitStream0", c);
    }
}