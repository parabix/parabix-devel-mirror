
#include "LZParabixSwizzledAioKernel.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;

namespace kernel{

    LZParabixSwizzledAioKernel::LZParabixSwizzledAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b)
    : LZParabixAioBaseKernel(b, "LZParabixSwizzledAioKernel") {
//        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(8, 1), "inputBitStream", BoundedRate(0, 1), AlwaysConsume()});
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(8, 1), "inputSwizzled0", BoundedRate(0, 1), AlwaysConsume()});
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(8, 1), "inputSwizzled1", RateEqualTo("inputSwizzled0"), AlwaysConsume()});

        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(4, 1), "outputSwizzle0", BoundedRate(0, 1)/*, Swizzled()*/});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(4, 1), "outputSwizzle1", RateEqualTo("outputSwizzle0")/*, Swizzled()*/});

        for (unsigned i = 0; i < 2; i++) {
            this->addScalar(b->getBitBlockType()->getPointerTo(), "currentOutputPtr" + std::to_string(i));
        }
    }

    void LZParabixSwizzledAioKernel::initDoSegmentMethod(const std::unique_ptr<KernelBuilder> &b) {
        for (unsigned i = 0; i < 2; i++) {
            Value* ptr = b->CreatePointerCast(b->getOutputStreamBlockPtr("outputSwizzle" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            b->setScalarField("currentOutputPtr" + std::to_string(i), ptr);
            b->CreateStore(ConstantVector::getNullValue(b->getBitBlockType()), ptr);
        }
    }

    llvm::Value* LZParabixSwizzledAioKernel::isAllItemAvailable(const std::unique_ptr<KernelBuilder> &b, llvm::Value* literalLength) {
        Value* literalStartPos = b->getProcessedItemCount("inputSwizzled0");
        Value* literalEndPos = b->CreateAdd(literalStartPos, literalLength);
        return b->CreateICmpULE(literalEndPos, b->getAvailableItemCount("inputSwizzled0"));
    }

    void LZParabixSwizzledAioKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos,
                                                   llvm::Value *literalLength) {
        // TODO continue here
        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        Value* remCursorPos = b->CreateURem(cursorPos, b->getCapacity("inputSwizzled0"));

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

        Value* cursorI64BlockIndex = b->CreateUDiv(phiRemCursorPos, b->getSize(64));
        Value* cursorI64BlockRem = b->CreateURem(phiRemCursorPos, b->getSize(64));
        Value* literalMask = b->CreateSub(
                b->CreateSelect(b->CreateICmpEQ(targetLiteralLength, b->getInt64(0x40)), b->getInt64(0), b->CreateShl(b->getInt64(1), targetLiteralLength)),
                b->getInt64(1)
        );


        std::vector<llvm::Value*> swizzledValues;

        for (unsigned i = 0; i < 2; i++) {
            Value* bitStreamBasePtr = b->CreatePointerCast(b->getRawInputPointer("inputSwizzled" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* targetBlockBasePtr = b->CreateGEP(bitStreamBasePtr, cursorI64BlockIndex);

            Value* swizzledValue = b->simd_srlv(64, b->CreateLoad(targetBlockBasePtr), b->simd_fill(64, cursorI64BlockRem));
            swizzledValue = b->simd_and(swizzledValue, b->simd_fill(64, literalMask));
            swizzledValues.push_back(swizzledValue);
        }

        this->appendBitStreamOutput(b, swizzledValues, targetLiteralLength);

        phiRemCursorPos->addIncoming(b->CreateURem(b->CreateAdd(phiRemCursorPos, targetLiteralLength), b->getCapacity("inputSwizzled0")), b->GetInsertBlock());
        phiRemainingLiteralLength->addIncoming(b->CreateSub(phiRemainingLiteralLength, targetLiteralLength), b->GetInsertBlock());

        b->CreateBr(processLiteralConBlock);

        // ---- processLiteralExitBlock
        b->SetInsertPoint(processLiteralExitBlock);
    }

    void LZParabixSwizzledAioKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *cursorPos,
                                                 llvm::Value *matchOffset, llvm::Value *matchMask) {
        Constant * PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
        std::vector<llvm::Value*> swizzledValues;

        // TODO logic here is incorrect, since outputStream here is in swizzled form now
        for (unsigned i = 0; i < 2; i++) {
            Value* outputBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputSwizzle" + std::to_string(i), b->getSize(0)), b->getBitBlockType()->getPointerTo());
            Value* currentOutputPtr = b->getScalarField("currentOutputPtr" + std::to_string(i));
            Value* currentOutputBlockIndex = b->CreatePtrDiff(currentOutputPtr, outputBasePtr);
            Value* targetOutputBlockIndex = b->CreateSub(currentOutputBlockIndex, matchOffset);

            Value* targetPtr = b->CreateGEP(outputBasePtr, targetOutputBlockIndex);
            Value* swizzledValue = b->CreateLoad(targetPtr);

            for (unsigned j = 0; j < 4; j++) {
                Value* value = b->CreateExtractElement(swizzledValue, j);
                Value * extractV = b->CreateCall(PEXT_func, {value, matchMask});
                swizzledValue = b->CreateInsertElement(swizzledValue, extractV, j);
            }
            swizzledValues.push_back(swizzledValue);
        }

        this->appendBitStreamOutput(b, swizzledValues, b->CreatePopcount(matchMask));
    }

    void LZParabixSwizzledAioKernel::appendBitStreamOutput(const unique_ptr<KernelBuilder> &b,
                                                           vector<Value *, allocator<Value *>> &swizzledValues,
                                                           Value *valueLength) {
        BasicBlock* exitBlock = b->CreateBasicBlock("exitBlock");

        Value* oldOutputPos = b->getScalarField("outputPos");
        Value* oldOutputPosRem64 = b->CreateURem(oldOutputPos, b->getSize(64));

        for (unsigned i = 0; i < 2; i++) {
            Value* outputPtr = b->getScalarField("currentOutputPtr" + std::to_string(i));
            Value* newValue = b->simd_or(
                    b->CreateLoad(outputPtr),
                    b->simd_sllv(64, swizzledValues[i], b->simd_fill(64, oldOutputPosRem64))
            );
            b->CreateStore(newValue, outputPtr);
        }

        BasicBlock* exceedOutputBlock =b->CreateBasicBlock("exceedOutputBlock");
        b->CreateCondBr(b->CreateICmpULT(b->CreateAdd(oldOutputPosRem64, valueLength), b->getSize(64)), exitBlock, exceedOutputBlock);

        // ---- storeOutputBlock
        b->SetInsertPoint(exceedOutputBlock);

        Value* shiftAmount = b->CreateSub(b->getSize(0x40), oldOutputPosRem64);
        Value* fullyShift = b->CreateICmpEQ(shiftAmount, b->getSize(0x40));

        for (unsigned i = 0; i < 2; i++) {
            Value* oldOutputPtr = b->getScalarField("currentOutputPtr" + std::to_string(i));
            Value* newOutputPtr = b->CreateGEP(oldOutputPtr, b->getSize(1));

            b->setScalarField("currentOutputPtr" + std::to_string(i), newOutputPtr);
            Value* newValue = b->CreateSelect(
                    fullyShift,
                    ConstantVector::getNullValue(b->getBitBlockType()),
                    b->simd_srlv(64, swizzledValues[i], b->simd_fill(64, shiftAmount))
            );
            b->CreateStore(newValue, newOutputPtr);
        }

        b->CreateBr(exitBlock);

        b->SetInsertPoint(exitBlock);
        b->setScalarField("outputPos", b->CreateAdd(oldOutputPos, valueLength));
    }

    void
    LZParabixSwizzledAioKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value *c) {
        b->setProducedItemCount("outputSwizzle0", c);
    }

    llvm::Value* LZParabixSwizzledAioKernel::getProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b) {
        return b->getProcessedItemCount("inputSwizzled0");
    }
    void LZParabixSwizzledAioKernel::setProcessedInputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* c) {
        b->setProcessedItemCount("inputSwizzled0", c);
    }
}