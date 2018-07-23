//
// Created by wxy325 on 2018/7/19.
//

#include "lz4_i4_bytestream_aio.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel {
    LZ4I4ByteStreamAioKernel::LZ4I4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned blockSize)
            : LZ4SequentialAioBaseKernel(b, "LZ4I4ByteStreamAioKernel", blockSize)
    {
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(1, 4), "inputI4Stream", RateEqualTo("byteStream")});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(1, 4), "outputI4Stream", BoundedRate(0, 1)});
    }

    void LZ4I4ByteStreamAioKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                               llvm::Value *literalLength, llvm::Value* blockStart) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        Value* inputCapacity = b->getCapacity("inputI4Stream");
        Value* outputCapacity = b->getCapacity("outputI4Stream");

        Value* inputByteBasePtr = b->CreatePointerCast(b->getRawInputPointer("inputI4Stream", b->getSize(0)), b->getInt8PtrTy());
        Value* outputByteBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputI4Stream", b->getSize(0)), b->getInt8PtrTy());

        Value* outputPos = b->getScalarField("outputPos");
//        b->CallPrintInt("literalCopy", outputPos);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();


        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");

        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiCopiedLength = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedLength->addIncoming(b->getSize(0), entryBlock);

        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, literalLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);
        Value* copyStart = b->CreateAdd(literalStart, phiCopiedLength);
        Value* copyStartRem = b->CreateURem(copyStart, inputCapacity);
        Value* copyStartRem2 = b->CreateURem(copyStart, b->getSize(2));
        Value* copyStartByteRem = b->CreateUDiv(copyStartRem, b->getSize(2));
        Value* remainingDataLength = b->CreateSub(inputCapacity, copyStartRem);

        Value* inputTargetPtr = b->CreateGEP(inputByteBasePtr, copyStartByteRem);
        inputTargetPtr = b->CreatePointerCast(inputTargetPtr, INT_FW_PTR);

        Value* outputStart = b->CreateAdd(outputPos, phiCopiedLength);
        Value* outputStartRem = b->CreateURem(outputStart, outputCapacity);
        Value* outputStartRem2 = b->CreateURem(outputStartRem, b->getSize(2));
        Value* outputStartByteRem = b->CreateUDiv(outputStartRem, b->getSize(2));
        Value* outputTargetPtr = b->CreateGEP(outputByteBasePtr, outputStartByteRem);
        outputTargetPtr = b->CreatePointerCast(outputTargetPtr, INT_FW_PTR);


        Value* inputTargetValue = b->CreateLoad(inputTargetPtr);
        inputTargetValue = b->CreateLShr(inputTargetValue, b->CreateMul(copyStartRem2, b->getIntN(fw, 4)));


        Value* outputValue = b->CreateLoad(outputTargetPtr);
        Value* outputMask = b->CreateSelect(
                b->CreateICmpEQ(outputStartRem2, b->getSize(1)),
                b->getIntN(fw, 0xf),
                b->getIntN(fw, 0)
        );
        outputValue = b->CreateAnd(outputValue, outputMask);
        inputTargetValue = b->CreateShl(inputTargetValue, b->CreateMul(outputStartRem2, b->getIntN(fw, 4)));
        outputValue = b->CreateOr(outputValue, inputTargetValue);
        b->CreateStore(outputValue, outputTargetPtr);

        Value* newCopyLength = b->getSize(fw / 4);
        newCopyLength = b->CreateSub(newCopyLength, b->CreateOr(outputStartRem2, copyStartRem2));
        newCopyLength = b->CreateUMin(newCopyLength, remainingDataLength);

        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, newCopyLength), b->GetInsertBlock());

        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }

    void LZ4I4ByteStreamAioKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                             llvm::Value *matchLength) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        Value* outputCapacity = b->getCapacity("outputI4Stream");

        Value* outputByteBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputI4Stream", b->getSize(0)), b->getInt8PtrTy());

        Value* outputPos = b->getScalarField("outputPos");
//        b->CallPrintInt("matchCopy", outputPos);
        Value* outputPosRem = b->CreateURem(outputPos, outputCapacity);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");
        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiCopiedLength = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedLength->addIncoming(b->getSize(0), entryBlock);

        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, matchLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);

        Value* outputStartRem = b->CreateAdd(outputPosRem, phiCopiedLength);
        Value* outputStartRem2 = b->CreateURem(outputStartRem, b->getSize(2));
        Value* outputStartByteRem = b->CreateUDiv(outputStartRem, b->getSize(2));

        Value* outputTargetPtr = b->CreateGEP(outputByteBasePtr, outputStartByteRem);
        outputTargetPtr = b->CreatePointerCast(outputTargetPtr, INT_FW_PTR);


        Value* copyStartRem = b->CreateSub(outputStartRem, matchOffset);
        Value* copyStartRem2 = b->CreateURem(copyStartRem, b->getSize(2));
        Value* copyStartByteRem = b->CreateUDiv(copyStartRem, b->getSize(2));

        Value* inputTargetPtr = b->CreateGEP(outputByteBasePtr, copyStartByteRem);
        inputTargetPtr = b->CreatePointerCast(inputTargetPtr, INT_FW_PTR);




        Value* inputTargetValue = b->CreateLoad(inputTargetPtr);
        inputTargetValue = b->CreateLShr(inputTargetValue, b->CreateMul(copyStartRem2, b->getIntN(fw, 4)));


        Value* outputValue = b->CreateLoad(outputTargetPtr);
        Value* outputMask = b->CreateSelect(
                b->CreateICmpEQ(outputStartRem2, b->getSize(1)),
                b->getIntN(fw, 0xf),
                b->getIntN(fw, 0)
        );
        outputValue = b->CreateAnd(outputValue, outputMask);
        inputTargetValue = b->CreateShl(inputTargetValue, b->CreateMul(outputStartRem2, b->getIntN(fw, 4)));
        outputValue = b->CreateOr(outputValue, inputTargetValue);
        b->CreateStore(outputValue, outputTargetPtr);



        Value* newCopyLength = b->getSize(fw / 4);
        newCopyLength = b->CreateSub(newCopyLength, b->CreateOr(outputStartRem2, copyStartRem2));
        newCopyLength = b->CreateUMin(newCopyLength, matchOffset);

        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, newCopyLength), b->GetInsertBlock());

        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);


        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }

    void LZ4I4ByteStreamAioKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) {
        b->setProducedItemCount("outputI4Stream", produced);
    }

}