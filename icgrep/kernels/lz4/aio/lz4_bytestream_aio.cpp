
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
    std::string LZ4ByteStreamAioKernel::getCopyByteStreamName() {
        return mCopyOtherByteStream ? "targetByteStream" : "byteStream";
    }

    LZ4ByteStreamAioKernel::LZ4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, bool copyOtherByteStream, unsigned blockSize)
            : LZ4SequentialAioBaseKernel(b, "LZ4ByteStreamAioKernel", blockSize),
              mCopyOtherByteStream(copyOtherByteStream) {
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)});
        this->addScalar(b->getInt8PtrTy(), "temporaryInputPtr");
        if (copyOtherByteStream) {
            mStreamSetInputs.push_back(Binding{b->getStreamSetTy(1, 8), "targetByteStream", RateEqualTo("byteStream")});
        }
    }

    void LZ4ByteStreamAioKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                               llvm::Value *literalLength, llvm::Value* blockStart) {
        unsigned fw = 64;
        Type* INT_FW_PTR = b->getIntNTy(fw)->getPointerTo();

        Value* inputBytePtr = b->getScalarField("temporaryInputPtr");
        inputBytePtr = b->CreateGEP(inputBytePtr, b->CreateSub(literalStart, blockStart));

        Value* inputPtr = b->CreatePointerCast(inputBytePtr, INT_FW_PTR);

        Value* outputPos = b->getScalarField("outputPos");
        Value* outputBufferSize = b->getCapacity("outputStream");
        Value* outputPtr = b->getRawOutputPointer("outputStream", b->CreateURem(outputPos, outputBufferSize));
        outputPtr = b->CreatePointerCast(outputPtr, INT_FW_PTR);

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

    void LZ4ByteStreamAioKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
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

    void LZ4ByteStreamAioKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) {
        b->setProducedItemCount("outputStream", produced);
    }

    void LZ4ByteStreamAioKernel::initializationMethod(const std::unique_ptr<KernelBuilder> &b) {
        b->setScalarField("temporaryInputPtr", b->CreateMalloc(b->getSize(mBlockSize)));
    }

    void LZ4ByteStreamAioKernel::prepareProcessBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart, llvm::Value* blockEnd) {
        Value* rawInputPtr = b->CreatePointerCast(b->getRawInputPointer(this->getCopyByteStreamName(), b->getSize(0)), b->getInt8PtrTy());
        Value* inputCapacity = b->getCapacity(this->getCopyByteStreamName());

        Value* blockStartRem = b->CreateURem(blockStart, inputCapacity);
        Value* remSize = b->CreateSub(inputCapacity, blockStartRem);

        Value* blockSize = b->CreateSub(blockEnd, blockStart);

        Value* copySize1 = b->CreateUMin(remSize, blockSize);
        Value* copySize2 = b->CreateSub(blockSize, copySize1);

        Value* temporayInputPtr = b->getScalarField("temporaryInputPtr");

        b->CreateMemCpy(temporayInputPtr, b->CreateGEP(rawInputPtr, blockStartRem), copySize1, 1);
        b->CreateMemCpy(b->CreateGEP(temporayInputPtr, copySize1), rawInputPtr, copySize2, 1);
    }

    void LZ4ByteStreamAioKernel::beforeTermination(const std::unique_ptr<KernelBuilder> &b) {
        b->CreateFree(b->getScalarField("temporaryInputPtr"));
//        b->CallPrintInt("beforeTermination", b->getSize(0));
    }

}