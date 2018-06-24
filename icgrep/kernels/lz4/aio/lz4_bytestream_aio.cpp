
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

    LZ4ByteStreamAioKernel::LZ4ByteStreamAioKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned blockSize)
            : LZ4SequentialAioBaseKernel(b, "LZ4ByteStreamAioKernel", blockSize) {
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(1, 8), "outputStream", BoundedRate(0, 1)});
    }

    void LZ4ByteStreamAioKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
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

}