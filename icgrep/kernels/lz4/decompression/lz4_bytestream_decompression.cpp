
#include "lz4_bytestream_decompression.h"


#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/core/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel {

std::string LZ4ByteStreamDecompressionKernel::getCopyByteStreamName() {
    return mCopyOtherByteStream ? "targetByteStream" : "byteStream";
}

LZ4ByteStreamDecompressionKernel::LZ4ByteStreamDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                                   // arguments
                                                                   Scalar * fileSize,
                                                                   // input
                                                                   StreamSet * inputStream,
                                                                   const LZ4BlockInfo & blockInfo,
                                                                   StreamSet * targetStream,
                                                                   // output
                                                                   StreamSet * outputStream)
: LZ4SequentialDecompressionKernel(b, "LZ4ByteStreamDecompression",
                                   fileSize, inputStream, blockInfo,
                                   4 * 1024 * 1024, false)
, mCopyOtherByteStream(targetStream != nullptr) {
    mOutputStreamSets.emplace_back("outputStream", outputStream, BoundedRate(0, 1));
    addInternalScalar(b->getInt8PtrTy(), "temporaryInputPtr");
    if (mCopyOtherByteStream) {
        mInputStreamSets.emplace_back("targetByteStream", targetStream, RateEqualTo("byteStream"));
    }
}

void LZ4ByteStreamDecompressionKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                                           llvm::Value *literalLength, llvm::Value* blockStart) {
    Value* LZ4_BLOCK_SIZE = b->getSize(mBlockSize);
    Value* INT_FW_1 = b->getIntN(COPY_FW, 1);
    Value* SIZE_FW_BYTE = b->getSize(COPY_FW / BYTE_WIDTH);

    Type* INT_FW_PTR = b->getIntNTy(COPY_FW)->getPointerTo();

    Value* inputBytePtr = b->getScalarField("temporaryInputPtr");
    inputBytePtr = b->CreateGEP(inputBytePtr, b->CreateSub(literalStart, blockStart));

    Value* inputPtr = b->CreatePointerCast(inputBytePtr, INT_FW_PTR);

    Value* outputPos = b->getScalarField("outputPos");
    Value* outputBufferSize = b->getCapacity("outputStream");
    Value* outputPosRem = b->CreateURem(outputPos, outputBufferSize);
    Value* outputPosRemBlockSize = b->CreateURem(outputPos, b->getSize(mBlockSize));
    Value* outputPtr = b->getRawOutputPointer("outputStream", outputPosRem);
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

    Value* remBufferSize = b->CreateSub(LZ4_BLOCK_SIZE, b->CreateAdd(phiCopiedLength, outputPosRemBlockSize));


    Value* inputValue = b->CreateLoad(phiInputPtr);

    Value* actualCopyLength = b->CreateUMin(SIZE_FW_BYTE, remBufferSize);

    Value* mask = b->CreateSub(
            b->CreateShl(INT_FW_1, b->CreateMul(actualCopyLength, b->getIntN(COPY_FW, 8))),
            INT_FW_1
    );
    Value* fullCopy = b->CreateICmpEQ(actualCopyLength, SIZE_FW_BYTE);
    mask = b->CreateSelect(
            fullCopy,
            b->CreateNot(b->getIntN(COPY_FW, 0)),
            mask
    );

    Value* exceedValue = b->CreateSelect(
            fullCopy,
            b->getIntN(COPY_FW, 0),
            b->CreateShl(oldOutputExceedFwData, b->CreateMul(actualCopyLength, b->getIntN(COPY_FW, 8)))
    );

    Value* actualOutput = b->CreateOr(
            b->CreateAnd(inputValue, mask),
            exceedValue
    );

    b->CreateStore(actualOutput, phiOutputPtr);

    phiInputPtr->addIncoming(b->CreateGEP(phiInputPtr, b->getSize(1)), b->GetInsertBlock());
    phiOutputPtr->addIncoming(b->CreateGEP(phiOutputPtr, b->getSize(1)), b->GetInsertBlock());
    phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, b->getSize(COPY_FW / BYTE_WIDTH)), b->GetInsertBlock());
    b->CreateBr(literalCopyCon);

    // ---- literalCopyExit
    b->SetInsertPoint(literalCopyExit);
    b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
}

void LZ4ByteStreamDecompressionKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                         llvm::Value *matchLength) {
    Value* LZ4_BLOCK_SIZE = b->getSize(mBlockSize);
    Type* INT_FW_PTR = b->getIntNTy(COPY_FW)->getPointerTo();
    Value* INT_FW_1 = b->getIntN(COPY_FW, 1);
    Value* SIZE_FW_BYTE = b->getSize(COPY_FW / BYTE_WIDTH);

    BasicBlock* entryBlock = b->GetInsertBlock();

    Value* outputPos = b->getScalarField("outputPos");
    Value* outputBufferSize = b->getCapacity("outputStream");
    Value* outputPosRemBlockSize = b->CreateURem(outputPos, b->getSize(mBlockSize));

    Value* outputPosRem = b->CreateURem(outputPos, outputBufferSize);
    Value* copyToPtr = b->getRawOutputPointer("outputStream", outputPosRem);

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

    Value* remBufferSize = b->CreateSub(LZ4_BLOCK_SIZE, b->CreateAdd(phiCopiedSize, outputPosRemBlockSize));
    Value* copyFromFwPtr = b->CreatePointerCast(phiFromPtr, INT_FW_PTR);
    Value* copyToFwPtr = b->CreatePointerCast(phiToPtr, INT_FW_PTR);

    Value* actualCopyLength = b->CreateUMin(SIZE_FW_BYTE, remBufferSize);
    Value* mask = b->CreateSub(
            b->CreateShl(INT_FW_1, b->CreateMul(actualCopyLength, b->getIntN(COPY_FW, 8))),
            INT_FW_1
    );

    Value* fullCopy = b->CreateICmpEQ(actualCopyLength, SIZE_FW_BYTE);
    mask = b->CreateSelect(
            fullCopy,
            b->CreateNot(b->getIntN(COPY_FW, 0)),
            mask
    );
    Value* exceedValue = b->CreateSelect(
            fullCopy,
            b->getIntN(COPY_FW, 0),
            b->CreateShl(oldOutputExceedFwData, b->CreateMul(actualCopyLength, b->getIntN(COPY_FW, 8)))
    );


    Value* actualOutput = b->CreateOr(
            b->CreateAnd(b->CreateLoad(copyFromFwPtr), mask),
            exceedValue
    );

    b->CreateStore(
            actualOutput,
            copyToFwPtr
    );

    Value* copySize = b->CreateUMin(matchOffset, b->getSize(COPY_FW / 8));
    phiFromPtr->addIncoming(b->CreateGEP(phiFromPtr, copySize), b->GetInsertBlock());
    phiToPtr->addIncoming(b->CreateGEP(phiToPtr, copySize), b->GetInsertBlock());
    phiCopiedSize->addIncoming(b->CreateAdd(phiCopiedSize, copySize), b->GetInsertBlock());
    b->CreateBr(matchCopyCon);

    // ---- matchCopyExit
    b->SetInsertPoint(matchCopyExit);
    b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
}

void LZ4ByteStreamDecompressionKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) {
    b->setProducedItemCount("outputStream", produced);
}

void LZ4ByteStreamDecompressionKernel::initializationMethod(const std::unique_ptr<KernelBuilder> &b) {
    b->setScalarField("temporaryInputPtr", b->CreateMalloc(b->getSize(mBlockSize)));
}

void LZ4ByteStreamDecompressionKernel::prepareProcessBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart, llvm::Value* blockEnd) {
    Value* rawInputPtr = b->CreatePointerCast(b->getRawInputPointer(getCopyByteStreamName(), b->getSize(0)), b->getInt8PtrTy());
    Value* inputCapacity = b->getCapacity(getCopyByteStreamName());

    Value* blockStartRem = b->CreateURem(blockStart, inputCapacity);
    Value* remSize = b->CreateSub(inputCapacity, blockStartRem);

    Value* blockSize = b->CreateSub(blockEnd, blockStart);

    Value* copySize1 = b->CreateUMin(remSize, blockSize);
    Value* copySize2 = b->CreateSub(blockSize, copySize1);

    Value* temporayInputPtr = b->getScalarField("temporaryInputPtr");

    b->CreateMemCpy(temporayInputPtr, b->CreateGEP(rawInputPtr, blockStartRem), copySize1, 1);
    b->CreateMemCpy(b->CreateGEP(temporayInputPtr, copySize1), rawInputPtr, copySize2, 1);


    Value* outputPos = b->getScalarField("outputPos");
    Value* outputBufferSize = b->getCapacity("outputStream");
    Value* outputPosRem = b->CreateURem(outputPos, outputBufferSize);
    Value* LZ4_BLOCK_SIZE = b->getSize(mBlockSize);
    Type* INT_FW_PTR = b->getIntNTy(COPY_FW)->getPointerTo();
    Value* outputEndPtr = b->CreateGEP(b->getRawOutputPointer("outputStream", b->getSize(0)), b->CreateMul(b->CreateUDiv(b->CreateAdd(outputPosRem, LZ4_BLOCK_SIZE), LZ4_BLOCK_SIZE), LZ4_BLOCK_SIZE));
    // Only load old output exceed data once for every LZ4 block
    oldOutputExceedFwData = b->CreateLoad(b->CreatePointerCast(outputEndPtr, INT_FW_PTR));

}

void LZ4ByteStreamDecompressionKernel::beforeTermination(const std::unique_ptr<KernelBuilder> &b) {
    b->CreateFree(b->getScalarField("temporaryInputPtr"));
}

}
