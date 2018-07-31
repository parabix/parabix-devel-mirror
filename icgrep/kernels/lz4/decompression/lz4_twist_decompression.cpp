
#include "lz4_twist_decompression.h"

#include <kernels/kernel_builder.h>
#include <iostream>
#include <string>
#include <llvm/Support/raw_ostream.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace std;


namespace kernel {
    size_t LZ4TwistDecompressionKernel::getNormalCopyLength() {
        return (COPY_FW - BYTE_WIDTH) / mTwistWidth;
    }
    llvm::Value* LZ4TwistDecompressionKernel::getNormalCopyLengthValue(const std::unique_ptr<KernelBuilder> &b) {
        return b->getSize(this->getNormalCopyLength());
    }


    LZ4TwistDecompressionKernel::LZ4TwistDecompressionKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned twistWidth, unsigned blockSize)
            : LZ4SequentialDecompressionKernel(b, "LZ4TwistDecompressionKernel", blockSize),
              mTwistWidth(twistWidth),
              mItemsPerByte(BYTE_WIDTH / twistWidth)
    {
        mStreamSetInputs.push_back(Binding{b->getStreamSetTy(1, twistWidth), "inputTwistStream", RateEqualTo("byteStream")});
        mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(1, twistWidth), "outputTwistStream", BoundedRate(0, 1)});

        this->addScalar(b->getInt8PtrTy(), "temporaryInputPtr");
    }



    void LZ4TwistDecompressionKernel::doLiteralCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *literalStart,
                               llvm::Value *literalLength, llvm::Value* blockStart) {
        // Constant and Type
        Constant* SIZE_0 = b->getSize(0);
        Constant* SIZE_ITEMS_PER_BYTE = b->getSize(mItemsPerByte);
        Constant* INT_FW_TWIST_WIDTH = b->getIntN(COPY_FW, mTwistWidth);
        Type* INT8_PTR_TY = b->getInt8PtrTy();
        Type* INT_FW_TY = b->getIntNTy(COPY_FW);
        Type* INT_FW_PTR_TY = INT_FW_TY->getPointerTo();



        Value* temporayInputPtr = b->getScalarField("temporaryInputPtr");
        Value* initInputOffset = b->CreateSub(
                b->CreateUDiv(literalStart, SIZE_ITEMS_PER_BYTE),
                b->CreateUDiv(blockStart, SIZE_ITEMS_PER_BYTE)
        );

        Value* initInputPtr = b->CreateGEP(temporayInputPtr, initInputOffset);

        Value* outputByteBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputTwistStream", SIZE_0), INT8_PTR_TY);
        Value* outputPos = b->getScalarField("outputPos");
        Value* outputCapacity = b->getCapacity("outputTwistStream");

        Value* outputPosRem = b->CreateURem(outputPos, outputCapacity);
        Value* outputPosByteRem = b->CreateUDiv(outputPosRem, SIZE_ITEMS_PER_BYTE);

        Value* initOutputPtr = b->CreateGEP(outputByteBasePtr, outputPosByteRem);
        Value* initOutputLastByte = b->CreateZExt(b->CreateLoad(initOutputPtr), INT_FW_TY);


        Value* literalStartRemByteItem = b->CreateURem(literalStart, SIZE_ITEMS_PER_BYTE);
        Value* outputPosRemByteItem = b->CreateURem(outputPos, SIZE_ITEMS_PER_BYTE);
        Value* outputMask = this->getOutputMask(b, outputPos);


        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();

        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");

        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiInputPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiInputPtr->addIncoming(initInputPtr, entryBlock);

        PHINode* phiOutputPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiOutputPtr->addIncoming(initOutputPtr, entryBlock);

        PHINode* phiCopiedLength = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedLength->addIncoming(SIZE_0, entryBlock);

        PHINode* phiOutputLastByte = b->CreatePHI(INT_FW_TY, 2);
        phiOutputLastByte->addIncoming(initOutputLastByte, entryBlock);

        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, literalLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);

        Value* inputFwPtr = b->CreatePointerCast(phiInputPtr, INT_FW_PTR_TY);
        Value* outputFwPtr = b->CreatePointerCast(phiOutputPtr, INT_FW_PTR_TY);

        Value* inputTargetValue = b->CreateLoad(inputFwPtr);
        inputTargetValue = b->CreateLShr(inputTargetValue, b->CreateMul(literalStartRemByteItem, INT_FW_TWIST_WIDTH));
        inputTargetValue = b->CreateShl(inputTargetValue, b->CreateMul(outputPosRemByteItem, INT_FW_TWIST_WIDTH));


        Value* newCopyLength = this->getNormalCopyLengthValue(b);

        Value* outputValue = b->CreateAnd(phiOutputLastByte, outputMask);
        outputValue = b->CreateOr(outputValue, inputTargetValue);
        b->CreateStore(outputValue, outputFwPtr);


        phiOutputLastByte->addIncoming(b->CreateLShr(outputValue, b->getSize(this->getNormalCopyLength() * mTwistWidth)), b->GetInsertBlock());
        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, newCopyLength), b->GetInsertBlock());
        phiInputPtr->addIncoming(b->CreateGEP(phiInputPtr, b->CreateUDiv(newCopyLength, SIZE_ITEMS_PER_BYTE)), b->GetInsertBlock());
        phiOutputPtr->addIncoming(b->CreateGEP(phiOutputPtr, b->CreateUDiv(newCopyLength, SIZE_ITEMS_PER_BYTE)), b->GetInsertBlock());

        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, literalLength));
    }



    void LZ4TwistDecompressionKernel::doShortMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                  llvm::Value *matchLength) {
        // Constant and Type
        Constant* SIZE_0 = b->getSize(0);
        Constant* SIZE_ITEMS_PER_BYTE = b->getSize(mItemsPerByte);
        Constant* INT_FW_TWIST_WIDTH = b->getIntN(COPY_FW, mTwistWidth);
        Type* INT8_PTR_TY = b->getInt8PtrTy();
        Type* INT_FW_TY = b->getIntNTy(COPY_FW);
        Type* INT_FW_PTR_TY = INT_FW_TY->getPointerTo();



        Value* outputCapacity = b->getCapacity("outputTwistStream");

        Value* outputByteBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputTwistStream", SIZE_0), INT8_PTR_TY);

        Value* outputPos = b->getScalarField("outputPos");
        Value* outputPosRem = b->CreateURem(outputPos, outputCapacity);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");
        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiCopiedLength = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedLength->addIncoming(SIZE_0, entryBlock);

        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, matchLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);
        Value* outputStartRem = b->CreateAdd(outputPosRem, phiCopiedLength);

        Value* outputStartRemByteItem = b->CreateURem(outputStartRem, SIZE_ITEMS_PER_BYTE);
        Value* outputStartByteRem = b->CreateUDiv(outputStartRem, SIZE_ITEMS_PER_BYTE);

        Value* outputTargetPtr = b->CreateGEP(outputByteBasePtr, outputStartByteRem);
        outputTargetPtr = b->CreatePointerCast(outputTargetPtr, INT_FW_PTR_TY);


        Value* copyStartRem = b->CreateSub(outputStartRem, matchOffset);
        Value* copyStartRemByteItem = b->CreateURem(copyStartRem, SIZE_ITEMS_PER_BYTE);
        Value* copyStartByteRem = b->CreateUDiv(copyStartRem, SIZE_ITEMS_PER_BYTE);

        Value* inputTargetPtr = b->CreateGEP(outputByteBasePtr, copyStartByteRem);
        inputTargetPtr = b->CreatePointerCast(inputTargetPtr, INT_FW_PTR_TY);

        Value* inputTargetValue = b->CreateLoad(inputTargetPtr);
        inputTargetValue = b->CreateLShr(inputTargetValue, b->CreateMul(copyStartRemByteItem, INT_FW_TWIST_WIDTH));

        Value* outputValue = b->CreateLoad(outputTargetPtr);
        Value* outputMask = this->getOutputMask(b, outputStartRemByteItem);
        outputValue = b->CreateAnd(outputValue, outputMask);

        inputTargetValue = b->CreateShl(inputTargetValue, b->CreateMul(outputStartRemByteItem, INT_FW_TWIST_WIDTH));
        outputValue = b->CreateOr(outputValue, inputTargetValue);
        b->CreateStore(outputValue, outputTargetPtr);

        Value* newCopyLength = matchOffset;

        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, newCopyLength), b->GetInsertBlock());

        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);
        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }

    void LZ4TwistDecompressionKernel::doLongMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                                 llvm::Value *matchLength) {
        // Constant and Type
        Constant* SIZE_0 = b->getSize(0);
        Constant* SIZE_ITEMS_PER_BYTE = b->getSize(mItemsPerByte);
        Constant* INT_FW_TWIST_WIDTH = b->getIntN(COPY_FW, mTwistWidth);
        Type* INT8_PTR_TY = b->getInt8PtrTy();
        Type* INT_FW_TY = b->getIntNTy(COPY_FW);
        Type* INT_FW_PTR_TY = INT_FW_TY->getPointerTo();



        Value* outputByteBasePtr = b->CreatePointerCast(b->getRawOutputPointer("outputTwistStream", b->getSize(0)), INT8_PTR_TY);
        Value* outputCapacity = b->getCapacity("outputTwistStream");
        Value* outputPos = b->getScalarField("outputPos");
        Value* outputPosRem = b->CreateURem(outputPos, outputCapacity);

        Value* outputPosRemByteItem = b->CreateURem(outputPosRem, SIZE_ITEMS_PER_BYTE);
        Value* outputMask = this->getOutputMask(b, outputPosRem);


        Value* outputBytePos = b->CreateUDiv(outputPosRem, SIZE_ITEMS_PER_BYTE);
        Value* initCopyToPtr = b->CreateGEP(outputByteBasePtr, outputBytePos);

        Value* initOutputLastByte = b->CreateZExt(b->CreateLoad(initCopyToPtr), INT_FW_TY);

        Value* copyFromPosRem = b->CreateSub(outputPosRem, matchOffset);

        Value* copyFromPosRemByteItem = b->CreateURem(copyFromPosRem, SIZE_ITEMS_PER_BYTE);
        Value* copyFromBytePos = b->CreateUDiv(copyFromPosRem, SIZE_ITEMS_PER_BYTE);
        Value* initCopyFromPtr = b->CreateGEP(outputByteBasePtr, copyFromBytePos);


        Value* copyLength = this->getNormalCopyLengthValue(b);
        Value* copyLengthByte = b->CreateUDiv(copyLength, SIZE_ITEMS_PER_BYTE);

        // ---- EntryBlock
        BasicBlock* entryBlock = b->GetInsertBlock();
        BasicBlock* literalCopyCon = b->CreateBasicBlock("literalCopyCon");
        b->CreateBr(literalCopyCon);

        // ---- literalCopyCon
        b->SetInsertPoint(literalCopyCon);
        PHINode* phiCopiedLength = b->CreatePHI(b->getSizeTy(), 2);
        phiCopiedLength->addIncoming(SIZE_0, entryBlock);

        PHINode* phiCopyFromPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiCopyFromPtr->addIncoming(initCopyFromPtr, entryBlock);

        PHINode* phiCopyToPtr = b->CreatePHI(b->getInt8PtrTy(), 2);
        phiCopyToPtr->addIncoming(initCopyToPtr, entryBlock);

        PHINode* phiOutputLastByte = b->CreatePHI(b->getIntNTy(COPY_FW), 2);
        phiOutputLastByte->addIncoming(initOutputLastByte, entryBlock);


        BasicBlock* literalCopyBody = b->CreateBasicBlock("literalCopyBody");
        BasicBlock* literalCopyExit = b->CreateBasicBlock("literalCopyExit");

        b->CreateCondBr(b->CreateICmpULT(phiCopiedLength, matchLength), literalCopyBody, literalCopyExit);

        // ---- literalCopyBody
        b->SetInsertPoint(literalCopyBody);
        Value* outputTargetPtr = b->CreatePointerCast(phiCopyToPtr, INT_FW_PTR_TY);
        Value* inputTargetPtr = b->CreatePointerCast(phiCopyFromPtr, INT_FW_PTR_TY);

        Value* inputTargetValue = b->CreateLoad(inputTargetPtr);
        inputTargetValue = b->CreateLShr(inputTargetValue, b->CreateMul(copyFromPosRemByteItem, INT_FW_TWIST_WIDTH));
        inputTargetValue = b->CreateShl(inputTargetValue, b->CreateMul(outputPosRemByteItem, INT_FW_TWIST_WIDTH));

        Value* outputValue = b->CreateAnd(phiOutputLastByte, outputMask);

        outputValue = b->CreateOr(outputValue, inputTargetValue);
        b->CreateStore(outputValue, outputTargetPtr);

        phiCopiedLength->addIncoming(b->CreateAdd(phiCopiedLength, copyLength), b->GetInsertBlock());
        phiCopyFromPtr->addIncoming(b->CreateGEP(phiCopyFromPtr, copyLengthByte), b->GetInsertBlock());
        phiCopyToPtr->addIncoming(b->CreateGEP(phiCopyToPtr, copyLengthByte), b->GetInsertBlock());
        phiOutputLastByte->addIncoming(b->CreateLShr(outputValue, b->getSize(this->getNormalCopyLength() * mTwistWidth)), b->GetInsertBlock());

        b->CreateBr(literalCopyCon);

        // ---- literalCopyExit
        b->SetInsertPoint(literalCopyExit);

        b->setScalarField("outputPos", b->CreateAdd(outputPos, matchLength));
    }


    void LZ4TwistDecompressionKernel::doMatchCopy(const std::unique_ptr<KernelBuilder> &b, llvm::Value *matchOffset,
                             llvm::Value *matchLength) {

        BasicBlock* shortMatchCopyBlock = b->CreateBasicBlock("shortMatchCopyBlock");
        BasicBlock* longMatchCopyBlock = b->CreateBasicBlock("longMatchCopyBlock");
        BasicBlock* matchCopyFinishBlock = b->CreateBasicBlock("matchCopyFinishBlock");

        b->CreateUnlikelyCondBr(
                b->CreateICmpULT(matchOffset, this->getNormalCopyLengthValue(b)),
                shortMatchCopyBlock,
                longMatchCopyBlock
        );

        // ---- shortMatchCopyBlock
        b->SetInsertPoint(shortMatchCopyBlock);
        this->doShortMatchCopy(b, matchOffset, matchLength);
        b->CreateBr(matchCopyFinishBlock);

        // ---- longMatchCopyBlock
        b->SetInsertPoint(longMatchCopyBlock);
        this->doLongMatchCopy(b, matchOffset, matchLength);
        b->CreateBr(matchCopyFinishBlock);

        b->SetInsertPoint(matchCopyFinishBlock);
    }

    void LZ4TwistDecompressionKernel::setProducedOutputItemCount(const std::unique_ptr<KernelBuilder> &b, llvm::Value* produced) {
        b->setProducedItemCount("outputTwistStream", produced);
    }


    void LZ4TwistDecompressionKernel::initializationMethod(const std::unique_ptr<KernelBuilder> &b) {
        b->setScalarField("temporaryInputPtr", b->CreateMalloc(b->getSize(mBlockSize / mItemsPerByte)));
    }

    void LZ4TwistDecompressionKernel::prepareProcessBlock(const std::unique_ptr<KernelBuilder> &b, llvm::Value* blockStart, llvm::Value* blockEnd) {
        Constant* SIZE_0 = b->getSize(0);
        Constant* SIZE_ITEMS_PER_BYTE = b->getSize(mItemsPerByte);
        Type* INT8_PTR_TY = b->getInt8PtrTy();


        Value* rawInputPtr = b->CreatePointerCast(b->getRawInputPointer("inputTwistStream", SIZE_0), INT8_PTR_TY);
        Value* inputCapacity = b->getCapacity("inputTwistStream");

        Value* inputByteCapacity = b->CreateUDiv(inputCapacity, SIZE_ITEMS_PER_BYTE);

        Value* blockStartRem = b->CreateURem(blockStart, inputCapacity);
        Value* blockStartByteRem = b->CreateUDiv(blockStartRem, SIZE_ITEMS_PER_BYTE);
        Value* remByte = b->CreateSub(inputByteCapacity, blockStartByteRem);

        Value* blockSize = b->CreateSub(blockEnd, blockStart);
        Value* copyTotalByte = b->CreateAdd(b->CreateUDiv(blockSize, SIZE_ITEMS_PER_BYTE), SIZE_ITEMS_PER_BYTE); // It will be safe if we copy a few bytes more

        Value* copyBytes1 = b->CreateUMin(remByte, copyTotalByte);
        Value* copyBytes2 = b->CreateSub(copyTotalByte, copyBytes1);

        Value* temporayInputPtr = b->getScalarField("temporaryInputPtr");

        b->CreateMemCpy(temporayInputPtr, b->CreateGEP(rawInputPtr, blockStartByteRem), copyBytes1, 1);
        b->CreateMemCpy(b->CreateGEP(temporayInputPtr, copyBytes1), rawInputPtr, copyBytes2, 1);
    }

    void LZ4TwistDecompressionKernel::beforeTermination(const std::unique_ptr<KernelBuilder> &b) {
        b->CreateFree(b->getScalarField("temporaryInputPtr"));
    }

    llvm::Value *LZ4TwistDecompressionKernel::getOutputMask(const std::unique_ptr<KernelBuilder> &b, llvm::Value *outputPos) {
        Value* remByteItems = b->CreateURem(outputPos, b->getSize(mItemsPerByte));
        Value* INT_FW_1 = b->getIntN(COPY_FW, 1);
        Value* shiftAmount = b->CreateMul(remByteItems, b->getIntN(COPY_FW, mTwistWidth));
        return b->CreateSub(
                b->CreateShl(INT_FW_1, shiftAmount),
                INT_FW_1
        );
    }
}