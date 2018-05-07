

#include "lz4_deposit_uncompressed.h"
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <kernels/streamset.h>

using namespace llvm;
using namespace kernel;
using namespace parabix;
using namespace std;

namespace kernel{

    void LZ4DepositUncompressedKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
        BasicBlock* exitBlock = iBuilder->CreateBasicBlock("exit_block");

        BasicBlock* uncompressedDataLoopCon = iBuilder->CreateBasicBlock("uncompressed_data_loop_con");
        BasicBlock* uncompressedDataLoopBody = iBuilder->CreateBasicBlock("uncompressed_data_loop_body");

        iBuilder->CreateBr(uncompressedDataLoopCon);

        // uncompressedDataLoopCon
        iBuilder->SetInsertPoint(uncompressedDataLoopCon);
        Value* uncompressedDataIndex = iBuilder->getScalarField("uncompressedDataIndex");
        Value* availableBlockData = iBuilder->getAvailableItemCount("uncompressedStartPos");
        iBuilder->CreateCondBr(
                iBuilder->CreateICmpULT(uncompressedDataIndex, availableBlockData),
                uncompressedDataLoopBody,
                exitBlock);

        // uncompressedDataLoopBody
        iBuilder->SetInsertPoint(uncompressedDataLoopBody);
        this->generateDepositUncompressed(iBuilder);
        this->increaseCurrentUncompressedDataIndex(iBuilder);
        iBuilder->CreateBr(uncompressedDataLoopCon);

        // Exit
        iBuilder->SetInsertPoint(exitBlock);

    }
    void LZ4DepositUncompressedKernel::generateDepositUncompressed(const std::unique_ptr<KernelBuilder> &iBuilder) {
        Value* uncompressedStartPos = this->loadCurrentUncompressedData(iBuilder, "uncompressedStartPos");
        Value* uncompressedLength = this->loadCurrentUncompressedData(iBuilder, "uncompressedLength");
        Value* uncompressedOutputPos = this->loadCurrentUncompressedData(iBuilder, "uncompressedOutputPos");

        Value* inputBufferBasePtr = iBuilder->getRawInputPointer("byteStream", iBuilder->getSize(0));
        Value* outputBufferBasePtr = iBuilder->getRawOutputPointer("outputStream", iBuilder->getSize(0));


        size_t outputBufferSize = this->getOutputStreamSetBuffer("outputStream")->getBufferBlocks() * iBuilder->getStride();
        Value* outputBufferSizeValue = iBuilder->getSize(outputBufferSize);
        Value* outputBufferSizeMask = iBuilder->getSize(outputBufferSize - 1);

        Value* maskedOutputOffset = iBuilder->CreateAnd(uncompressedOutputPos, outputBufferSizeMask);
        Value* remainBuffer = iBuilder->CreateSub(outputBufferSizeValue, maskedOutputOffset);
        Value* copyLength1 = iBuilder->CreateSelect(iBuilder->CreateICmpUGE(remainBuffer, uncompressedLength), uncompressedLength, remainBuffer);
        Value* copyLength2 = iBuilder->CreateSub(uncompressedLength, copyLength1);

        iBuilder->CreateMemCpy(
                iBuilder->CreateGEP(outputBufferBasePtr, maskedOutputOffset),
                iBuilder->CreateGEP(inputBufferBasePtr, uncompressedStartPos),
                copyLength1,
                1); // no alignment guaranteed
        // Assumed output buffer is Circular buffer
        iBuilder->CreateMemCpy(
                outputBufferBasePtr,
                iBuilder->CreateGEP(inputBufferBasePtr, iBuilder->CreateAdd(uncompressedStartPos, copyLength1)),
                copyLength2,
                8
        );

        iBuilder->setProducedItemCount("outputStream", iBuilder->CreateAdd(uncompressedOutputPos, uncompressedLength));
    }

    void LZ4DepositUncompressedKernel::increaseCurrentUncompressedDataIndex(const std::unique_ptr<KernelBuilder> &iBuilder) {
        Value* i = iBuilder->getScalarField("uncompressedDataIndex");
        i = iBuilder->CreateAdd(i, iBuilder->getSize(1));
        iBuilder->setScalarField("uncompressedDataIndex", i);
    }

    Value* LZ4DepositUncompressedKernel::loadCurrentUncompressedData(const unique_ptr<kernel::KernelBuilder> & iBuilder, const string& name) {
        Value* blockDataIndex = iBuilder->getScalarField("uncompressedDataIndex");
        return this->generateLoadCircularInput(iBuilder, name, blockDataIndex, iBuilder->getInt64Ty());
    }

    size_t LZ4DepositUncompressedKernel::getInputBufferSize(const unique_ptr<KernelBuilder> &iBuilder, string bufferName) {
        return this->getInputStreamSetBuffer(bufferName)->getBufferBlocks() * iBuilder->getStride();
    }

    Value* LZ4DepositUncompressedKernel::generateLoadCircularInput(const unique_ptr<KernelBuilder> &iBuilder, string inputBufferName, Value* offset, Type* pointerType) {
        size_t inputSize = this->getInputBufferSize(iBuilder, inputBufferName);
        Value* offsetMask = iBuilder->getSize(inputSize - 1);
        Value* maskedOffset = iBuilder->CreateAnd(offsetMask, offset);

        Value* inputBufferPtr = iBuilder->getRawInputPointer(inputBufferName, iBuilder->getSize(0));

        inputBufferPtr = iBuilder->CreatePointerCast(inputBufferPtr, pointerType);
        return iBuilder->CreateLoad(iBuilder->CreateGEP(inputBufferPtr, maskedOffset));
    }

    LZ4DepositUncompressedKernel::LZ4DepositUncompressedKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) :
            SegmentOrientedKernel(
                    "lz4_deposit_uncompressed_kernel",
                    {//Inputs
                            Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"},

                            // Uncompressed Data
                            Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedStartPos"},
                            Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedLength"},
                            Binding{iBuilder->getStreamSetTy(1, 64), "uncompressedOutputPos"}
                    },
                    {//Outputs
                            Binding{iBuilder->getStreamSetTy(1, 8), "outputStream", UnknownRate()}
                    },
                    {//Arguments

                    },
                    {},
                    {//Internal States
                            Binding{iBuilder->getSizeTy(), "uncompressedDataIndex"},
                    }){
//        setNoTerminateAttribute(true);
    }
}

