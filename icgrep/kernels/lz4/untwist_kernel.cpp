
#include "untwist_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

void untwistByPEXT(const std::unique_ptr <kernel::KernelBuilder> &b, Value* inputBasePtr, const unsigned twistWidth, Value *outputBlocks[]){
    Function* PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
    uint64_t pextBaseMask = twistWidth == 2 ? 0x5555555555555555: 0x1111111111111111;

    for (unsigned i = 0; i < twistWidth; i++) {
        outputBlocks[i] = ConstantVector::getNullValue(b->getBitBlockType());
    }

    for (unsigned i = 0; i < b->getBitBlockWidth() / 64; i++) {
        Value* currentOutput[twistWidth];

        for (unsigned iIndex = 0; iIndex < twistWidth; iIndex++) {
            currentOutput[iIndex] = b->getInt64(0);
        }

        for (unsigned j = 0; j < twistWidth; j++) {
            unsigned inputIndex = i * twistWidth + j;

            Value* currentInput = b->CreateLoad(b->CreateGEP(inputBasePtr, b->getInt32(inputIndex)));
            for (unsigned k = 0; k < twistWidth; k++) {

                Value* newBits = b->CreateCall(
                            PEXT_func,{
                                currentInput,
                                b->getInt64(pextBaseMask << k)
                            }
                            );

                currentOutput[k] = b->CreateOr(currentOutput[k], b->CreateShl(newBits, 64 / twistWidth * j));
            }
        }

        for (unsigned iIndex = 0; iIndex < twistWidth; iIndex++) {
            outputBlocks[iIndex] = b->CreateInsertElement(outputBlocks[iIndex], currentOutput[iIndex], i);
        }
    }
}




UntwistByPEXTKernel::UntwistByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * inputStream, StreamSet * outputStream)
: BlockOrientedKernel(b, "UntwistByPEXTKernel",
// input
{Binding{"byteStream", inputStream}},
// output
{Binding{"basisBits", outputStream}}
, {}, {}, {}),
      mNumberOfOutputStream(outputStream->getFieldWidth()),
      mTwistWidth(inputStream->getFieldWidth())
{
    assert(mTwistWidth == 2 || mTwistWidth == 4);
    assert(mNumberOfOutputStream <= mTwistWidth);

}

void UntwistByPEXTKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
    Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());
    Value* outputBlocks[mTwistWidth];

    untwistByPEXT(b, inputBasePtr, mTwistWidth, outputBlocks);

    for (unsigned i = 0; i < mNumberOfOutputStream; i++) {
        b->storeOutputStreamBlock("basisBits", b->getInt32(i), outputBlocks[i]);
    }
}


UntwistMultipleByPEXTKernel::UntwistMultipleByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                         StreamSet * inputStream,
                                                         const StreamSets & outputStreams)
: BlockOrientedKernel(b, "UntwistMultipleByPEXTKernel",
// input
{Binding{"byteStream", inputStream}},
// output

{},
{}, {}, {}),
    mTwistWidth(inputStream->getFieldWidth()) {
    assert(mTwistWidth == 2 || mTwistWidth == 4);
    for (unsigned i = 0; i < outputStreams.size(); i++) {
        mOutputStreamSets.emplace_back("basisBits_" + std::to_string(i), outputStreams[i]);
    }
}

void UntwistMultipleByPEXTKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
    Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());

    Value* outputBlocks[mTwistWidth];

    untwistByPEXT(b, inputBasePtr, mTwistWidth, outputBlocks);

    for (unsigned i = 0, k = 0; i < getNumOfStreamOutputs(); i++) {
        const auto m = getOutputStreamSet(i)->getNumElements();
        for (unsigned j = 0; j < m; ++j, ++k) {
            b->storeOutputStreamBlock("basisBits_" + std::to_string(i), b->getInt32(j), outputBlocks[k]);
        }
    }
}

}
