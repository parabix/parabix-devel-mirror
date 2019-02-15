

#include "twist_kernel.h"
#include <llvm/IR/Intrinsics.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/Compiler.h>

namespace llvm { class Value; }

using namespace llvm;

namespace kernel {

void twistByPDEP(const std::unique_ptr <kernel::KernelBuilder> &b, Value *inputBlocks[], const unsigned twistWidth, Value* outputBasePtr) {
    Function *PDEPFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pdep_64);
    uint64_t pdepBaseMask = twistWidth == 2? 0x5555555555555555 : 0x1111111111111111;

    SmallVector<Value *, 16> currentInput(twistWidth);
    for (unsigned i = 0; i < b->getBitBlockWidth() / 64; i++) {
        for (unsigned iIndex = 0; iIndex < twistWidth; iIndex++) {
            currentInput[iIndex] = b->CreateExtractElement(inputBlocks[iIndex], i);
        }

        for (unsigned j = 0; j < twistWidth; j++) {
            unsigned outputIndex = i * twistWidth + j;
            Value *retI64 = b->getInt64(0);
            for (unsigned k = 0; k < twistWidth; k++) {
                Value *newBits = b->CreateCall(
                            PDEPFunc, {
                                b->CreateLShr(currentInput[k], b->getInt64(j * 64 / twistWidth)),
                                b->getInt64(pdepBaseMask << k)
                            }
                            );
                retI64 = b->CreateOr(retI64, newBits);
            }
            b->CreateStore(retI64, b->CreateGEP(outputBasePtr, b->getInt32(outputIndex)));
        }
    }
}

TwistByPDEPKernel::TwistByPDEPKernel(const std::unique_ptr <kernel::KernelBuilder> &b, unsigned numberOfInputStream, unsigned twistWidth)
: BlockOrientedKernel(b, "TwistByPDEPKernel",
{Binding{b->getStreamSetTy(numberOfInputStream, 1), "basisBits"}},
{Binding{b->getStreamSetTy(1, twistWidth), "byteStream"}},
{}, {}, {}),
      mNumberOfInputStream(numberOfInputStream),
      mTwistWidth(twistWidth)
{
    assert(twistWidth == 2 || twistWidth == 4);
    assert(numberOfInputStream <= twistWidth);
}

void TwistByPDEPKernel::generateDoBlockMethod(const std::unique_ptr <kernel::KernelBuilder> &b) {
    SmallVector<Value *, 16> inputBlocks(mTwistWidth);
    for (unsigned i = 0; i < mTwistWidth; i++) {
        if (i < mNumberOfInputStream) {
            inputBlocks[i] = b->loadInputStreamBlock("basisBits", b->getInt32(i));
        } else {
            inputBlocks[i] = ConstantVector::getNullValue(b->getBitBlockType());
        }
    }

    Value *outputBasePtr = b->CreatePointerCast(b->getOutputStreamBlockPtr("byteStream", b->getSize(0)),
                                                b->getInt64Ty()->getPointerTo());
    twistByPDEP(b, inputBlocks.data(), mTwistWidth, outputBasePtr);
}


TwistMultipleByPDEPKernel::TwistMultipleByPDEPKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                     const StreamSets & inputStreams,
                                                     StreamSet * outputStream)
: BlockOrientedKernel(b, "TwistMultipleByPDEPKernel",
{},
{Binding{"byteStream", outputStream}},
{}, {}, {}),
mTwistWidth(outputStream->getFieldWidth()) {
    assert(mTwistWidth == 2 || mTwistWidth == 4);
    for (unsigned i = 0; i < inputStreams.size(); i++) {
        mInputStreamSets.push_back(Binding{"basisBits_" + std::to_string(i), inputStreams[i] });
    }
}

void TwistMultipleByPDEPKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &b) {
    SmallVector<Value *, 16> input(mTwistWidth);
    unsigned k = 0;
    for (unsigned i = 0; i < getNumOfStreamInputs(); ++i) {
        const auto m = getInputStreamSet(i)->getNumElements();
        for (unsigned j = 0; j < m; j++) {
            input[k++] = b->loadInputStreamBlock("basisBits_" + std::to_string(i), b->getInt32(j));
        }
    }
    assert (k <= mTwistWidth);
    while (k < mTwistWidth) {
        input[k++] = ConstantVector::getNullValue(b->getBitBlockType());
    }


    Value *outputBasePtr = b->CreatePointerCast(b->getOutputStreamBlockPtr("byteStream", b->getSize(0)),
                                                b->getInt64Ty()->getPointerTo());
    twistByPDEP(b, input.data(), mTwistWidth, outputBasePtr);
}

}
