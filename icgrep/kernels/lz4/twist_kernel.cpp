

#include "twist_kernel.h"
#include <llvm/IR/Intrinsics.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/Compiler.h>

namespace llvm { class Value; }

using namespace llvm;
using namespace parabix;

namespace kernel {

    TwistByPDEPKernel::TwistByPDEPKernel(const std::unique_ptr <kernel::KernelBuilder> &b, unsigned numberOfInputStream, unsigned twistWidth)
            : BlockOrientedKernel("TwistByPDEPKernel",
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
        Function *PDEPFunc = Intrinsic::getDeclaration(getModule(), Intrinsic::x86_bmi_pdep_64);
        uint64_t pdepBaseMask = mTwistWidth == 2? 0x5555555555555555 : 0x1111111111111111;

        Value *inputBlocks[mTwistWidth];
        for (unsigned i = 0; i < mTwistWidth; i++) {
            if (i < mNumberOfInputStream) {
                inputBlocks[i] = b->loadInputStreamBlock("basisBits", b->getInt32(i));
//            b->CallPrintRegister("input" + std::to_string(i), inputBlocks[i]);

            } else {
                inputBlocks[i] = ConstantVector::getNullValue(b->getBitBlockType());
            }
        }

        Value *outputBasePtr = b->CreatePointerCast(b->getOutputStreamBlockPtr("byteStream", b->getSize(0)),
                                                    b->getInt64Ty()->getPointerTo());

        for (unsigned i = 0; i < b->getBitBlockWidth() / 64; i++) {
            Value *currentInput[mTwistWidth];
            for (unsigned iIndex = 0; iIndex < mTwistWidth; iIndex++) {
                currentInput[iIndex] = b->CreateExtractElement(inputBlocks[iIndex], i);
            }

            for (unsigned j = 0; j < mTwistWidth; j++) {
                unsigned outputIndex = i * mTwistWidth + j;
                Value *retI64 = b->getInt64(0);
                for (unsigned k = 0; k < mTwistWidth; k++) {
                    Value *newBits = b->CreateCall(
                            PDEPFunc, {
                                    b->CreateLShr(currentInput[k], b->getInt64(j * 64 / mTwistWidth)),
                                    b->getInt64(pdepBaseMask << k)
                            }
                    );
                    retI64 = b->CreateOr(retI64, newBits);
                }
                b->CreateStore(retI64, b->CreateGEP(outputBasePtr, b->getInt32(outputIndex)));
            }
        }
    }

}