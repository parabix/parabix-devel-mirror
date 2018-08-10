
#include "untwist_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {
    UntwistByPEXTKernel::UntwistByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> &b, unsigned numberOfOutputStream, unsigned twistWidth)
            :BlockOrientedKernel("UntwistByPEXTKernel",
                                 {
                                         Binding{b->getStreamSetTy(1, twistWidth), "byteStream", FixedRate(), Principal()}
                                 },
                                 {
                                         Binding{b->getStreamSetTy(numberOfOutputStream, 1), "basisBits"}
                                 }, {}, {}, {}),
             mNumberOfOutputStream(numberOfOutputStream),
             mTwistWidth(twistWidth)
    {
        assert(twistWidth == 2 || twistWidth == 4);
        assert(numberOfInputStream <= twistWidth);

    }

    void UntwistByPEXTKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
        Function* PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);

        uint64_t pextBaseMask = mTwistWidth == 2 ? 0x5555555555555555: 0x1111111111111111;

        Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());

        Value* outputBlocks[mTwistWidth];
        for (unsigned i = 0; i < mTwistWidth; i++) {
            outputBlocks[i] = ConstantVector::getNullValue(b->getBitBlockType());
        }

        for (unsigned i = 0; i < b->getBitBlockWidth() / 64; i++) {
            Value* currentOutput[mTwistWidth];

            for (unsigned iIndex = 0; iIndex < mTwistWidth; iIndex++) {
                currentOutput[iIndex] = b->getInt64(0);
            }

            for (unsigned j = 0; j < mTwistWidth; j++) {
                unsigned inputIndex = i * mTwistWidth + j;

                Value* currentInput = b->CreateLoad(b->CreateGEP(inputBasePtr, b->getInt32(inputIndex)));
                for (unsigned k = 0; k < mTwistWidth; k++) {

                    Value* newBits = b->CreateCall(
                            PEXT_func,{
                                    currentInput,
                                    b->getInt64(pextBaseMask << k)
                            }
                    );

                    currentOutput[k] = b->CreateOr(currentOutput[k], b->CreateShl(newBits, 64 / mTwistWidth * j));
                }
            }

            for (unsigned iIndex = 0; iIndex < mTwistWidth; iIndex++) {
                outputBlocks[iIndex] = b->CreateInsertElement(outputBlocks[iIndex], currentOutput[iIndex], i);
            }
        }

        for (unsigned i = 0; i < mNumberOfOutputStream; i++) {
            b->storeOutputStreamBlock("basisBits", b->getInt32(i), outputBlocks[i]);
        }
    }

    StreamCompareKernel::StreamCompareKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                             unsigned int numberOfStream):
            BlockOrientedKernel("UntwistByPEXTKernel",
                                {
                                        Binding{b->getStreamSetTy(numberOfStream, 1), "stream1", FixedRate(), Principal()},
                                        Binding{b->getStreamSetTy(numberOfStream, 1), "stream2", FixedRate()}
                                },
                                {
//                                        Binding{b->getStreamSetTy(numberOfOutputStream, 1), "basisBits"}
                                }, {}, {}, {}),mNumberOfStream(numberOfStream)
    {
//        this->setStride(4 * 1024 * 1024);
        this->addScalar(b->getSizeTy(), "pos");
    }

    void StreamCompareKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
        Value* s1 = b->loadInputStreamBlock("stream1", b->getSize(0));
        Value* s2 = b->loadInputStreamBlock("stream2", b->getSize(0));

        for (unsigned i = 0 ; i < 4; i++) {
            Value* v1 = b->CreateExtractElement(s1, i);
            Value* v2 = b->CreateExtractElement(s2, i);
            Value* shouldPrint = b->CreateICmpNE(v1, v2);
            b->CallPrintIntCond("---pos", b->getScalarField("pos"), shouldPrint);

//            b->CallPrintIntCond("s1_available", b->getAvailableItemCount("stream1"), shouldPrint);
            b->CallPrintRegisterCond("s1", s1, shouldPrint);
            b->CallPrintRegisterCond("s2", s2, shouldPrint);
        }
        b->setScalarField("pos", b->CreateAdd(b->getScalarField("pos"), b->getSize(b->getBitBlockWidth())));
    };
}