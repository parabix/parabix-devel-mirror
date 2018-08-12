
#include "untwist_kernel.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

    void untwistByPEXT(const std::unique_ptr <kernel::KernelBuilder> &b, Value* inputBasePtr, unsigned mTwistWidth, Value *outputBlocks[]){
        Function* PEXT_func = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_bmi_pext_64);
        uint64_t pextBaseMask = mTwistWidth == 2 ? 0x5555555555555555: 0x1111111111111111;

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
    }




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
        Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());
        Value* outputBlocks[mTwistWidth];

        untwistByPEXT(b, inputBasePtr, mTwistWidth, outputBlocks);

        for (unsigned i = 0; i < mNumberOfOutputStream; i++) {
            b->storeOutputStreamBlock("basisBits", b->getInt32(i), outputBlocks[i]);
        }
    }


    UntwistMultipleByPEXTKernel::UntwistMultipleByPEXTKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                                             std::vector<unsigned> numberOfOutputStreams,
                                                             unsigned twistWidth) :
            BlockOrientedKernel("UntwistMultipleByPEXTKernel",
                                {
                                        Binding{b->getStreamSetTy(1, twistWidth), "byteStream", FixedRate(),
                                                Principal()}
                                },
                                {
                                        //Binding{b->getStreamSetTy(numberOfOutputStream, 1), "basisBits"}
                                }, {}, {}, {}),
            mNumberOfOutputStreams(numberOfOutputStreams),
            mTwistWidth(twistWidth) {
        assert(twistWidth == 2 || twistWidth == 4);

        size_t totalNumOfStreams = 0;
        for (unsigned i = 0; i < numberOfOutputStreams.size(); i++) {
            totalNumOfStreams += numberOfOutputStreams[i];
            mStreamSetOutputs.push_back(Binding{b->getStreamSetTy(numberOfOutputStreams[i]), "basisBits_" + std::to_string(i) });
        }
        assert(totalNumOfStreams < twistWidth);
    }

    void UntwistMultipleByPEXTKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &b) {
        Value* inputBasePtr = b->CreatePointerCast(b->getInputStreamBlockPtr("byteStream", b->getSize(0)), b->getInt64Ty()->getPointerTo());
        Value* outputBlocks[mTwistWidth];
        untwistByPEXT(b, inputBasePtr, mTwistWidth, outputBlocks);

        unsigned iStreamIndex = 0;
        for (unsigned i = 0; i < mNumberOfOutputStreams.size(); i++) {
            for (unsigned j = 0; j < mNumberOfOutputStreams[i]; j++) {
                b->storeOutputStreamBlock("basisBits_" + std::to_string(i), b->getInt32(j), outputBlocks[iStreamIndex]);
                iStreamIndex++;
            }
        }
    }
}