/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "swizzle.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

SwizzleGenerator::SwizzleGenerator(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned bitStreamCount, unsigned outputSets, unsigned inputSets, unsigned fieldWidth)
: BlockOrientedKernel("swizzle" + std::to_string(fieldWidth) + ":" + std::to_string(bitStreamCount), {}, {}, {}, {}, {})
, mBitStreamCount(bitStreamCount)
, mFieldWidth(fieldWidth)
, mSwizzleFactor(iBuilder->getBitBlockWidth() / fieldWidth)
, mInputSets(inputSets)
, mOutputSets(outputSets) {
    assert((fieldWidth > 0) && ((fieldWidth & (fieldWidth - 1)) == 0) && "fieldWidth must be a power of 2");
    assert(fieldWidth < iBuilder->getBitBlockWidth() && "fieldWidth must be less than the block width");
    assert(mSwizzleFactor > 1 && "fieldWidth must be less than the block width");
    unsigned inputStreamsPerSet = (bitStreamCount + inputSets - 1)/inputSets;
    unsigned outputStreamsPerSet = (bitStreamCount + outputSets - 1)/outputSets;
    // Maybe the following is unnecessary.
    //assert(inputStreamsPerSet % swizzleFactor == 0 && "input sets must be an exact multiple of the swizzle factor");
    assert(outputStreamsPerSet % mSwizzleFactor == 0 && "output sets must be an exact multiple of the swizzle factor");
    for (unsigned i = 0; i < mInputSets; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(inputStreamsPerSet, 1), "inputGroup" + std::to_string(i)});
    }
    for (unsigned i = 0; i < mOutputSets; i++) {
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(outputStreamsPerSet, 1), "outputGroup" + std::to_string(i)});
    }
}

void SwizzleGenerator::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
        
    // We may need a few passes depending on the swizzle factor
    const unsigned swizzleFactor = mSwizzleFactor;
    const unsigned passes = std::log2(mSwizzleFactor);
    const unsigned swizzleGroups = (mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor;
    const unsigned inputStreamsPerSet = (mBitStreamCount + mInputSets - 1)/mInputSets;
    const unsigned outputStreamsPerSet = (mBitStreamCount + mOutputSets - 1)/mOutputSets;

    Value * sourceBlocks[swizzleFactor];
    Value * targetBlocks[swizzleFactor];

    for (unsigned grp = 0; grp < swizzleGroups; grp++) {
        // First load all the data.       
        for (unsigned i = 0; i < swizzleFactor; i++) {
            unsigned streamNo = grp * swizzleFactor + i;
            if (streamNo < mBitStreamCount) {
                unsigned inputSetNo = streamNo / inputStreamsPerSet;
                unsigned j = streamNo % inputStreamsPerSet;
                sourceBlocks[i] = iBuilder->loadInputStreamBlock("inputGroup" + std::to_string(inputSetNo), iBuilder->getInt32(j));
            } else {
                // Fill in the remaining logically required streams of the last swizzle group with null values.
                sourceBlocks[i] = Constant::getNullValue(iBuilder->getBitBlockType());
            }
        }
        // Now perform the swizzle passes.
        for (unsigned p = 0; p < passes; p++) {
            for (unsigned i = 0; i < swizzleFactor / 2; i++) {
                targetBlocks[i * 2] = iBuilder->esimd_mergel(mFieldWidth, sourceBlocks[i], sourceBlocks[i + (swizzleFactor / 2)]);
                targetBlocks[(i * 2) + 1] = iBuilder->esimd_mergeh(mFieldWidth, sourceBlocks[i], sourceBlocks[i + (swizzleFactor / 2)]);
            }
            for (unsigned i = 0; i < swizzleFactor; i++) {
                sourceBlocks[i] = targetBlocks[i];
            }
        }
        for (unsigned i = 0; i < swizzleFactor; i++) {
            unsigned streamNo = grp * swizzleFactor + i;
            unsigned outputSetNo = streamNo / outputStreamsPerSet;
            unsigned j = streamNo % outputStreamsPerSet;
            iBuilder->storeOutputStreamBlock("outputGroup" + std::to_string(outputSetNo), iBuilder->getInt32(j), iBuilder->bitCast(sourceBlocks[i]));
        }
    }
}


}
