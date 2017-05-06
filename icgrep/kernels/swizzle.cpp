/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "swizzle.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Value.h>

using namespace llvm;

namespace kernel {

SwizzleGenerator::SwizzleGenerator(const std::unique_ptr<IDISA::IDISA_Builder> & iBuilder, unsigned bitStreamCount, unsigned outputSets, unsigned inputSets, unsigned fieldWidth)
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

void SwizzleGenerator::generateDoBlockMethod() {
        
    // We may need a few passes depending on the swizzle factor
    unsigned passes = std::log2(mSwizzleFactor);
    
    unsigned swizzleGroups = (mBitStreamCount + mSwizzleFactor - 1)/mSwizzleFactor;
    unsigned inputStreamsPerSet = (mBitStreamCount + mInputSets - 1)/mInputSets;
    unsigned outputStreamsPerSet = (mBitStreamCount + mOutputSets - 1)/mOutputSets;

    for (unsigned grp = 0; grp < swizzleGroups; grp++) {
        // First load all the data.
        std::vector<Value *> sourceBlocks;        
        std::vector<Value *> targetBlocks;        
        for (unsigned i = 0; i < mSwizzleFactor; i++) {
            unsigned streamNo = grp * mSwizzleFactor + i;
            if (streamNo < mBitStreamCount) {
                unsigned inputSetNo = streamNo / inputStreamsPerSet;
                unsigned j = streamNo % inputStreamsPerSet;
                sourceBlocks.push_back(loadInputStreamBlock("inputGroup" + std::to_string(inputSetNo), iBuilder->getInt32(j)));
            }
            else {
                // Fill in the remaining logically required streams of the last swizzle group with null values.
                sourceBlocks.push_back(Constant::getNullValue(iBuilder->getBitBlockType()));
            }
        }
        // Now perform the swizzle passes.
        for (unsigned p = 0; p < passes; p++) {
            std::vector<Value *> targetBlocks;
            for (unsigned i = 0; i < mSwizzleFactor/2; i++) {
                targetBlocks.push_back(iBuilder->esimd_mergel(mFieldWidth, sourceBlocks[i], sourceBlocks[i+mSwizzleFactor/2]));
                targetBlocks.push_back(iBuilder->esimd_mergeh(mFieldWidth, sourceBlocks[i], sourceBlocks[i+mSwizzleFactor/2]));
            }
            sourceBlocks = targetBlocks;
        }
        for (unsigned i = 0; i < mSwizzleFactor; i++) {
            unsigned streamNo = grp * mSwizzleFactor + i;
            unsigned outputSetNo = streamNo / outputStreamsPerSet;
            unsigned j = streamNo % outputStreamsPerSet;
            storeOutputStreamBlock("outputGroup" + std::to_string(outputSetNo), iBuilder->getInt32(j), iBuilder->bitCast(sourceBlocks[i]));
        }
    }
}


}
