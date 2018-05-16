/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streams_merge.h"
#include <kernels/kernel_builder.h>

using namespace llvm;

namespace kernel {

StreamsMerge::StreamsMerge(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamsPerSet, unsigned inputSets)
    : BlockOrientedKernel("streamsMerge" + std::to_string(streamsPerSet) + "_" + std::to_string(inputSets) , {}, {}, {}, {}, {})
, mStreamsPerSet(streamsPerSet)
, mInputSets(inputSets) {
    for (unsigned i = 0; i < mInputSets; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "inputGroup" + std::to_string(i)});
    }
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "output"});
}

void StreamsMerge::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {

    std::vector<Value *> resultStreams;

    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        resultStreams.push_back(iBuilder->loadInputStreamBlock("inputGroup" + std::to_string(0), iBuilder->getInt32(j)));
    }

    for (unsigned i = 1; i < mInputSets; i++) {
        for (unsigned j = 0; j < mStreamsPerSet; j++) {
            resultStreams[j] = iBuilder->CreateOr(resultStreams[j], iBuilder->loadInputStreamBlock("inputGroup" + std::to_string(i), iBuilder->getInt32(j)));
        }
    }
    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        iBuilder->storeOutputStreamBlock("output", iBuilder->getInt32(j), resultStreams[j]);
    }
}

StreamsIntersect::StreamsIntersect(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamsPerSet, unsigned inputSets)
: BlockOrientedKernel("streamsIntersect" + std::to_string(streamsPerSet) + "_" + std::to_string(inputSets) , {}, {}, {}, {}, {})
, mStreamsPerSet(streamsPerSet)
, mInputSets(inputSets) {
    for (unsigned i = 0; i < mInputSets; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "inputGroup" + std::to_string(i)});
    }
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "output"});
}

void StreamsIntersect::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
    
    std::vector<Value *> resultStreams;
    
    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        resultStreams.push_back(iBuilder->loadInputStreamBlock("inputGroup" + std::to_string(0), iBuilder->getInt32(j)));
    }
    
    for (unsigned i = 1; i < mInputSets; i++) {
        for (unsigned j = 0; j < mStreamsPerSet; j++) {
            resultStreams[j] = iBuilder->CreateAnd(resultStreams[j], iBuilder->loadInputStreamBlock("inputGroup" + std::to_string(i), iBuilder->getInt32(j)));
        }
    }
    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        iBuilder->storeOutputStreamBlock("output", iBuilder->getInt32(j), resultStreams[j]);
    }
}

StreamsCombineKernel::StreamsCombineKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder,
                                                     std::vector<unsigned> streamsNumOfSets)
        : BlockOrientedKernel("StreamsCombineKernel" , {}, {}, {}, {}, {}),
          mStreamsNumOfSets(streamsNumOfSets) {
    int total = 0;
    for (unsigned i = 0; i < streamsNumOfSets.size(); i++) {
        total += streamsNumOfSets[i];
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamsNumOfSets[i], 1), "inputGroup" + std::to_string(i)});
    }
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(total, 1), "output"});
}

void StreamsCombineKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    unsigned outputIndex = 0;
    for (unsigned i = 0; i < mStreamsNumOfSets.size(); i++) {
        int streamNum = mStreamsNumOfSets[i];
        for (unsigned j = 0; j < streamNum; j++) {
            iBuilder->storeOutputStreamBlock(
                    "output",
                    iBuilder->getInt32(outputIndex),
                    iBuilder->loadInputStreamBlock(
                            "inputGroup" + std::to_string(i),
                            iBuilder->getInt32(j)
                    )
            );
            ++outputIndex;
        }
    }
}


StreamsSplitKernel::StreamsSplitKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder,
                                       std::vector<unsigned> streamsNumOfSets)
        : BlockOrientedKernel("StreamsSplitKernel" , {}, {}, {}, {}, {}),
          mStreamsNumOfSets(streamsNumOfSets){
    int total = 0;
    for (unsigned i = 0; i < streamsNumOfSets.size(); i++) {
        total += streamsNumOfSets[i];
        mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamsNumOfSets[i], 1), "outputGroup" + std::to_string(i)});
    }
    mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(total, 1), "input"});
}

void StreamsSplitKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    unsigned inputIndex = 0;
    for (unsigned i = 0; i < mStreamsNumOfSets.size(); i++) {
        int streamNum = mStreamsNumOfSets[i];
        for (unsigned j = 0; j < streamNum; j++) {
            iBuilder->storeOutputStreamBlock(
                    "outputGroup" + std::to_string(i),
                    iBuilder->getInt32(j),
                    iBuilder->loadInputStreamBlock("input", iBuilder->getInt32(inputIndex))
            );
            ++inputIndex;
        }
    }
}
}
