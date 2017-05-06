/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streams_merge.h"
#include <IR_Gen/idisa_builder.h>
#include <llvm/IR/Value.h>

using namespace llvm;

namespace kernel {

StreamsMerge::StreamsMerge(const std::unique_ptr<IDISA::IDISA_Builder> & iBuilder, unsigned streamsPerSet, unsigned inputSets)
: BlockOrientedKernel("streamsMerge", {}, {}, {}, {}, {})
, mStreamsPerSet(streamsPerSet)
, mInputSets(inputSets) {
    for (unsigned i = 0; i < mInputSets; i++) {
        mStreamSetInputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "inputGroup" + std::to_string(i)});
    }
    mStreamSetOutputs.push_back(Binding{iBuilder->getStreamSetTy(streamsPerSet, 1), "output"});
}

void StreamsMerge::generateDoBlockMethod() {

    std::vector<Value *> resultStreams;

    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        resultStreams.push_back(loadInputStreamBlock("inputGroup" + std::to_string(0), iBuilder->getInt32(j)));
    }

    for (unsigned i = 1; i < mInputSets; i++) {
        for (unsigned j = 0; j < mStreamsPerSet; j++) {
            resultStreams[j] = iBuilder->CreateOr(resultStreams[j], loadInputStreamBlock("inputGroup" + std::to_string(i), iBuilder->getInt32(j)));
        }
    }
    for (unsigned j = 0; j < mStreamsPerSet; j++) {
        storeOutputStreamBlock("output", iBuilder->getInt32(j), resultStreams[j]);
    }
}


}
