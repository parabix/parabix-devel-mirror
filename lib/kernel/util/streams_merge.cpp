/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "streams_merge.h"
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

std::string makeKernelName(const std::string & prefix, const std::vector<StreamSet *> & inputs, StreamSet * const output) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << prefix << inputs.size();
    char joiner = 'x';
    unsigned maxNumOfInputStreams = 0;
    for (unsigned i = 0; i < inputs.size(); ++i) {
        if (LLVM_UNLIKELY(inputs[i]->getFieldWidth() != 1)) {
            report_fatal_error("input streams must be of field width 1");
        }
        maxNumOfInputStreams = std::max(maxNumOfInputStreams, inputs[i]->getNumElements());
        out << joiner << inputs[i]->getNumElements();
        joiner = '_';
    }
    if (LLVM_UNLIKELY(output->getNumElements() < maxNumOfInputStreams)) {
        report_fatal_error("output streamset requires " + std::to_string(maxNumOfInputStreams) + " streams");
    }
    out << ':' << output->getNumElements();
    out.flush();
    return tmp;
}

StreamsMerge::StreamsMerge(const std::unique_ptr<kernel::KernelBuilder> & b, const std::vector<StreamSet *> & inputs, StreamSet * output)
: BlockOrientedKernel(b, makeKernelName("streamsMerge", inputs, output), {}, {}, {}, {}, {}) {
    for (unsigned i = 0; i < inputs.size(); i++) {
        mInputStreamSets.push_back(Binding{"input" + std::to_string(i), inputs[i]});
    }
    mOutputStreamSets.push_back(Binding{"output", output});
}

void StreamsMerge::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {

    const auto n = getOutputStreamSet(0)->getNumElements();
    std::vector<Value *> resultStreams(n, nullptr);
    for (unsigned i = 0; i < getNumOfStreamInputs(); ++i) {
        const auto m = getInputStreamSet(i)->getNumElements(); assert (m <= n);
        for (unsigned j = 0; j < m; ++j) {
            Value * const inputValue = iBuilder->loadInputStreamBlock("input" + std::to_string(i), iBuilder->getInt32(j));
            if (resultStreams[j]) {
                resultStreams[j] = iBuilder->CreateOr(resultStreams[j], inputValue);
            } else {
                resultStreams[j] = inputValue;
            }
        }
    }

    for (unsigned j = 0; j < n; j++) {
        Value * output = resultStreams[j];
        if (LLVM_UNLIKELY(output == nullptr)) {
            output = iBuilder->allZeroes();
        }
        iBuilder->storeOutputStreamBlock("output", iBuilder->getInt32(j), output);
    }
}

StreamsIntersect::StreamsIntersect(const std::unique_ptr<kernel::KernelBuilder> & b, const std::vector<StreamSet *> & inputs, StreamSet * output)
: BlockOrientedKernel(b, makeKernelName("streamsIntersect", inputs, output), {}, {}, {}, {}, {}) {
    for (unsigned i = 0; i < inputs.size(); i++) {
        mInputStreamSets.push_back(Binding{"input" + std::to_string(i), inputs[i]});
    }
    mOutputStreamSets.push_back(Binding{"output", output});
}

void StreamsIntersect::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> &iBuilder) {
    const auto n = getOutputStreamSet(0)->getNumElements();
    std::vector<Value *> resultStreams(n, nullptr);
    for (unsigned i = 0; i < getNumOfStreamInputs(); ++i) {
        const auto m = getInputStreamSet(i)->getNumElements(); assert (m <= n);
        for (unsigned j = 0; j < m; ++j) {
            Value * const inputValue = iBuilder->loadInputStreamBlock("input" + std::to_string(i), iBuilder->getInt32(j));
            if (resultStreams[j]) {
                resultStreams[j] = iBuilder->CreateAnd(resultStreams[j], inputValue);
            } else {
                resultStreams[j] = inputValue;
            }
        }
    }

    for (unsigned j = 0; j < n; j++) {
        Value * output = resultStreams[j];
        if (LLVM_UNLIKELY(output == nullptr)) {
            output = iBuilder->allZeroes();
        }
        iBuilder->storeOutputStreamBlock("output", iBuilder->getInt32(j), output);
    }
}

StreamsCombineKernel::StreamsCombineKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                                                     std::vector<unsigned> streamsNumOfSets)
: BlockOrientedKernel(b, "StreamsCombineKernel" , {}, {}, {}, {}, {})
, mStreamsNumOfSets(streamsNumOfSets) {
    int total = 0;
    for (unsigned i = 0; i < streamsNumOfSets.size(); i++) {
        total += streamsNumOfSets[i];
        mInputStreamSets.push_back(Binding{b->getStreamSetTy(streamsNumOfSets[i], 1), "inputGroup" + std::to_string(i)});
    }
    mOutputStreamSets.push_back(Binding{b->getStreamSetTy(total, 1), "output"});
}

void StreamsCombineKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    unsigned outputIndex = 0;
    for (unsigned i = 0; i < mStreamsNumOfSets.size(); i++) {
        unsigned streamNum = mStreamsNumOfSets[i];
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


StreamsSplitKernel::StreamsSplitKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
                                       std::vector<unsigned> streamsNumOfSets)
: BlockOrientedKernel(b, "StreamsSplitKernel" , {}, {}, {}, {}, {})
, mStreamsNumOfSets(streamsNumOfSets){
    int total = 0;
    for (unsigned i = 0; i < streamsNumOfSets.size(); i++) {
        total += streamsNumOfSets[i];
        mOutputStreamSets.push_back(Binding{b->getStreamSetTy(streamsNumOfSets[i], 1), "outputGroup" + std::to_string(i)});
    }
    mInputStreamSets.push_back(Binding{b->getStreamSetTy(total, 1), "input"});
}

void StreamsSplitKernel::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &iBuilder) {
    unsigned inputIndex = 0;
    for (unsigned i = 0; i < mStreamsNumOfSets.size(); i++) {
        unsigned streamNum = mStreamsNumOfSets[i];
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
