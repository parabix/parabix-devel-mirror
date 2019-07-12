/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/swizzle.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>
#include <string>
#include <vector>

using namespace llvm;

namespace kernel {

inline static bool is_power_2(const uint64_t n) {
    return ((n & (n - 1)) == 0) && n;
}

LLVM_READNONE inline unsigned getBitStreamCount(const std::vector<StreamSet *> & inputs) {
    unsigned count = 0;


    for (StreamSet * input : inputs) {
        count += input->getNumElements();
    }
    return count;
}


inline std::string makeSwizzleName(const std::vector<StreamSet *> & inputs, const std::vector<StreamSet *> & outputs, const unsigned fieldWidth) {
    const auto inputStreamCount = getBitStreamCount(inputs);
    const auto outputStreamCount = getBitStreamCount(outputs);
    if (LLVM_UNLIKELY(inputStreamCount != outputStreamCount)) {
        report_fatal_error("total number of input elements does not match the output elements");
    }
    std::string tmp;
    raw_string_ostream out(tmp);
    out << "swizzle" << fieldWidth << ':' << inputStreamCount << '_' << inputs.size() << '_' << outputs.size();
    out.flush();
    return tmp;
}

inline size_t ceil_udiv(const size_t n, const size_t m) {
    return (n + m - 1) / m;
}

inline Bindings makeSwizzledInputs(const std::vector<StreamSet *> & inputs) {
    Bindings bindings;
    const auto n = inputs.size();
    bindings.reserve(n);
    const auto numElements = inputs[0]->getNumElements();
    for (unsigned i = 0; i < n; ++i) {
        if (LLVM_UNLIKELY(inputs[i]->getNumElements() != numElements)) {
            report_fatal_error("not all inputs have the same number of elements");
        }
        bindings.emplace_back("inputGroup" + std::to_string(i), inputs[i]);
    }
    return bindings;
}

inline Bindings makeSwizzledOutputs(const std::vector<StreamSet *> & outputs, const unsigned fieldWidth) {
    Bindings bindings;
    const auto n = outputs.size();
    bindings.reserve(n);
    const auto numElements = outputs[0]->getNumElements();
    for (unsigned i = 0; i < n; ++i) {
        if (LLVM_UNLIKELY(outputs[i]->getNumElements() != numElements)) {
            report_fatal_error("not all outputs have the same number of elements");
        }
        bindings.emplace_back("outputGroup" + std::to_string(i), outputs[i], FixedRate(1), BlockSize(fieldWidth));
    }
    return bindings;
}

SwizzleGenerator::SwizzleGenerator(const std::unique_ptr<kernel::KernelBuilder> & b,
                                   const std::vector<StreamSet *> & inputs,
                                   const std::vector<StreamSet *> & outputs,
                                   const unsigned fieldWidth)
: BlockOrientedKernel(b, makeSwizzleName(inputs, outputs, fieldWidth),
makeSwizzledInputs(inputs),
makeSwizzledOutputs(outputs, fieldWidth),
{}, {}, {})
, mBitStreamCount(getBitStreamCount(inputs))
, mFieldWidth(fieldWidth) {

}

void SwizzleGenerator::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {

    // We may need a few passes depending on the swizzle factor

    if (LLVM_UNLIKELY(!is_power_2(mFieldWidth))) {
        report_fatal_error("fieldWidth must be a power of 2");
    }
    if (LLVM_UNLIKELY(mFieldWidth > b->getBitBlockWidth())) {
        report_fatal_error("fieldWidth must be a power of 2");
    }

    const auto swizzleFactor = b->getBitBlockWidth() / mFieldWidth;
    const auto passes = std::log2(swizzleFactor);
    const auto swizzleGroups = ceil_udiv(mBitStreamCount, swizzleFactor);
    const auto inputStreamsPerSet = ceil_udiv(mBitStreamCount, getNumOfStreamInputs());
    const auto outputStreamsPerSet = ceil_udiv(mBitStreamCount, getNumOfStreamOutputs());

    SmallVector<Value *, 16> sourceBlocks(swizzleFactor);
    SmallVector<Value *, 16> targetBlocks(swizzleFactor);
    for (unsigned grp = 0; grp < swizzleGroups; grp++) {
        // First load all the data.
        for (unsigned i = 0; i < swizzleFactor; i++) {
            const auto streamNo = grp * swizzleFactor + i;
            if (streamNo < mBitStreamCount) {
                const auto inputSetNo = streamNo / inputStreamsPerSet;
                const auto j = streamNo % inputStreamsPerSet;
                sourceBlocks[i] = b->loadInputStreamBlock("inputGroup" + std::to_string(inputSetNo), b->getInt32(j));
            } else {
                // Fill in the remaining logically required streams of the last swizzle group with null values.
                sourceBlocks[i] = Constant::getNullValue(b->getBitBlockType());
            }
        }
        // Now perform the swizzle passes.
        for (unsigned p = 0; p < passes; p++) {
            for (unsigned i = 0; i < swizzleFactor / 2; i++) {
                targetBlocks[i * 2] = b->esimd_mergel(mFieldWidth, sourceBlocks[i], sourceBlocks[i + (swizzleFactor / 2)]);
                targetBlocks[(i * 2) + 1] = b->esimd_mergeh(mFieldWidth, sourceBlocks[i], sourceBlocks[i + (swizzleFactor / 2)]);
            }
            for (unsigned i = 0; i < swizzleFactor; i++) {
                sourceBlocks[i] = targetBlocks[i];
            }
        }
        for (unsigned i = 0; i < swizzleFactor; i++) {
            unsigned streamNo = grp * swizzleFactor + i;
            unsigned outputSetNo = streamNo / outputStreamsPerSet;
            unsigned j = streamNo % outputStreamsPerSet;
            b->storeOutputStreamBlock("outputGroup" + std::to_string(outputSetNo), b->getInt32(j), b->bitCast(sourceBlocks[i]));
        }
    }
}


SwizzleByGather::SwizzleByGather(const std::unique_ptr<KernelBuilder> & b)
: BlockOrientedKernel(b, "swizzleByGather", {}, {}, {}, {}, {}){
    for (unsigned i = 0; i < 2; i++) {
        mInputStreamSets.push_back(Binding{b->getStreamSetTy(4, 1), "inputGroup" + std::to_string(i)});
    }
    for (unsigned i = 0; i < 1; i++) {
        mOutputStreamSets.push_back(Binding{b->getStreamSetTy(8, 1), "outputGroup" + std::to_string(i), FixedRate(1)});
    }
}

void SwizzleByGather::generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> &b) {
    Value* outputStreamPtr = b->getOutputStreamBlockPtr("outputGroup0", b->getSize(0));

    for (unsigned i = 0; i < 2; i++) {
        std::vector<llvm::Value*> inputStream;
        Value* inputPtr = b->getInputStreamBlockPtr("inputGroup" + std::to_string(i), b->getSize(0));

        Value* inputBytePtr = b->CreatePointerCast(inputPtr, b->getInt8PtrTy());
        Function *gatherFunc = Intrinsic::getDeclaration(b->getModule(), Intrinsic::x86_avx2_gather_d_q_256);
        Value *addresses = ConstantVector::get(
                {b->getInt32(0), b->getInt32(32), b->getInt32(64), b->getInt32(96)});

        for (unsigned j = 0; j < 4; j++) {
            Value *gather_result = b->CreateCall(
                    gatherFunc,
                    {
                            UndefValue::get(b->getBitBlockType()),
                            inputBytePtr,
                            addresses,
                            Constant::getAllOnesValue(b->getBitBlockType()),
                            b->getInt8(1)
                    }
            );

            inputBytePtr = b->CreateGEP(inputBytePtr, b->getInt32(8));

            b->CreateStore(gather_result, outputStreamPtr);
            outputStreamPtr = b->CreateGEP(outputStreamPtr, b->getSize(1));
        }
    }
}

}
