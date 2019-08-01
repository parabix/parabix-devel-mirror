/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/scan/reader.h>

#include <kernel/core/kernel_builder.h>

using namespace llvm;

namespace kernel {

void ScanReader::generateMultiBlockLogic(BuilderRef b, Value * const numOfStrides) {
    Module * const module = b->getModule();
    Type * const sizeTy = b->getSizeTy();
    Value * const sz_ZERO = b->getSize(0);
    Value * const sz_ONE = b->getSize(1);

    BasicBlock * const entryBlock = b->GetInsertBlock();
    BasicBlock * const readItem = b->CreateBasicBlock("readItem");
    BasicBlock * const exitBlock = b->CreateBasicBlock("exitBlock");
    Value * const initialStride = b->getProcessedItemCount("scan");
    Value * const isInvalidFinalItem = b->CreateAnd(mIsFinal, b->CreateICmpEQ(b->getSize(0), b->getAccessibleItemCount("scan")));
    b->CreateCondBr(isInvalidFinalItem, exitBlock, readItem);

    b->SetInsertPoint(readItem);
    PHINode * const strideNo = b->CreatePHI(sizeTy, 2);
    strideNo->addIncoming(sz_ZERO, entryBlock);
    Value * const nextStrideNo = b->CreateAdd(strideNo, sz_ONE);
    strideNo->addIncoming(nextStrideNo, readItem);
    std::vector<Value *> callbackParams{};
    Value * maxScanIndex = nullptr;
    Value * const index = b->CreateAdd(strideNo, initialStride);
    for (uint32_t i = 0; i < mNumScanStreams; ++i) {
        Value * const scanItem = b->CreateLoad(b->getRawInputPointer("scan", b->getInt32(i), index));
        if (maxScanIndex != nullptr) {
            maxScanIndex = b->CreateUMax(maxScanIndex, scanItem);
        } else {
            maxScanIndex = scanItem;
        }
        // FIXME: We are assuming that we have access to the entire source stream, this may not always be the case.
        Value * const scanPtr = b->getRawInputPointer("source", scanItem);
        callbackParams.push_back(scanPtr);
    }
    b->setProcessedItemCount("source", maxScanIndex);
    Value * const nextIndex = b->CreateAdd(nextStrideNo, initialStride);
    b->setProcessedItemCount("scan", nextIndex);
    for (auto const & name : mAdditionalStreamNames) {
        Value * const item = b->CreateLoad(b->getRawInputPointer(name, b->getInt32(0), index));
        callbackParams.push_back(item);
        b->setProcessedItemCount(name, nextIndex);
    }
    Function * const callback = module->getFunction(mCallbackName);
    if (callback == nullptr) {
        llvm::report_fatal_error(mKernelName + ": failed to get function: " + mCallbackName);
    }
    b->CreateCall(callback, ArrayRef<Value *>(callbackParams));
    b->CreateCondBr(b->CreateICmpNE(nextStrideNo, numOfStrides), readItem, exitBlock);

    b->SetInsertPoint(exitBlock);
}

static std::string ScanReader_GenerateName(StreamSet * scan, std::string const & callbackName) {
    return "ScanReader_" + std::to_string(scan->getNumElements()) + "xscan" + "_" + std::string(callbackName);
}

static std::string ScanReader_GenerateName(StreamSet * scan, std::string const & callbackName, std::initializer_list<StreamSet *> const & additionalStreams) {
    std::string name = ScanReader_GenerateName(scan, callbackName);
    for (auto const & stream : additionalStreams) {
        name += "_" + std::to_string(stream->getNumElements()) + "xi" + std::to_string(stream->getFieldWidth());
    }
    return name;
}

ScanReader::ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, std::string const & callbackName)
: MultiBlockKernel(b, ScanReader_GenerateName(scanIndices, callbackName), {
    {"scan", scanIndices, BoundedRate(0, 1), Principal()},
    {"source", source, BoundedRate(0, 1)}
  }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
, mNumScanStreams(scanIndices->getNumElements())
{
    assert (scanIndices->getFieldWidth() == 64);
    assert (source->getNumElements() == 1);
    addAttribute(SideEffecting());
    setStride(1);
}

ScanReader::ScanReader(BuilderRef b, StreamSet * source, StreamSet * scanIndices, std::string const & callbackName, std::initializer_list<StreamSet *> additionalStreams)
: MultiBlockKernel(b, ScanReader_GenerateName(scanIndices, callbackName, additionalStreams), {
    {"scan", scanIndices, BoundedRate(0, 1), Principal()},
    {"source", source, BoundedRate(0, 1)}
  }, {}, {}, {}, {})
, mCallbackName(callbackName)
, mAdditionalStreamNames()
, mNumScanStreams(scanIndices->getNumElements())
{
    assert (scanIndices->getFieldWidth() == 64);
    assert (source->getNumElements() == 1);
    addAttribute(SideEffecting());
    setStride(1);
    size_t i = 0;
    for (auto const & stream : additionalStreams) {
        std::string name = "__additional_" + std::to_string(i++);
        mInputStreamSets.push_back({name, stream, BoundedRate(0, 1)});
        mAdditionalStreamNames.push_back(name);
    }
}

}
