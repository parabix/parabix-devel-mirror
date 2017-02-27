/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Verifier.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Transforms/Utils/Local.h>

static const auto DO_BLOCK_SUFFIX = "_DoBlock";

static const auto FINAL_BLOCK_SUFFIX = "_FinalBlock";

static const auto LOGICAL_SEGMENT_NO_SCALAR = "logicalSegNo";

static const auto PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";

static const auto PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";

static const auto TERMINATION_SIGNAL = "terminationSignal";

static const auto BUFFER_PTR_SUFFIX = "_bufferPtr";

static const auto BLOCK_MASK_SUFFIX = "_blkMask";

using namespace llvm;
using namespace kernel;
using namespace parabix;

unsigned KernelBuilder::addScalar(Type * const type, const std::string & name) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add field " + name + " to " + getName() + " after kernel state finalized");
    }
    if (LLVM_UNLIKELY(mKernelMap.count(name))) {
        report_fatal_error(getName() + " already contains scalar field " + name);
    }
    const auto index = mKernelFields.size();
    mKernelMap.emplace(name, index);
    mKernelFields.push_back(type);
    return index;
}

unsigned KernelBuilder::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed kernel field after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}

void KernelBuilder::prepareKernelSignature() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamSetNameMap.emplace(mStreamSetInputs[i].name, i);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamSetNameMap.emplace(mStreamSetOutputs[i].name, i);
    }
}
    
void KernelBuilder::prepareKernel() {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot prepare kernel after kernel state finalized");
    }
    if (mStreamSetInputs.size() != mStreamSetInputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetInputBuffers.size() << " input buffers for "
            << mStreamSetInputs.size() << " input stream sets.";
        report_fatal_error(out.str());
    }
    if (mStreamSetOutputs.size() != mStreamSetOutputBuffers.size()) {
        std::string tmp;
        raw_string_ostream out(tmp);
        out << "kernel contains " << mStreamSetOutputBuffers.size() << " output buffers for "
            << mStreamSetOutputs.size() << " output stream sets.";
        report_fatal_error(out.str());
    }
    const auto blockSize = iBuilder->getBitBlockWidth();
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferBlocks() > 0) && (mStreamSetInputBuffers[i]->getBufferBlocks() < codegen::SegmentSize + (blockSize + mLookAheadPositions - 1)/blockSize)) {
            report_fatal_error("Kernel preparation: Buffer size too small " + mStreamSetInputs[i].name);
        }
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getPointerType(), mStreamSetInputs[i].name + BUFFER_PTR_SUFFIX);
        if ((i == 0) || !mStreamSetInputs[i].rate.isExact()) {
            addScalar(iBuilder->getSizeTy(), mStreamSetInputs[i].name + PROCESSED_ITEM_COUNT_SUFFIX);
        }
        
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getPointerType(), mStreamSetOutputs[i].name + BUFFER_PTR_SUFFIX);
        if ((mStreamSetInputs.empty() && (i == 0)) || !mStreamSetOutputs[i].rate.isExact()) {
            addScalar(iBuilder->getSizeTy(), mStreamSetOutputs[i].name + PRODUCED_ITEM_COUNT_SUFFIX);
        }
    }
    for (const auto binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (const auto binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    if (mStreamSetNameMap.empty()) {
        prepareKernelSignature();
    }
    for (auto binding : mInternalScalars) {
        addScalar(binding.type, binding.name);
    }
    addScalar(iBuilder->getSizeTy(), LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(iBuilder->getInt1Ty(), TERMINATION_SIGNAL);
    mKernelStateType = StructType::create(iBuilder->getContext(), mKernelFields, getName());
}

std::unique_ptr<Module> KernelBuilder::createKernelModule(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    auto saveModule = iBuilder->getModule();
    auto savePoint = iBuilder->saveIP();
    auto module = make_unique<Module>(getName() + "_" + iBuilder->getBitBlockTypeName(), iBuilder->getContext());
    iBuilder->setModule(module.get());
    generateKernel(inputs, outputs);
    iBuilder->setModule(saveModule);
    iBuilder->restoreIP(savePoint);
    return module;
}

void KernelBuilder::generateKernel(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {

    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); ++i) {
        if (LLVM_UNLIKELY(mStreamSetInputBuffers[i] == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " cannot be null when calling generateKernel()");
        }
    }
    if (LLVM_UNLIKELY(mStreamSetInputs.size() != mStreamSetInputBuffers.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetInputs.size()) +
                           " input stream sets but generateKernel() was given "
                           + std::to_string(mStreamSetInputBuffers.size()));
    }

    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        if (LLVM_UNLIKELY(mStreamSetOutputBuffers[i] == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " cannot be null when calling generateKernel()");
        }
    }
    if (LLVM_UNLIKELY(mStreamSetOutputs.size() != mStreamSetOutputBuffers.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetOutputs.size())
                           + " output stream sets but generateKernel() was given "
                           + std::to_string(mStreamSetOutputBuffers.size()));
    }


    auto savePoint = iBuilder->saveIP();
    prepareKernel(); // possibly overridden by the KernelBuilder subtype
    addKernelDeclarations(iBuilder->getModule());
    callGenerateInitMethod();
    callGenerateDoSegmentMethod();
    // Implement the accumulator get functions
    for (auto binding : mScalarOutputs) {
        Function * f = getAccumulatorFunction(binding.name);
        iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "get_" + binding.name, f));
        Value * self = &*(f->arg_begin());
        Value * ptr = iBuilder->CreateGEP(self, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        Value * retVal = iBuilder->CreateLoad(ptr);
        iBuilder->CreateRet(retVal);
    }
    iBuilder->restoreIP(savePoint);
}

void KernelBuilder::callGenerateDoSegmentMethod() {
    mCurrentMethod = getDoSegmentFunction();
    iBuilder->SetInsertPoint(CreateBasicBlock(getName() + "_entry"));
    auto args = mCurrentMethod->arg_begin();
    mSelf = &*(args++);
    Value * doFinal = &*(args++);
    std::vector<Value *> producerPos;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        producerPos.push_back(&*(args++));
    }
    generateDoSegmentMethod(doFinal, producerPos); // must be overridden by the KernelBuilder subtype
    iBuilder->CreateRetVoid();
}

void KernelBuilder::callGenerateInitMethod() {
    mCurrentMethod = getInitFunction();
    iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    mSelf = &*(args++);
    iBuilder->CreateStore(ConstantAggregateZero::get(mKernelStateType), mSelf);
    for (auto binding : mScalarInputs) {
        Value * param = &*(args++);
        Value * ptr = iBuilder->CreateGEP(mSelf, {iBuilder->getInt32(0), getScalarIndex(binding.name)});
        iBuilder->CreateStore(param, ptr);
    }
    generateInitMethod();
    iBuilder->CreateRetVoid();
}

ConstantInt * KernelBuilder::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return iBuilder->getInt32(f->second);
}

unsigned KernelBuilder::getScalarCount() const {
    return mKernelFields.size();
}

Value * KernelBuilder::getScalarFieldPtr(Value * instance, Value * index) const {
    return iBuilder->CreateGEP(instance, {iBuilder->getInt32(0), index});
}

Value * KernelBuilder::getScalarFieldPtr(Value * instance, const std::string & fieldName) const {
    return getScalarFieldPtr(instance, getScalarIndex(fieldName));
}

Value * KernelBuilder::getScalarField(Value * instance, const std::string & fieldName) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(instance, fieldName));
}

Value * KernelBuilder::getScalarField(Value * instance, Value * index) const {
    return iBuilder->CreateLoad(getScalarFieldPtr(instance, index));
}

void KernelBuilder::setScalarField(Value * instance, const std::string & fieldName, Value * value) const {
    iBuilder->CreateStore(value, getScalarFieldPtr(instance, fieldName));
}

void KernelBuilder::setScalarField(Value * instance, Value * index, Value * value) const {
    iBuilder->CreateStore(value, getScalarFieldPtr(instance, index));
}

Value * KernelBuilder::getProcessedItemCount(Value * instance, const std::string & name) const {
    unsigned ssIdx = getStreamSetIndex(name);
    if (mStreamSetInputs[ssIdx].rate.isExact()) {
        Value * principalItemsProcessed = getScalarField(instance, mStreamSetInputs[0].name + PROCESSED_ITEM_COUNT_SUFFIX);
        return mStreamSetInputs[ssIdx].rate.CreateRatioCalculation(iBuilder, principalItemsProcessed);
    }
    return getScalarField(instance, name + PROCESSED_ITEM_COUNT_SUFFIX);
}

Value * KernelBuilder::getProducedItemCount(Value * instance, const std::string & name) const {
    unsigned ssIdx = getStreamSetIndex(name);
    if (mStreamSetOutputs[ssIdx].rate.isExact()) {
        std::string principalField = mStreamSetInputs.empty() ? mStreamSetOutputs[0].name + PRODUCED_ITEM_COUNT_SUFFIX : mStreamSetInputs[0].name + PROCESSED_ITEM_COUNT_SUFFIX;
        Value * principalItemsProcessed = getScalarField(instance, principalField);
        return mStreamSetOutputs[ssIdx].rate.CreateRatioCalculation(iBuilder, principalItemsProcessed);
    }
    return getScalarField(instance, name + PRODUCED_ITEM_COUNT_SUFFIX);
}

void KernelBuilder::setProcessedItemCount(Value * instance, const std::string & name, Value * value) const {
    setScalarField(instance, name + PROCESSED_ITEM_COUNT_SUFFIX, value);
}

void KernelBuilder::setProducedItemCount(Value * instance, const std::string & name, Value * value) const {
    setScalarField(instance, name + PRODUCED_ITEM_COUNT_SUFFIX, value);
}

Value * KernelBuilder::getTerminationSignal(Value * instance) const {
    return getScalarField(instance, TERMINATION_SIGNAL);
}

void KernelBuilder::setTerminationSignal(Value * instance) const {
    setScalarField(instance, TERMINATION_SIGNAL, iBuilder->getInt1(true));
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo(Value * instance) const {
    return iBuilder->CreateAtomicLoadAcquire(getScalarFieldPtr(instance, LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * instance, Value * newCount) const {
    iBuilder->CreateAtomicStoreRelease(newCount, getScalarFieldPtr(instance, LOGICAL_SEGMENT_NO_SCALAR));
}

inline Value * KernelBuilder::computeBlockIndex(const std::vector<Binding> & bindings, const std::string & name, Value * itemCount) const {
    for (const Binding & b : bindings) {
        if (b.name == name) {
            const auto divisor = iBuilder->getBitBlockWidth();
            if (LLVM_LIKELY((divisor & (divisor - 1)) == 0)) {
                return iBuilder->CreateLShr(itemCount, std::log2(divisor));
            } else {
                return iBuilder->CreateUDiv(itemCount, iBuilder->getSize(divisor));
            }
        }
    }
    report_fatal_error("Error: no binding in " + getName() + " for " + name);
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * streamIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamPackPtr(getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, true);
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex) const {
    return iBuilder->CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * KernelBuilder::getInputStreamSetCount(const std::string & name) const {
    return getInputStreamSetBuffer(name)->getStreamSetCount(getStreamSetBufferPtr(name));
}

llvm::Value * KernelBuilder::getAdjustedInputStreamBlockPtr(Value * blockAdjustment, const std::string & name, llvm::Value * streamIndex) const {
    Value * blockIndex = computeBlockIndex(mStreamSetInputs, name, getProcessedItemCount(name));
    blockIndex = iBuilder->CreateAdd(blockIndex, blockAdjustment);
    const StreamSetBuffer * const buf = getInputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(getStreamSetBufferPtr(name), streamIndex, blockIndex, true);
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamBlockPtr(getStreamSetBufferPtr(name), streamIndex, blockIndex, false);
}

void KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) const {
    Value * const blockIndex = computeBlockIndex(mStreamSetOutputs, name, getProducedItemCount(name));
    const StreamSetBuffer * const buf = getOutputStreamSetBuffer(name);
    return buf->getStreamPackPtr(getStreamSetBufferPtr(name), streamIndex, blockIndex, packIndex, false);
}

void KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * toStore) const {
    return iBuilder->CreateBlockAlignedStore(toStore, getOutputStreamPackPtr(name, streamIndex, packIndex));
}

llvm::Value * KernelBuilder::getOutputStreamSetCount(const std::string & name) const {
    return getOutputStreamSetBuffer(name)->getStreamSetCount(getStreamSetBufferPtr(name));
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getInputStreamSetBuffer(name)->getRawItemPointer(getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * streamIndex, Value * absolutePosition) const {
    return getOutputStreamSetBuffer(name)->getRawItemPointer(getStreamSetBufferPtr(name), streamIndex, absolutePosition);
}

unsigned KernelBuilder::getStreamSetIndex(const std::string & name) const {
    const auto f = mStreamSetNameMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamSetNameMap.end())) {
        report_fatal_error(getName() + " does not contain stream set: " + name);
    }
    return f->second;
}

Value * KernelBuilder::getStreamSetBufferPtr(const std::string & name) const {
    return getScalarField(getSelf(), name + BUFFER_PTR_SUFFIX);
}

Argument * KernelBuilder::getParameter(Function * const f, const std::string & name) const {
    for (auto & arg : f->getArgumentList()) {
        if (arg.getName().equals(name)) {
            return &arg;
        }
    }
    report_fatal_error(getName() + " does not have parameter " + name);
}

Value * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) const {
    return iBuilder->CreateCall(getDoSegmentFunction(), args);
}

Value * KernelBuilder::createGetAccumulatorCall(Value * self, const std::string & accumName) const {
    return iBuilder->CreateCall(getAccumulatorFunction(accumName), {self});
}

BasicBlock * KernelBuilder::CreateBasicBlock(std::string && name) const {
    return BasicBlock::Create(iBuilder->getContext(), name, mCurrentMethod);
}

void KernelBuilder::createInstance() {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot instantiate " + getName() + " before calling prepareKernel()");
    }
    mKernelInstance = iBuilder->CreateCacheAlignedAlloca(mKernelStateType);

    std::vector<Value *> args;
    args.reserve(mInitialArguments.size() + mStreamSetInputBuffers.size() + mStreamSetOutputBuffers.size() + 1);
    args.push_back(mKernelInstance);
    for (unsigned i = 0; i < mInitialArguments.size(); ++i) {
        Value * arg = mInitialArguments[i];
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": initial argument " + std::to_string(i)
                               + " cannot be null when calling createInstance()");
        }
        args.push_back(arg);
    }
    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); ++i) {
        assert (mStreamSetInputBuffers[i]);
        Value * arg = mStreamSetInputBuffers[i]->getStreamSetBasePtr();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetInputs.size() == mStreamSetInputBuffers.size());
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        assert (mStreamSetOutputBuffers[i]);
        Value * arg = mStreamSetOutputBuffers[i]->getStreamSetBasePtr();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetOutputs.size() == mStreamSetOutputBuffers.size());
    iBuilder->CreateCall(getInitFunction(), args);
}

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.

void BlockOrientedKernel::generateDoSegmentMethod(Value * doFinal, const std::vector<Value *> & producerPos) {

    BasicBlock * const entryBlock = iBuilder->GetInsertBlock();
    BasicBlock * const strideLoopCond = CreateBasicBlock(getName() + "_strideLoopCond");
    mStrideLoopBody = CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = CreateBasicBlock(getName() + "_stridesDone");

    ConstantInt * stride = iBuilder->getSize(iBuilder->getStride());
    Value * availablePos = producerPos[0];
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * p = producerPos[i];
        availablePos = iBuilder->CreateSelect(iBuilder->CreateICmpULT(availablePos, p), availablePos, p);
    }

    Value * processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsAvail = iBuilder->CreateSub(availablePos, processed);
    Value * stridesToDo = iBuilder->CreateUDiv(itemsAvail, stride);

    iBuilder->CreateBr(strideLoopCond);

    iBuilder->SetInsertPoint(strideLoopCond);
    PHINode * stridesRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "stridesRemaining");
    stridesRemaining->addIncoming(stridesToDo, entryBlock);
    Value * notDone = iBuilder->CreateICmpNE(stridesRemaining, iBuilder->getSize(0));
    iBuilder->CreateCondBr(notDone, mStrideLoopBody, stridesDone);

    iBuilder->SetInsertPoint(mStrideLoopBody);

    if (useIndirectBr()) {
        mStrideLoopBranchAddress = iBuilder->CreatePHI(iBuilder->getInt8PtrTy(), 2);
        mStrideLoopBranchAddress->addIncoming(BlockAddress::get(strideLoopCond), strideLoopCond);        
    }

    /// GENERATE DO BLOCK METHOD

    writeDoBlockMethod();

    /// UPDATE PROCESSED COUNTS

    processed = getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsDone = iBuilder->CreateAdd(processed, stride);
    setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);

    stridesRemaining->addIncoming(iBuilder->CreateSub(stridesRemaining, iBuilder->getSize(1)), iBuilder->GetInsertBlock());

    if (useIndirectBr()) {
        mStrideLoopBranch = iBuilder->CreateIndirectBr(mStrideLoopBranchAddress, 2);
        mStrideLoopBranch->addDestination(strideLoopCond);
    } else {
        iBuilder->CreateBr(strideLoopCond);
    }

    iBuilder->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    BasicBlock * const doFinalBlock = CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = CreateBasicBlock(getName() + "_segmentDone");
    iBuilder->CreateCondBr(doFinal, doFinalBlock, segmentDone);
    iBuilder->SetInsertPoint(doFinalBlock);

    Value * remainingItems = iBuilder->CreateSub(producerPos[0], getProcessedItemCount(mStreamSetInputs[0].name));
    writeFinalBlockMethod(remainingItems);
    // if remainingItems was not used, this will eliminate it.
    RecursivelyDeleteTriviallyDeadInstructions(remainingItems);

    itemsDone = producerPos[0];
    setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);
    setTerminationSignal();
    iBuilder->CreateBr(segmentDone);

    if (useIndirectBr()) {
        const auto destinations = mStrideLoopBranch->getNumDestinations();
        assert (mStrideLoopBranchAddress->getNumIncomingValues() == destinations);
        if (destinations == 1) {
            // Final block does not call DoBlock. Replace the indirect branch with a direct one.
            iBuilder->SetInsertPoint(mStrideLoopBranch);
            iBuilder->CreateBr(strideLoopCond);
            mStrideLoopBranch->eraseFromParent();
            mStrideLoopBranch = nullptr;
            mStrideLoopBranchAddress->eraseFromParent();
            mStrideLoopBranchAddress = nullptr;
        } else {
            MDBuilder mdb(iBuilder->getContext());
            uint32_t weights[destinations] = { 100, 0 };
            ArrayRef<uint32_t> bw(weights, destinations);
            mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(bw));
        }
    }

    segmentDone->moveAfter(iBuilder->GetInsertBlock());

    iBuilder->SetInsertPoint(segmentDone);

}

void BlockOrientedKernel::writeDoBlockMethod() {

    Value * const self = mSelf;
    Function * const cp = mCurrentMethod;
    auto ip = iBuilder->saveIP();

    /// Check if the do block method is called and create the function if necessary    
    if (!useIndirectBr()) {
        FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), {mSelf->getType()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::ExternalLinkage, getName() + DO_BLOCK_SUFFIX, iBuilder->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        mCurrentMethod = mCurrentMethod;
        mSelf = &*args;
        mSelf->setName("self");
        iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    }

    std::vector<Value *> priorProduced;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        if (isa<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            priorProduced.push_back(getProducedItemCount(mStreamSetOutputs[i].name));
        }
    }

    generateDoBlockMethod(); // must be implemented by the BlockOrientedKernelBuilder subtype

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        unsigned priorIdx = 0;
        if (auto cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = iBuilder->CreateSub(getProducedItemCount(mStreamSetOutputs[i].name), priorProduced[priorIdx]);
            Value * accessible = cb->getLinearlyAccessibleItems(priorProduced[priorIdx]);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(getStreamSetBufferPtr(mStreamSetOutputs[i].name), copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
    }

    /// Call the do block method if necessary then restore the current function state to the do segement method

    if (!useIndirectBr()) {
        iBuilder->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        iBuilder->restoreIP(ip);
        iBuilder->CreateCall(mCurrentMethod, self);
        mSelf = self;
        mCurrentMethod = cp;
    }

}

void BlockOrientedKernel::writeFinalBlockMethod(Value * remainingItems) {

    Value * const self = mSelf;
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = iBuilder->saveIP();

    if (!useIndirectBr()) {
        FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), {mSelf->getType(), iBuilder->getSizeTy()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::ExternalLinkage, getName() + FINAL_BLOCK_SUFFIX, iBuilder->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        mSelf = &*args;
        mSelf->setName("self");
        remainingItems = &*(++args);
        remainingItems->setName("remainingItems");
        iBuilder->SetInsertPoint(CreateBasicBlock("entry"));
    }

    generateFinalBlockMethod(remainingItems); // may be implemented by the BlockOrientedKernelBuilder subtype

    if (!useIndirectBr()) {
        iBuilder->CreateRetVoid();        
        iBuilder->restoreIP(ip);
        iBuilder->CreateCall(mCurrentMethod, {self, remainingItemCount});
        mCurrentMethod = cp;
        mSelf = self;
    }

}

//  The default finalBlock method simply dispatches to the doBlock routine.
void BlockOrientedKernel::generateFinalBlockMethod(Value * remainingItems) {
    CreateDoBlockMethodCall();
}

void BlockOrientedKernel::CreateDoBlockMethodCall() {
    if (useIndirectBr()) {
        BasicBlock * bb = CreateBasicBlock("resume");
        mStrideLoopBranch->addDestination(bb);
        mStrideLoopBranchAddress->addIncoming(BlockAddress::get(bb), iBuilder->GetInsertBlock());
        iBuilder->CreateBr(mStrideLoopBody);
        iBuilder->SetInsertPoint(bb);
    } else {
        iBuilder->CreateCall(mDoBlockMethod, mSelf);
    }
}

// CONSTRUCTOR

BlockOrientedKernel::BlockOrientedKernel(IDISA::IDISA_Builder * builder,
                                                           std::string && kernelName,
                                                           std::vector<Binding> && stream_inputs,
                                                           std::vector<Binding> && stream_outputs,
                                                           std::vector<Binding> && scalar_parameters,
                                                           std::vector<Binding> && scalar_outputs,
                                                           std::vector<Binding> && internal_scalars)
: KernelBuilder(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopBranchAddress(nullptr) {

}


// CONSTRUCTOR
KernelBuilder::KernelBuilder(IDISA::IDISA_Builder * builder,
                             std::string && kernelName,
                             std::vector<Binding> && stream_inputs,
                             std::vector<Binding> && stream_outputs,
                             std::vector<Binding> && scalar_parameters,
                             std::vector<Binding> && scalar_outputs,
                             std::vector<Binding> && internal_scalars)
: KernelInterface(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mSelf(nullptr)
, mCurrentMethod(nullptr)
, mNoTerminateAttribute(false)
, mDoBlockUpdatesProducedItemCountsAttribute(false) {

}

KernelBuilder::~KernelBuilder() { }

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(IDISA::IDISA_Builder * builder,
                                             std::string && kernelName,
                                             std::vector<Binding> && stream_inputs,
                                             std::vector<Binding> && stream_outputs,
                                             std::vector<Binding> && scalar_parameters,
                                             std::vector<Binding> && scalar_outputs,
                                             std::vector<Binding> && internal_scalars)
: KernelBuilder(builder, std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

}
