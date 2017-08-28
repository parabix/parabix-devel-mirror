/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <kernels/streamset.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>
#include <llvm/Support/Debug.h>

using namespace llvm;
using namespace parabix;

namespace kernel {

const std::string Kernel::DO_BLOCK_SUFFIX = "_DoBlock";
const std::string Kernel::FINAL_BLOCK_SUFFIX = "_FinalBlock";
const std::string Kernel::MULTI_BLOCK_SUFFIX = "_MultiBlock";
const std::string Kernel::LOGICAL_SEGMENT_NO_SCALAR = "logicalSegNo";
const std::string Kernel::PROCESSED_ITEM_COUNT_SUFFIX = "_processedItemCount";
const std::string Kernel::CONSUMED_ITEM_COUNT_SUFFIX = "_consumedItemCount";
const std::string Kernel::PRODUCED_ITEM_COUNT_SUFFIX = "_producedItemCount";
const std::string Kernel::TERMINATION_SIGNAL = "terminationSignal";
const std::string Kernel::BUFFER_PTR_SUFFIX = "_bufferPtr";
const std::string Kernel::CONSUMER_SUFFIX = "_consumerLocks";
const std::string Kernel::CYCLECOUNT_SCALAR = "CPUcycles";

unsigned Kernel::addScalar(Type * const type, const std::string & name) {
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

unsigned Kernel::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed field  to " + getName() + " after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}

void Kernel::prepareStreamSetNameMap() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamMap.emplace(mStreamSetInputs[i].name, std::make_pair(Port::Input, i));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamMap.emplace(mStreamSetOutputs[i].name, std::make_pair(Port::Output, i));
    }
}

void Kernel::bindPorts(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs) {
    assert (mModule == nullptr);
    assert (mStreamSetInputBuffers.empty());
    assert (mStreamSetOutputBuffers.empty());

    if (LLVM_UNLIKELY(mStreamSetInputs.size() != inputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetInputs.size()) +
                           " input stream sets but was given "
                           + std::to_string(inputs.size()));
    }

    for (unsigned i = 0; i < inputs.size(); ++i) {
        StreamSetBuffer * const buf = inputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " cannot be null");
        }
        buf->addConsumer(this);
    }

    if (LLVM_UNLIKELY(mStreamSetOutputs.size() != outputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetOutputs.size())
                           + " output stream sets but was given "
                           + std::to_string(outputs.size()));
    }

    for (unsigned i = 0; i < outputs.size(); ++i) {
        StreamSetBuffer * const buf = outputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i) + " cannot be null");
        }
        if (LLVM_LIKELY(buf->getProducer() == nullptr)) {
            buf->setProducer(this);
        } else {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " is already produced by kernel " + buf->getProducer()->getName());
        }
    }

    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());
}

Module * Kernel::makeModule(const std::unique_ptr<KernelBuilder> & idb) {
    assert (mModule == nullptr);
    std::stringstream cacheName;   
    cacheName << getName() << '_' << idb->getBuilderUniqueName();
    for (const StreamSetBuffer * b: mStreamSetInputBuffers) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    for (const StreamSetBuffer * b: mStreamSetOutputBuffers) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    mModule = new Module(cacheName.str(), idb->getContext());
    prepareKernel(idb);
    return mModule;
}

Module * Kernel::setModule(const std::unique_ptr<KernelBuilder> & idb, llvm::Module * const module) {
    assert (mModule == nullptr);
    mModule = module;
    prepareKernel(idb);
    return mModule;
}

void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot prepare kernel after kernel state finalized");
    }
    const auto blockSize = idb->getBitBlockWidth();
    if (mStride == 0) {
        // Set the default kernel stride.
        mStride = blockSize;
    }
    const auto requiredBlocks = codegen::SegmentSize + ((blockSize + mLookAheadPositions - 1) / blockSize);

    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferBlocks() != 0) && (mStreamSetInputBuffers[i]->getBufferBlocks() < requiredBlocks)) {
            //report_fatal_error(getName() + ": " + mStreamSetInputs[i].name + " requires buffer size " + std::to_string(requiredBlocks));
        }
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetInputs[i].name + BUFFER_PTR_SUFFIX);
        if ((i == 0) || !mStreamSetInputs[i].rate.isExact()) {
            addScalar(idb->getSizeTy(), mStreamSetInputs[i].name + PROCESSED_ITEM_COUNT_SUFFIX);
        }
    }

    IntegerType * const sizeTy = idb->getSizeTy();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetOutputs[i].name + BUFFER_PTR_SUFFIX);
        if ((mStreamSetInputs.empty() && (i == 0)) || !mStreamSetOutputs[i].rate.isExact()) {
            addScalar(sizeTy, mStreamSetOutputs[i].name + PRODUCED_ITEM_COUNT_SUFFIX);
        }
    }
    for (const auto & binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (const auto & binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    if (mStreamMap.empty()) {
        prepareStreamSetNameMap();
    }
    for (const auto & binding : mInternalScalars) {
        addScalar(binding.type, binding.name);
    }

    Type * const consumerSetTy = StructType::get(sizeTy, sizeTy->getPointerTo()->getPointerTo(), nullptr)->getPointerTo();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(consumerSetTy, mStreamSetOutputs[i].name + CONSUMER_SUFFIX);
    }

    addScalar(sizeTy, LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(idb->getInt1Ty(), TERMINATION_SIGNAL);

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(sizeTy, mStreamSetOutputs[i].name + CONSUMED_ITEM_COUNT_SUFFIX);
    }

    // We compile in a 64-bit CPU cycle counter into every kernel.   It will remain unused
    // in normal execution, but when codegen::EnableCycleCounter is specified, pipelines
    // will be able to add instrumentation to cached modules without recompilation.
    addScalar(idb->getInt64Ty(), CYCLECOUNT_SCALAR);
    // NOTE: StructType::create always creates a new type even if an identical one exists.
    mKernelStateType = getModule()->getTypeByName(getName());
    if (LLVM_LIKELY(mKernelStateType == nullptr)) {
        mKernelStateType = StructType::create(idb->getContext(), mKernelFields, getName());
    }
    processingRateAnalysis();
}
    
    
void Kernel::processingRateAnalysis() {
    
    const unsigned inputSetCount = mStreamSetInputs.size();
    const unsigned outputSetCount = mStreamSetOutputs.size();
    const unsigned totalSetCount = inputSetCount + outputSetCount;
    
    mItemsPerStride.resize(totalSetCount);
    mIsDerived.resize(totalSetCount);

    mItemsPerStride[0] = mStride;
    mIsDerived[0] = true;
    
    for (unsigned i = 0; i < inputSetCount; i++) {
        // Default reference stream set is the principal input stream set.
        auto & rate = mStreamSetInputs[i].rate;
        if (rate.referenceStreamSet() == "") {
            rate.setReferenceStreamSet(mStreamSetInputs[0].name);
        }
        Port port; unsigned ssIdx;
        std::tie(port, ssIdx) = getStreamPort(rate.referenceStreamSet());
        if ((port == Port::Output) || (ssIdx > i) || ((ssIdx == i) && (i > 0))) {
            report_fatal_error(getName() + ": input set " + mStreamSetInputs[i].name + ": forward or circular rate dependency");
        }
        if ((rate.isExact() || rate.isMaxRatio()) && mIsDerived[ssIdx]) {
            if ((mItemsPerStride[ssIdx] % rate.getRatioDenominator()) != 0) {
                report_fatal_error(getName() + ": " + mStreamSetInputs[i].name + " processing rate denominator does not exactly divide items per stride.");
            }
            mItemsPerStride[i] = rate.calculateRatio(mItemsPerStride[ssIdx]);
            mIsDerived[i] = rate.isExact();
        }
        else {
            mIsDerived[i] = false;
            mItemsPerStride[i] = 0;  // For unknown input rate, no items will be copied to temp buffers.
        }
    }
    
    for (unsigned i = inputSetCount; i < totalSetCount; i++) {
        auto & rate = mStreamSetOutputs[i-inputSetCount].rate;
        // Default reference stream set is the principal input stream set for the principal output stream set.
        // Default reference stream set is the principal output stream set for other output stream sets.
        if (rate.referenceStreamSet() == "") {
            if ((mStreamSetInputs.size() > 0) && (i == inputSetCount)) {
                rate.setReferenceStreamSet(mStreamSetInputs[0].name);
            }
            else {
                rate.setReferenceStreamSet(mStreamSetOutputs[0].name);
            }
        }
        Port port; unsigned ssIdx;
        std::tie(port, ssIdx) = getStreamPort(rate.referenceStreamSet());
        if (port == Port::Output) ssIdx += inputSetCount;
        if ((ssIdx > i) || ((ssIdx == i) && (i > 0))) {
            report_fatal_error(getName() + ": output set " + mStreamSetOutputs[i].name + ": forward or circular rate dependency");
        }
        if ((rate.isExact() || rate.isMaxRatio()) && mIsDerived[ssIdx]) {
            if ((mItemsPerStride[ssIdx] % rate.getRatioDenominator()) != 0) {
                report_fatal_error(getName() + ": " + mStreamSetOutputs[i-inputSetCount].name + " processing rate denominator does not exactly divide items per stride.");
            }
            mItemsPerStride[i] = rate.calculateRatio(mItemsPerStride[ssIdx]);
            mIsDerived[i] = rate.isExact();
        }
        else {
            mIsDerived[i] = false;
            mItemsPerStride[i] = 0;  // For unknown output rate, no items will be copied to temp buffers.
        }
    }
}


// Default kernel signature: generate the IR and emit as byte code.
std::string Kernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb.get());
    if (LLVM_UNLIKELY(hasSignature())) {
        generateKernel(idb);
        std::string signature;
        raw_string_ostream OS(signature);
        WriteBitcodeToFile(getModule(), OS);
        return signature;
    } else {
        return getModule()->getModuleIdentifier();
    }
}

void Kernel::generateKernel(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb.get());
    // If the module id cannot uniquely identify this kernel, "generateKernelSignature()" will have already
    // generated the unoptimized IR.
    if (!mIsGenerated) {
        const auto m = idb->getModule();
        const auto ip = idb->saveIP();
        const auto saveInstance = getInstance();
        idb->setModule(mModule);
        addKernelDeclarations(idb);
        callGenerateInitializeMethod(idb);
        callGenerateDoSegmentMethod(idb);
        callGenerateFinalizeMethod(idb);
        setInstance(saveInstance);
        idb->setModule(m);
        idb->restoreIP(ip);
        mIsGenerated = true;
    }
}

inline void Kernel::callGenerateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    mCurrentMethod = getInitFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    idb->CreateStore(ConstantAggregateZero::get(mKernelStateType), getInstance());
    for (const auto & binding : mScalarInputs) {
        idb->setScalarField(binding.name, &*(args++));
    }
    for (const auto & binding : mStreamSetOutputs) {
        idb->setConsumerLock(binding.name, &*(args++));
    }
    generateInitializeMethod(idb);
    idb->CreateRetVoid();
}

inline void Kernel::callGenerateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    mCurrentMethod = getDoSegmentFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    mIsFinal = &*(args++);
    const auto n = mStreamSetInputs.size();
    mAvailableItemCount.resize(n, nullptr);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mAvailableItemCount[i] = &*(args++);
    }
    generateDoSegmentMethod(idb); // must be overridden by the KernelBuilder subtype
    mIsFinal = nullptr;
    mAvailableItemCount.clear();
    idb->CreateRetVoid();
}

inline void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & idb) {
    mCurrentMethod = getTerminateFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    generateFinalizeMethod(idb); // may be overridden by the KernelBuilder subtype
    const auto n = mScalarOutputs.size();
    if (n == 0) {
        idb->CreateRetVoid();
    } else {
        Value * outputs[n];
        for (unsigned i = 0; i < n; ++i) {
            outputs[i] = idb->getScalarField(mScalarOutputs[i].name);
        }
        if (n == 1) {
            idb->CreateRet(outputs[0]);
        } else {
            idb->CreateAggregateRet(outputs, n);
        }
    }
}

unsigned Kernel::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return f->second;
}

Value * Kernel::createInstance(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot instantiate " + getName() + " before calling prepareKernel()");
    }
    setInstance(idb->CreateCacheAlignedAlloca(mKernelStateType));
    return getInstance();
}

void Kernel::initializeInstance(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(getInstance() == nullptr)) {
        report_fatal_error("Cannot initialize " + getName() + " before calling createInstance()");
    }
    std::vector<Value *> args;
    args.reserve(1 + mInitialArguments.size() + mStreamSetInputBuffers.size() + (mStreamSetOutputBuffers.size() * 2));
    args.push_back(getInstance());
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
        Value * arg = mStreamSetInputBuffers[i]->getStreamSetHandle();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": input stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetInputs.size() == mStreamSetInputBuffers.size());
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        assert (mStreamSetOutputBuffers[i]);
        Value * arg = mStreamSetOutputBuffers[i]->getStreamSetHandle();
        if (LLVM_UNLIKELY(arg == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i)
                               + " was not allocated prior to calling createInstance()");
        }
        args.push_back(arg);
    }
    assert (mStreamSetOutputs.size() == mStreamSetOutputBuffers.size());
    IntegerType * const sizeTy = idb->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    PointerType * const sizePtrPtrTy = sizePtrTy->getPointerTo();
    StructType * const consumerTy = StructType::get(sizeTy, sizePtrPtrTy, nullptr);
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        const auto output = mStreamSetOutputBuffers[i];
        const auto & consumers = output->getConsumers();
        const auto n = consumers.size();
        AllocaInst * const outputConsumers = idb->CreateAlloca(consumerTy);
        Value * const consumerSegNoArray = idb->CreateAlloca(ArrayType::get(sizePtrTy, n));
        for (unsigned i = 0; i < n; ++i) {
            Kernel * const consumer = consumers[i];
            assert ("all instances must be created prior to initialization of any instance" && consumer->getInstance());
            idb->setKernel(consumer);
            Value * const segmentNoPtr = idb->getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR);
            idb->CreateStore(segmentNoPtr, idb->CreateGEP(consumerSegNoArray, { idb->getInt32(0), idb->getInt32(i) }));
        }
        idb->setKernel(this);
        Value * const consumerCountPtr = idb->CreateGEP(outputConsumers, {idb->getInt32(0), idb->getInt32(0)});
        idb->CreateStore(idb->getSize(n), consumerCountPtr);
        Value * const consumerSegNoArrayPtr = idb->CreateGEP(outputConsumers, {idb->getInt32(0), idb->getInt32(1)});
        idb->CreateStore(idb->CreatePointerCast(consumerSegNoArray, sizePtrPtrTy), consumerSegNoArrayPtr);
        args.push_back(outputConsumers);
    }
    idb->CreateCall(getInitFunction(idb->getModule()), args);
}

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.

void BlockOrientedKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & idb) {
    BasicBlock * const entryBlock = idb->GetInsertBlock();
    BasicBlock * const strideLoopCond = idb->CreateBasicBlock(getName() + "_strideLoopCond");
    mStrideLoopBody = idb->CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = idb->CreateBasicBlock(getName() + "_stridesDone");
    BasicBlock * const doFinalBlock = idb->CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = idb->CreateBasicBlock(getName() + "_segmentDone");

    Value * baseTarget = nullptr;
    if (idb->supportsIndirectBr()) {
        baseTarget = idb->CreateSelect(mIsFinal, BlockAddress::get(doFinalBlock), BlockAddress::get(segmentDone));
    }

    ConstantInt * stride = idb->getSize(idb->getStride());
    Value * availablePos = mAvailableItemCount[0];
    Value * processed = idb->getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsAvail = idb->CreateSub(availablePos, processed);
    Value * stridesToDo = idb->CreateUDiv(itemsAvail, stride);

    idb->CreateBr(strideLoopCond);

    idb->SetInsertPoint(strideLoopCond);

    PHINode * branchTarget = nullptr;
    if (idb->supportsIndirectBr()) {
        branchTarget = idb->CreatePHI(baseTarget->getType(), 2, "branchTarget");
        branchTarget->addIncoming(baseTarget, entryBlock);
    }

    PHINode * const stridesRemaining = idb->CreatePHI(idb->getSizeTy(), 2, "stridesRemaining");
    stridesRemaining->addIncoming(stridesToDo, entryBlock);
    // NOTE: stridesRemaining may go to a negative number in the final block if the generateFinalBlockMethod(...)
    // calls CreateDoBlockMethodCall(). Do *not* replace the comparator with an unsigned one!
    Value * notDone = idb->CreateICmpSGT(stridesRemaining, idb->getSize(0));
    idb->CreateLikelyCondBr(notDone, mStrideLoopBody, stridesDone);

    idb->SetInsertPoint(mStrideLoopBody);

    if (idb->supportsIndirectBr()) {
        mStrideLoopTarget = idb->CreatePHI(baseTarget->getType(), 2, "strideTarget");
        mStrideLoopTarget->addIncoming(branchTarget, strideLoopCond);
    }

    /// GENERATE DO BLOCK METHOD

    writeDoBlockMethod(idb);

    /// UPDATE PROCESSED COUNTS

    processed = idb->getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsDone = idb->CreateAdd(processed, stride);
    idb->setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);

    stridesRemaining->addIncoming(idb->CreateSub(stridesRemaining, idb->getSize(1)), idb->GetInsertBlock());

    BasicBlock * bodyEnd = idb->GetInsertBlock();
    if (idb->supportsIndirectBr()) {
        branchTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }
    idb->CreateBr(strideLoopCond);

    stridesDone->moveAfter(bodyEnd);

    idb->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    if (idb->supportsIndirectBr()) {
        mStrideLoopBranch = idb->CreateIndirectBr(branchTarget, 3);
        mStrideLoopBranch->addDestination(doFinalBlock);
        mStrideLoopBranch->addDestination(segmentDone);
    } else {
        idb->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, segmentDone);
    }

    doFinalBlock->moveAfter(stridesDone);

    idb->SetInsertPoint(doFinalBlock);

    Value * remainingItems = idb->CreateSub(mAvailableItemCount[0], idb->getProcessedItemCount(mStreamSetInputs[0].name));

    writeFinalBlockMethod(idb, remainingItems);

    itemsDone = mAvailableItemCount[0];
    idb->setProcessedItemCount(mStreamSetInputs[0].name, itemsDone);
    idb->setTerminationSignal();
    idb->CreateBr(segmentDone);

    segmentDone->moveAfter(idb->GetInsertBlock());

    idb->SetInsertPoint(segmentDone);

    // Update the branch prediction metadata to indicate that the likely target will be segmentDone
    if (idb->supportsIndirectBr()) {
        MDBuilder mdb(idb->getContext());
        const auto destinations = mStrideLoopBranch->getNumDestinations();
        uint32_t weights[destinations];
        for (unsigned i = 0; i < destinations; ++i) {
            weights[i] = (mStrideLoopBranch->getDestination(i) == segmentDone) ? 100 : 1;
        }
        ArrayRef<uint32_t> bw(weights, destinations);
        mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(bw));
    }

}

inline void BlockOrientedKernel::writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & idb) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    auto ip = idb->saveIP();
    std::vector<Value *> availableItemCount(0);

    /// Check if the do block method is called and create the function if necessary    
    if (!idb->supportsIndirectBr()) {

        std::vector<Type *> params;
        params.reserve(1 + mAvailableItemCount.size());
        params.push_back(self->getType());
        for (Value * avail : mAvailableItemCount) {
            params.push_back(avail->getType());
        }

        FunctionType * const type = FunctionType::get(idb->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + DO_BLOCK_SUFFIX, idb->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        availableItemCount.reserve(mAvailableItemCount.size());
        while (++args != mCurrentMethod->arg_end()) {
            availableItemCount.push_back(&*args);
        }
        assert (availableItemCount.size() == mAvailableItemCount.size());
        mAvailableItemCount.swap(availableItemCount);
        idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    }

    std::vector<Value *> priorProduced;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        if (isa<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]) || isa<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            priorProduced.push_back(idb->getProducedItemCount(mStreamSetOutputs[i].name));
        }
    }

    generateDoBlockMethod(idb); // must be implemented by the BlockOrientedKernelBuilder subtype

    unsigned priorIdx = 0;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * log2BlockSize = idb->getSize(std::log2(idb->getBitBlockWidth()));
        if (SwizzledCopybackBuffer * const cb = dyn_cast<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = idb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = idb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = idb->CreateSub(idb->getProducedItemCount(mStreamSetOutputs[i].name), priorProduced[priorIdx]);
            Value * priorBlock = idb->CreateLShr(priorProduced[priorIdx], log2BlockSize);
            Value * priorOffset = idb->CreateAnd(priorProduced[priorIdx], idb->getSize(idb->getBitBlockWidth() - 1));
            Value * instance = idb->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * accessibleBlocks = idb->getLinearlyAccessibleBlocks(mStreamSetOutputs[i].name, priorBlock);
            Value * accessible = idb->CreateSub(idb->CreateShl(accessibleBlocks, log2BlockSize), priorOffset);
            Value * wraparound = idb->CreateICmpULT(accessible, newlyProduced);
            idb->CreateCondBr(wraparound, copyBack, done);
            idb->SetInsertPoint(copyBack);
            Value * copyItems = idb->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(idb.get(), instance, copyItems);
            idb->CreateBr(done);
            idb->SetInsertPoint(done);
            priorIdx++;
        }
        if (CircularCopybackBuffer * const cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = idb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = idb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * instance = idb->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * newlyProduced = idb->CreateSub(idb->getProducedItemCount(mStreamSetOutputs[i].name), priorProduced[priorIdx]);
            Value * accessible = idb->getLinearlyAccessibleItems(mStreamSetOutputs[i].name, priorProduced[priorIdx]);
            Value * wraparound = idb->CreateICmpULT(accessible, newlyProduced);
            idb->CreateCondBr(wraparound, copyBack, done);
            idb->SetInsertPoint(copyBack);
            Value * copyItems = idb->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(idb.get(), instance, copyItems);
            idb->CreateBr(done);
            idb->SetInsertPoint(done);
            priorIdx++;
        }
    }

    if (!idb->supportsIndirectBr()) {
        // Restore the DoSegment function state then call the DoBlock method
        idb->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        idb->restoreIP(ip);
        setInstance(self);
        mCurrentMethod = cp;
        mAvailableItemCount.swap(availableItemCount);
        CreateDoBlockMethodCall(idb);
    }

}

inline void BlockOrientedKernel::writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, Value * remainingItems) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = idb->saveIP();
    std::vector<Value *> availableItemCount(0);

    if (!idb->supportsIndirectBr()) {
        std::vector<Type *> params;
        params.reserve(2 + mAvailableItemCount.size());
        params.push_back(self->getType());
        params.push_back(idb->getSizeTy());
        for (Value * avail : mAvailableItemCount) {
            params.push_back(avail->getType());
        }
        FunctionType * const type = FunctionType::get(idb->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + FINAL_BLOCK_SUFFIX, idb->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        remainingItems = &*(++args);
        remainingItems->setName("remainingItems");
        availableItemCount.reserve(mAvailableItemCount.size());
        while (++args != mCurrentMethod->arg_end()) {
            availableItemCount.push_back(&*args);
        }
        assert (availableItemCount.size() == mAvailableItemCount.size());
        mAvailableItemCount.swap(availableItemCount);
        idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    }

    generateFinalBlockMethod(idb, remainingItems); // may be implemented by the BlockOrientedKernel subtype

    RecursivelyDeleteTriviallyDeadInstructions(remainingItems); // if remainingItems was not used, this will eliminate it.

    if (!idb->supportsIndirectBr()) {
        idb->CreateRetVoid();
        idb->restoreIP(ip);
        setInstance(self);
        mAvailableItemCount.swap(availableItemCount);
        // Restore the DoSegment function state then call the DoFinal method
        std::vector<Value *> args;
        args.reserve(2 + mAvailableItemCount.size());
        args.push_back(self);
        args.push_back(remainingItemCount);
        for (Value * avail : mAvailableItemCount) {
            args.push_back(avail);
        }
        idb->CreateCall(mCurrentMethod, args);
        mCurrentMethod = cp;
    }

}

//  The default finalBlock method simply dispatches to the doBlock routine.
void BlockOrientedKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, Value * /* remainingItems */) {
    CreateDoBlockMethodCall(idb);
}

void BlockOrientedKernel::CreateDoBlockMethodCall(const std::unique_ptr<KernelBuilder> & idb) {
    if (idb->supportsIndirectBr()) {
        BasicBlock * bb = idb->CreateBasicBlock("resume");
        mStrideLoopBranch->addDestination(bb);
        mStrideLoopTarget->addIncoming(BlockAddress::get(bb), idb->GetInsertBlock());
        idb->CreateBr(mStrideLoopBody);
        bb->moveAfter(idb->GetInsertBlock());
        idb->SetInsertPoint(bb);
    } else {
        std::vector<Value *> args;
        args.reserve(1 + mAvailableItemCount.size());
        args.push_back(getInstance());
        for (Value * avail : mAvailableItemCount) {
            args.push_back(avail);
        }
        idb->CreateCall(mDoBlockMethod, args);
    }
}

void MultiBlockKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

    // Stream set and buffer analysis.  When near the end of buffers
    // or for final block processing, data for each streamset may need
    // to be copied into temporary buffers to ensure linear access.
    // Data is always copied as a number of whole blocks, dependent
    // on the stream set processing rate.
    
    const unsigned bitBlockWidth = kb->getBitBlockWidth();
    const unsigned inputSetCount = mStreamSetInputs.size();
    const unsigned outputSetCount = mStreamSetOutputs.size();
    const unsigned totalSetCount = inputSetCount + outputSetCount;
    
    int maxBlocksToCopy[totalSetCount];
    for (unsigned i = 0; i < totalSetCount; i++) {
        if (mIsDerived[i]) {
            if (mItemsPerStride[i] % bitBlockWidth == 0) {
                maxBlocksToCopy[i] = mItemsPerStride[i] / bitBlockWidth;
            }
            else {
                // May not be block aligned, can overlap partial blocks at both ends.
                maxBlocksToCopy[i] = mItemsPerStride[i]/bitBlockWidth + 2;
            }
        }
        else {
            // For variable input stream sets, we make a single stride of items
            // available, if possible, but this stride could be nonaligned.
            maxBlocksToCopy[i] = mStride / bitBlockWidth + 2;
        }
    }
    auto ip = kb->saveIP();
    Function * const cp = mCurrentMethod;
    const auto saveInstance = getInstance();

    // First prepare the multi-block method that will be used.

    std::vector<Type *> multiBlockParmTypes;
    multiBlockParmTypes.push_back(mKernelStateType->getPointerTo());
    multiBlockParmTypes.push_back(kb->getSizeTy());
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        if (!mIsDerived[i]) multiBlockParmTypes.push_back(kb->getSizeTy());
    }
    for (auto buffer : mStreamSetInputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }
    for (auto buffer : mStreamSetOutputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }

    FunctionType * const type = FunctionType::get(kb->getVoidTy(), multiBlockParmTypes, false);
    Function * multiBlockFunction = Function::Create(type, GlobalValue::InternalLinkage, getName() + MULTI_BLOCK_SUFFIX, kb->getModule());
    multiBlockFunction->setCallingConv(CallingConv::C);
    multiBlockFunction->setDoesNotThrow();
    mCurrentMethod = multiBlockFunction;
    kb->SetInsertPoint(BasicBlock::Create(kb->getContext(), "multiBlockEntry", multiBlockFunction, 0));

    auto args = multiBlockFunction->arg_begin();
    args->setName("self");
    setInstance(&*args);
    (++args)->setName("itemsToDo");
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        if (!mIsDerived[i]) (++args)->setName(mStreamSetInputs[i].name + "_availItems");
    }
    for (auto binding : mStreamSetInputs) {
        (++args)->setName(binding.name + "BufPtr");
    }
    for (auto binding : mStreamSetOutputs) {
        (++args)->setName(binding.name + "BufPtr");
    }

    // Now use the generateMultiBlockLogic method of the MultiBlockKernelBuilder subtype to
    // provide the required multi-block kernel logic.
    generateMultiBlockLogic(kb);

    kb->CreateRetVoid();

    kb->restoreIP(ip);
    mCurrentMethod = cp;
    setInstance(saveInstance);

    // Now proceed with creation of the doSegment method.

    BasicBlock * const entry = kb->GetInsertBlock();
    BasicBlock * const doSegmentOuterLoop = kb->CreateBasicBlock(getName() + "_doSegmentOuterLoop");
    BasicBlock * const doMultiBlockCall = kb->CreateBasicBlock(getName() + "_doMultiBlockCall");
    BasicBlock * const tempBlockCheck = kb->CreateBasicBlock(getName() + "_tempBlockCheck");
    BasicBlock * const doTempBufferBlock = kb->CreateBasicBlock(getName() + "_doTempBufferBlock");
    BasicBlock * const segmentDone = kb->CreateBasicBlock(getName() + "_segmentDone");

    Value * blockBaseMask = kb->CreateNot(kb->getSize(kb->getBitBlockWidth() - 1));
    //
    // Define and allocate the temporary buffer area.
    //
    Type * tempBuffers[totalSetCount];
    for (unsigned i = 0; i < totalSetCount; i++) {
        unsigned blocks = maxBlocksToCopy[i];
        Type * bufType = i < inputSetCount ? mStreamSetInputBuffers[i]->getStreamSetBlockType() : mStreamSetOutputBuffers[i -inputSetCount]->getStreamSetBlockType();
        if (blocks > 1) {
            tempBuffers[i] = ArrayType::get(bufType, blocks);
        }
        else {
            tempBuffers[i] = bufType;
        }
    }
    Type * tempParameterStructType = StructType::create(kb->getContext(), ArrayRef<Type *>(tempBuffers, totalSetCount), "tempBuf");
    Value * tempParameterArea = kb->CreateCacheAlignedAlloca(tempParameterStructType);
    ConstantInt * blockSize = kb->getSize(kb->getBitBlockWidth());
    ConstantInt * strideSize = kb->getSize(mStride);
    
    Value * availablePos = mAvailableItemCount[0];
    Value * itemsAvail = availablePos;

    //  Make sure that corresponding data is available depending on processing rate
    //  for all derived input stream sets.
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * a = mAvailableItemCount[i];
        auto & rate = mStreamSetInputs[i].rate;
        if (mIsDerived[i]) {
            Value * maxItems = rate.CreateMaxReferenceItemsCalculation(kb.get(), a);
            itemsAvail = kb->CreateSelect(kb->CreateICmpULT(itemsAvail, maxItems), itemsAvail, maxItems);
        }
    }

    Value * processed = kb->getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsToDo = kb->CreateSub(itemsAvail, processed);
    Value * fullStridesToDo = kb->CreateUDiv(itemsToDo, strideSize);
    Value * excessItems = kb->CreateURem(itemsToDo, strideSize);

    //  Now we iteratively process these blocks using the doMultiBlock method.
    //  In each iteration, we process the maximum number of linearly accessible
    //  blocks on the principal input, reduced to ensure that the corresponding
    //  data is linearly available at the specified processing rates for the other inputs,
    //  and that each of the output buffers has sufficient linearly available space
    //  (using overflow areas, if necessary) for the maximum output that can be
    //  produced.

    kb->CreateBr(doSegmentOuterLoop);
    kb->SetInsertPoint(doSegmentOuterLoop);
    PHINode * const stridesRemaining = kb->CreatePHI(kb->getSizeTy(), 2, "stridesRemaining");
    stridesRemaining->addIncoming(fullStridesToDo, entry);

    // For each input buffer, determine the processedItemCount, the block pointer for the
    // buffer block containing the next item, and the number of linearly available items.

    std::vector<Value *> processedItemCount;
    std::vector<Value *> inputBlockPtr;
    std::vector<Value *> producedItemCount;
    std::vector<Value *> outputBlockPtr;

    //  Now determine the linearly available blocks, based on blocks remaining reduced
    //  by limitations of linearly available input buffer space.

    Value * linearlyAvailStrides = stridesRemaining;
    for (unsigned i = 0; i < inputSetCount; i++) {
        Value * p = kb->getProcessedItemCount(mStreamSetInputs[i].name);
        Value * blkNo = kb->CreateUDiv(p, blockSize);
        Value * b = kb->getInputStreamBlockPtr(mStreamSetInputs[i].name, kb->getInt32(0));
        processedItemCount.push_back(p);
        inputBlockPtr.push_back(b);
        auto & rate = mStreamSetInputs[i].rate;
        if (rate.isUnknownRate()) continue;  // No calculation possible for unknown rates.
        Value * maxReferenceItems = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator())) {
            maxReferenceItems = kb->CreateMul(kb->getLinearlyAccessibleBlocks(mStreamSetInputs[i].name, blkNo), blockSize);

        } else {
            Value * linearlyAvailItems = kb->getLinearlyAccessibleItems(mStreamSetInputs[i].name, p);
            maxReferenceItems = rate.CreateMaxReferenceItemsCalculation(kb.get(), linearlyAvailItems);
        }
        Value * maxStrides = kb->CreateUDiv(maxReferenceItems, strideSize);
        linearlyAvailStrides = kb->CreateSelect(kb->CreateICmpULT(maxStrides, linearlyAvailStrides), maxStrides, linearlyAvailStrides);
    }
    //  Now determine the linearly writeable blocks, based on available blocks reduced
    //  by limitations of output buffer space.
    Value * linearlyWritableStrides = linearlyAvailStrides;
    for (unsigned i = 0; i < outputSetCount; i++) {
        Value * p = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * blkNo = kb->CreateUDiv(p, blockSize);
        Value * b = kb->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, kb->getInt32(0));
        producedItemCount.push_back(p);
        outputBlockPtr.push_back(b);
        auto & rate = mStreamSetOutputs[i].rate;
        if (rate.isUnknownRate()) continue;  // No calculation possible for unknown rates.
        Value * maxReferenceItems = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator())) {
            maxReferenceItems = kb->CreateMul(kb->getLinearlyWritableBlocks(mStreamSetOutputs[i].name, blkNo), blockSize);
        } else {
            Value * writableItems = kb->getLinearlyWritableItems(mStreamSetOutputs[i].name, p);
            maxReferenceItems = rate.CreateMaxReferenceItemsCalculation(kb.get(), writableItems);
        }
        Value * maxStrides = kb->CreateUDiv(maxReferenceItems, strideSize);
        linearlyWritableStrides = kb->CreateSelect(kb->CreateICmpULT(maxStrides, linearlyWritableStrides), maxStrides, linearlyWritableStrides);
    }
    Value * haveStrides = kb->CreateICmpUGT(linearlyWritableStrides, kb->getSize(0));
    kb->CreateCondBr(haveStrides, doMultiBlockCall, tempBlockCheck);

    //  At this point we have verified the availability of one or more blocks of input data and output buffer space for all stream sets.
    //  Now prepare the doMultiBlock call.
    kb->SetInsertPoint(doMultiBlockCall);

    Value * linearlyAvailItems = kb->CreateMul(linearlyWritableStrides, strideSize);

    std::vector<Value *> doMultiBlockArgs;
    doMultiBlockArgs.push_back(getInstance());
    doMultiBlockArgs.push_back(linearlyAvailItems);
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        if (!mIsDerived[i]) {
            Value * avail = kb->CreateSub(mAvailableItemCount[i], processedItemCount[i]);
            Value * linearlyAvail = kb->getLinearlyAccessibleItems(mStreamSetInputs[i].name, processedItemCount[i]);
            doMultiBlockArgs.push_back(kb->CreateSelect(kb->CreateICmpULT(avail, linearlyAvail), avail, linearlyAvail));
        }
    }
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * bufPtr = kb->CreatePointerCast(inputBlockPtr[i], mStreamSetInputBuffers[i]->getPointerType());
        doMultiBlockArgs.push_back(bufPtr);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * bufPtr = kb->CreatePointerCast(outputBlockPtr[i], mStreamSetOutputBuffers[i]->getPointerType());
        doMultiBlockArgs.push_back(bufPtr);
    }

    kb->CreateCall(multiBlockFunction, doMultiBlockArgs);
    // Do copybacks if necessary.
    unsigned priorIdx = 0;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * log2BlockSize = kb->getSize(std::log2(kb->getBitBlockWidth()));
        if (auto cb = dyn_cast<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = kb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = kb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = kb->CreateSub(kb->getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * priorBlock = kb->CreateLShr(producedItemCount[i], log2BlockSize);
            Value * priorOffset = kb->CreateAnd(producedItemCount[i], kb->getSize(kb->getBitBlockWidth() - 1));
            Value * instance = kb->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * accessibleBlocks = kb->getLinearlyAccessibleBlocks(mStreamSetOutputs[i].name, priorBlock);
            Value * accessible = kb->CreateSub(kb->CreateShl(accessibleBlocks, log2BlockSize), priorOffset);
            Value * wraparound = kb->CreateICmpULT(accessible, newlyProduced);
            kb->CreateCondBr(wraparound, copyBack, done);
            kb->SetInsertPoint(copyBack);
            Value * copyItems = kb->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(kb.get(), instance, copyItems);
            kb->CreateBr(done);
            kb->SetInsertPoint(done);
            priorIdx++;
        }
        if (auto cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = kb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = kb->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * instance = kb->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * newlyProduced = kb->CreateSub(kb->getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * accessible = kb->getLinearlyAccessibleItems(mStreamSetOutputs[i].name, producedItemCount[i]);
            Value * wraparound = kb->CreateICmpULT(accessible, newlyProduced);
            kb->CreateCondBr(wraparound, copyBack, done);
            kb->SetInsertPoint(copyBack);
            Value * copyItems = kb->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(kb.get(), instance, copyItems);
            kb->CreateBr(done);
            kb->SetInsertPoint(done);
            priorIdx++;
        }
    }

    Value * nowProcessed = kb->CreateAdd(processedItemCount[0], linearlyAvailItems);
    kb->setProcessedItemCount(mStreamSetInputs[0].name, nowProcessed);
    Value * reducedStridesToDo = kb->CreateSub(stridesRemaining, linearlyWritableStrides);
    BasicBlock * multiBlockFinal = kb->GetInsertBlock();
    stridesRemaining->addIncoming(reducedStridesToDo, multiBlockFinal);
    kb->CreateBr(doSegmentOuterLoop);
    //
    // We use temporary buffers in 3 different cases that preclude full block processing.
    // (a) One or more input buffers does not have a sufficient number of input items linearly available.
    // (b) One or more output buffers does not have sufficient linearly available buffer space.
    // (c) We have processed all the full blocks of input and only the excessItems remain.
    // In each case we set up temporary buffers for input and output and then
    // call the Multiblock routine.
    //

    kb->SetInsertPoint(tempBlockCheck);
    haveStrides = kb->CreateICmpUGT(stridesRemaining, kb->getSize(0));
    kb->CreateCondBr(kb->CreateOr(mIsFinal, haveStrides), doTempBufferBlock, segmentDone);

    kb->SetInsertPoint(doTempBufferBlock);
    Value * tempBlockItems = kb->CreateSelect(haveStrides, strideSize, excessItems);
    Value * doFinal = kb->CreateNot(haveStrides);

    // Begin constructing the doMultiBlock args.
    std::vector<Value *> tempArgs;
    tempArgs.push_back(getInstance());
    tempArgs.push_back(tempBlockItems);
    // For non-derived inputs, add the available items.
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        if (!mIsDerived[i]) {
            Value * avail = kb->CreateSub(mAvailableItemCount[i], processedItemCount[i]);
            tempArgs.push_back(kb->CreateSelect(kb->CreateICmpULT(avail, strideSize), avail, strideSize));
        }
    }
    // Prepare the temporary buffer area.
    //
    // First zero it out.
    Constant * const tempAreaSize = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(tempParameterStructType), kb->getSizeTy(), false);
    kb->CreateMemZero(tempParameterArea, tempAreaSize);
    // For each input and output buffer, copy over necessary data starting from the last
    // block boundary.
    Value * finalItemCountNeeded[inputSetCount];
    finalItemCountNeeded[0] = kb->CreateAdd(processedItemCount[0], tempBlockItems);

    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); i++) {
        Type * bufPtrType = mStreamSetInputBuffers[i]->getPointerType();
        if (mItemsPerStride[i] != 0) {
            Value * tempBufPtr = kb->CreateGEP(tempParameterArea, {kb->getInt32(0), kb->getInt32(i)});
            tempBufPtr = kb->CreatePointerCast(tempBufPtr, bufPtrType);
            ConstantInt * strideItems = kb->getSize(mItemsPerStride[i]);
            Value * strideBasePos = kb->CreateSub(processedItemCount[i], kb->CreateURem(processedItemCount[i], strideItems));
            Value * blockBasePos = (mItemsPerStride[i] % bitBlockWidth == 0) ? strideBasePos : kb->CreateAnd(strideBasePos, blockBaseMask);

            // The number of items to copy is determined by the processing rate requirements.
            if (i >= 1) {
                auto & rate = mStreamSetInputs[i].rate;
                std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
                Port port; unsigned ssIdx;
                std::tie(port, ssIdx) = getStreamPort(refSet);
                finalItemCountNeeded[i] = rate.CreateRatioCalculation(kb.get(), finalItemCountNeeded[ssIdx], doFinal);
            }
            
            Value * inputPtr = kb->CreatePointerCast(kb->getRawInputPointer(mStreamSetInputs[i].name, kb->getInt32(0), blockBasePos), bufPtrType);
            
            if (maxBlocksToCopy[i] == 1) {
                // copy one block
                mStreamSetInputBuffers[i]->createBlockCopy(kb.get(), tempBufPtr, inputPtr, kb->getSize(1));
            }
            else {
                Value * neededItems = kb->CreateSub(finalItemCountNeeded[i], blockBasePos);
                Value * availFromBase = kb->getLinearlyAccessibleItems(mStreamSetInputs[i].name, blockBasePos);
                Value * allAvail = kb->CreateICmpULE(neededItems, availFromBase);
                Value * copyItems1 = kb->CreateSelect(allAvail, neededItems, availFromBase);
                //mStreamSetInputBuffers[i]->createBlockAlignedCopy(kb.get(), tempBufPtr, inputPtr, copyItems1);
                Value * copyBlocks1 = kb->CreateUDivCeil(copyItems1, blockSize);
                mStreamSetInputBuffers[i]->createBlockCopy(kb.get(), tempBufPtr, inputPtr, copyBlocks1);
                BasicBlock * copyRemaining = kb->CreateBasicBlock("copyRemaining");
                BasicBlock * copyDone = kb->CreateBasicBlock("copyDone");
                kb->CreateCondBr(allAvail, copyDone, copyRemaining);
                kb->SetInsertPoint(copyRemaining);
                Value * copyItems2 = kb->CreateSub(neededItems, copyItems1);
                Value * copyBlocks2 = kb->CreateUDivCeil(copyItems2, blockSize);
                //Value * nextBasePos = kb->CreateAdd(blockBasePos, copyItems1);
                Value * nextBasePos = kb->CreateAdd(blockBasePos, kb->CreateMul(copyBlocks2, blockSize));
                Value * nextInputPtr = kb->CreatePointerCast(kb->getRawInputPointer(mStreamSetInputs[i].name, kb->getInt32(0), nextBasePos), bufPtrType);
                Value * nextBufPtr = kb->CreateGEP(tempBufPtr, kb->CreateUDiv(copyItems1, blockSize));
                //mStreamSetInputBuffers[i]->createBlockAlignedCopy(kb.get(), nextBufPtr, nextInputPtr, copyItems2);
                mStreamSetInputBuffers[i]->createBlockCopy(kb.get(), nextBufPtr, nextInputPtr, copyBlocks2);
                kb->CreateBr(copyDone);
                kb->SetInsertPoint(copyDone);
            }
            tempArgs.push_back(tempBufPtr);
        }
        else {
            Value * bufPtr = kb->getInputStreamBlockPtr(mStreamSetInputs[i].name, kb->getInt32(0));
            bufPtr = kb->CreatePointerCast(bufPtr, mStreamSetInputBuffers[i]->getPointerType());
            tempArgs.push_back(bufPtr);            
        }
    }
    Value * outputBasePos[outputSetCount];
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = kb->CreateGEP(tempParameterArea,  {kb->getInt32(0), kb->getInt32(mStreamSetInputs.size() + i)});
        Type * bufPtrType = mStreamSetOutputBuffers[i]->getPointerType();
        tempBufPtr = kb->CreatePointerCast(tempBufPtr, bufPtrType);
        producedItemCount[i] = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        outputBasePos[i] = kb->CreateAnd(producedItemCount[i], blockBaseMask);
        //mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), tempBufPtr, outputBlockPtr[i], kb->CreateSub(producedItemCount[i], outputBasePos[i]));
        Value * copyBlocks = kb->CreateUDivCeil(kb->CreateSub(producedItemCount[i], outputBasePos[i]), blockSize);
        mStreamSetOutputBuffers[i]->createBlockCopy(kb.get(), tempBufPtr, outputBlockPtr[i], copyBlocks);
        tempArgs.push_back(tempBufPtr);
    }

    kb->CreateCall(multiBlockFunction, tempArgs);
    
    //  The items have been processed and output generated to the temporary areas.
    //  Update the processed item count (and hence all the counts derived automatically
    //  therefrom).
    kb->setProcessedItemCount(mStreamSetInputs[0].name, finalItemCountNeeded[0]);
    
    // Copy back data to the actual output buffers.
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = kb->CreateGEP(tempParameterArea,  {kb->getInt32(0), kb->getInt32(mStreamSetInputs.size() + i)});
        tempBufPtr = kb->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        Value * finalOutputItems = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * copyItems = kb->CreateSub(finalOutputItems, outputBasePos[i]);
        // Round up to exact multiple of block size.
        //copyItems = kb->CreateAnd(kb->CreateAdd(copyItems, kb->getSize(bitBlockWidth - 1)), blockBaseMask);
        Value * writableFromBase = kb->getLinearlyWritableItems(mStreamSetOutputs[i].name, outputBasePos[i]); // must be a whole number of blocks.
        Value * allWritable = kb->CreateICmpULE(copyItems, writableFromBase);
        Value * copyItems1 = kb->CreateSelect(allWritable, copyItems, writableFromBase);
        //mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), outputBlockPtr[i], tempBufPtr, copyItems1);
        Value * copyBlocks1 = kb->CreateUDivCeil(copyItems1, blockSize);
        mStreamSetOutputBuffers[i]->createBlockCopy(kb.get(), outputBlockPtr[i], tempBufPtr, copyBlocks1);
        BasicBlock * copyBackRemaining = kb->CreateBasicBlock("copyBackRemaining");
        BasicBlock * copyBackDone = kb->CreateBasicBlock("copyBackDone");
        kb->CreateCondBr(allWritable, copyBackDone, copyBackRemaining);
        kb->SetInsertPoint(copyBackRemaining);
        Value * copyItems2 = kb->CreateSub(copyItems, copyItems1);
        Value * nextBasePos = kb->CreateAdd(outputBasePos[i], copyItems1);
        Type * bufPtrType = mStreamSetOutputBuffers[i]->getPointerType();
        Value * nextOutputPtr = kb->CreatePointerCast(kb->getRawOutputPointer(mStreamSetOutputs[i].name, kb->getInt32(0), nextBasePos), bufPtrType);
        tempBufPtr = kb->CreateGEP(tempBufPtr, kb->CreateUDiv(copyItems1, blockSize));
        //mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), nextOutputPtr, tempBufPtr, copyItems2);
        Value * copyBlocks2 = kb->CreateUDivCeil(copyItems2, blockSize);
        mStreamSetOutputBuffers[i]->createBlockCopy(kb.get(), nextOutputPtr, tempBufPtr, copyBlocks2);
        kb->CreateBr(copyBackDone);
        kb->SetInsertPoint(copyBackDone);
    }


    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    //
    stridesRemaining->addIncoming(kb->CreateSub(stridesRemaining, kb->CreateZExt(haveStrides, kb->getSizeTy())), kb->GetInsertBlock());
    BasicBlock * setTermination = kb->CreateBasicBlock("mBsetTermination");
    kb->CreateCondBr(haveStrides, doSegmentOuterLoop, setTermination);
    kb->SetInsertPoint(setTermination);
    kb->setTerminationSignal();
    kb->CreateBr(segmentDone);
    kb->SetInsertPoint(segmentDone);
}

void Kernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    mOutputScalarResult = idb->CreateCall(getTerminateFunction(idb->getModule()), { getInstance() });
}

Kernel::StreamPort Kernel::getStreamPort(const std::string & name) const {
    const auto f = mStreamMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamMap.end())) {
        report_fatal_error(getName() + " does not contain stream set " + name);
    }
    return f->second;
}

static inline std::string annotateKernelNameWithDebugFlags(std::string && name) {
    if (codegen::EnableAsserts) {
        name += "_EA";
    }
    name += "_O" + std::to_string((int)codegen::OptLevel);
    return name;
}

// CONSTRUCTOR
Kernel::Kernel(std::string && kernelName,
               std::vector<Binding> && stream_inputs,
               std::vector<Binding> && stream_outputs,
               std::vector<Binding> && scalar_parameters,
               std::vector<Binding> && scalar_outputs,
               std::vector<Binding> && internal_scalars)
: KernelInterface(annotateKernelNameWithDebugFlags(std::move(kernelName))
                  , std::move(stream_inputs), std::move(stream_outputs)
                  , std::move(scalar_parameters), std::move(scalar_outputs)
                  , std::move(internal_scalars))
, mCurrentMethod(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr)
, mStride(0) {

}

Kernel::~Kernel() {

}

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(std::string && kernelName,
                                         std::vector<Binding> && stream_inputs,
                                         std::vector<Binding> && stream_outputs,
                                         std::vector<Binding> && scalar_parameters,
                                         std::vector<Binding> && scalar_outputs,
                                         std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr) {

}

// CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(std::string && kernelName,
                                   std::vector<Binding> && stream_inputs,
                                   std::vector<Binding> && stream_outputs,
                                   std::vector<Binding> && scalar_parameters,
                                   std::vector<Binding> && scalar_outputs,
                                   std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {
}

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(std::string && kernelName,
                                             std::vector<Binding> && stream_inputs,
                                             std::vector<Binding> && stream_outputs,
                                             std::vector<Binding> && scalar_parameters,
                                             std::vector<Binding> && scalar_outputs,
                                             std::vector<Binding> && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {
    
}
  
    
void applyOutputBufferExpansions(const std::unique_ptr<KernelBuilder> & kb,
                                 std::vector<Value *> inputAvailable,
                                 Value * doFinal) {
    auto kernel = kb->getKernel();
    const unsigned inputSetCount = inputAvailable.size();
    if (inputSetCount == 0) return;  //  Cannot calculate buffer items expected from input.
    auto & outputs = kernel->getStreamSetOutputBuffers();
    const unsigned outputSetCount = outputs.size();

    Constant * blockSize = kb->getSize(kb->getBitBlockWidth());
    Value * newlyAvailInputItems[inputSetCount];
    Value * requiredOutputBufferSpace[outputSetCount];
    for (unsigned i = 0; i < inputSetCount; i++) {
        Value * processed = kb->getProcessedItemCount(kernel->getStreamInput(i).name);
        newlyAvailInputItems[i] = kb->CreateSub(inputAvailable[i], processed);
    }
    //kb->GetInsertBlock()->dump();
    for (unsigned i = 0; i < outputSetCount; i++) {
        requiredOutputBufferSpace[i] = nullptr;
        auto & rate = kernel->getStreamOutput(i).rate;
        if (rate.isUnknownRate()) continue;  // No calculations possible.
        Kernel::Port port; unsigned ssIdx;
        std::tie(port, ssIdx) = kernel->getStreamPort(rate.referenceStreamSet());
        if (port == Kernel::Port::Output) {
            requiredOutputBufferSpace[i] = rate.CreateRatioCalculation(kb.get(), requiredOutputBufferSpace[ssIdx], doFinal);
        }
        else {
            requiredOutputBufferSpace[i] = rate.CreateRatioCalculation(kb.get(), newlyAvailInputItems[ssIdx], doFinal);
        }
        if (auto db = dyn_cast<DynamicBuffer>(outputs[i])) {
            Value * handle = db->getStreamSetHandle();
            // This buffer can be expanded.
            Value * producedBlock = kb->CreateUDivCeil(kb->getProducedItemCount(kernel->getStreamOutput(i).name), blockSize);
            Value * consumedBlock = kb->CreateUDiv(kb->getConsumedItemCount(kernel->getStreamOutput(i).name), blockSize);
            Value * blocksInUse = kb->CreateSub(producedBlock, consumedBlock);
            Value * blocksRequired = kb->CreateAdd(blocksInUse, kb->CreateUDivCeil(requiredOutputBufferSpace[i], blockSize));
            Value * spaceRequired = kb->CreateMul(blocksRequired, blockSize);
            Value * expansionNeeded = kb->CreateICmpUGT(spaceRequired, db->getBufferedSize(kb.get(), handle));
            BasicBlock * doExpand = kb->CreateBasicBlock("doExpand");
            BasicBlock * bufferReady = kb->CreateBasicBlock("bufferReady");
            kb->CreateCondBr(expansionNeeded, doExpand, bufferReady);
            kb->SetInsertPoint(doExpand);
            db->doubleCapacity(kb.get(), handle);
            // Ensure that capacity is sufficient by successive doubling, if necessary.
            expansionNeeded = kb->CreateICmpUGT(spaceRequired, db->getBufferedSize(kb.get(), handle));
            kb->CreateCondBr(expansionNeeded, doExpand, bufferReady);
            kb->SetInsertPoint(bufferReady);
        }
    }

}

}
