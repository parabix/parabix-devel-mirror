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
    const auto requiredBlocks = codegen::SegmentSize + ((blockSize + mLookAheadPositions - 1) / blockSize);

    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferBlocks() != 0) && (mStreamSetInputBuffers[i]->getBufferBlocks() < requiredBlocks)) {
            report_fatal_error(getName() + ": " + mStreamSetInputs[i].name + " requires buffer size " + std::to_string(requiredBlocks));
        }
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getPointerType(), mStreamSetInputs[i].name + BUFFER_PTR_SUFFIX);
        if ((i == 0) || !mStreamSetInputs[i].rate.isExact()) {
            addScalar(idb->getSizeTy(), mStreamSetInputs[i].name + PROCESSED_ITEM_COUNT_SUFFIX);
        }
    }

    IntegerType * const sizeTy = idb->getSizeTy();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getPointerType(), mStreamSetOutputs[i].name + BUFFER_PTR_SUFFIX);
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
    
    mKernelStateType = StructType::create(idb->getContext(), mKernelFields, getName());
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
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(idb.get(), priorBlock);
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
            Value * accessible = cb->getLinearlyAccessibleItems(idb.get(), priorProduced[priorIdx]);
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

    auto ip = kb->saveIP();
    Function * const cp = mCurrentMethod;
    const auto saveInstance = getInstance();

    // First prepare the multi-block method that will be used.

    DataLayout DL(kb->getModule());
    IntegerType * const intAddrTy = DL.getIntPtrType(kb->getContext());

    std::vector<Type *> multiBlockParmTypes;
    multiBlockParmTypes.push_back(mKernelStateType->getPointerTo());
    multiBlockParmTypes.push_back(kb->getSizeTy());
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
    auto args = multiBlockFunction->arg_begin();
    args->setName("self");
    setInstance(&*args);
    (++args)->setName("itemsToDo");
    for (auto binding : mStreamSetInputs) {
        (++args)->setName(binding.name + "BufPtr");
    }
    for (auto binding : mStreamSetOutputs) {
        (++args)->setName(binding.name + "BufPtr");
    }

    // Now use the generateMultiBlockLogic method of the MultiBlockKernelBuilder subtype to
    // provide the required multi-block kernel logic.
    mCurrentMethod = multiBlockFunction;
    kb->SetInsertPoint(BasicBlock::Create(kb->getContext(), "multiBlockEntry", multiBlockFunction, 0));
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
    //  A. Temporary Buffer Area Determination
    //
    // For final block processing and for processing near the end of physical buffer
    // boundaries, we need to allocate temporary space for processing a full block of input.
    // Compute the size requirements to store stream set data at the declared processing
    // rates in reference to one block of the principal input stream.
    //

    unsigned bitBlockWidth = kb->getBitBlockWidth();
    std::vector<Type *> tempBuffers;
    std::vector<unsigned> itemsPerPrincipalBlock;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        auto & rate = mStreamSetInputs[i].rate;
        std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
        if (refSet.empty()) {
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(bitBlockWidth));
        }
        else {
            Port port; unsigned ssIdx;
            std::tie(port, ssIdx) = getStreamPort(mStreamSetInputs[i].name);
            assert (port == Port::Input && ssIdx < i);
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(itemsPerPrincipalBlock[ssIdx]));
        }
        // 
        unsigned blocks = (itemsPerPrincipalBlock.back() + bitBlockWidth - 1)/bitBlockWidth +2;
        if (blocks > 1) {
            tempBuffers.push_back(ArrayType::get(mStreamSetInputBuffers[i]->getType(), blocks));
        }
        else {
            tempBuffers.push_back(mStreamSetInputBuffers[i]->getType());
        }
    }

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        auto & rate = mStreamSetOutputs[i].rate;
        std::string refSet = mStreamSetOutputs[i].rate.referenceStreamSet();
        if (refSet.empty()) {
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(bitBlockWidth));
        }
        else {
            Port port; unsigned ssIdx;
            std::tie(port, ssIdx) = getStreamPort(mStreamSetOutputs[i].name);
            if (port == Port::Output) ssIdx += mStreamSetInputs.size();
            itemsPerPrincipalBlock.push_back(rate.calculateRatio(itemsPerPrincipalBlock[ssIdx]));
        }
        unsigned blocks = (itemsPerPrincipalBlock.back() + bitBlockWidth - 1)/bitBlockWidth +2;
        if (blocks > 1) {
            tempBuffers.push_back(ArrayType::get(mStreamSetOutputBuffers[i]->getType(), blocks));
        }
        else {
            tempBuffers.push_back(mStreamSetOutputBuffers[i]->getType());
        }
    }

    Type * tempParameterStructType = StructType::create(kb->getContext(), tempBuffers);
    Value * tempParameterArea = kb->CreateCacheAlignedAlloca(tempParameterStructType);

    ConstantInt * blockSize = kb->getSize(kb->getBitBlockWidth());

    Value * availablePos = mAvailableItemCount[0];
    Value * itemsAvail = availablePos;

    //  Make sure that corresponding data is available depending on processing rate
    //  for all input stream sets.

    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * a = mAvailableItemCount[i];
        auto & rate = mStreamSetInputs[i].rate;
        assert (((rate.referenceStreamSet().empty()) || (rate.referenceStreamSet() == mStreamSetInputs[0].name)) && "Multiblock kernel input rate not with respect to principal stream.");
        Value * maxItems = rate.CreateMaxReferenceItemsCalculation(kb.get(), a);
        itemsAvail = kb->CreateSelect(kb->CreateICmpULT(itemsAvail, maxItems), itemsAvail, maxItems);
    }

    Value * processed = kb->getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsToDo = kb->CreateSub(itemsAvail, processed);
    Value * fullBlocksToDo = kb->CreateUDiv(itemsToDo, blockSize);
    Value * excessItems = kb->CreateURem(itemsToDo, blockSize);

    //  Now we iteratively process these blocks using the doMultiBlock method.
    //  In each iteration, we process the maximum number of linearly accessible
    //  blocks on the principal input, reduced to ensure that the corresponding
    //  data is linearly available at the specified processing rates for the other inputs,
    //  and that each of the output buffers has sufficient linearly available space
    //  (using overflow areas, if necessary) for the maximum output that can be
    //  produced.

    kb->CreateBr(doSegmentOuterLoop);
    kb->SetInsertPoint(doSegmentOuterLoop);
    PHINode * const blocksRemaining = kb->CreatePHI(kb->getSizeTy(), 2, "blocksRemaining");
    blocksRemaining->addIncoming(fullBlocksToDo, entry);

    // For each input buffer, determine the processedItemCount, the block pointer for the
    // buffer block containing the next item, and the number of linearly available items.

    std::vector<Value *> processedItemCount;
    std::vector<Value *> inputBlockPtr;
    std::vector<Value *> producedItemCount;
    std::vector<Value *> outputBlockPtr;

    //  Now determine the linearly available blocks, based on blocks remaining reduced
    //  by limitations of linearly available input buffer space.

    Value * linearlyAvailBlocks = blocksRemaining;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * p = kb->getProcessedItemCount(mStreamSetInputs[i].name);
        Value * blkNo = kb->CreateUDiv(p, blockSize);
        Value * b = kb->getInputStreamBlockPtr(mStreamSetInputs[i].name, kb->getInt32(0));
        processedItemCount.push_back(p);
        inputBlockPtr.push_back(b);
        auto & rate = mStreamSetInputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator()) && (rate.referenceStreamSet() == "")) {
            blocks = mStreamSetInputBuffers[i]->getLinearlyAccessibleBlocks(kb.get(), blkNo);
        } else {
            Value * linearlyAvailItems = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(kb.get(), p);
            Value * items = rate.CreateMaxReferenceItemsCalculation(kb.get(), linearlyAvailItems);
            blocks = kb->CreateUDiv(items, blockSize);
        }
        linearlyAvailBlocks = kb->CreateSelect(kb->CreateICmpULT(blocks, linearlyAvailBlocks), blocks, linearlyAvailBlocks);
    }
    //  Now determine the linearly writeable blocks, based on available blocks reduced
    //  by limitations of output buffer space.
    Value * linearlyWritableBlocks = linearlyAvailBlocks;

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * p = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * blkNo = kb->CreateUDiv(p, blockSize);
        Value * b = kb->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, kb->getInt32(0));
        producedItemCount.push_back(p);
        outputBlockPtr.push_back(b);
        auto & rate = mStreamSetOutputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator())) {
            blocks = mStreamSetOutputBuffers[0]->getLinearlyWritableBlocks(kb.get(), blkNo);
        } else {
            Value * writableItems = mStreamSetOutputBuffers[0]->getLinearlyWritableItems(kb.get(), p);
            blocks = kb->CreateUDiv(writableItems, blockSize);
        }
        linearlyWritableBlocks = kb->CreateSelect(kb->CreateICmpULT(blocks, linearlyWritableBlocks), blocks, linearlyWritableBlocks);
    }

    Value * haveBlocks = kb->CreateICmpUGT(linearlyWritableBlocks, kb->getSize(0));
    kb->CreateCondBr(haveBlocks, doMultiBlockCall, tempBlockCheck);

    //  At this point we have verified the availability of one or more blocks of input data and output buffer space for all stream sets.
    //  Now prepare the doMultiBlock call.
    kb->SetInsertPoint(doMultiBlockCall);

    Value * linearlyAvailItems = kb->CreateMul(linearlyWritableBlocks, blockSize);

    std::vector<Value *> doMultiBlockArgs;
    doMultiBlockArgs.push_back(getInstance());
    doMultiBlockArgs.push_back(linearlyAvailItems);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * bufPtr = kb->getRawInputPointer(mStreamSetInputs[i].name, kb->getInt32(0), processedItemCount[i]);
        bufPtr = kb->CreatePointerCast(bufPtr, mStreamSetInputBuffers[i]->getPointerType());
        doMultiBlockArgs.push_back(bufPtr);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * bufPtr = kb->getRawOutputPointer(mStreamSetOutputs[i].name, kb->getInt32(0), producedItemCount[i]);
        bufPtr = kb->CreatePointerCast(bufPtr, mStreamSetOutputBuffers[i]->getPointerType());
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
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(kb.get(), priorBlock);
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
            Value * accessible = cb->getLinearlyAccessibleItems(kb.get(), producedItemCount[i]);
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
    Value * reducedBlocksToDo = kb->CreateSub(blocksRemaining, linearlyWritableBlocks);
    BasicBlock * multiBlockFinal = kb->GetInsertBlock();
    blocksRemaining->addIncoming(reducedBlocksToDo, multiBlockFinal);
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
    haveBlocks = kb->CreateICmpUGT(blocksRemaining, kb->getSize(0));
    kb->CreateCondBr(kb->CreateOr(mIsFinal, haveBlocks), doTempBufferBlock, segmentDone);

    // We use temporary buffers in 3 different cases that preclude full block processing.
    // (a) One or more input buffers does not have a sufficient number of input items linearly available.
    // (b) One or more output buffers does not have sufficient linearly available buffer space.
    // (c) We have processed all the full blocks of input and only the excessItems remain.
    // In each case we set up temporary buffers for input and output and then
    // call the Multiblock routine.
    //
    kb->SetInsertPoint(doTempBufferBlock);
    Value * tempBlockItems = kb->CreateSelect(haveBlocks, blockSize, excessItems);
    Value * doFinal = kb->CreateNot(haveBlocks);

    // Begin constructing the doMultiBlock args.
    std::vector<Value *> tempArgs;
    tempArgs.push_back(getInstance());
    tempArgs.push_back(tempBlockItems);

    // Prepare the temporary buffer area.
    //
    // First zero it out.
    Constant * const tempAreaSize = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(tempParameterStructType), kb->getSizeTy(), false);
    kb->CreateMemZero(tempParameterArea, tempAreaSize);

    // For each input and output buffer, copy over necessary data starting from the last
    // block boundary.
    std::vector<Value *> finalItemCountNeeded;
    finalItemCountNeeded.push_back(kb->CreateAdd(processedItemCount[0], tempBlockItems));

    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); i++) {
        Value * tempBufPtr = kb->CreateGEP(tempParameterArea, kb->getInt32(i));
        Type * bufPtrType = mStreamSetInputBuffers[i]->getPointerType();
        tempBufPtr = kb->CreatePointerCast(tempBufPtr, bufPtrType);
        Value * blockBasePos = kb->CreateAnd(processedItemCount[i], blockBaseMask);
        // The number of items to copy is determined by the processing rate requirements.
        if (i > 1) {
            auto & rate = mStreamSetInputs[i].rate;
            std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
            if (refSet.empty()) {
                finalItemCountNeeded.push_back(rate.CreateRatioCalculation(kb.get(), finalItemCountNeeded[0], doFinal));
            }
            else {
                Port port; unsigned ssIdx;
                std::tie(port, ssIdx) = getStreamPort(mStreamSetInputs[i].name);
                assert (port == Port::Input && ssIdx < i);
                finalItemCountNeeded.push_back(rate.CreateRatioCalculation(kb.get(), finalItemCountNeeded[ssIdx], doFinal));
            }
        }
        Value * neededItems = kb->CreateSub(finalItemCountNeeded[i], blockBasePos);
        Value * availFromBase = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(kb.get(), blockBasePos);
        Value * copyItems1 = kb->CreateSelect(kb->CreateICmpULT(neededItems, availFromBase), neededItems, availFromBase);
        Value * copyItems2 = kb->CreateSub(neededItems, copyItems1);
        Value * inputPtr = kb->CreatePointerCast(kb->getRawInputPointer(mStreamSetInputs[i].name, kb->getInt32(0), blockBasePos), bufPtrType);
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(kb.get(), tempBufPtr, inputPtr, copyItems1);
        Value * nextBufPtr = kb->CreateGEP(tempBufPtr, kb->CreateUDiv(copyItems1, blockSize));
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(kb.get(), nextBufPtr, kb->getStreamSetBufferPtr(mStreamSetInputs[i].name), copyItems2);
        Value * itemAddress = kb->getRawInputPointer(mStreamSetInputs[i].name, kb->getInt32(0), processedItemCount[i]);
        itemAddress = kb->CreatePtrToInt(itemAddress, intAddrTy);
        Value * baseAddress = inputBlockPtr[i];
        baseAddress = kb->CreatePtrToInt(baseAddress, intAddrTy);
        Value * tempAddress = kb->CreateAdd(kb->CreatePtrToInt(tempBufPtr, intAddrTy), kb->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(kb->CreateIntToPtr(tempAddress, bufPtrType));
    }

    std::vector<Value *> blockBasePos;
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = kb->CreateGEP(tempParameterArea, kb->getInt32(mStreamSetInputs.size() + i));
        Type * bufPtrType = mStreamSetOutputBuffers[i]->getPointerType();
        tempBufPtr = kb->CreatePointerCast(tempBufPtr, bufPtrType);
        producedItemCount[i] = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        blockBasePos.push_back(kb->CreateAnd(producedItemCount[i], blockBaseMask));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), tempBufPtr, outputBlockPtr[i], kb->CreateSub(producedItemCount[i], blockBasePos[i]));
        Value * itemAddress = kb->CreatePtrToInt(kb->getRawOutputPointer(mStreamSetOutputs[i].name, kb->getInt32(0), producedItemCount[i]), intAddrTy);
        Value * outputPtr = kb->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, kb->getInt32(0));
        Value * baseAddress = kb->CreatePtrToInt(outputPtr, intAddrTy);
        Value * tempAddress = kb->CreateAdd(kb->CreatePtrToInt(tempBufPtr, intAddrTy), kb->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(kb->CreateIntToPtr(tempAddress, bufPtrType));
    }


    kb->CreateCall(multiBlockFunction, tempArgs);

    // Copy back data to the actual output buffers.

    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = kb->CreateGEP(tempParameterArea, kb->getInt32(mStreamSetInputs.size() + i));
        tempBufPtr = kb->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        Value * finalItems = kb->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * copyItems = kb->CreateSub(finalItems, blockBasePos[i]);
        
        Value * writableFromBase = mStreamSetOutputBuffers[i]->getLinearlyWritableItems(kb.get(), blockBasePos[i]); // must be a whole number of blocks.
        Value * copyItems1 = kb->CreateSelect(kb->CreateICmpULT(copyItems, writableFromBase), copyItems, writableFromBase);
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), outputBlockPtr[i], tempBufPtr, copyItems1);
        Value * copyItems2 = kb->CreateSub(copyItems, copyItems1);
        tempBufPtr = kb->CreateGEP(tempBufPtr, kb->CreateUDiv(copyItems1, blockSize));
        Value * outputBaseBlockPtr = kb->CreateGEP(kb->getBaseAddress(mStreamSetOutputs[i].name), kb->getInt32(0));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(kb.get(), outputBaseBlockPtr, tempBufPtr, copyItems2);
    }

    kb->setProcessedItemCount(mStreamSetInputs[0].name, finalItemCountNeeded[0]);

    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    //
    blocksRemaining->addIncoming(kb->CreateSub(blocksRemaining, kb->CreateZExt(haveBlocks, kb->getSizeTy())), kb->GetInsertBlock());
    kb->CreateCondBr(haveBlocks, doSegmentOuterLoop, segmentDone);
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
    return name;
}

// CONSTRUCTOR
Kernel::Kernel(std::string && kernelName,
               std::vector<Binding> && stream_inputs,
               std::vector<Binding> && stream_outputs,
               std::vector<Binding> && scalar_parameters,
               std::vector<Binding> && scalar_outputs,
               std::vector<Binding> && internal_scalars)
: KernelInterface(std::move(annotateKernelNameWithDebugFlags(std::move(kernelName)))
                  , std::move(stream_inputs), std::move(stream_outputs)
                  , std::move(scalar_parameters), std::move(scalar_outputs)
                  , std::move(internal_scalars))
, mCurrentMethod(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr) {

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
    
}
