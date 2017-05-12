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
    
void Kernel::createKernelStub(const std::unique_ptr<KernelBuilder> & idb, const StreamSetBuffers & inputs, const StreamSetBuffers & outputs) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    assert ("IDISA Builder does not have a valid Module" && idb->getModule());
    std::stringstream cacheName;   
    cacheName << getName() << '_' << idb->getBuilderUniqueName();
    for (const StreamSetBuffer * b: inputs) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    for (const StreamSetBuffer * b: outputs) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    Module * const kernelModule = new Module(cacheName.str(), idb->getContext());
    createKernelStub(idb, inputs, outputs, kernelModule);
}

void Kernel::createKernelStub(const std::unique_ptr<KernelBuilder> & idb, const StreamSetBuffers & inputs, const StreamSetBuffers & outputs, Module * const kernelModule) {
    assert (mModule == nullptr);
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
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

    mModule = kernelModule;
    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());
    prepareKernel(idb);
}

void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
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
    const auto blockSize = idb->getBitBlockWidth();
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        if ((mStreamSetInputBuffers[i]->getBufferBlocks() > 0) && (mStreamSetInputBuffers[i]->getBufferBlocks() < codegen::SegmentSize + (blockSize + mLookAheadPositions - 1)/blockSize)) {
            report_fatal_error("Kernel preparation: Buffer size too small " + mStreamSetInputs[i].name);
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
    for (const auto binding : mScalarInputs) {
        addScalar(binding.type, binding.name);
    }
    for (const auto binding : mScalarOutputs) {
        addScalar(binding.type, binding.name);
    }
    if (mStreamMap.empty()) {
        prepareStreamSetNameMap();
    }
    for (auto binding : mInternalScalars) {
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

    mKernelStateType = StructType::create(idb->getContext(), mKernelFields, getName());
}

// Default kernel signature: generate the IR and emit as byte code.
std::string Kernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb.get());
    if (LLVM_LIKELY(moduleIDisSignature())) {
        return getModule()->getModuleIdentifier();
    } else {
        generateKernel(idb);
        std::string signature;
        raw_string_ostream OS(signature);
        WriteBitcodeToFile(getModule(), OS);
        return signature;
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
    //CurrentMethod->dump();
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
        assert (false);
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

    /// Check if the do block method is called and create the function if necessary    
    if (!idb->supportsIndirectBr()) {
        FunctionType * const type = FunctionType::get(idb->getVoidTy(), {self->getType()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + DO_BLOCK_SUFFIX, idb->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        idb->SetInsertPoint(idb->CreateBasicBlock("entry"));
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


    /// Call the do block method if necessary then restore the current function state to the do segement method
    if (!idb->supportsIndirectBr()) {
        idb->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        idb->restoreIP(ip);
        idb->CreateCall(mCurrentMethod, self);
        setInstance(self);
        mCurrentMethod = cp;
    }

}

inline void BlockOrientedKernel::writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, Value * remainingItems) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = idb->saveIP();

    if (!idb->supportsIndirectBr()) {
        FunctionType * const type = FunctionType::get(idb->getVoidTy(), {self->getType(), idb->getSizeTy()}, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + FINAL_BLOCK_SUFFIX, idb->getModule());
        mCurrentMethod->setCallingConv(CallingConv::C);
        mCurrentMethod->setDoesNotThrow();
        mCurrentMethod->setDoesNotCapture(1);
        auto args = mCurrentMethod->arg_begin();
        args->setName("self");
        setInstance(&*args);
        remainingItems = &*(++args);
        remainingItems->setName("remainingItems");
        idb->SetInsertPoint(idb->CreateBasicBlock("entry"));
    }

    generateFinalBlockMethod(idb, remainingItems); // may be implemented by the BlockOrientedKernel subtype

    RecursivelyDeleteTriviallyDeadInstructions(remainingItems); // if remainingItems was not used, this will eliminate it.

    if (!idb->supportsIndirectBr()) {
        idb->CreateRetVoid();
        idb->restoreIP(ip);
        idb->CreateCall(mCurrentMethod, {self, remainingItemCount});
        mCurrentMethod = cp;
        setInstance(self);
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
        idb->CreateCall(mDoBlockMethod, getInstance());
    }
}

void MultiBlockKernel::generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) {

    KernelBuilder * const iBuilder = kb.get();
    auto ip = iBuilder->saveIP();
    Function * const cp = mCurrentMethod;
    
    // First prepare the multi-block method that will be used.

    std::vector<Type *> multiBlockParmTypes;
    multiBlockParmTypes.push_back(mKernelStateType->getPointerTo());
    multiBlockParmTypes.push_back(iBuilder->getSizeTy());
    for (auto buffer : mStreamSetInputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }
    for (auto buffer : mStreamSetOutputBuffers) {
        multiBlockParmTypes.push_back(buffer->getPointerType());
    }
    
    FunctionType * const type = FunctionType::get(iBuilder->getVoidTy(), multiBlockParmTypes, false);
    Function * multiBlockFunction = Function::Create(type, GlobalValue::InternalLinkage, getName() + MULTI_BLOCK_SUFFIX, iBuilder->getModule());
    multiBlockFunction->setCallingConv(CallingConv::C);
    multiBlockFunction->setDoesNotThrow();
    auto args = multiBlockFunction->arg_begin();
    args->setName("self");
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
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "multiBlockEntry", multiBlockFunction, 0));
    generateMultiBlockLogic(kb);

    iBuilder->CreateRetVoid();
    
    iBuilder->restoreIP(ip);
    mCurrentMethod = cp;
    
    // Now proceed with creation of the doSegment method.

    BasicBlock * const entry = iBuilder->GetInsertBlock();
    BasicBlock * const doSegmentOuterLoop = iBuilder->CreateBasicBlock(getName() + "_doSegmentOuterLoop");
    BasicBlock * const doMultiBlockCall = iBuilder->CreateBasicBlock(getName() + "_doMultiBlockCall");
    BasicBlock * const tempBlockCheck = iBuilder->CreateBasicBlock(getName() + "_tempBlockCheck");
    BasicBlock * const doTempBufferBlock = iBuilder->CreateBasicBlock(getName() + "_doTempBufferBlock");
    BasicBlock * const segmentDone = iBuilder->CreateBasicBlock(getName() + "_segmentDone");

    Value * blockBaseMask = iBuilder->CreateNot(iBuilder->getSize(iBuilder->getBitBlockWidth() - 1));

    //
    //  A. Temporary Buffer Area Determination
    //
    // For final block processing and for processing near the end of physical buffer
    // boundaries, we need to allocate temporary space for processing a full block of input.
    // Compute the size requirements to store stream set data at the declared processing
    // rates in reference to one block of the principal input stream.
    //

    unsigned bitBlockWidth = iBuilder->getBitBlockWidth();
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
    Type * tempParameterStructType = StructType::create(iBuilder->getContext(), tempBuffers);
    Value * tempParameterArea = iBuilder->CreateCacheAlignedAlloca(tempParameterStructType);

    ConstantInt * blockSize = iBuilder->getSize(iBuilder->getBitBlockWidth());

    Value * availablePos = mAvailableItemCount[0];
    Value * itemsAvail = availablePos;
    //  Make sure that corresponding data is available depending on processing rate
    //  for all input stream sets.
    for (unsigned i = 1; i < mStreamSetInputs.size(); i++) {
        Value * a = mAvailableItemCount[i];
        auto & rate = mStreamSetInputs[i].rate;
        assert (((rate.referenceStreamSet().empty()) || (rate.referenceStreamSet() == mStreamSetInputs[0].name)) && "Multiblock kernel input rate not with respect to principal stream.");
        Value * maxItems = rate.CreateMaxReferenceItemsCalculation(iBuilder, a);
        itemsAvail = iBuilder->CreateSelect(iBuilder->CreateICmpULT(itemsAvail, maxItems), itemsAvail, maxItems);
    }

    Value * processed = iBuilder->getProcessedItemCount(mStreamSetInputs[0].name);
    Value * itemsToDo = iBuilder->CreateSub(itemsAvail, processed);
    Value * fullBlocksToDo = iBuilder->CreateUDiv(itemsToDo, blockSize);
    Value * excessItems = iBuilder->CreateURem(itemsToDo, blockSize);

    //  Now we iteratively process these blocks using the doMultiBlock method.
    //  In each iteration, we process the maximum number of linearly accessible
    //  blocks on the principal input, reduced to ensure that the corresponding
    //  data is linearly available at the specified processing rates for the other inputs,
    //  and that each of the output buffers has sufficient linearly available space
    //  (using overflow areas, if necessary) for the maximum output that can be
    //  produced.

    //iBuilder->CreateCondBr(iBuilder->CreateICmpUGT(fullBlocksToDo, iBuilder->getSize(0)), doSegmentOuterLoop, finalBlockCheck);
    
    iBuilder->CreateBr(doSegmentOuterLoop);
    iBuilder->SetInsertPoint(doSegmentOuterLoop);
    PHINode * const blocksRemaining = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "blocksRemaining");
    blocksRemaining->addIncoming(fullBlocksToDo, entry);
    // For each input buffer, determine the processedItemCount, the block pointer for the
    // buffer block containing the next item, and the number of linearly available items.
    //
    std::vector<Value *> processedItemCount;
    std::vector<Value *> inputBlockPtr;
    std::vector<Value *> producedItemCount;
    std::vector<Value *> outputBlockPtr;

    //  Now determine the linearly available blocks, based on blocks remaining reduced
    //  by limitations of linearly available input buffer space.
    Value * linearlyAvailBlocks = blocksRemaining;
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * p = iBuilder->getProcessedItemCount(mStreamSetInputs[i].name);
        Value * blkNo = iBuilder->CreateUDiv(p, blockSize);
        Value * b = iBuilder->getInputStreamBlockPtr(mStreamSetInputs[i].name, iBuilder->getInt32(0));
        processedItemCount.push_back(p);
        inputBlockPtr.push_back(b);
        auto & rate = mStreamSetInputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator()) && (rate.referenceStreamSet() == "")) {
            blocks = mStreamSetInputBuffers[i]->getLinearlyAccessibleBlocks(iBuilder, blkNo);
        } else {
            Value * linearlyAvailItems = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(iBuilder, p);
            Value * items = rate.CreateMaxReferenceItemsCalculation(iBuilder, linearlyAvailItems);
            blocks = iBuilder->CreateUDiv(items, blockSize);
        }
        linearlyAvailBlocks = iBuilder->CreateSelect(iBuilder->CreateICmpULT(blocks, linearlyAvailBlocks), blocks, linearlyAvailBlocks);
    }
    //  Now determine the linearly writeable blocks, based on available blocks reduced
    //  by limitations of output buffer space.
    Value * linearlyWritableBlocks = linearlyAvailBlocks;

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * p = iBuilder->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * blkNo = iBuilder->CreateUDiv(p, blockSize);
        Value * b = iBuilder->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, iBuilder->getInt32(0));
        producedItemCount.push_back(p);
        outputBlockPtr.push_back(b);
        auto & rate = mStreamSetOutputs[i].rate;
        Value * blocks = nullptr;
        if ((rate.isFixedRatio()) && (rate.getRatioNumerator() == rate.getRatioDenominator())) {
            blocks = mStreamSetOutputBuffers[0]->getLinearlyWritableBlocks(iBuilder, blkNo);
        } else {
            Value * writableItems = mStreamSetOutputBuffers[0]->getLinearlyWritableItems(iBuilder, p);
            blocks = iBuilder->CreateUDiv(writableItems, blockSize);
        }
        linearlyWritableBlocks = iBuilder->CreateSelect(iBuilder->CreateICmpULT(blocks, linearlyWritableBlocks), blocks, linearlyWritableBlocks);
    }
    Value * haveBlocks = iBuilder->CreateICmpUGT(linearlyWritableBlocks, iBuilder->getSize(0));
    iBuilder->CreateCondBr(haveBlocks, doMultiBlockCall, tempBlockCheck);

    //  At this point we have verified the availability of one or more blocks of input data and output buffer space for all stream sets.
    //  Now prepare the doMultiBlock call.
    iBuilder->SetInsertPoint(doMultiBlockCall);

    Value * linearlyAvailItems = iBuilder->CreateMul(linearlyWritableBlocks, blockSize);

    std::vector<Value *> doMultiBlockArgs;
    doMultiBlockArgs.push_back(getInstance());
    doMultiBlockArgs.push_back(linearlyAvailItems);
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        Value * bufPtr = iBuilder->getRawInputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), processedItemCount[i]);
        bufPtr = iBuilder->CreatePointerCast(bufPtr, mStreamSetInputBuffers[i]->getPointerType());
        doMultiBlockArgs.push_back(bufPtr);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * bufPtr = iBuilder->getRawOutputPointer(mStreamSetOutputs[i].name, iBuilder->getInt32(0), producedItemCount[i]);
        bufPtr = iBuilder->CreatePointerCast(bufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        doMultiBlockArgs.push_back(bufPtr);
    }

    iBuilder->CreateCall(multiBlockFunction, doMultiBlockArgs);
    // Do copybacks if necessary.
    unsigned priorIdx = 0;
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * log2BlockSize = iBuilder->getSize(std::log2(iBuilder->getBitBlockWidth()));
        if (auto cb = dyn_cast<SwizzledCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = iBuilder->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = iBuilder->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * newlyProduced = iBuilder->CreateSub(iBuilder->getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * priorBlock = iBuilder->CreateLShr(producedItemCount[i], log2BlockSize);
            Value * priorOffset = iBuilder->CreateAnd(producedItemCount[i], iBuilder->getSize(iBuilder->getBitBlockWidth() - 1));
            Value * instance = iBuilder->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * accessibleBlocks = cb->getLinearlyAccessibleBlocks(iBuilder, priorBlock);
            Value * accessible = iBuilder->CreateSub(iBuilder->CreateShl(accessibleBlocks, log2BlockSize), priorOffset);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
        if (auto cb = dyn_cast<CircularCopybackBuffer>(mStreamSetOutputBuffers[i]))  {
            BasicBlock * copyBack = iBuilder->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBack");
            BasicBlock * done = iBuilder->CreateBasicBlock(mStreamSetOutputs[i].name + "_copyBackDone");
            Value * instance = iBuilder->getStreamSetBufferPtr(mStreamSetOutputs[i].name);
            Value * newlyProduced = iBuilder->CreateSub(iBuilder->getProducedItemCount(mStreamSetOutputs[i].name), producedItemCount[i]);
            Value * accessible = cb->getLinearlyAccessibleItems(iBuilder, producedItemCount[i]);
            Value * wraparound = iBuilder->CreateICmpULT(accessible, newlyProduced);
            iBuilder->CreateCondBr(wraparound, copyBack, done);
            iBuilder->SetInsertPoint(copyBack);
            Value * copyItems = iBuilder->CreateSub(newlyProduced, accessible);
            cb->createCopyBack(iBuilder, instance, copyItems);
            iBuilder->CreateBr(done);
            iBuilder->SetInsertPoint(done);
            priorIdx++;
        }
    }
    iBuilder->setProcessedItemCount(mStreamSetInputs[0].name, iBuilder->CreateAdd(processed, linearlyAvailItems));
    Value * reducedBlocksToDo = iBuilder->CreateSub(blocksRemaining, linearlyWritableBlocks);
    Value * fullBlocksRemain = iBuilder->CreateICmpUGT(reducedBlocksToDo, iBuilder->getSize(0));
    BasicBlock * multiBlockFinal = iBuilder->GetInsertBlock();
    blocksRemaining->addIncoming(reducedBlocksToDo, multiBlockFinal);
    iBuilder->CreateCondBr(fullBlocksRemain, doSegmentOuterLoop, tempBlockCheck);
    //iBuilder->CreateBr(doSegmentOuterLoop);
    //
    // We use temporary buffers in 3 different cases that preclude full block processing.
    // (a) One or more input buffers does not have a sufficient number of input items linearly available.
    // (b) One or more output buffers does not have sufficient linearly available buffer space.
    // (c) We have processed all the full blocks of input and only the excessItems remain.
    // In each case we set up temporary buffers for input and output and then
    // call the Multiblock routine.
    //

    iBuilder->SetInsertPoint(tempBlockCheck);
    PHINode * const tempBlocksRemain = iBuilder->CreatePHI(iBuilder->getSizeTy(), 2, "tempBlocksRemain");
    tempBlocksRemain->addIncoming(blocksRemaining, doSegmentOuterLoop);
    tempBlocksRemain->addIncoming(reducedBlocksToDo, multiBlockFinal);
    
    haveBlocks = iBuilder->CreateICmpUGT(tempBlocksRemain, iBuilder->getSize(0));
    iBuilder->CreateCondBr(iBuilder->CreateOr(mIsFinal, haveBlocks), doTempBufferBlock, segmentDone);

    //
    // We use temporary buffers in 3 different cases that preclude full block processing.
    // (a) One or more input buffers does not have a sufficient number of input items linearly available.
    // (b) One or more output buffers does not have sufficient linearly available buffer space.
    // (c) We have processed all the full blocks of input and only the excessItems remain.
    // In each case we set up temporary buffers for input and output and then
    // call the Multiblock routine.
    //
    iBuilder->SetInsertPoint(doTempBufferBlock);
    Value * tempBlockItems = iBuilder->CreateSelect(haveBlocks, blockSize, excessItems);

    // Begin constructing the doMultiBlock args.
    std::vector<Value *> tempArgs;
    tempArgs.push_back(getInstance());
    tempArgs.push_back(tempBlockItems);

    // Prepare the temporary buffer area.
    //
    // First zero it out.
    Constant * const tempAreaSize = ConstantExpr::getIntegerCast(ConstantExpr::getSizeOf(tempParameterStructType), iBuilder->getSizeTy(), false);
    iBuilder->CreateMemZero(tempParameterArea, tempAreaSize);
    
    // For each input and output buffer, copy over necessary data starting from the last
    // block boundary.
    std::vector<Value *> finalItemPos;
    finalItemPos.push_back(iBuilder->CreateAdd(processedItemCount[0], tempBlockItems));

    for (unsigned i = 0; i < mStreamSetInputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetInputBuffers[i]->getPointerType());

        Value * blockItemPos = iBuilder->CreateAnd(processedItemCount[i], blockBaseMask);

        // The number of items to copy is determined by the processing rate requirements.
        if (i > 1) {
            auto & rate = mStreamSetInputs[i].rate;
            std::string refSet = mStreamSetInputs[i].rate.referenceStreamSet();
            if (refSet.empty()) {
                finalItemPos.push_back(rate.CreateRatioCalculation(iBuilder, finalItemPos[0], iBuilder->CreateNot(haveBlocks)));
            }
            else {
                Port port; unsigned ssIdx;
                std::tie(port, ssIdx) = getStreamPort(mStreamSetInputs[i].name);
                assert (port == Port::Input && ssIdx < i);
                finalItemPos.push_back(rate.CreateRatioCalculation(iBuilder, finalItemPos[ssIdx], iBuilder->CreateNot(haveBlocks)));
            }
        }
        Value * neededItems = iBuilder->CreateSub(finalItemPos[i], blockItemPos);
        Value * availFromBase = mStreamSetInputBuffers[i]->getLinearlyAccessibleItems(iBuilder, blockItemPos);
        Value * copyItems1 = iBuilder->CreateSelect(iBuilder->CreateICmpULT(neededItems, availFromBase), neededItems, availFromBase);
        Value * copyItems2 = iBuilder->CreateSub(neededItems, copyItems1);
        Value * inputPtr = iBuilder->getInputStreamBlockPtr(mStreamSetInputs[i].name, iBuilder->getInt32(0));
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(iBuilder, tempBufPtr, inputPtr, copyItems1);
        Value * nextBufPtr = iBuilder->CreateGEP(tempBufPtr, iBuilder->CreateUDiv(availFromBase, blockSize));
        mStreamSetInputBuffers[i]->createBlockAlignedCopy(iBuilder, nextBufPtr, iBuilder->getStreamSetBufferPtr(mStreamSetInputs[i].name), copyItems2);
        Value * itemAddress = iBuilder->CreatePtrToInt(iBuilder->getRawInputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), processedItemCount[i]), iBuilder->getSizeTy());
        Value * baseAddress = iBuilder->CreatePtrToInt(inputBlockPtr[i], iBuilder->getSizeTy());
        Value * tempAddress = iBuilder->CreateAdd(iBuilder->CreatePtrToInt(tempBufPtr, iBuilder->getSizeTy()), iBuilder->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(iBuilder->CreateIntToPtr(tempAddress, mStreamSetInputBuffers[i]->getPointerType()));
    }

    std::vector<Value *> blockItemPos;
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(mStreamSetInputs.size() + i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        blockItemPos.push_back(iBuilder->CreateAnd(producedItemCount[i], blockBaseMask));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, tempBufPtr, outputBlockPtr[i], iBuilder->CreateSub(producedItemCount[i], blockItemPos[i]));
        Value * itemAddress = iBuilder->CreatePtrToInt(iBuilder->getRawOutputPointer(mStreamSetInputs[i].name, iBuilder->getInt32(0), producedItemCount[i]), iBuilder->getSizeTy());
        Value * outputPtr = iBuilder->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, iBuilder->getInt32(0));
        Value * baseAddress = iBuilder->CreatePtrToInt(outputPtr, iBuilder->getSizeTy());
        Value * tempAddress = iBuilder->CreateAdd(iBuilder->CreatePtrToInt(tempBufPtr, iBuilder->getSizeTy()), iBuilder->CreateSub(itemAddress, baseAddress));
        tempArgs.push_back(iBuilder->CreateIntToPtr(tempAddress, mStreamSetOutputBuffers[i]->getPointerType()));
    }

    
    iBuilder->CreateCall(multiBlockFunction, tempArgs);

    // Copy back data to the actual output buffers.

    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); i++) {
        Value * tempBufPtr = iBuilder->CreateGEP(tempParameterArea, iBuilder->getInt32(mStreamSetInputs.size() + i));
        tempBufPtr = iBuilder->CreatePointerCast(tempBufPtr, mStreamSetOutputBuffers[i]->getPointerType());
        Value * final_items = iBuilder->getProducedItemCount(mStreamSetOutputs[i].name);
        Value * copyItems = iBuilder->CreateSub(final_items, blockItemPos[i]);
        Value * copyItems1 = mStreamSetOutputBuffers[i]->getLinearlyWritableItems(iBuilder, blockItemPos[i]); // must be a whole number of blocks.
        Value * outputPtr = iBuilder->getOutputStreamBlockPtr(mStreamSetOutputs[i].name, iBuilder->getInt32(0));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, outputPtr, tempBufPtr, copyItems1);
        Value * copyItems2 = iBuilder->CreateSelect(iBuilder->CreateICmpULT(copyItems, copyItems), iBuilder->getSize(0), iBuilder->CreateSub(copyItems, copyItems1));
        tempBufPtr = iBuilder->CreateGEP(tempBufPtr, iBuilder->CreateUDiv(copyItems1, blockSize));
        mStreamSetOutputBuffers[i]->createBlockAlignedCopy(iBuilder, iBuilder->getStreamSetBufferPtr(mStreamSetOutputs[i].name), tempBufPtr, copyItems2);
    }

    iBuilder->setProcessedItemCount(mStreamSetInputs[0].name, finalItemPos[0]);

    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    //
    blocksRemaining->addIncoming(iBuilder->CreateSub(tempBlocksRemain, iBuilder->CreateZExt(haveBlocks, iBuilder->getSizeTy())), iBuilder->GetInsertBlock());
    iBuilder->CreateCondBr(haveBlocks, doSegmentOuterLoop, segmentDone);
    iBuilder->SetInsertPoint(segmentDone);
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

// CONSTRUCTOR
Kernel::Kernel(std::string && kernelName,
                             std::vector<Binding> && stream_inputs,
                             std::vector<Binding> && stream_outputs,
                             std::vector<Binding> && scalar_parameters,
                             std::vector<Binding> && scalar_outputs,
                             std::vector<Binding> && internal_scalars)
: KernelInterface(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
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
