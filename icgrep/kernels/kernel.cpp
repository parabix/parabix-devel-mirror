/*
 *  Copyright (c) 2016-7 International Characters.
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
#if LLVM_VERSION_INTEGER < LLVM_4_0_0
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>
#include <boost/math/common_factor_rt.hpp>
#include <llvm/Support/Debug.h>

using namespace llvm;
using namespace parabix;
using namespace boost::math;

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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addScalar
 ** ------------------------------------------------------------------------------------------------------------- */
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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addUnnamedScalar
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned Kernel::addUnnamedScalar(Type * const type) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error("Cannot add unnamed field  to " + getName() + " after kernel state finalized");
    }
    const auto index = mKernelFields.size();
    mKernelFields.push_back(type);
    return index;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareStreamSetNameMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareStreamSetNameMap() {
    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamMap.emplace(mStreamSetInputs[i].getName(), std::make_pair(Port::Input, i));
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamMap.emplace(mStreamSetOutputs[i].getName(), std::make_pair(Port::Output, i));
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindPorts
 ** ------------------------------------------------------------------------------------------------------------- */
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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCacheName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getCacheName(const std::unique_ptr<KernelBuilder> & idb) const {
    std::stringstream cacheName;
    cacheName << getName() << '_' << idb->getBuilderUniqueName();
    for (const StreamSetBuffer * b: mStreamSetInputBuffers) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    for (const StreamSetBuffer * b: mStreamSetOutputBuffers) {
        cacheName <<  ':' <<  b->getUniqueID();
    }
    return cacheName.str();
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * Kernel::setModule(Module * const module) {
    assert (mModule == nullptr || mModule == module);
    assert (module != nullptr);
    mModule = module;
    return mModule;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * Kernel::makeModule(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    return setModule(new Module(getCacheName(idb), idb->getContext()));
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    addBaseKernelProperties(idb);
    addInternalKernelProperties(idb);
    // NOTE: StructType::create always creates a new type even if an identical one exists.
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        setModule(new Module(getCacheName(idb), idb->getContext()));
    }
    mKernelStateType = mModule->getTypeByName(getName());
    if (LLVM_LIKELY(mKernelStateType == nullptr)) {
        mKernelStateType = StructType::create(idb->getContext(), mKernelFields, getName());
        assert (mKernelStateType);
    }    
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareCachedKernel(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    assert (getModule());
    addBaseKernelProperties(idb);
    mKernelStateType = getModule()->getTypeByName(getName());
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Kernel definition for " + getName() + " could not be found in the cache object");
    }    
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemsPerStride
 ** ------------------------------------------------------------------------------------------------------------- */
std::pair<unsigned, unsigned> Kernel::getStreamRate(const Port p, const unsigned i) const {
    const ProcessingRate & rate = (p == Port::Input) ? mStreamSetInputs[i].getRate() : mStreamSetOutputs[i].getRate();
    unsigned min = 0, max = 0;
    if (rate.isFixed()) {
        min = max = rate.getRate();
    } else if (rate.isBounded()) {
        min = rate.getLowerBound();
        max = rate.getUpperBound();
    } else if (rate.isUnknown()) {
        min = rate.getLowerBound();
        max = 0;
    } else if (rate.isExactlyRelative()) {
        for (unsigned j = 0; j < mStreamSetInputs.size(); ++j) {
            if (mStreamSetInputs[j].getName() == rate.getReference()) {
                std::tie(min, max) = getStreamRate(Port::Input, j);
                min = (min * rate.getNumerator()) / rate.getDenominator();
                assert (max == 0 || (max * rate.getNumerator()) % rate.getDenominator() == 0);
                max = (max * rate.getNumerator()) / rate.getDenominator();
                return std::make_pair(min, max);
            }
        }
        for (unsigned j = 0; j < mStreamSetOutputs.size(); ++j) {
            if (mStreamSetOutputs[j].getName() == rate.getReference()) {
                assert (p == Port::Output);
                std::tie(min, max) = getStreamRate(Port::Output, j);
                min = (min * rate.getNumerator()) / rate.getDenominator();
                assert (max == 0 || (max * rate.getNumerator()) % rate.getDenominator() == 0);
                max = (max * rate.getNumerator()) / rate.getDenominator();
                return std::make_pair(min, max);
            }
        }
        llvm_unreachable("Reference rate must be associated with an input or output!");
    }
    return std::make_pair(min, max);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addBaseKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & idb) {
    
    const unsigned inputSetCount = mStreamSetInputs.size();
    const unsigned outputSetCount = mStreamSetOutputs.size();
    
    assert (inputSetCount == mStreamSetInputBuffers.size());
    assert (outputSetCount == mStreamSetOutputBuffers.size());

    if (mStride == 0) {
        // Set the default kernel stride.
        mStride = idb->getBitBlockWidth();
    }

    IntegerType * const sizeTy = idb->getSizeTy();

    for (unsigned i = 0; i < inputSetCount; i++) {
        const Binding & b = mStreamSetInputs[i];
        //const ProcessingRate & rate = b.getRate();
        //if (rate.isBounded() || rate.isUnknown()) {
            addScalar(sizeTy, b.getName() + PROCESSED_ITEM_COUNT_SUFFIX);
        //}
    }

    for (unsigned i = 0; i < outputSetCount; i++) {
        const Binding & b = mStreamSetOutputs[i];
        //const ProcessingRate & rate = b.getRate();
        //if (rate.isBounded() || rate.isUnknown()) {
            addScalar(sizeTy, b.getName() + PRODUCED_ITEM_COUNT_SUFFIX);
        //}
    }

    for (unsigned i = 0; i < inputSetCount; i++) {
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetInputs[i].getName() + BUFFER_PTR_SUFFIX);
    }
    for (unsigned i = 0; i < outputSetCount; i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetOutputs[i].getName() + BUFFER_PTR_SUFFIX);
    }
    for (const auto & binding : mScalarInputs) {
        addScalar(binding.getType(), binding.getName());
    }
    for (const auto & binding : mScalarOutputs) {
        addScalar(binding.getType(), binding.getName());
    }
    if (mStreamMap.empty()) {
        prepareStreamSetNameMap();
    }
    for (const auto & binding : mInternalScalars) {
        addScalar(binding.getType(), binding.getName());
    }
    Type * const consumerSetTy = StructType::get(idb->getContext(), {sizeTy, sizeTy->getPointerTo()->getPointerTo()})->getPointerTo();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(consumerSetTy, mStreamSetOutputs[i].getName() + CONSUMER_SUFFIX);
    }
    addScalar(sizeTy, LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(idb->getInt1Ty(), TERMINATION_SIGNAL);
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(sizeTy, mStreamSetOutputs[i].getName() + CONSUMED_ITEM_COUNT_SUFFIX);
    }
    // We compile in a 64-bit CPU cycle counter into every kernel.   It will remain unused
    // in normal execution, but when codegen::EnableCycleCounter is specified, pipelines
    // will be able to add instrumentation to cached modules without recompilation.
    addScalar(idb->getInt64Ty(), CYCLECOUNT_SCALAR);

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeSignature
 *
 * Default kernel signature: generate the IR and emit as byte code.
 ** ------------------------------------------------------------------------------------------------------------- */
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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::generateKernel(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb.get());
    // If the module id cannot uniquely identify this kernel, "generateKernelSignature()" will have already
    // generated the unoptimized IR.
    if (!mIsGenerated) {
        const auto m = idb->getModule();
        const auto ip = idb->saveIP();
        // const auto saveInstance = getInstance();
        idb->setModule(mModule);
        addKernelDeclarations(idb);
        callGenerateInitializeMethod(idb);
        callGenerateDoSegmentMethod(idb);
        callGenerateFinalizeMethod(idb);
        // setInstance(saveInstance);
        idb->setModule(m);
        idb->restoreIP(ip);
        mIsGenerated = true;
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    mCurrentMethod = getInitFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    idb->CreateStore(ConstantAggregateZero::get(mKernelStateType), getInstance());
    for (const auto & binding : mScalarInputs) {
        idb->setScalarField(binding.getName(), &*(args++));
    }
    for (const auto & binding : mStreamSetOutputs) {
        idb->setConsumerLock(binding.getName(), &*(args++));
    }
    generateInitializeMethod(idb);
    idb->CreateRetVoid();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    mCurrentMethod = getDoSegmentFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    mIsFinal = &*(args++);
    mAvailablePrincipleItemCount = nullptr;
//    if (mHasPrincipleItemCount) {
//        mAvailablePrincipleItemCount = &*(args++);
//    }
    const auto n = mStreamSetInputs.size();
    mAvailableItemCount.resize(n, nullptr);
    for (unsigned i = 0; i < n; i++) {
//        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
//        Value * itemCount = nullptr;
//        if (rate.isFixed()) {
//            itemCount = mAvailablePrincipleItemCount;
//            if (rate.getRate() != 1) {
//                itemCount = idb->CreateMul(itemCount, ConstantInt::get(itemCount->getType(), rate.getRate()));
//            }
//        } else if (rate.isBounded() || rate.isUnknown()) {
//            itemCount = &*(args++);
//        } else if (rate.isRelative()) {
//            for (unsigned j = 0; j < i; ++j) {
//                if (mStreamSetInputs[j].getName() == rate.getReference()) {
//                    itemCount = mAvailableItemCount[j];
//                    break;
//                }
//            }
//            if (LLVM_UNLIKELY(itemCount == nullptr)) {
//                report_fatal_error(mStreamSetInputs[i].getName() + " is declared before " + rate.getReference());
//            }
//            if (rate.getNumerator() != 1) {
//                itemCount = idb->CreateMul(itemCount, ConstantInt::get(itemCount->getType(), rate.getNumerator()));
//            }
//            if (rate.getDenominator() != 1) {
//                itemCount = idb->CreateUDiv(itemCount, ConstantInt::get(itemCount->getType(), rate.getDenominator()));
//            }
//        }
//        assert (itemCount);
//        mAvailableItemCount[i] = itemCount;

        assert (args != mCurrentMethod->arg_end());
        mAvailableItemCount[i] = &*(args++);
    }
    assert (args == mCurrentMethod->arg_end());

    generateKernelMethod(idb); // must be overridden by the Kernel subtype
    mIsFinal = nullptr;
    mAvailableItemCount.clear();
    idb->CreateRetVoid();
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & idb) {
    mCurrentMethod = getTerminateFunction(idb->getModule());
    idb->SetInsertPoint(BasicBlock::Create(idb->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    generateFinalizeMethod(idb); // may be overridden by the Kernel subtype
    const auto n = mScalarOutputs.size();
    if (n == 0) {
        idb->CreateRetVoid();
    } else {
        Value * outputs[n];
        for (unsigned i = 0; i < n; ++i) {
            outputs[i] = idb->getScalarField(mScalarOutputs[i].getName());
        }
        if (n == 1) {
            idb->CreateRet(outputs[0]);
        } else {
            idb->CreateAggregateRet(outputs, n);
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarIndex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned Kernel::getScalarIndex(const std::string & name) const {
    const auto f = mKernelMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelMap.end())) {
        assert (false);
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return f->second;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::createInstance(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Cannot instantiate " + getName() + " before calling prepareKernel()");
    }
    setInstance(idb->CreateCacheAlignedAlloca(mKernelStateType));
    return getInstance();
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
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
    StructType * const consumerTy = StructType::get(idb->getContext(), {sizeTy, sizePtrPtrTy});
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & idb) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && idb);
    mOutputScalarResult = idb->CreateCall(getTerminateFunction(idb->getModule()), { getInstance() });
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStreamPort
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel::StreamPort Kernel::getStreamPort(const std::string & name) const {
    const auto f = mStreamMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamMap.end())) {
        report_fatal_error(getName() + " does not contain stream set " + name);
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void SegmentOrientedKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    Constant * const log2BlockWidth = b->getSize(std::log2(b->getBitBlockWidth()));

    const auto inputSetCount = mStreamSetInputs.size();
    mStreamSetInputBufferPtr.resize(inputSetCount);
    for (unsigned i = 0; i < inputSetCount; ++i) {
        const auto & name = mStreamSetInputs[i].getName();
        Value * ic = b->getProcessedItemCount(name);
        Value * const blockIndex = b->CreateLShr(ic, log2BlockWidth);
        mStreamSetInputBufferPtr[i] = b->getInputStreamPtr(name, blockIndex);
    }

    const auto outputSetCount = mStreamSetOutputs.size();
    mStreamSetOutputBufferPtr.resize(outputSetCount);
    for (unsigned i = 0; i < outputSetCount; ++i) {
        const auto & name = mStreamSetOutputs[i].getName();
        Value * ic = b->getProducedItemCount(name);
        Value * const blockIndex = b->CreateLShr(ic, log2BlockWidth);
        mStreamSetOutputBufferPtr[i] = b->getOutputStreamPtr(name, blockIndex);
    }

    generateDoSegmentMethod(b);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & kb) {

    const auto inputSetCount = mStreamSetInputs.size();
    const auto outputSetCount = mStreamSetOutputs.size();
    const auto totalSetCount = inputSetCount + outputSetCount;

    // Scan through and see if any of our input streams is marked as the principle

    bool hasPrinciple = false;
    unsigned principleInput = 0;

    for (unsigned i = 0; i < inputSetCount; i++) {
        for (const auto attr : mStreamSetInputs[i].getAttributes()) {
            if (attr.isPrinciple()) {
                hasPrinciple = true;
                principleInput = i;
                break;
            }
        }
    }

    // Now we iteratively process these blocks using the doMultiBlock method.
    // In each iteration, we check how many linearly accessible / writable
    // items can be processed with our current input / output buffers. If we
    // cannot support an full stride, we check whether (a) there is enough
    // input data to process but it is not linearly accessible, in which case
    // we move the data into temporary buffers or (b) there is not enough data
    // to process, in which case we abort unless IsFinal was set.

    // Now proceed with creation of the doSegment method.
    BasicBlock * const doSegmentLoop = kb->CreateBasicBlock("DoSegmentLoop");
    kb->CreateBr(doSegmentLoop);

    /// DO SEGMENT LOOP

    kb->SetInsertPoint(doSegmentLoop);

    // For each input buffer, determine the processedItemCount, the block pointer for the
    // buffer block containing the next item, and the number of linearly available items.

    Value * processedItemCount[inputSetCount];
    Value * baseInputBuffer[inputSetCount];
    Value * unprocessed[inputSetCount];
    Value * linearlyAvailable[inputSetCount];
    Value * readableStrides[inputSetCount];

    Constant * const log2BlockWidth = kb->getSize(std::log2(kb->getBitBlockWidth()));

    Value * numOfStrides = nullptr;

    for (unsigned i = 0; i < inputSetCount; i++) {
        const auto name = mStreamSetInputs[i].getName();
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();

        processedItemCount[i] = kb->getProcessedItemCount(name);

        assert (processedItemCount[i]->getType() == mAvailableItemCount[i]->getType());

        Value * const blockIndex = kb->CreateLShr(processedItemCount[i], log2BlockWidth);
        baseInputBuffer[i] = kb->getInputStreamPtr(name, blockIndex);

        if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
            kb->CreateAssert(kb->CreateICmpUGE(mAvailableItemCount[i], processedItemCount[i]),
                             "Processed item count cannot exceed the available item count");
        }

        unprocessed[i] = kb->CreateSub(mAvailableItemCount[i], processedItemCount[i]);

        //kb->CallPrintInt(getName() + "_" + name + "_unprocessed", unprocessed[i]);

        // INVESTIGATE: If the input rate of this stream is constant and known a priori, we could
        // avoid checking whether it is linearly accessible. Should we have an attribute for this?

        linearlyAvailable[i] = kb->getLinearlyAccessibleItems(name, processedItemCount[i], unprocessed[i]);

        //kb->CallPrintInt(getName() + "_" + name + "_linearlyAvailable", linearlyAvailable[i]);

        readableStrides[i] = nullptr;

        if (rate.isFixed() || rate.isBounded()) {
            Constant * const maxStrideSize = kb->getSize(rate.getUpperBound() * mStride);
            readableStrides[i] = kb->CreateUDiv(linearlyAvailable[i], maxStrideSize);
            if (numOfStrides) {
                numOfStrides = kb->CreateUMin(numOfStrides, readableStrides[i]);
            } else {
                numOfStrides = readableStrides[i];
            }
        }
    }

    //kb->CallPrintInt(getName() + "_numOfStrides", numOfStrides);

    // Now determine the linearly writeable blocks, based on available blocks reduced
    // by limitations of output buffer space.

    Value * producedItemCount[outputSetCount];
    Value * baseOutputBuffer[outputSetCount];
    Value * writableStrides[outputSetCount];
    Value * linearlyWritable[outputSetCount];

    for (unsigned i = 0; i < outputSetCount; i++) {
        const auto & name = mStreamSetOutputs[i].getName();
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        producedItemCount[i] = kb->getProducedItemCount(name);

        //kb->CallPrintInt(getName() + "_" + name + "_producedItemCount", producedItemCount[i]);

        Value * const blockIndex = kb->CreateLShr(producedItemCount[i], log2BlockWidth);
        baseOutputBuffer[i] = kb->getOutputStreamPtr(name, blockIndex);
        linearlyWritable[i] = nullptr;
        writableStrides[i] = nullptr;
        if (rate.isFixed() || rate.isBounded()) {
            linearlyWritable[i] = kb->getLinearlyWritableItems(name, producedItemCount[i]);

            //kb->CallPrintInt(getName() + "_" + name + "_linearlyWritable", linearlyWritable[i]);

            Constant * const maxStrideSize = kb->getSize(rate.getUpperBound() * mStride);
            writableStrides[i] = kb->CreateUDiv(linearlyWritable[i], maxStrideSize);
            if (numOfStrides) {
                numOfStrides = kb->CreateUMin(numOfStrides, writableStrides[i]);
            } else {
                numOfStrides = writableStrides[i];
            }
        }
    }

    //kb->CallPrintInt(getName() + "_numOfStrides'", numOfStrides);

    for (unsigned i = 0; i < inputSetCount; i++) {
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (rate.isFixed()) {
            mAvailableItemCount[i] = kb->CreateMul(numOfStrides, kb->getSize(rate.getRate() * mStride));
        } else {
            mAvailableItemCount[i] = linearlyAvailable[i];
        }

        //kb->CallPrintInt(getName() + "_" + mStreamSetInputs[i].getName() + "_avail", mAvailableItemCount[i]);
    }

    // Define and allocate the temporary buffer area.
    Type * tempBuffers[totalSetCount];
    for (unsigned i = 0; i < inputSetCount; ++i) {
        Type * bufType = baseInputBuffer[i]->getType()->getPointerElementType();
        assert (baseInputBuffer[i]->getType()->getPointerAddressSpace() == 0);
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        unsigned count = 0;
        if (rate.isFixed()) {
            count = rate.getRate();
        } else if (rate.isBounded()) {
            count = rate.getUpperBound() + 2;
        }
        tempBuffers[i] = ArrayType::get(bufType, count);
    }
    for (unsigned i = 0; i < outputSetCount; i++) {
        Type * const bufType = baseOutputBuffer[i]->getType()->getPointerElementType();
        assert (baseOutputBuffer[i]->getType()->getPointerAddressSpace() == 0);
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        unsigned count = 0;
        if (rate.isFixed()) {
            count = rate.getRate();
        } else if (rate.isBounded()) {
            count = rate.getUpperBound() + 2;
        }
        tempBuffers[i + inputSetCount] = ArrayType::get(bufType, count);
    }

    Type * const tempParameterStructType = StructType::create(kb->getContext(), ArrayRef<Type *>(tempBuffers, totalSetCount));

    Value * const tempBufferArea = kb->CreateCacheAlignedAlloca(tempParameterStructType);

    BasicBlock * const temporaryBufferCheck = kb->CreateBasicBlock("temporaryBufferCheck");
    BasicBlock * const doMultiBlock = kb->CreateBasicBlock("doMultiBlock");
    BasicBlock * const copyToTemporaryBuffers = kb->CreateBasicBlock("copyToTemporaryBuffers");
    BasicBlock * const segmentDone = kb->CreateBasicBlock("segmentDone");

    Value * const hasFullStride = numOfStrides ? kb->CreateICmpNE(numOfStrides, kb->getSize(0)) : kb->getTrue();
    kb->CreateCondBr(hasFullStride, doMultiBlock, temporaryBufferCheck);

    // We use temporary buffers in 3 different cases that preclude full stride processing.

    //  (a) One or more input buffers does not have a sufficient number of input items linearly available.
    //  (b) One or more output buffers does not have sufficient linearly available buffer space.
    //  (c) We have processed all the full strides of input and only the final block remains.

    kb->SetInsertPoint(temporaryBufferCheck);

    // Even if we copy the input data into a linear arrays, is there enough data to perform this stride?
    // If not, proceed only if this is our final block.
    Value * hasFullFragmentedStride = nullptr;
    for (unsigned i = 0; i < inputSetCount; i++) {
        const ProcessingRate & r = mStreamSetInputs[i].getRate();
        if (r.isBounded() || (r.isUnknown() && r.getLowerBound() > 0)) {
            const auto l = r.isBounded() ? r.getUpperBound() : r.getLowerBound();
            Constant * const strideSize = kb->getSize(l * mStride);
            Value * enoughAvail = kb->CreateICmpUGE(unprocessed[i], strideSize);
            if (hasFullFragmentedStride) {
                hasFullFragmentedStride = kb->CreateAnd(hasFullFragmentedStride, enoughAvail);
            } else {
                hasFullFragmentedStride = enoughAvail;
            }
        }
    }

    Value * hasFragmentedOrFinalStride = nullptr;
    if (hasFullFragmentedStride) {
        hasFragmentedOrFinalStride = kb->CreateOr(hasFullFragmentedStride, mIsFinal);
        // Although this might be the final segment, we may have a full fragmented stride to process prior
        // to the actual final stride.
        mIsFinal = kb->CreateAnd(mIsFinal, kb->CreateNot(hasFullFragmentedStride));
    } else {
        hasFragmentedOrFinalStride = mIsFinal;
    }
    kb->CreateCondBr(hasFragmentedOrFinalStride, copyToTemporaryBuffers, segmentDone);

    /// COPY TO TEMPORARY BUFFERS
    kb->SetInsertPoint(copyToTemporaryBuffers);

    kb->CreateAlignedStore(Constant::getNullValue(tempParameterStructType), tempBufferArea, kb->getCacheAlignment());

    // For each input and output buffer, copy over necessary data starting from the last block boundary.

    Value * temporaryInputBuffer[inputSetCount];
    Value * temporaryAvailable[inputSetCount];

    for (unsigned i = 0; i < inputSetCount; i++) {
        temporaryInputBuffer[i] = baseInputBuffer[i];
        if (readableStrides[i]) {
            const auto name = mStreamSetInputs[i].getName();
            const ProcessingRate & rate = mStreamSetInputs[i].getRate();
            assert (rate.getUpperBound() > 0);
            Constant * const maxStrideSize = kb->getSize(rate.getUpperBound() * mStride);
            temporaryAvailable[i] = kb->CreateUMin(unprocessed[i], maxStrideSize);

            BasicBlock * entry = kb->GetInsertBlock();
            BasicBlock * copy = kb->CreateBasicBlock(name + "Copy");
            BasicBlock * resume = kb->CreateBasicBlock(name + "ResumeCopy");
            Value * const test = kb->CreateOr(kb->CreateICmpNE(readableStrides[i], kb->getSize(0)), mIsFinal);
            kb->CreateCondBr(test, resume, copy);

            kb->SetInsertPoint(copy);
            Value * const tempBufferPtr = kb->CreateGEP(tempBufferArea, {kb->getInt32(0), kb->getInt32(i), kb->getInt32(0)});
            assert (tempBufferPtr->getType() == baseInputBuffer[i]->getType());
            Value * const neededItems = linearlyAvailable[i];
            Value * const bytesCopied = kb->copy(name, tempBufferPtr, baseInputBuffer[i], neededItems);
            Value * const nextInputPtr = kb->getRawInputPointer(name, kb->getSize(0));
            Value * const remaining = kb->CreateSub(temporaryAvailable[i], neededItems);
            Value * nextBufPtr = kb->CreatePointerCast(tempBufferPtr, kb->getInt8PtrTy());
            nextBufPtr = kb->CreateGEP(nextBufPtr, bytesCopied);
            kb->copy(name, nextBufPtr, nextInputPtr, remaining);

            kb->CreateBr(resume);

            kb->SetInsertPoint(resume);
            PHINode * bufferPtr = kb->CreatePHI(baseInputBuffer[i]->getType(), 2);
            bufferPtr->addIncoming(baseInputBuffer[i], entry);
            bufferPtr->addIncoming(tempBufferPtr, copy);
            temporaryInputBuffer[i] = bufferPtr;
        }
    }

    Value * temporaryOutputBuffer[outputSetCount];
    for (unsigned i = 0; i < outputSetCount; i++) {
        temporaryOutputBuffer[i] = baseOutputBuffer[i];
        if (writableStrides[i]) {
            const auto name = mStreamSetOutputs[i].getName();

            BasicBlock * const entry = kb->GetInsertBlock();
            BasicBlock * const copy = kb->CreateBasicBlock(name + "Copy");
            BasicBlock * const resume = kb->CreateBasicBlock(name + "ResumeCopy");

            Value * const test = kb->CreateOr(kb->CreateICmpNE(writableStrides[i], kb->getSize(0)), mIsFinal);
            kb->CreateCondBr(test, resume, copy);

            kb->SetInsertPoint(copy);
            Value * const tempBufferPtr = kb->CreateGEP(tempBufferArea,  {kb->getInt32(0), kb->getInt32(inputSetCount + i), kb->getInt32(0)});
            assert (tempBufferPtr->getType() == baseOutputBuffer[i]->getType());
            Value * const itemsToCopy = kb->CreateAnd(producedItemCount[i], kb->getSize(kb->getBitBlockWidth() - 1));
            kb->copy(name, tempBufferPtr, baseOutputBuffer[i], itemsToCopy);
            kb->CreateBr(resume);

            kb->SetInsertPoint(resume);
            PHINode * bufferPtr = kb->CreatePHI(tempBufferPtr->getType(), 2);
            bufferPtr->addIncoming(baseOutputBuffer[i], entry);
            bufferPtr->addIncoming(tempBufferPtr, copy);
            temporaryOutputBuffer[i] = bufferPtr;
        }
    }

    kb->CreateBr(doMultiBlock);
    BasicBlock * const usingTemporaryBuffers = kb->GetInsertBlock();
    doMultiBlock->moveAfter(usingTemporaryBuffers);

    /// DO MULTI BLOCK

    //  At this point we have verified the availability of one or more blocks of input data and output buffer space for all stream sets.
    //  Now prepare the doMultiBlock call.
    kb->SetInsertPoint(doMultiBlock);

    PHINode * const isFinal = kb->CreatePHI(mIsFinal->getType(), 2);
    isFinal->addIncoming(kb->getFalse(), doSegmentLoop);
    isFinal->addIncoming(mIsFinal, usingTemporaryBuffers);
    mIsFinal = isFinal;

    mStreamSetInputBufferPtr.resize(inputSetCount);
    for (unsigned i = 0; i < inputSetCount; ++i) {
        assert (baseInputBuffer[i] && temporaryInputBuffer[i]);
        if (baseInputBuffer[i] != temporaryInputBuffer[i]) {
            PHINode * const avail = kb->CreatePHI(kb->getSizeTy(), 2);
            avail->addIncoming(mAvailableItemCount[i], doSegmentLoop);
            avail->addIncoming(temporaryAvailable[i], usingTemporaryBuffers);
            mAvailableItemCount[i] = avail;
            PHINode * const bufferPtr = kb->CreatePHI(baseInputBuffer[i]->getType(), 2);
            bufferPtr->addIncoming(baseInputBuffer[i], doSegmentLoop);
            assert (baseInputBuffer[i]->getType() == temporaryInputBuffer[i]->getType());
            bufferPtr->addIncoming(temporaryInputBuffer[i], usingTemporaryBuffers);
            temporaryInputBuffer[i] = bufferPtr;
        }
        mStreamSetInputBufferPtr[i] = temporaryInputBuffer[i];
    }

    mStreamSetOutputBufferPtr.resize(outputSetCount);
    for (unsigned i = 0; i < outputSetCount; ++i) {
        assert (baseOutputBuffer[i] && temporaryOutputBuffer[i]);
        if (baseOutputBuffer[i] != temporaryOutputBuffer[i]) {
            PHINode * const bufferPtr = kb->CreatePHI(baseOutputBuffer[i]->getType(), 2);
            bufferPtr->addIncoming(baseOutputBuffer[i], doSegmentLoop);
            assert (baseOutputBuffer[i]->getType() == temporaryOutputBuffer[i]->getType());
            bufferPtr->addIncoming(temporaryOutputBuffer[i], usingTemporaryBuffers);
            temporaryOutputBuffer[i] = bufferPtr;
        }
        mStreamSetOutputBufferPtr[i] = temporaryOutputBuffer[i];
    }

    // Now use the generateMultiBlockLogic method of the MultiBlockKernelBuilder subtype to
    // provide the required multi-block kernel logic.
    generateMultiBlockLogic(kb, numOfStrides);

    // If we have no fixed rate inputs, we won't know when we're done parsing until we test
    // whether any input data was processed.
    bool mayMakeNoProgress = true;

    // Update the processed item count of any Fixed input or output stream. While doing so, also
    // calculate the LCM of their rates. The LCM is used to calculate the final item counts.

    unsigned rateLCM = 1;

    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (rate.isFixed()) {
            mayMakeNoProgress = false;
            rateLCM = lcm(rateLCM, rate.getRate());
            Value * const processed = mAvailableItemCount[i]; // kb->CreateMul(numOfStrides, kb->getSize(mStride * rate.getRate()));
            Value * const ic = kb->CreateAdd(processedItemCount[i], processed);
            kb->setProcessedItemCount(mStreamSetInputs[i].getName(), ic);
        }
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        if (rate.isFixed()) {
            rateLCM = lcm(rateLCM, rate.getRate());
            Value * const produced = kb->CreateMul(numOfStrides, kb->getSize(mStride * rate.getRate()));
            Value * const ic = kb->CreateAdd(producedItemCount[i], produced);
            kb->setProducedItemCount(mStreamSetOutputs[i].getName(), ic);
        }
    }

    BasicBlock * const finalStrideCheck = kb->CreateBasicBlock("finalStrideCheck");
    BasicBlock * const finalStrideAdjustment = kb->CreateBasicBlock("finalStrideAdjustment");
    BasicBlock * const standardCopyBack = kb->CreateBasicBlock("standardCopyBack");
    BasicBlock * const temporaryBufferCopyBack = kb->CreateBasicBlock("temporaryBufferCopyBack");

    kb->CreateLikelyCondBr(hasFullStride, standardCopyBack, finalStrideCheck);


    /// FINAL STRIDE CHECK
    kb->SetInsertPoint(finalStrideCheck);
    kb->CreateUnlikelyCondBr(mIsFinal, finalStrideAdjustment, temporaryBufferCopyBack);

    /// FINAL STRIDE ADJUSTMENT
    kb->SetInsertPoint(finalStrideAdjustment);

    // If this is our final stride, adjust the Fixed output item counts. The main loop assumes that
    // the ITEM COUNT % FIXED RATE = 0 for all Fixed Input and Output streams. We correct that here
    // to calculate them based on the actual input item counts.

    // NOTE: This appears overly complex to avoid an integer overflow without reducing the maximum
    // integer size. For each Fixed output stream, this calculates:

    //       CEILING(MIN(Total Available Item Count / Fixed Input Rate) * Fixed Output Rate)

    Value * basePreviouslyProcessedItemCount = nullptr;
    Value * scaledInverseOfStrideItemCount = nullptr;

    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & r = mStreamSetInputs[i].getRate();
        if (r.isFixed()) {
            assert (rateLCM % r.getRate() == 0);
            Value * const a = kb->CreateMul(mAvailableItemCount[i], kb->getSize(rateLCM / r.getRate())); // unprocessed
            Value * const p = kb->CreateUDiv(processedItemCount[i], kb->getSize(r.getRate()));
            if (scaledInverseOfStrideItemCount) {
                scaledInverseOfStrideItemCount = kb->CreateUMin(scaledInverseOfStrideItemCount, a);
                basePreviouslyProcessedItemCount = kb->CreateUMin(basePreviouslyProcessedItemCount, p);
            } else {
                scaledInverseOfStrideItemCount = a;
                basePreviouslyProcessedItemCount = p;
            }
        }
//        const auto name = mStreamSetInputs[i].getName();
//        Value * const processed = kb->CreateAdd(processedItemCount[i], unprocessed[i]);
//        kb->setProcessedItemCount(name, processed);
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        const auto name = mStreamSetOutputs[i].getName();
        const ProcessingRate & r = mStreamSetOutputs[i].getRate();
        Value * produced = nullptr;
        if (r.isFixed()) {
            assert (rateLCM % r.getRate() == 0);
            assert (basePreviouslyProcessedItemCount && scaledInverseOfStrideItemCount);
            Value * const p = kb->CreateMul(basePreviouslyProcessedItemCount, kb->getSize(r.getRate()));
            Value * const ic = kb->CreateUDivCeil(scaledInverseOfStrideItemCount, kb->getSize(rateLCM / r.getRate()));
            produced = kb->CreateAdd(p, ic);
        } else { // check if we have an attribute; if so, get the current produced count and adjust it
            bool noAttributes = true;
            for (const Attribute & attr : mStreamSetOutputs[i].getAttributes()) {
                if (attr.isAdd() || attr.isRoundUpTo()) {
                    noAttributes = false;
                    break;
                }
            }
            if (noAttributes) {
                continue;
            }
            produced = kb->getProducedItemCount(name);
        }
        for (const Attribute & attr : mStreamSetOutputs[i].getAttributes()) {
            if (attr.isAdd()) {
                produced = kb->CreateAdd(produced, kb->getSize(attr.getAmount()));
            } else if (attr.isRoundUpTo()) {
                produced = kb->CreateRoundUp(produced, kb->getSize(attr.getAmount()));
            }
        }
        kb->setProducedItemCount(name, produced);
    }

    kb->CreateBr(temporaryBufferCopyBack);

    /// TEMPORARY BUFFER COPY BACK
    kb->SetInsertPoint(temporaryBufferCopyBack);

    // Copy back data to the actual output buffers.
    for (unsigned i = 0; i < outputSetCount; i++) {

        if (baseOutputBuffer[i] != temporaryOutputBuffer[i]) {

            const auto name = mStreamSetOutputs[i].getName();

            BasicBlock * const copy = kb->CreateBasicBlock(name + "CopyBack");
            BasicBlock * const resume = kb->CreateBasicBlock(name + "ResumeCopyBack");
            Value * const usedTemporary = kb->CreateICmpNE(temporaryOutputBuffer[i], baseOutputBuffer[i]);

            // If we used a temporary buffer ...
            kb->CreateCondBr(usedTemporary, copy, resume);

            kb->SetInsertPoint(copy);
            Value * bytesCopied = kb->copy(name, baseOutputBuffer[i], temporaryOutputBuffer[i], linearlyWritable[i]);
            Value * nextOutputPtr = kb->getRawOutputPointer(name, kb->getSize(0));
            Value * producedCount = kb->getProducedItemCount(name);

            Value * remaining = kb->CreateSub(producedCount, linearlyWritable[i]);
            Value * nextBufPtr = kb->CreatePointerCast(temporaryOutputBuffer[i], kb->getInt8PtrTy());
            nextBufPtr = kb->CreateGEP(nextBufPtr, bytesCopied);

            kb->copy(name, nextOutputPtr, nextBufPtr, remaining);
            kb->CreateBr(resume);

            kb->SetInsertPoint(resume);
        }
    }

    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    BasicBlock * setTermination = nullptr;
    if (hasNoTerminateAttribute()) {
        kb->CreateCondBr(mIsFinal, segmentDone, standardCopyBack);
    } else {
        setTermination = kb->CreateBasicBlock("setTermination");
        kb->CreateCondBr(mIsFinal, setTermination, standardCopyBack);
    }

    /// STANDARD COPY BACK
    kb->SetInsertPoint(standardCopyBack);

    // Do copybacks if necessary.
    for (unsigned i = 0; i < outputSetCount; i++) {
        if (mStreamSetOutputBuffers[i]->supportsCopyBack()) {
            const auto name = mStreamSetOutputs[i].getName();
            Value * newProduced = kb->getProducedItemCount(name);
            kb->CreateCopyBack(name, producedItemCount[i], newProduced);
        }
    }

    // If it is possible to make no progress, verify we processed some of the input. If we haven't,
    // we're finished this segment.
    if (mayMakeNoProgress) {
        Value * madeProgress = nullptr;
        for (unsigned i = 0; i < inputSetCount; ++i) {
            Value * const processed = kb->getProcessedItemCount(mStreamSetInputs[i].getName());
            Value * const progress = kb->CreateICmpNE(processed, processedItemCount[i]);
            if (madeProgress) {
                madeProgress = kb->CreateOr(madeProgress, progress);
            } else {
                madeProgress = progress;
            }
        }
        assert (madeProgress);
        kb->CreateCondBr(madeProgress, doSegmentLoop, segmentDone);
    } else {
        kb->CreateBr(doSegmentLoop);
    }

    if (hasNoTerminateAttribute()) {
        segmentDone->moveAfter(kb->GetInsertBlock());
    } else {
        /// SET TERMINATION
        setTermination->moveAfter(kb->GetInsertBlock());
        kb->SetInsertPoint(setTermination);
        kb->setTerminationSignal();
        kb->CreateBr(segmentDone);
        segmentDone->moveAfter(setTermination);
    }

    kb->SetInsertPoint(segmentDone);

}

//bool MultiBlockKernel::requiresCopyBack(const ProcessingRate & rate) const {
//    if (rate.isBounded() || rate.isUnknown()) {
//        return true;
//    } else if (rate.isDirectlyRelative()) {
//        Port port; unsigned i;
//        std::tie(port, i) = getStreamPort(rate.getReference());
//        const auto & binding = (port == Port::Input) ? mStreamSetInputs[i] : mStreamSetOutputs[i];
//        return requiresCopyBack(binding.getRate());
//    }
//    return false;
//}

//  The default doSegment method dispatches to the doBlock routine for
//  each block of the given number of blocksToDo, and then updates counts.

void BlockOrientedKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & idb, llvm::Value * const numOfStrides) {

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

    Constant * const log2BlockSize = idb->getSize(std::log2(idb->getBitBlockWidth()));

    const auto inputSetCount = mStreamSetInputs.size();
    Value * baseProcessedIndex[inputSetCount];
    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (rate.isFixed()) {
            baseProcessedIndex[i] = nullptr;
        } else {
            Value * ic = idb->getProcessedItemCount(mStreamSetInputs[i].getName());
            ic = idb->CreateLShr(ic, log2BlockSize);
            baseProcessedIndex[i] = ic;
        }
    }

    const auto outputSetCount = mStreamSetOutputs.size();
    Value * baseProducedIndex[outputSetCount];
    for (unsigned i = 0; i < outputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        if (rate.isFixed()) {
            baseProducedIndex[i] = nullptr;
        } else {
            Value * ic = idb->getProducedItemCount(mStreamSetOutputs[i].getName());
            ic = idb->CreateLShr(ic, log2BlockSize);
            baseProducedIndex[i] = ic;
        }
    }

    Value * const numOfBlocksToProcess = idb->CreateMul(numOfStrides, idb->getSize(mStride / idb->getBitBlockWidth()));

    idb->CreateBr(strideLoopCond);

    /// BLOCK COND

    idb->SetInsertPoint(strideLoopCond);

    PHINode * branchTarget = nullptr;
    if (baseTarget) {
        branchTarget = idb->CreatePHI(baseTarget->getType(), 2, "branchTarget");
        branchTarget->addIncoming(baseTarget, entryBlock);
    }

    PHINode * const blockIndex = idb->CreatePHI(idb->getSizeTy(), 2, "index");
    blockIndex->addIncoming(idb->getSize(0), entryBlock);

    for (unsigned i = 0; i < inputSetCount; ++i) {
        Value * offset = blockIndex;
        if (baseProcessedIndex[i]) {
            offset = idb->getProcessedItemCount(mStreamSetInputs[i].getName());
            offset = idb->CreateLShr(offset, log2BlockSize);
            offset = idb->CreateSub(offset, baseProcessedIndex[i]);
        }
        mStreamSetInputBufferPtr[i] = idb->CreateGEP(mStreamSetInputBufferPtr[i], offset);
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        Value * offset = blockIndex;
        if (baseProducedIndex[i]) {
            offset = idb->getProducedItemCount(mStreamSetOutputs[i].getName());
            offset = idb->CreateLShr(offset, log2BlockSize);
            offset = idb->CreateSub(offset, baseProducedIndex[i]);
        }
        mStreamSetOutputBufferPtr[i] = idb->CreateGEP(mStreamSetOutputBufferPtr[i], offset);
    }

    Value * const notDone = idb->CreateICmpULT(blockIndex, numOfBlocksToProcess);
    idb->CreateLikelyCondBr(notDone, mStrideLoopBody, stridesDone);

    /// BLOCK BODY

    idb->SetInsertPoint(mStrideLoopBody);

    if (idb->supportsIndirectBr()) {
        mStrideLoopTarget = idb->CreatePHI(baseTarget->getType(), 2, "strideTarget");
        mStrideLoopTarget->addIncoming(branchTarget, strideLoopCond);
    }

    /// GENERATE DO BLOCK METHOD

    writeDoBlockMethod(idb);

    BasicBlock * const bodyEnd = idb->GetInsertBlock();
    blockIndex->addIncoming(idb->CreateAdd(blockIndex, idb->getSize(1)), bodyEnd);
    if (branchTarget) {
        branchTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }
    idb->CreateBr(strideLoopCond);

    stridesDone->moveAfter(bodyEnd);

    /// STRIDE DONE

    idb->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    if (branchTarget) {
        mStrideLoopBranch = idb->CreateIndirectBr(branchTarget, 3);
        mStrideLoopBranch->addDestination(doFinalBlock);
        mStrideLoopBranch->addDestination(segmentDone);
    } else {
        idb->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, segmentDone);
    }

    doFinalBlock->moveAfter(stridesDone);

    idb->SetInsertPoint(doFinalBlock);

    Value * remainingItems = nullptr;
    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & r = mStreamSetInputs[i].getRate();
        if (r.isFixed()) {
            Value * ic = idb->CreateUDiv(mAvailableItemCount[i], idb->getSize(r.getRate()));
            if (remainingItems) {
                remainingItems = idb->CreateUMax(remainingItems, ic);
            } else {
                remainingItems = ic;
            }
        }
    }

    writeFinalBlockMethod(idb, remainingItems);

    idb->CreateBr(segmentDone);

    segmentDone->moveAfter(idb->GetInsertBlock());

    idb->SetInsertPoint(segmentDone);

    // Update the branch prediction metadata to indicate that the likely target will be segmentDone
    if (branchTarget) {
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

    generateDoBlockMethod(idb); // must be implemented by the BlockOrientedKernelBuilder subtype

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

static inline std::string annotateKernelNameWithDebugFlags(std::string && name) {
    if (codegen::DebugOptionIsSet(codegen::EnableAsserts)) {
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
, mAvailablePrincipleItemCount(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mStride(0)
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
: MultiBlockKernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr) {

}

// MULTI-BLOCK KERNEL CONSTRUCTOR
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
