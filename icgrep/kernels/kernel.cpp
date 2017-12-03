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
#include <boost/math/common_factor.hpp>
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
    if (LLVM_UNLIKELY(mKernelFieldMap.count(name))) {
        report_fatal_error(getName() + " already contains scalar field " + name);
    }
    const auto index = mKernelFields.size();
    mKernelFieldMap.emplace(name, index);
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
    Module * m = new Module(getCacheName(idb), idb->getContext());
    m->setTargetTriple(idb->getModule()->getTargetTriple());
    m->setDataLayout(idb->getModule()->getDataLayout());
    return setModule(m);
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
        makeModule(idb);
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
 * @brief addBaseKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & idb) {

    if (mStreamMap.empty()) {
        prepareStreamSetNameMap();
    }

    normalizeStreamProcessingRates();

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
    mAvailablePrincipalItemCount = nullptr;
    const auto n = mStreamSetInputs.size();
    mAvailableItemCount.resize(n, nullptr);
    for (unsigned i = 0; i < n; i++) {
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
    const auto f = mKernelFieldMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelFieldMap.end())) {
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
 * @brief getStreamPort
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & Kernel::getBinding(const std::string & name) const {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    return (port == Port::Input) ? getStreamInput(index) : getStreamOutput(index);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalizeRelativeToFixedProcessingRate
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::normalizeRelativeToFixedProcessingRate(const ProcessingRate & base, ProcessingRate & toUpdate) {
    if (base.isFixed()) {
        return true;
    } else if (LLVM_UNLIKELY(base.isRelative())) {
        const auto & ref = getBinding(base.getReference()).getRate();
        if (normalizeRelativeToFixedProcessingRate(ref, toUpdate)) {
            toUpdate.getRate() *= ref.getRate();
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief normalizeStreamProcessingRates
 *
 * If we allow a stream to be transitively relative to a fixed rate stream, it complicates detection of fixed
 * rate streams later. Find any such occurance and transform them. This implies, however, that a fixed rate
 * stream could have a rational processing rate (which should not occur normally.)
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::normalizeStreamProcessingRates() {
    for (Binding & input : mStreamSetInputs) {
        normalizeRelativeToFixedProcessingRate(input.getRate(), input.getRate());
    }
    for (Binding & output : mStreamSetOutputs) {
        normalizeRelativeToFixedProcessingRate(output.getRate(), output.getRate());
    }
    // TODO: we want to consume whole units. Once the pipeline is able to schedule kernels based on their stride
    // and input/output rates, modify them here.
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void SegmentOrientedKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    const auto inputSetCount = mStreamSetInputs.size();
    mStreamSetInputBaseAddress.resize(inputSetCount);
    for (unsigned i = 0; i < inputSetCount; ++i) {
        mStreamSetInputBaseAddress[i] = nullptr;
    }
    const auto outputSetCount = mStreamSetOutputs.size();
    mStreamSetOutputBaseAddress.resize(outputSetCount);
    for (unsigned i = 0; i < outputSetCount; ++i) {
        mStreamSetOutputBaseAddress[i] = nullptr;
    }
    generateDoSegmentMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresBufferedFinalStride
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool requiresBufferedFinalStride(const Binding & b) {
    if (LLVM_LIKELY(isa<ArrayType>(b.getType()))) {
        return b.getType()->getArrayNumElements() == 1;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemWidth
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned getItemWidth(const Binding & b) {
    Type * ty = b.getType();
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLowerBound
 ** ------------------------------------------------------------------------------------------------------------- */
ProcessingRate::RateValue MultiBlockKernel::getLowerBound(const ProcessingRate & rate) const {
    if (rate.isFixed() || rate.isBounded()) {
        return rate.getLowerBound();
    } else if (rate.isRelative()) {
        return rate.getRate() * getLowerBound(getBinding(rate.getReference()).getRate());
    } else { // if (rate.isUnknown())
        return 0;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getUpperBound
 ** ------------------------------------------------------------------------------------------------------------- */
ProcessingRate::RateValue MultiBlockKernel::getUpperBound(const ProcessingRate &rate) const {
    if (rate.isFixed() || rate.isBounded()) {
        return rate.getUpperBound();
    } else if (rate.isRelative()) {
        return rate.getRate() * getUpperBound(getBinding(rate.getReference()).getRate());
    } else { // if (rate.isUnknown())
        return 0;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getUpperBound
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiBlockKernel::isTransitivelyUnknownRate(const ProcessingRate & rate) const {
    if (rate.isUnknown()) {
        return true;
    } else if (rate.isDerived()) {
        return isTransitivelyUnknownRate(getBinding(rate.getReference()).getRate());
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief roundUp
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned roundUp(const ProcessingRate::RateValue & r) {
    if (LLVM_LIKELY(r.denominator() == 1)) {
        return r.numerator();
    } else {
        return (r.numerator() + r.denominator() - 1) / r.denominator();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemAlignment
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned MultiBlockKernel::getItemAlignment(const Binding & binding) const {
    const auto & rate = binding.getRate();
    if (rate.isFixed()) {
        const auto & r = rate.getRate();
        const auto n = (r.numerator() * mStride);
        if (LLVM_LIKELY(r.denominator() == 1)) {
            return n;
        } else if (LLVM_LIKELY((n % r.denominator()) == 0)) {
            return n / r.denominator();
        }
    }
    return 1; // ∀x GCD(x, x + 1) = 1
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStrideSize
 ** ------------------------------------------------------------------------------------------------------------- */
llvm::Value * MultiBlockKernel::getStrideSize(const std::unique_ptr<KernelBuilder> & b, const ProcessingRate & rate) {
    // NOTE: if we ever support feedback loops, using upper bound could lead to a deadlock due to data starvation
    const auto r = getUpperBound(rate);
    if (r.numerator() == 0) {
        return nullptr;
    } else {
        assert ((r.numerator() * mStride) % r.denominator() == 0);
        return b->getSize((r.numerator() * mStride) / r.denominator());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    if (LLVM_UNLIKELY((mStride % b->getBitBlockWidth()) != 0)) {
        report_fatal_error(getName() + ": the Stride (" + std::to_string(mStride) + ") of MultiBlockKernel "
                           "must be a multiple of the BitBlockWidth (" + std::to_string(b->getBitBlockWidth()) + ")");
    }

    const auto inputSetCount = mStreamSetInputs.size();
    const auto outputSetCount = mStreamSetOutputs.size();

    // Define and allocate the temporary buffer area in the prolog.
    const auto alignment = b->getBitBlockWidth() / 8;
    Value * temporaryInputBuffer[inputSetCount];
    for (unsigned i = 0; i < inputSetCount; ++i) {

        // TODO: if this is a fixed rate input stream and the pipeline guarantees it will not call the kernel unless
        // there is sufficient input and all buffers will be sized sufficiently for the input, we ought to be able to
        // avoid the temporary buffer checks.

        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        Type * const ty = mStreamSetInputBuffers[i]->getStreamSetBlockType();
        const auto ub = getUpperBound(rate);
        if (ub.numerator() == 0) {
            report_fatal_error("MultiBlock kernels do not support unknown rate input streams or streams relative to an unknown rate input.");
        } else {            
            temporaryInputBuffer[i] = b->CreateAlignedAlloca(ty, alignment, b->getSize(roundUp(ub)));
            Type * const sty = temporaryInputBuffer[i]->getType()->getPointerElementType();
            b->CreateStore(Constant::getNullValue(sty), temporaryInputBuffer[i]);
        }        
    }

    Value * temporaryOutputBuffer[outputSetCount];
    for (unsigned i = 0; i < outputSetCount; i++) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        Type * const ty = mStreamSetOutputBuffers[i]->getStreamSetBlockType();
        if (LLVM_UNLIKELY(isTransitivelyUnknownRate(rate))) {
            temporaryOutputBuffer[i] = nullptr;
        } else {            
            auto ub = getUpperBound(rate);
            if (LLVM_UNLIKELY(mStreamSetOutputBuffers[i]->supportsCopyBack() && requiresCopyBack(rate))) {
                ub += mStreamSetOutputBuffers[i]->overflowSize();
            }
            temporaryOutputBuffer[i] = b->CreateAlignedAlloca(ty, alignment, b->getSize(roundUp(ub)));
            Type * const sty = temporaryOutputBuffer[i]->getType()->getPointerElementType();
            b->CreateStore(Constant::getNullValue(sty), temporaryOutputBuffer[i]);
        }
    }

    // Now we iteratively process these blocks using the doMultiBlock method.
    // In each iteration, we check how many linearly accessible / writable
    // items can be processed with our current input / output buffers. If we
    // cannot support an full stride, we check whether (a) there is enough
    // input data to process but it is not linearly accessible, in which case
    // we move the data into temporary buffers or (b) there is not enough data
    // to process, in which case we abort unless IsFinal was set.

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(std::log2(b->getBitBlockWidth()));
    Constant * const BLOCK_WIDTH_MASK = b->getSize(b->getBitBlockWidth() - 1);

    // Now proceed with creation of the doSegment method.
    BasicBlock * const segmentLoop = b->CreateBasicBlock("SegmentLoop");

    b->CreateBr(segmentLoop);

    /// DO SEGMENT LOOP

    b->SetInsertPoint(segmentLoop);

    // For each input buffer, get the initial processed item count, base input pointer, and the number of
    // linearly available strides.
    Value * numOfStrides = nullptr;
    mInitialAvailableItemCount.resize(inputSetCount);
    mInitialProcessedItemCount.resize(inputSetCount);
    mStreamSetInputBaseAddress.resize(inputSetCount);
    Value * inputStrideSize[inputSetCount];
    for (unsigned i = 0; i < inputSetCount; i++) {
        const auto & input = mStreamSetInputs[i];
        const auto & name = input.getName();
        const ProcessingRate & rate = input.getRate();
        Value * const ic = b->getProcessedItemCount(name);
        mInitialProcessedItemCount[i] = ic;
        b->CreateAssert(b->CreateICmpUGE(mAvailableItemCount[i], ic), "processed item count cannot exceed the available item count");
        assert (ic->getType() == mAvailableItemCount[i]->getType());
        Value * const unprocessed = b->CreateSub(mAvailableItemCount[i], ic);
        mStreamSetInputBaseAddress[i]  = b->getBlockAddress(name, b->CreateLShr(ic, LOG_2_BLOCK_WIDTH));
        mInitialAvailableItemCount[i] = mAvailableItemCount[i];
        mAvailableItemCount[i] = b->getLinearlyAccessibleItems(name, ic, unprocessed);
        // Are our linearly accessible items sufficient for a stride?
        inputStrideSize[i] = getStrideSize(b, rate);
        Value * accessibleStrides = b->CreateUDiv(mAvailableItemCount[i], inputStrideSize[i]);
        if (!rate.isFixed() || requiresBufferedFinalStride(input)) {

            // Since we trust that the pipeline won't call this kernel unless there is enough data to process a stride, whenever
            // we discover that there isn't enough linearly available data, optimistically copy the data to the temporary buffer.

            BasicBlock * const entry = b->GetInsertBlock();
            BasicBlock * const copyFromBack = b->CreateBasicBlock(name + "CopyFromBack");
            BasicBlock * const copyFromFront = b->CreateBasicBlock(name + "CopyFromFront");
            BasicBlock * const resume = b->CreateBasicBlock(name + "Resume");

            b->CreateUnlikelyCondBr(b->CreateICmpEQ(accessibleStrides, ZERO), copyFromBack, resume);

            b->SetInsertPoint(copyFromBack);
            Value * const temporaryAvailable = b->CreateUMin(unprocessed, inputStrideSize[i]);
            b->CreateAssert(b->CreateICmpULE(mAvailableItemCount[i], temporaryAvailable), "linearly available cannot be greater than temporarily available");
            Value * const tempBufferPtr = temporaryInputBuffer[i];
            Value * const offset = b->CreateAnd(ic, BLOCK_WIDTH_MASK);
            const auto alignment = getItemAlignment(mStreamSetInputs[i]);
            b->CreateStreamCpy(name, tempBufferPtr, ZERO, mStreamSetInputBaseAddress[i] , offset, mAvailableItemCount[i], alignment);
            Value * const temporaryStrides = b->CreateSelect(b->CreateICmpULT(unprocessed, inputStrideSize[i]), ZERO, ONE);
            BasicBlock * const copyToBackEnd = b->GetInsertBlock();
            b->CreateCondBr(b->CreateICmpNE(mAvailableItemCount[i], temporaryAvailable), copyFromFront, resume);

            b->SetInsertPoint(copyFromFront);
            Value * const remaining = b->CreateSub(temporaryAvailable, mAvailableItemCount[i]);
            Value * const baseAddress = b->getBaseAddress(name);
            b->CreateStreamCpy(name, tempBufferPtr, mAvailableItemCount[i], baseAddress, ZERO, remaining, alignment);
            BasicBlock * const copyToFrontEnd = b->GetInsertBlock();
            b->CreateBr(resume);

            b->SetInsertPoint(resume);
            PHINode * const bufferPtr = b->CreatePHI(mStreamSetInputBaseAddress[i] ->getType(), 3);
            bufferPtr->addIncoming(mStreamSetInputBaseAddress[i] , entry);
            bufferPtr->addIncoming(tempBufferPtr, copyToBackEnd);
            bufferPtr->addIncoming(tempBufferPtr, copyToFrontEnd);
            mStreamSetInputBaseAddress[i] = bufferPtr;

            PHINode * const phiAvailItemCount = b->CreatePHI(b->getSizeTy(), 3);
            phiAvailItemCount->addIncoming(mAvailableItemCount[i], entry);
            phiAvailItemCount->addIncoming(temporaryAvailable, copyToBackEnd);
            phiAvailItemCount->addIncoming(temporaryAvailable, copyToFrontEnd);
            mAvailableItemCount[i] = phiAvailItemCount;

            PHINode * const phiNumOfStrides = b->CreatePHI(b->getSizeTy(), 2);
            phiNumOfStrides->addIncoming(accessibleStrides, entry);
            phiNumOfStrides->addIncoming(temporaryStrides, copyToBackEnd);
            phiNumOfStrides->addIncoming(temporaryStrides, copyToFrontEnd);
            accessibleStrides = phiNumOfStrides;
        }
        numOfStrides = b->CreateUMin(numOfStrides, accessibleStrides);
    }

    // Now determine the linearly writeable strides
    Value * linearlyWritable[outputSetCount];
    Value * baseOutputBuffer[outputSetCount];
    Value * outputStrideSize[outputSetCount];
    mInitialProducedItemCount.resize(outputSetCount);
    mStreamSetOutputBaseAddress.resize(outputSetCount);
    for (unsigned i = 0; i < outputSetCount; i++) {
        const auto & output = mStreamSetOutputs[i];
        const auto & name = output.getName();
        const ProcessingRate & rate = output.getRate();
        Value * const ic = b->getProducedItemCount(name);
        baseOutputBuffer[i] = b->getBlockAddress(name, b->CreateLShr(ic, LOG_2_BLOCK_WIDTH));
        assert (baseOutputBuffer[i]->getType()->isPointerTy());
        linearlyWritable[i] = b->getLinearlyWritableItems(name, ic);
        mInitialProducedItemCount[i] = ic;
        outputStrideSize[i] = nullptr;
        if (temporaryOutputBuffer[i]) {
            outputStrideSize[i] = getStrideSize(b, rate);
            // Is the number of linearly writable items sufficient for a stride?
            Value * writableStrides = b->CreateUDiv(linearlyWritable[i], outputStrideSize[i]);
            if (!rate.isFixed() || requiresBufferedFinalStride(output)) {
                Value * const requiresCopy = b->CreateICmpEQ(writableStrides, ZERO);
                assert (temporaryOutputBuffer[i]->getType() == baseOutputBuffer[i]->getType());
                baseOutputBuffer[i] = b->CreateSelect(requiresCopy, temporaryOutputBuffer[i], baseOutputBuffer[i]);
                writableStrides = b->CreateSelect(requiresCopy, ONE, writableStrides);
            }
            numOfStrides = b->CreateUMin(numOfStrides, writableStrides);
            assert (temporaryOutputBuffer[i]->getType() == baseOutputBuffer[i]->getType());
        }
        mStreamSetOutputBaseAddress[i] = baseOutputBuffer[i];
    }

    Value * const initiallyFinal = mIsFinal;
    if (LLVM_LIKELY(numOfStrides != nullptr)) {
        mIsFinal = b->CreateAnd(mIsFinal, b->CreateICmpEQ(numOfStrides, ZERO));
        Value * const processStride = b->CreateOr(b->CreateICmpNE(numOfStrides, ZERO), mIsFinal);
        b->CreateAssert(processStride, getName() + " does not have sufficient input data or output space for one stride");
        for (unsigned i = 0; i < inputSetCount; ++i) {
            const ProcessingRate & rate = mStreamSetInputs[i].getRate();
            if (rate.isFixed() && mStreamSetInputs[i].notDeferred()) {
                mAvailableItemCount[i] = b->CreateSelect(mIsFinal, mAvailableItemCount[i], b->CreateMul(numOfStrides, inputStrideSize[i]));
            }
        }
    }

    //  We have one or more blocks of input data and output buffer space for all stream sets.
    generateMultiBlockLogic(b, numOfStrides);

    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (rate.isFixed() && mStreamSetInputs[i].notDeferred()) {
            Value * const ic = b->CreateAdd(mInitialProcessedItemCount[i], mAvailableItemCount[i]);
            b->setProcessedItemCount(mStreamSetInputs[i].getName(), ic);
        }
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        if (rate.isFixed()) {
            assert (mStreamSetOutputs[i].notDeferred());
            Value * const produced = b->CreateMul(numOfStrides, outputStrideSize[i]);
            Value * const ic = b->CreateAdd(mInitialProducedItemCount[i], produced);
            b->setProducedItemCount(mStreamSetOutputs[i].getName(), ic);
        }
    }

    BasicBlock * const handleFinalBlock = b->CreateBasicBlock("HandleFinalBlock");
    BasicBlock * const temporaryBufferCopyBack = b->CreateBasicBlock("TemporaryBufferCopyBack");
    BasicBlock * const strideDone = b->CreateBasicBlock("MultiBlockDone");

    b->CreateLikelyCondBr(b->CreateICmpNE(numOfStrides, ZERO), temporaryBufferCopyBack, handleFinalBlock);


    /// FINAL STRIDE ADJUSTMENT
    b->SetInsertPoint(handleFinalBlock);

    // If this is our final stride, adjust the Fixed output item counts. The main loop assumes that
    // the ITEM COUNT % FIXED RATE = 0 for all Fixed Input and Output streams. We correct that here
    // to calculate them based on the actual input item counts.

    reviseFinalProducedItemCounts(b);

    b->CreateBr(temporaryBufferCopyBack);

    /// TEMPORARY BUFFER COPY BACK
    b->SetInsertPoint(temporaryBufferCopyBack);

    // Copy back data to the actual output buffers.
    for (unsigned i = 0; i < outputSetCount; i++) {
        Value * const tempBuffer = temporaryOutputBuffer[i];
        if (LLVM_UNLIKELY(tempBuffer == nullptr)) {
            continue;
        }
        Value * const baseBuffer = baseOutputBuffer[i];
        assert ("stack overflow" && (tempBuffer->getType() == baseBuffer->getType()));
        const auto & name = mStreamSetOutputs[i].getName();
        BasicBlock * const copyToBack = b->CreateBasicBlock(name + "CopyToBack");
        BasicBlock * const copyToFront = b->CreateBasicBlock(name + "CopyToFront");
        BasicBlock * const resume = b->CreateBasicBlock(name + "ResumeCopyBack");
        // If we used a temporary buffer, copy it back to the original output buffer
        b->CreateCondBr(b->CreateICmpEQ(tempBuffer, baseBuffer), copyToBack, resume);

        b->SetInsertPoint(copyToBack);        
        Value * const offset = b->CreateAnd(mInitialProducedItemCount[i], BLOCK_WIDTH_MASK);
        Value * const newProducedItemCount = b->getProducedItemCount(name);
        Value * const newlyProduced = b->CreateSub(newProducedItemCount, mInitialProducedItemCount[i]);
        Value * const toWrite = b->CreateUMin(newlyProduced, linearlyWritable[i]);
        const auto alignment = getItemAlignment(mStreamSetOutputs[i]);
        b->CreateStreamCpy(name, baseBuffer, offset, tempBuffer, ZERO, toWrite, alignment);
        // If we required a temporary output buffer, we will probably need to write to the beginning of the buffer as well.
        b->CreateLikelyCondBr(b->CreateICmpULT(toWrite, newlyProduced), copyToFront, resume);

        b->SetInsertPoint(copyToFront);
        Value * const remaining = b->CreateSub(newlyProduced, toWrite);
        Value * const baseAddress = b->getBaseAddress(name);
        b->CreateStreamCpy(name, baseAddress, ZERO, tempBuffer, toWrite, remaining, alignment);
        b->CreateBr(resume);

        b->SetInsertPoint(resume);
    }

    strideDone->moveAfter(b->GetInsertBlock());

    BasicBlock * const segmentDone = b->CreateBasicBlock("SegmentDone");
    //  We've dealt with the partial block processing and copied information back into the
    //  actual buffers.  If this isn't the final block, loop back for more multiblock processing.
    if (hasNoTerminateAttribute()) {
        b->CreateCondBr(mIsFinal, segmentDone, strideDone);
    } else {
        BasicBlock * const setTermination = b->CreateBasicBlock("setTermination");
        b->CreateCondBr(mIsFinal, setTermination, strideDone);

        b->SetInsertPoint(setTermination);
        b->setTerminationSignal();
        b->CreateBr(segmentDone);        
    }

    /// STRIDE DONE
    b->SetInsertPoint(strideDone);

    // do we have enough data for another stride?
    Value * pendingStrides = nullptr;
    for (unsigned i = 0; i < inputSetCount; ++i) {
        Value * const processed = b->getProcessedItemCount(mStreamSetInputs[i].getName());
        Value * const remaining = b->CreateSub(mInitialAvailableItemCount[i], processed);
        Value * const remainingStrides = b->CreateUDiv(remaining, inputStrideSize[i]);
        pendingStrides = b->CreateUMin(pendingStrides, remainingStrides);
    }

    // do we have enough room for another stride?
    for (unsigned i = 0; i < outputSetCount; ++i) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        const auto & name = mStreamSetOutputs[i].getName();
        Value * const newProduced = b->getProducedItemCount(name);
        // If this output has a Fixed/Bounded rate, determine whether we have room for another stride.
        if (LLVM_LIKELY(outputStrideSize[i] != nullptr)) {
            Value * const unconsumed = b->CreateSub(newProduced, b->getConsumedItemCount(name));
            Value * const remaining = b->CreateSub(b->getCapacity(name), unconsumed);
            Value * const remainingStrides = b->CreateUDiv(remaining, outputStrideSize[i]);
            pendingStrides = b->CreateUMin(pendingStrides, remainingStrides);
        }
        // Do copybacks if necessary.
        if (mStreamSetOutputBuffers[i]->supportsCopyBack() && requiresCopyBack(rate)) {
            b->CreateCopyBack(name, mInitialProducedItemCount[i], newProduced);
        }
    }

    Value * const hasMoreStrides = b->CreateOr(b->CreateICmpNE(pendingStrides, ZERO), initiallyFinal);
    b->CreateCondBr(hasMoreStrides, segmentLoop, segmentDone);

    /// SEGMENT DONE
    segmentDone->moveAfter(b->GetInsertBlock());
    b->SetInsertPoint(segmentDone);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
bool MultiBlockKernel::requiresCopyBack(const ProcessingRate & rate) const {
    if (rate.isBounded() || rate.isUnknown()) {
        return true;
    } else if (rate.isRelative()) {
        return requiresCopyBack(getBinding(rate.getReference()).getRate());
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateUDivCeil
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * CreateUDivCeil(const std::unique_ptr<KernelBuilder> & b, Value * const number, const ProcessingRate::RateValue divisor, const Twine & Name = "") {
    Constant * const n = ConstantInt::get(number->getType(), divisor.numerator());
    if (LLVM_LIKELY(divisor.denominator() == 1)) {
        return b->CreateUDivCeil(number, n, Name);
    } else {
        //   ⌊(num + ratio - 1) / ratio⌋
        // = ⌊(num - 1) / (n/d)⌋ + (ratio/ratio)
        // = ⌊(d * (num - 1)) / n⌋ + 1
        Constant * const ONE = ConstantInt::get(number->getType(), 1);
        Constant * const d = ConstantInt::get(number->getType(), divisor.denominator());
        return b->CreateAdd(b->CreateUDiv(b->CreateMul(b->CreateSub(number, ONE), d), n), ONE, Name);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reviseFinalProducedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void MultiBlockKernel::reviseFinalProducedItemCounts(const std::unique_ptr<KernelBuilder> & b) {

    if (LLVM_UNLIKELY(mStreamSetInputs.empty())) {
        return;
    }

    const auto inputSetCount = mStreamSetInputs.size();

    ProcessingRate::RateValue rateLCM(1);
    unsigned first = 0;
    unsigned last = inputSetCount;

    for (unsigned i = 0; i < inputSetCount; ++i) {
        const ProcessingRate & pr = mStreamSetInputs[i].getRate();
        if (pr.isFixed()) {
            rateLCM = lcm(rateLCM, pr.getRate());
            if (mStreamSetInputs[i].isPrincipal()) {
                assert ("A kernel cannot have multiple principle input streams" && (first == 0 && last == inputSetCount));
                first = i;
                last = i + 1;
            }
        }       
    }

    bool noFixedRateOutput = true;

    for (const Binding & output : mStreamSetOutputs) {
        const ProcessingRate & pr = output.getRate();
        if (pr.isFixed()) {
            rateLCM = lcm(rateLCM, pr.getRate());
            noFixedRateOutput = false;
        }
    }

    if (noFixedRateOutput) {
        return;
    }

    Value * baseInitialProcessedItemCount = nullptr;
    Value * scaledInverseOfAvailItemCount = nullptr;

    // For each Fixed output stream, this calculates:

    //    CEILING(MIN(Available Item Count / Fixed Input Rate) * Fixed Output Rate)

    // But avoids the possibility of overflow errors (assuming that each processed item count does not overflow)

    for (unsigned i = first; i < last; ++i) {
        const ProcessingRate & pr = mStreamSetInputs[i].getRate();
        if (pr.isFixed()) {
            Value * p = mInitialProcessedItemCount[i];
            Value * a = b->CreateSub(mInitialAvailableItemCount[i], p);
            const auto & rate = pr.getRate();
            if (LLVM_UNLIKELY(rateLCM != rate)) {
                const auto factor = rateLCM / rate;
                if (LLVM_UNLIKELY(factor.numerator() > 1)) {
                    a = b->CreateMul(a, b->getSize(factor.numerator()));
                }
                if (LLVM_UNLIKELY(factor.denominator() > 1)) {
                    a = b->CreateUDiv(a, b->getSize(factor.denominator()));
                }
            }
            if (LLVM_UNLIKELY(rate.denominator() > 1)) {
                p = b->CreateMul(p, b->getSize(rate.denominator()));
            }
            if (LLVM_UNLIKELY(rate.numerator() > 1)) {
                p = b->CreateUDiv(p, b->getSize(rate.numerator()));
            }
            if (scaledInverseOfAvailItemCount) {
                scaledInverseOfAvailItemCount = b->CreateUMin(scaledInverseOfAvailItemCount, a);
                baseInitialProcessedItemCount = b->CreateUMin(baseInitialProcessedItemCount, p);
            } else {
                scaledInverseOfAvailItemCount = a;
                baseInitialProcessedItemCount = p;
            }
        }
    }

    for (const Binding & output : mStreamSetOutputs) {
        const auto name = output.getName();
        const ProcessingRate & pr = output.getRate();
        Value * produced = nullptr;
        if (pr.isFixed() && output.notDeferred()) {
            assert (baseInitialProcessedItemCount && scaledInverseOfAvailItemCount);
            const auto rate = pr.getRate();
            Value * p = baseInitialProcessedItemCount;
            if (LLVM_UNLIKELY(rate.numerator() != 1)) {
                p = b->CreateMul(p, b->getSize(rate.numerator()));
            }
            if (LLVM_UNLIKELY(rate.denominator() != 1)) {
                p = b->CreateUDiv(p, b->getSize(rate.denominator()));
            }
            Value * const ic = CreateUDivCeil(b, scaledInverseOfAvailItemCount, rateLCM / pr.getRate());
            produced = b->CreateAdd(p, ic);
        } else { // check if we have an attribute; if so, get the current produced count and adjust it
            bool noAttributes = true;
            for (const Attribute & attr : output.getAttributes()) {
                if (attr.isAdd() || attr.isRoundUpTo()) {
                    noAttributes = false;
                    break;
                }
            }
            if (noAttributes) {
                continue;
            }
            produced = b->getProducedItemCount(name);
        }
        for (const Attribute & attr : output.getAttributes()) {
            if (attr.isAdd()) {
                produced = b->CreateAdd(produced, b->getSize(attr.getAmount()));
            } else if (attr.isRoundUpTo()) {
                produced = b->CreateRoundUp(produced, b->getSize(attr.getAmount()));
            }
        }
        b->setProducedItemCount(name, produced);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateMultiBlockLogic
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockOrientedKernel::generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, Value * const numOfBlocks) {

    if (LLVM_UNLIKELY(mStride != b->getBitBlockWidth())) {
        report_fatal_error(getName() + ": the Stride (" + std::to_string(mStride) + ") of BlockOrientedKernel "
                           "equal to the BitBlockWidth (" + std::to_string(b->getBitBlockWidth()) + ")");
    }

    Constant * const LOG_2_BLOCK_WIDTH = b->getSize(std::log2(b->getBitBlockWidth()));

    BasicBlock * const entryBlock = b->GetInsertBlock();
    mStrideLoopBody = b->CreateBasicBlock(getName() + "_strideLoopBody");
    BasicBlock * const stridesDone = b->CreateBasicBlock(getName() + "_stridesDone");
    BasicBlock * const doFinalBlock = b->CreateBasicBlock(getName() + "_doFinalBlock");
    BasicBlock * const segmentDone = b->CreateBasicBlock(getName() + "_segmentDone");
    b->CreateAssert(b->CreateXor(b->CreateIsNotNull(numOfBlocks), mIsFinal),
                    "numOfStrides cannot be 0 unless this is the final stride and must be 0 if it is");
    const auto inputSetCount = mStreamSetInputs.size();
    Value * baseProcessedIndex[inputSetCount];
    Value * baseInputAddress[inputSetCount];
    for (unsigned i = 0; i < inputSetCount; i++) {
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (LLVM_UNLIKELY(!rate.isFixed())) {
            Value * const ic = mInitialProcessedItemCount[i];
            baseProcessedIndex[i] = b->CreateLShr(ic, LOG_2_BLOCK_WIDTH);
        }
        baseInputAddress[i] = mStreamSetInputBaseAddress[i];
    }

    const auto outputSetCount = mStreamSetOutputs.size();
    Value * baseProducedIndex[outputSetCount];
    Value * baseOutputAddress[inputSetCount];
    for (unsigned i = 0; i < outputSetCount; i++) {
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        if (LLVM_UNLIKELY(!rate.isFixed())) {
            Value * const ic = b->getProducedItemCount(mStreamSetOutputs[i].getName());
            baseProducedIndex[i] = b->CreateLShr(ic, LOG_2_BLOCK_WIDTH);
        }
        baseOutputAddress[i] = mStreamSetOutputBaseAddress[i];
    }

    b->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, mStrideLoopBody);

    /// BLOCK BODY

    b->SetInsertPoint(mStrideLoopBody);

    if (b->supportsIndirectBr()) {
        Value * const baseTarget = BlockAddress::get(segmentDone);
        mStrideLoopTarget = b->CreatePHI(baseTarget->getType(), 2, "strideTarget");
        mStrideLoopTarget->addIncoming(baseTarget, entryBlock);
    }

    mStrideBlockIndex = b->CreatePHI(b->getSizeTy(), 2);
    mStrideBlockIndex->addIncoming(b->getSize(0), entryBlock);

    /// GENERATE DO BLOCK METHOD

    for (unsigned i = 0; i < inputSetCount; ++i) {
        Value * index = mStrideBlockIndex;
        const ProcessingRate & rate = mStreamSetInputs[i].getRate();
        if (LLVM_UNLIKELY(!rate.isFixed())) {
            Value * ic = b->getProcessedItemCount(mStreamSetInputs[i].getName());
            index = b->CreateSub(b->CreateLShr(ic, LOG_2_BLOCK_WIDTH), baseProcessedIndex[i]);
        }
        mStreamSetInputBaseAddress[i] = b->CreateGEP(mStreamSetInputBaseAddress[i], index);
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        Value * index = mStrideBlockIndex;
        const ProcessingRate & rate = mStreamSetOutputs[i].getRate();
        if (LLVM_UNLIKELY(!rate.isFixed())) {
            Value * ic = b->getProducedItemCount(mStreamSetOutputs[i].getName());
            index = b->CreateSub(b->CreateLShr(ic, LOG_2_BLOCK_WIDTH), baseProducedIndex[i]);
        }
        mStreamSetOutputBaseAddress[i] = b->CreateGEP(mStreamSetOutputBaseAddress[i], index);
    }

    writeDoBlockMethod(b);

    BasicBlock * const bodyEnd = b->GetInsertBlock();
    if (mStrideLoopTarget) {
        mStrideLoopTarget->addIncoming(mStrideLoopTarget, bodyEnd);
    }

    Value * const nextIndex = b->CreateAdd(mStrideBlockIndex, b->getSize(1));
    mStrideBlockIndex->addIncoming(nextIndex, bodyEnd);
    Value * const notDone = b->CreateICmpULT(nextIndex, numOfBlocks);
    b->CreateCondBr(notDone, mStrideLoopBody, stridesDone);

    stridesDone->moveAfter(bodyEnd);

    /// STRIDE DONE

    b->SetInsertPoint(stridesDone);

    // Now conditionally perform the final block processing depending on the doFinal parameter.
    if (mStrideLoopTarget) {
        mStrideLoopBranch = b->CreateIndirectBr(mStrideLoopTarget, 3);
        mStrideLoopBranch->addDestination(doFinalBlock);
        mStrideLoopBranch->addDestination(segmentDone);
    } else {
        b->CreateUnlikelyCondBr(mIsFinal, doFinalBlock, segmentDone);
    }

    doFinalBlock->moveAfter(stridesDone);

    /// DO FINAL BLOCK

    b->SetInsertPoint(doFinalBlock);
    for (unsigned i = 0; i < inputSetCount; ++i) {
        mStreamSetInputBaseAddress[i] = baseInputAddress[i];
    }

    for (unsigned i = 0; i < outputSetCount; ++i) {
        mStreamSetOutputBaseAddress[i] = baseOutputAddress[i];
    }

    writeFinalBlockMethod(b, getRemainingItems(b));

    b->CreateBr(segmentDone);

    segmentDone->moveAfter(b->GetInsertBlock());

    b->SetInsertPoint(segmentDone);

    // Update the branch prediction metadata to indicate that the likely target will be segmentDone
    if (mStrideLoopTarget) {
        MDBuilder mdb(b->getContext());
        const auto destinations = mStrideLoopBranch->getNumDestinations();
        uint32_t weights[destinations];
        for (unsigned i = 0; i < destinations; ++i) {
            weights[i] = (mStrideLoopBranch->getDestination(i) == segmentDone) ? 100 : 1;
        }
        ArrayRef<uint32_t> bw(weights, destinations);
        mStrideLoopBranch->setMetadata(LLVMContext::MD_prof, mdb.createBranchWeights(bw));
    }

    return numOfBlocks;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRemainingItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * BlockOrientedKernel::getRemainingItems(const std::unique_ptr<KernelBuilder> & b) {
    Value * remainingItems = nullptr;
    const auto count = mStreamSetInputs.size();
    if (count == 1) {
        return mAvailableItemCount[0];
    } else {
        for (unsigned i = 0; i < count; i++) {
            if (mStreamSetInputs[i].isPrincipal()) {
                return mAvailableItemCount[i];
            }
        }
        for (unsigned i = 0; i < count; ++i) {
            const ProcessingRate & r = mStreamSetInputs[i].getRate();
            if (r.isFixed()) {
                Value * ic = CreateUDivCeil(b, mAvailableItemCount[i], r.getRate());
                if (remainingItems) {
                    remainingItems = b->CreateUMin(remainingItems, ic);
                } else {
                    remainingItems = ic;
                }
            }
        }
    }
    return remainingItems;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeDoBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockOrientedKernel::writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    auto ip = b->saveIP();
    std::vector<Value *> availableItemCount(0);

    /// Check if the do block method is called and create the function if necessary
    if (!b->supportsIndirectBr()) {

        std::vector<Type *> params;
        params.reserve(1 + mAvailableItemCount.size());
        params.push_back(self->getType());
        for (Value * avail : mAvailableItemCount) {
            params.push_back(avail->getType());
        }

        FunctionType * const type = FunctionType::get(b->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + DO_BLOCK_SUFFIX, b->getModule());
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
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    }

    generateDoBlockMethod(b); // must be implemented by the BlockOrientedKernelBuilder subtype

    if (!b->supportsIndirectBr()) {
        // Restore the DoSegment function state then call the DoBlock method
        b->CreateRetVoid();
        mDoBlockMethod = mCurrentMethod;
        b->restoreIP(ip);
        setInstance(self);
        mCurrentMethod = cp;
        mAvailableItemCount.swap(availableItemCount);
        CreateDoBlockMethodCall(b);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void BlockOrientedKernel::writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * remainingItems) {

    Value * const self = getInstance();
    Function * const cp = mCurrentMethod;
    Value * const remainingItemCount = remainingItems;
    auto ip = b->saveIP();
    std::vector<Value *> availableItemCount(0);

    if (!b->supportsIndirectBr()) {
        std::vector<Type *> params;
        params.reserve(2 + mAvailableItemCount.size());
        params.push_back(self->getType());
        params.push_back(b->getSizeTy());
        for (Value * avail : mAvailableItemCount) {
            params.push_back(avail->getType());
        }
        FunctionType * const type = FunctionType::get(b->getVoidTy(), params, false);
        mCurrentMethod = Function::Create(type, GlobalValue::InternalLinkage, getName() + FINAL_BLOCK_SUFFIX, b->getModule());
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
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    }

    generateFinalBlockMethod(b, remainingItems); // may be implemented by the BlockOrientedKernel subtype

    if (!b->supportsIndirectBr()) {
        b->CreateRetVoid();
        b->restoreIP(ip);
        setInstance(self);
        mAvailableItemCount.swap(availableItemCount);
        // Restore the DoSegment function state then call the DoFinal method
        std::vector<Value *> args;
        args.reserve(2 + mAvailableItemCount.size());
        args.push_back(self);
        args.push_back(remainingItemCount);
        args.insert(args.end(), mAvailableItemCount.begin(), mAvailableItemCount.end());
        b->CreateCall(mCurrentMethod, args);
        mCurrentMethod = cp;
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalBlockMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void BlockOrientedKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * /* remainingItems */) {
    //  The default finalBlock method simply dispatches to the doBlock routine.
    CreateDoBlockMethodCall(b);
}

void BlockOrientedKernel::CreateDoBlockMethodCall(const std::unique_ptr<KernelBuilder> & b) {
    if (b->supportsIndirectBr()) {
        BasicBlock * const bb = b->CreateBasicBlock("resume");
        mStrideLoopBranch->addDestination(bb);
        BasicBlock * const current = b->GetInsertBlock();
        mStrideLoopTarget->addIncoming(BlockAddress::get(bb), current);
        mStrideBlockIndex->addIncoming(b->getSize(0), current);
        b->CreateBr(mStrideLoopBody);
        bb->moveAfter(current);
        b->SetInsertPoint(bb);
    } else {
        std::vector<Value *> args;
        args.reserve(1 + mAvailableItemCount.size());
        args.push_back(getInstance());
        args.insert(args.end(), mAvailableItemCount.begin(), mAvailableItemCount.end());
        b->CreateCall(mDoBlockMethod, args);
    }
}

static inline std::string annotateKernelNameWithDebugFlags(std::string && name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        name += "_EA";
    }
    name += "_O" + std::to_string((int)codegen::OptLevel);
    return name;
}

// CONSTRUCTOR
Kernel::Kernel(std::string && kernelName,
               Bindings && stream_inputs,
               Bindings && stream_outputs,
               Bindings && scalar_parameters,
               Bindings && scalar_outputs,
               Bindings && internal_scalars)
: KernelInterface(annotateKernelNameWithDebugFlags(std::move(kernelName))
                  , std::move(stream_inputs), std::move(stream_outputs)
                  , std::move(scalar_parameters), std::move(scalar_outputs)
                  , std::move(internal_scalars))
, mCurrentMethod(nullptr)
, mAvailablePrincipalItemCount(nullptr)
, mNoTerminateAttribute(false)
, mIsGenerated(false)
, mStride(0)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr) {

}

Kernel::~Kernel() {

}

// MULTI-BLOCK KERNEL CONSTRUCTOR
MultiBlockKernel::MultiBlockKernel(std::string && kernelName,
                                   Bindings && stream_inputs,
                                   Bindings && stream_outputs,
                                   Bindings && scalar_parameters,
                                   Bindings && scalar_outputs,
                                   Bindings && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

}

// CONSTRUCTOR
BlockOrientedKernel::BlockOrientedKernel(std::string && kernelName,
                                         Bindings && stream_inputs,
                                         Bindings && stream_outputs,
                                         Bindings && scalar_parameters,
                                         Bindings && scalar_outputs,
                                         Bindings && internal_scalars)
: MultiBlockKernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars))
, mDoBlockMethod(nullptr)
, mStrideLoopBody(nullptr)
, mStrideLoopBranch(nullptr)
, mStrideLoopTarget(nullptr)
, mStrideBlockIndex(nullptr) {

}

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(std::string && kernelName,
                                             Bindings && stream_inputs,
                                             Bindings && stream_outputs,
                                             Bindings && scalar_parameters,
                                             Bindings && scalar_outputs,
                                             Bindings && internal_scalars)
: Kernel(std::move(kernelName), std::move(stream_inputs), std::move(stream_outputs), std::move(scalar_parameters), std::move(scalar_outputs), std::move(internal_scalars)) {

}


}
