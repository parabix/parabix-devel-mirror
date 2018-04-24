/*
 *  Copyright (c) 2018 International Characters.
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
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/Bitcode/ReaderWriter.h>
#else
#include <llvm/Bitcode/BitcodeWriter.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <kernels/streamset.h>
#include <sstream>
#include <kernels/kernel_builder.h>
#include <llvm/Support/Debug.h>

using namespace llvm;
using namespace parabix;

namespace kernel {

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
 * @brief bindPorts
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::bindPorts(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs) {

    if (LLVM_UNLIKELY(mStreamSetInputs.size() != inputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetInputs.size()) +
                           " input stream sets but was given "
                           + std::to_string(inputs.size()));
    }

    for (unsigned i = 0; i < mStreamSetInputs.size(); i++) {
        mStreamMap.emplace(mStreamSetInputs[i].getName(), std::make_pair(Port::Input, i));
    }

    for (unsigned i = 0; i < inputs.size(); ++i) {
        StreamSetBuffer * const buf = inputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": input stream " + std::to_string(i) + " cannot be null");
        }
       // const Binding & input = mStreamSetInputs[i];
       // verifyBufferSize(input, buf);
        buf->addConsumer(this);
    }

    if (LLVM_UNLIKELY(mStreamSetOutputs.size() != outputs.size())) {
        report_fatal_error(getName() + ": expected " + std::to_string(mStreamSetOutputs.size())
                           + " output stream sets but was given "
                           + std::to_string(outputs.size()));
    }

    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        mStreamMap.emplace(mStreamSetOutputs[i].getName(), std::make_pair(Port::Output, i));
    }

    for (unsigned i = 0; i < outputs.size(); ++i) {
        StreamSetBuffer * const buf = outputs[i];
        if (LLVM_UNLIKELY(buf == nullptr)) {
            report_fatal_error(getName() + ": output stream set " + std::to_string(i) + " cannot be null");
        }
        const Binding & output = mStreamSetOutputs[i];
       // verifyBufferSize(output, buf);
        if (LLVM_LIKELY(buf->getProducer() == nullptr)) {
            buf->setProducer(this);
        } else {
            report_fatal_error(getName() + ": output stream set " + output.getName()
                               + " is already produced by kernel " + buf->getProducer()->getName());
        }
    }

    mStreamSetInputBuffers.assign(inputs.begin(), inputs.end());
    mStreamSetOutputBuffers.assign(outputs.begin(), outputs.end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCacheName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getCacheName(const std::unique_ptr<KernelBuilder> & b) const {
    std::stringstream cacheName;
    cacheName << getName() << '_' << b->getBuilderUniqueName();
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
void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & b) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && b);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    // verifyStreamSetDefinitions();
    addBaseKernelProperties(b);
    addInternalKernelProperties(b);
    // NOTE: StructType::create always creates a new type even if an identical one exists.
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        makeModule(b);
    }
    mKernelStateType = mModule->getTypeByName(getName());
    if (LLVM_LIKELY(mKernelStateType == nullptr)) {
        mKernelStateType = StructType::create(b->getContext(), mKernelFields, getName());
        assert (mKernelStateType);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareCachedKernel(const std::unique_ptr<KernelBuilder> & b) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && b);
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    assert (getModule());    
    addBaseKernelProperties(b);
    mKernelStateType = getModule()->getTypeByName(getName());
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Kernel definition for " + getName() + " could not be found in the cache object");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief containsFixedRate
 ** ------------------------------------------------------------------------------------------------------------- */
bool containsFixedRate(const Bindings & bindings) {
    for (const Binding & binding : bindings) {
        const ProcessingRate & rate = binding.getRate();
        if (rate.isFixed()) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addBaseKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & b) {

    const unsigned inputSetCount = mStreamSetInputs.size();
    const unsigned outputSetCount = mStreamSetOutputs.size();

    assert (inputSetCount == mStreamSetInputBuffers.size());
    assert (outputSetCount == mStreamSetOutputBuffers.size());

    if (mStride == 0) {
        // Set the default kernel stride.
        mStride = b->getBitBlockWidth();
    }

    IntegerType * const sizeTy = b->getSizeTy();

    addScalar(sizeTy, LOGICAL_SEGMENT_NO_SCALAR);
    addScalar(sizeTy, TERMINATION_SIGNAL);
    // TODO: if we had a way of easily calculating the number of processed/produced items of the
    // final stride of a non-deferred fixed rate stream, we could avoid storing the item counts.
    for (unsigned i = 0; i < inputSetCount; i++) {
        const Binding & input = mStreamSetInputs[i];
        addScalar(sizeTy, input.getName() + PROCESSED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(input.isDeferred())) {
            addScalar(sizeTy, input.getName() + NON_DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    for (unsigned i = 0; i < outputSetCount; i++) {
        const Binding & output = mStreamSetOutputs[i];
        addScalar(sizeTy, output.getName() + PRODUCED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(output.isDeferred())) {
            addScalar(sizeTy, output.getName() + NON_DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }
    for (unsigned i = 0; i < inputSetCount; i++) {
        mScalarInputs.emplace_back(mStreamSetInputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetInputs[i].getName() + BUFFER_SUFFIX);
    }
    for (unsigned i = 0; i < outputSetCount; i++) {
        mScalarInputs.emplace_back(mStreamSetOutputBuffers[i]->getStreamSetHandle()->getType(), mStreamSetOutputs[i].getName() + BUFFER_SUFFIX);
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
    Type * const consumerSetTy = StructType::get(b->getContext(), {sizeTy, sizeTy->getPointerTo()->getPointerTo()})->getPointerTo();
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(consumerSetTy, mStreamSetOutputs[i].getName() + CONSUMER_SUFFIX);
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        addScalar(sizeTy, mStreamSetOutputs[i].getName() + CONSUMED_ITEM_COUNT_SUFFIX);
    }
    // We compile in a 64-bit CPU cycle counter into every kernel.   It will remain unused
    // in normal execution, but when codegen::EnableCycleCounter is specified, pipelines
    // will be able to add instrumentation to cached modules without recompilation.
    addScalar(b->getInt64Ty(), CYCLECOUNT_SCALAR);
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
        std::string tmp;
        raw_string_ostream signature(tmp);
        WriteBitcodeToFile(getModule(), signature);
        return signature.str();
    } else {
        return getModule()->getModuleIdentifier();
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::generateKernel(const std::unique_ptr<kernel::KernelBuilder> & idb) {
    assert ("Kernel does not have a valid IDISA Builder" && idb.get());
    if (LLVM_UNLIKELY(mIsGenerated)) return;
    idb->setModule(mModule);
    addKernelDeclarations(idb);
    callGenerateInitializeMethod(idb);
    callGenerateDoSegmentMethod(idb);
    callGenerateFinalizeMethod(idb);
    mIsGenerated = true;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCurrentMethod = getInitFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    Function::arg_iterator args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    b->CreateStore(ConstantAggregateZero::get(mKernelStateType), getInstance());
    for (const auto & binding : mScalarInputs) {
        b->setScalarField(binding.getName(), &*(args++));
    }
    for (const auto & binding : mStreamSetOutputs) {
        b->setConsumerLock(binding.getName(), &*(args++));
    }
    generateInitializeMethod(b);
    b->CreateRetVoid();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateDoSegmentMethod(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCurrentMethod = getDoSegmentFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    mIsFinal = &*(args++);
    const auto n = mStreamSetInputs.size();
    mAvailableItemCount.resize(n, nullptr);
    for (unsigned i = 0; i < n; i++) {
        assert (args != mCurrentMethod->arg_end());
        mAvailableItemCount[i] = &*(args++);
    }
    assert (args == mCurrentMethod->arg_end());
    generateKernelMethod(b); // must be overridden by the Kernel subtype
    mIsFinal = nullptr;
    mAvailableItemCount.clear();
    b->CreateRetVoid();
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCurrentMethod = getTerminateFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setInstance(&*(args++));
    generateFinalizeMethod(b); // may be overridden by the Kernel subtype
    const auto n = mScalarOutputs.size();
    if (n == 0) {
        b->CreateRetVoid();
    } else {
        Value * outputs[n];
        for (unsigned i = 0; i < n; ++i) {
            outputs[i] = b->getScalarField(mScalarOutputs[i].getName());
        }
        if (n == 1) {
            b->CreateRet(outputs[0]);
        } else {
            b->CreateAggregateRet(outputs, n);
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarIndex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned Kernel::getScalarIndex(const std::string & name) const {
    const auto f = mKernelFieldMap.find(name);
    if (LLVM_UNLIKELY(f == mKernelFieldMap.end())) {
        assert ("kernel does not contain the requested scalar" && false);
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
void Kernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b) {
    assert ("KernelBuilder does not have a valid IDISA Builder" && b);
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
    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    PointerType * const sizePtrPtrTy = sizePtrTy->getPointerTo();
    StructType * const consumerTy = StructType::get(b->getContext(), {sizeTy, sizePtrPtrTy});
    for (unsigned i = 0; i < mStreamSetOutputBuffers.size(); ++i) {
        const auto output = mStreamSetOutputBuffers[i];
        const auto & consumers = output->getConsumers();
        const auto n = consumers.size();
        AllocaInst * const outputConsumers = b->CreateAlloca(consumerTy);
        Value * const consumerSegNoArray = b->CreateAlloca(ArrayType::get(sizePtrTy, n));
        for (unsigned i = 0; i < n; ++i) {
            Kernel * const consumer = consumers[i];
            assert ("all instances must be created prior to initialization of any instance" && consumer->getInstance());
            b->setKernel(consumer);
            Value * const segmentNoPtr = b->getScalarFieldPtr(LOGICAL_SEGMENT_NO_SCALAR);
            b->CreateStore(segmentNoPtr, b->CreateGEP(consumerSegNoArray, { b->getInt32(0), b->getInt32(i) }));
        }
        b->setKernel(this);
        Value * const consumerCountPtr = b->CreateGEP(outputConsumers, {b->getInt32(0), b->getInt32(0)});
        b->CreateStore(b->getSize(n), consumerCountPtr);
        Value * const consumerSegNoArrayPtr = b->CreateGEP(outputConsumers, {b->getInt32(0), b->getInt32(1)});
        b->CreateStore(b->CreatePointerCast(consumerSegNoArray, sizePtrPtrTy), consumerSegNoArrayPtr);
        args.push_back(outputConsumers);
    }
    b->CreateCall(getInitFunction(b->getModule()), args);
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
        assert (!mStreamMap.empty());
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
 * @brief getLowerBound
 ** ------------------------------------------------------------------------------------------------------------- */
ProcessingRate::RateValue Kernel::getLowerBound(const ProcessingRate & rate) const {
    if (rate.isFixed() || rate.isBounded()) {
        return rate.getLowerBound();
    } else if (rate.hasReference()) {
        return rate.getLowerBound() * getLowerBound(getBinding(rate.getReference()).getRate());
    } else { // if (rate.isUnknown())
        return 0;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getUpperBound
 ** ------------------------------------------------------------------------------------------------------------- */
ProcessingRate::RateValue Kernel::getUpperBound(const ProcessingRate &rate) const {
    if (rate.isFixed() || rate.isBounded() || rate.isPopCount()) {
        return rate.getUpperBound();
    } else if (rate.hasReference()) {
        return rate.getUpperBound() * getUpperBound(getBinding(rate.getReference()).getRate());
    } else { // if (rate.isUnknown())
        return 0;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyStreamSetDefinitions
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::verifyStreamSetDefinitions() const {
    unsigned numOfPrincipalStreams = 0;
    for (unsigned i = 0; i < mStreamSetInputs.size(); ++i) {
        const Binding & input = mStreamSetInputs[i];
        const ProcessingRate & rate = input.getRate();
        // If a stream can be relative to a relative or fixed rate stream, it complicates the pipeline and
        // multiblock kernel. For now, report an error.
        if (LLVM_UNLIKELY(rate.hasReference())) {
            Port port; unsigned index;
            std::tie(port, index) = getStreamPort(rate.getReference());
            if (LLVM_UNLIKELY(port == Port::Output)) {
                report_fatal_error(getName() + ": input stream \"" + input.getName() + "\" cannot refer to an output stream");
            }
            if (LLVM_UNLIKELY(index >= i)) {
                report_fatal_error(getName() + ": \"" + input.getName() + "\" must be ordered after its reference stream");
            }
            if (rate.isRelative()) {
                const ProcessingRate & refRate = getStreamInput(index).getRate();
                if (LLVM_UNLIKELY(refRate.isRelative() || refRate.isFixed())) {
                    report_fatal_error(getName() + ": \"" + input.getName() + "\" cannot be relative to a fixed or relative rate stream");
                }
            }
        } else if (LLVM_UNLIKELY(rate.isUnknown())) {
            report_fatal_error(getName() + ": \"" + input.getName() + "\" cannot be an unknown rate");
        }
        if (LLVM_UNLIKELY(input.isPrincipal())) {
            ++numOfPrincipalStreams;
        }
        bool hasFixedOnlyAttribute = false;
        for (const Attribute & attr : input.getAttributes()) {
            switch (attr.getKind()) {
                case Attribute::KindId::Add:
                case Attribute::KindId::RoundUpTo:
                case Attribute::KindId::Deferred:
                    hasFixedOnlyAttribute = false;
                    break;
                default: break;
            }
        }
        if (rate.isFixed()) {



        } else if (LLVM_UNLIKELY(hasFixedOnlyAttribute)) {
            report_fatal_error(getName() + ": Add, RoundUpTo and Deferred cannot be applied to non-Fixed rate input stream \"" + input.getName() + "\"");
        }
    }
    if (LLVM_UNLIKELY(numOfPrincipalStreams > 1)) {
        report_fatal_error(getName() + ": may only have one principal stream set");
    }
    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
        const Binding & output = mStreamSetOutputs[i];
        const ProcessingRate & rate = output.getRate();
        if (LLVM_UNLIKELY(rate.hasReference())) {
            Port port; unsigned index;
            std::tie(port, index) = getStreamPort(rate.getReference());
            if (LLVM_UNLIKELY(rate.isPopCount() && port == Port::Output)) {
                report_fatal_error(getName() + ": the popcount rate of \"" + output.getName() + "\" cannot refer to another output stream");
            }
            if (LLVM_UNLIKELY(port == Port::Output && index >= i)) {
                report_fatal_error(getName() + ": \"" + output.getName() + "\" must be ordered after its reference stream");
            }
            if (rate.isRelative()) {
                const Binding & ref = (port == Port::Input) ? getStreamInput(index) : getStreamOutput(index);
                const ProcessingRate & refRate = ref.getRate();
                if (LLVM_UNLIKELY(refRate.isRelative() || refRate.isFixed())) {
                    report_fatal_error(getName() + ": \"" + output.getName() + "\" cannot be relative to a fixed or relative rate stream");
                }
            }
        }
        if (LLVM_UNLIKELY(output.isPrincipal())) {
            report_fatal_error(getName() + ": output stream \"" + output.getName() + "\" cannot be a principal stream");
        }

        bool hasAddOrRoundUpTo = false;
        bool hasDeferred = false;
        for (const Attribute & attr : output.getAttributes()) {
            switch (attr.getKind()) {
                case Attribute::KindId::Add:
                case Attribute::KindId::RoundUpTo:
                    hasAddOrRoundUpTo = true;
                    break;
                case Attribute::KindId::Deferred:
                    hasDeferred = false;
                    break;
                default: break;
            }
        }

        if (LLVM_UNLIKELY((hasAddOrRoundUpTo || hasDeferred) && !(rate.isFixed() || rate.isPopCount()))) {
            report_fatal_error(getName() + ": " + output.getName() + " cannot have an Add, RoundUpTo or Deferred attribute");
        }       
        if (LLVM_UNLIKELY(hasDeferred && hasAddOrRoundUpTo)) {
            report_fatal_error(getName() + ": cannot apply Add or RoundUpTo attributes to the Deferred output stream " + output.getName());
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyBufferSize
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::verifyBufferSize(const Binding & binding, const StreamSetBuffer * const buffer) const {
    if (LLVM_UNLIKELY(isa<SourceBuffer>(buffer) || isa<ExternalBuffer>(buffer))) {
        return true;
    }
    const ProcessingRate & rate = binding.getRate();
    if (requiresCopyBack(binding)) {
        const auto minOverflow = ceiling(rate.getUpperBound());
        if (LLVM_UNLIKELY(buffer->overflowSize() < minOverflow)) {
            report_fatal_error(getName() + ": " + binding.getName() + " requires " +
                               std::to_string(minOverflow) + " overflow blocks");
        }
    } else if (rate.isFixed() || binding.hasAttribute(Attribute::KindId::BlockSize)) {
        const auto r = rate.getUpperBound();
        if (LLVM_LIKELY(r.denominator() == 1)) {
            if (LLVM_UNLIKELY((buffer->getBufferBlocks() % r.numerator())) != 0) {
                report_fatal_error(getName() + ": " + binding.getName() + " requires a multiple of " +
                                   std::to_string(r.numerator()) + " buffer blocks");
                return false;
            }
        } else { // if (b % (n/d) != 0)
            const auto b = buffer->getBufferBlocks();
            const auto x = (b * r.denominator()) / r.numerator();
            if (LLVM_UNLIKELY((b * r.denominator()) != (r.numerator() * x))) {
                report_fatal_error(getName() + ": " + binding.getName() + " requires a multiple of " +
                                   std::to_string(r.numerator()) + "/" + std::to_string(r.denominator()) + " buffer blocks");
                return false;
            }
        }
    }
    return true;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresCopyBack
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::requiresCopyBack(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || binding.hasAttribute(Attribute::KindId::BlockSize)) {
        return false;
    } else if (rate.isRelative()) {
        return requiresCopyBack(getBinding(rate.getReference()));
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresLinearAccess
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::requiresLinearAccess(const Binding & binding) const {
    return binding.hasAttribute(Attribute::KindId::RequiresLinearAccess);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief strideOffsetIsTriviallyCalculable
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::strideOffsetIsTriviallyCalculable(const Binding & binding) const {
    if (requiresCopyBack(binding)) {
        const ProcessingRate & rate = binding.getRate();
        return rate.isPopCount() || rate.isNegatedPopCount();
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief permitsNonLinearAccess
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::permitsNonLinearAccess(const Binding & binding) const {
    if (LLVM_UNLIKELY(requiresLinearAccess(binding))) {
        return false;
    } else if (LLVM_UNLIKELY(binding.hasAttribute(Attribute::KindId::PermitsNonLinearAccess))) {
        return true;
    } else {
        return strideOffsetIsTriviallyCalculable(binding);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief mustClearOverflowPriorToCopyback
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::mustClearOverflowPriorToCopyback(const Binding & binding) const {
    return requiresCopyBack(binding) && permitsNonLinearAccess(binding) && !strideOffsetIsTriviallyCalculable(binding);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief anyBindingRequiresLinearSpace
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::anyBindingRequiresLinearSpace() const {
    for (const Binding & input : mStreamSetInputs) {
        if (requiresLinearAccess(input)) {
            return true;
        }
    }
    for (const Binding & output : mStreamSetOutputs) {
        if (!permitsNonLinearAccess(output)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void SegmentOrientedKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    mTreatUnsafeKernelOperationsAsErrors = false;
    generateDoSegmentMethod(b);
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
, mStride(0)
, mTreatUnsafeKernelOperationsAsErrors(false)
, mIsFinal(nullptr)
, mOutputScalarResult(nullptr)
, mIsGenerated(false) {

}

Kernel::~Kernel() {

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
