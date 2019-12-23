/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/core/kernel.h>
#include <kernel/core/kernel_compiler.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/Format.h>
#include <util/sha1.hpp>

using namespace llvm;
using namespace boost;

namespace kernel {

using AttrId = Attribute::KindId;
using Rational = ProcessingRate::Rational;
using RateId = ProcessingRate::KindId;
using StreamSetPort = Kernel::StreamSetPort;
using KernelCompilerRef = Kernel::KernelCompilerRef;
using PortType = Kernel::PortType;

const static auto INITIALIZE_SUFFIX = "_Initialize";
const static auto INITIALIZE_THREAD_LOCAL_SUFFIX = "_InitializeThreadLocal";
const static auto DO_SEGMENT_SUFFIX = "_DoSegment";
const static auto FINALIZE_THREAD_LOCAL_SUFFIX = "_FinalizeThreadLocal";
const static auto FINALIZE_SUFFIX = "_Finalize";

const static auto SHARED_SUFFIX = "_shared_state";
const static auto THREAD_LOCAL_SUFFIX = "_thread_local";

#define BEGIN_SCOPED_REGION {
#define END_SCOPED_REGION }

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isLocalBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
/* static */ bool Kernel::isLocalBuffer(const Binding & output) {
    return output.getRate().isUnknown() || output.hasAttribute(AttrId::ManagedBuffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresExplicitPartialFinalStride
 ** ------------------------------------------------------------------------------------------------------------- */
/* static */ bool Kernel::requiresExplicitPartialFinalStride(const Kernel * const kernel) {
    if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::InternallySynchronized))) {
        return false;
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasFixedRate
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::hasFixedRate() const {
    const auto n = getNumOfStreamInputs();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            return true;
        }
    }
    const auto m = getNumOfStreamOutputs();
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFixedRateLCM
 ** ------------------------------------------------------------------------------------------------------------- */
Rational Kernel::getFixedRateLCM() const {
    Rational rateLCM(1);
    bool hasFixedRate = false;
    const auto n = getNumOfStreamInputs();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRate = true;
        }
    }
    const auto m = getNumOfStreamOutputs();
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRate = true;
        }
    }
    return hasFixedRate ? rateLCM : Rational{0};
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief canSetTerminateSignal
 ** ------------------------------------------------------------------------------------------------------------ */
bool Kernel::canSetTerminateSignal() const {
    for (const Attribute & attr : getAttributes()) {
        switch (attr.getKind()) {
            case AttrId::MayFatallyTerminate:
            case AttrId::CanTerminateEarly:
            case AttrId::MustExplicitlyTerminate:
                return true;
            default: continue;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCacheName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getCacheName(BuilderRef b) const {
    std::stringstream cacheName;
    cacheName << getName() << '_' << b->getBuilderUniqueName();
    return cacheName.str();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiateKernelCompiler
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<KernelCompiler> Kernel::instantiateKernelCompiler(BuilderRef /* b */) const noexcept {
    return llvm::make_unique<KernelCompiler>(const_cast<Kernel *>(this));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeModule
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::makeModule(BuilderRef b) {
    Module * const m = new Module(getCacheName(b), b->getContext());
    Module * const prior = b->getModule();
    m->setTargetTriple(prior->getTargetTriple());
    m->setDataLayout(prior->getDataLayout());
    mModule = m;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::generateKernel(BuilderRef b) {
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        report_fatal_error(getName() + " does not have a module");
    }
    if (LLVM_UNLIKELY(mStride == 0)) {
        report_fatal_error(getName() + ": stride cannot be 0");
    }
    b->setModule(mModule);
    instantiateKernelCompiler(b)->generateKernel(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief nullIfEmpty
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * nullIfEmpty(StructType * type) {
    return (type && type->isEmptyTy()) ? nullptr : type;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::loadCachedKernel(BuilderRef b) {
    if (LLVM_UNLIKELY(mModule != nullptr)) {
        report_fatal_error("loadCachedKernel was called after associating " + getName() + " with a module");
    }
    Module * const m = b->getModule(); assert (m);
    mSharedStateType = nullIfEmpty(m->getTypeByName(getName() + SHARED_SUFFIX));
    mThreadLocalStateType = nullIfEmpty(m->getTypeByName(getName() + THREAD_LOCAL_SUFFIX));
    mModule = m;
    linkExternalMethods(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkExternalMethods
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::linkExternalMethods(BuilderRef b) {
    auto & driver = b->getDriver();
    Module * const m = b->getModule(); assert (m);
    for (const LinkedFunction & linked : mLinkedFunctions) {
        driver.addLinkFunction(m, linked.Name, linked.Type, linked.FunctionPtr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructStateTypes
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::constructStateTypes(BuilderRef b) {
    Module * const m = getModule(); assert (b->getModule() == m);
    mSharedStateType = m->getTypeByName(getName() + SHARED_SUFFIX);
    mThreadLocalStateType = m->getTypeByName(getName() + THREAD_LOCAL_SUFFIX);
    if (LLVM_LIKELY(mSharedStateType == nullptr && mThreadLocalStateType == nullptr)) {
        SmallVector<Type *, 64> shared;
        SmallVector<Type *, 64> threadLocal;
        shared.reserve(mInputScalars.size() + mOutputScalars.size() + mInternalScalars.size());
        for (const auto & scalar : mInputScalars) {
            assert (scalar.getType());
            shared.push_back(scalar.getType());
        }
        for (const auto & scalar : mOutputScalars) {
            assert (scalar.getType());
            shared.push_back(scalar.getType());
        }
        // TODO: make "grouped" internal scalars that are automatically packed into cache-aligned structs
        // within the kernel state to hide the complexity from the user?
        for (const auto & scalar : mInternalScalars) {
            assert (scalar.getValueType());
            switch (scalar.getScalarType()) {
                case ScalarType::Internal:
                    shared.push_back(scalar.getValueType());
                    break;
                case ScalarType::ThreadLocal:
                    threadLocal.push_back(scalar.getValueType());
                    break;
                default: break;
            }
        }
        // NOTE: StructType::create always creates a new type even if an identical one exists.
        StructType * const sharedTy = StructType::create(b->getContext(), shared, getName() + SHARED_SUFFIX);
        assert (mSharedStateType == nullptr || mSharedStateType == nullIfEmpty(sharedTy));
        mSharedStateType = sharedTy;

        StructType * const threadLocalTy = StructType::create(b->getContext(), threadLocal, getName() + THREAD_LOCAL_SUFFIX);
        assert (mThreadLocalStateType == nullptr || mThreadLocalStateType == nullIfEmpty(threadLocalTy));
        mThreadLocalStateType = threadLocalTy;
    }
    mSharedStateType = nullIfEmpty(mSharedStateType);
    mThreadLocalStateType = nullIfEmpty(mThreadLocalStateType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addKernelDeclarations(BuilderRef b) {
    addInitializeDeclaration(b);
    addInitializeThreadLocalDeclaration(b);
    addDoSegmentDeclaration(b);
    addFinalizeThreadLocalDeclaration(b);
    addFinalizeDeclaration(b);
    linkExternalMethods(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getInitializeFunction(BuilderRef b, const bool alwayReturnDeclaration) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + INITIALIZE_SUFFIX);
    if (LLVM_UNLIKELY(f == nullptr && alwayReturnDeclaration)) {
        f = addInitializeDeclaration(b);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addInitializeDeclaration(BuilderRef b) const {
    const auto funcName = getName() + INITIALIZE_SUFFIX;
    Module * const m = b->getModule();
    Function * initFunc = m->getFunction(funcName);
    if (LLVM_LIKELY(initFunc == nullptr)) {

        InitArgTypes params;
        if (LLVM_LIKELY(isStateful())) {
            params.push_back(getSharedStateType()->getPointerTo());
        }
        for (const Binding & binding : mInputScalars) {
            params.push_back(binding.getType());
        }
        addFamilyInitializationArgTypes(b, params);
        FunctionType * const initType = FunctionType::get(b->getSizeTy(), params, false);
        initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, funcName, m);
        initFunc->setCallingConv(CallingConv::C);
        initFunc->setDoesNotRecurse();

        auto arg = initFunc->arg_begin();
        auto setNextArgName = [&](const StringRef name) {
            assert (arg != initFunc->arg_end());
            arg->setName(name);
            std::advance(arg, 1);
        };
        if (LLVM_LIKELY(isStateful())) {
            setNextArgName("shared");
        }
        for (const Binding & binding : mInputScalars) {
            setNextArgName(binding.getName());
        }
    }
    return initFunc;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyInitializationArgTypes
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addFamilyInitializationArgTypes(BuilderRef /* b */, InitArgTypes & /* argTypes */) const {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializeThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getInitializeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration) const {
    assert (hasThreadLocal());
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + INITIALIZE_THREAD_LOCAL_SUFFIX);
    if (LLVM_UNLIKELY(f == nullptr && alwayReturnDeclaration)) {
        f = addInitializeThreadLocalDeclaration(b);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeThreadLocalDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addInitializeThreadLocalDeclaration(BuilderRef b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {
        const auto funcName = getName() + INITIALIZE_THREAD_LOCAL_SUFFIX;
        Module * const m = b->getModule();
        func = m->getFunction(funcName);
        if (LLVM_LIKELY(func == nullptr)) {

            SmallVector<Type *, 1> params;
            if (LLVM_LIKELY(isStateful())) {
                params.push_back(getSharedStateType()->getPointerTo());
            }
            PointerType * const retTy = getThreadLocalStateType()->getPointerTo();
            FunctionType * const funcType = FunctionType::get(retTy, params, false);
            func = Function::Create(funcType, GlobalValue::ExternalLinkage, funcName, m);
            func->setCallingConv(CallingConv::C);
            func->setDoesNotRecurse();

            auto arg = func->arg_begin();
            auto setNextArgName = [&](const StringRef name) {
                assert (arg != func->arg_end());
                arg->setName(name);
                std::advance(arg, 1);
            };
            if (LLVM_LIKELY(isStateful())) {
                setNextArgName("shared");
            }
            assert (arg == func->arg_end());
        }
        assert (func);
    }
    return func;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFields
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Type *> Kernel::getDoSegmentFields(BuilderRef b) const {

    // WARNING: any change to this must be reflected in addDoSegmentDeclaration,
    // KernelCompiler::getDoSegmentProperties, KernelCompiler::setDoSegmentProperties,
    // and PipelineCompiler::writeKernelCall

    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    const auto n = mInputStreamSets.size();
    const auto m = mOutputStreamSets.size();

    std::vector<Type *> fields;
    fields.reserve(4 + 3 * (n + m));

    if (LLVM_LIKELY(isStateful())) {
        fields.push_back(getSharedStateType()->getPointerTo());  // handle
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        fields.push_back(getThreadLocalStateType()->getPointerTo());  // handle
    }
    fields.push_back(sizeTy); // numOfStrides
    if (LLVM_LIKELY(!requiresExplicitPartialFinalStride(this))) {
        fields.push_back(b->getInt1Ty()); // isFinal
    }
    if (LLVM_UNLIKELY(supportsInternalSynchronization())) {
        fields.push_back(sizeTy); // external segNo for any internal synchronization
    }
    if (LLVM_LIKELY(hasFixedRate())) {
        fields.push_back(sizeTy); // fixedRateFactor
    }
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = mInputStreamSets[i];
        auto const bufferType = StreamSetBuffer::resolveType(b, input.getType())->getPointerTo();
        // virtual base input address
        fields.push_back(bufferType);
        // processed input items
        if (isAddressable(input)) {
            fields.push_back(sizePtrTy); // updatable
        } else if (isCountable(input)) {
            fields.push_back(sizeTy); // constant
        }
        // accessible input items
        if (requiresItemCount(input)) {
            fields.push_back(sizeTy);
        }
    }

    const auto hasTerminationSignal = canSetTerminateSignal();

    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = mOutputStreamSets[i];
        // virtual base output address
        const auto isLocal = isLocalBuffer(output);
        auto const bufferType = StreamSetBuffer::resolveType(b, output.getType())->getPointerTo();
        auto bufferParamType = bufferType;
        if (LLVM_UNLIKELY(isLocal)) {
            // If this buffer is owned by this kernel, it is responsible for
            // informing the calling kernel what its virtual base address is.
            bufferParamType = bufferType->getPointerTo();
        }
        fields.push_back(bufferParamType);
        // produced output items
        if (hasTerminationSignal|| isAddressable(output)) {
            fields.push_back(sizePtrTy); // updatable
        } else if (isCountable(output)) {
            fields.push_back(sizeTy); // constant
        }
        // writable / requested item count
        if (requiresItemCount(output)) {
            fields.push_back(sizeTy);
        }
        // consumed item count
        if (LLVM_UNLIKELY(isLocal)) {
            fields.push_back(sizeTy);
        }
    }
    return fields;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addDoSegmentDeclaration
 ** ------------------------------------------------------------------------------------------------------------ */
Function * Kernel::addDoSegmentDeclaration(BuilderRef b) const {

    // WARNING: any change to this must be reflected in getDoSegmentProperties, setDoSegmentProperties,
    // getDoSegmentFields, and PipelineCompiler::writeKernelCall

    const auto funcName = getName() + DO_SEGMENT_SUFFIX;
    Module * const m = b->getModule();
    Function * doSegment = m->getFunction(funcName);
    if (LLVM_LIKELY(doSegment == nullptr)) {

        Type * const retTy = canSetTerminateSignal() ? b->getSizeTy() : b->getVoidTy();
        FunctionType * const doSegmentType = FunctionType::get(retTy, getDoSegmentFields(b), false);
        doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, funcName, m);
        doSegment->setCallingConv(CallingConv::C);
        doSegment->setDoesNotRecurse();

        auto arg = doSegment->arg_begin();
        auto setNextArgName = [&](const StringRef name) {
            assert (arg != doSegment->arg_end());
            arg->setName(name);
            std::advance(arg, 1);
        };
        if (LLVM_LIKELY(isStateful())) {
            setNextArgName("shared");
        }
        if (LLVM_UNLIKELY(hasThreadLocal())) {
            setNextArgName("thread_local");
        }
        setNextArgName("numOfStrides");
        if (LLVM_LIKELY(!requiresExplicitPartialFinalStride(this))) {
            setNextArgName("isFinal");
        }
        if (hasAttribute(AttrId::InternallySynchronized)) {
            setNextArgName("externalSegNo");
        }
        if (LLVM_LIKELY(hasFixedRate())) {
            setNextArgName("fixedRateFactor");
        }
        for (unsigned i = 0; i < mInputStreamSets.size(); ++i) {
            const Binding & input = mInputStreamSets[i];
            setNextArgName(input.getName());
            if (LLVM_LIKELY(isAddressable(input) || isCountable(input))) {
                setNextArgName(input.getName() + "_processed");
            }
            if (requiresItemCount(input)) {
                setNextArgName(input.getName() + "_accessible");
            }
        }

        const auto hasTerminationSignal = canSetTerminateSignal();

        for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
            const Binding & output = mOutputStreamSets[i];
            setNextArgName(output.getName());
            if (LLVM_LIKELY(hasTerminationSignal || isAddressable(output) || isCountable(output))) {
                setNextArgName(output.getName() + "_produced");
            }
            if (requiresItemCount(output)) {
                setNextArgName(output.getName() + "_writable");
            }
            if (LLVM_UNLIKELY(isLocalBuffer(output))) {
                setNextArgName(output.getName() + "_consumed");
            }
        }
        assert (arg == doSegment->arg_end());
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getDoSegmentFunction(BuilderRef b, const bool alwayReturnDeclaration) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + DO_SEGMENT_SUFFIX);
    if (LLVM_UNLIKELY(f == nullptr && alwayReturnDeclaration)) {
        f = addDoSegmentDeclaration(b);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getFinalizeThreadLocalFunction(BuilderRef b, const bool alwayReturnDeclaration) const {
    assert (hasThreadLocal());
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + FINALIZE_THREAD_LOCAL_SUFFIX);
    if (LLVM_UNLIKELY(f == nullptr && alwayReturnDeclaration)) {
        f = addFinalizeThreadLocalDeclaration(b);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeThreadLocalDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addFinalizeThreadLocalDeclaration(BuilderRef b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {
        const auto funcName = getName() + FINALIZE_THREAD_LOCAL_SUFFIX;
        Module * const m = b->getModule();
        func = m->getFunction(funcName);
        if (LLVM_LIKELY(func == nullptr)) {
            SmallVector<Type *, 2> params;
            if (LLVM_LIKELY(isStateful())) {
                params.push_back(getSharedStateType()->getPointerTo());
            }
            params.push_back(getThreadLocalStateType()->getPointerTo());

            FunctionType * const funcType = FunctionType::get(b->getVoidTy(), params, false);
            func = Function::Create(funcType, GlobalValue::ExternalLinkage, funcName, m);
            func->setCallingConv(CallingConv::C);
            func->setDoesNotRecurse();

            auto arg = func->arg_begin();
            auto setNextArgName = [&](const StringRef name) {
                assert (arg != func->arg_end());
                arg->setName(name);
                std::advance(arg, 1);
            };
            if (LLVM_LIKELY(isStateful())) {
                setNextArgName("shared");
            }
            setNextArgName("thread_local");
            assert (arg == func->arg_end());

        }
    }
    return func;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getFinalizeFunction(BuilderRef b, const bool alwayReturnDeclaration) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + FINALIZE_SUFFIX);
    if (LLVM_UNLIKELY(f == nullptr && alwayReturnDeclaration)) {
        f = addFinalizeDeclaration(b);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addFinalizeDeclaration(BuilderRef b) const {
    const auto funcName = getName() + FINALIZE_SUFFIX;
    Module * const m = b->getModule();
    Function * terminateFunc = m->getFunction(funcName);
    if (LLVM_LIKELY(terminateFunc == nullptr)) {
        Type * resultType = nullptr;
        if (mOutputScalars.empty()) {
            resultType = b->getVoidTy();
        } else {
            const auto n = mOutputScalars.size();
            SmallVector<Type *, 16> outputType(n);
            for (unsigned i = 0; i < n; ++i) {
                outputType[i] = mOutputScalars[i].getType();
            }
            if (n == 1) {
                resultType = outputType[0];
            } else {
                resultType = StructType::get(b->getContext(), outputType);
            }
        }
        std::vector<Type *> params;
        if (LLVM_LIKELY(isStateful())) {
            params.push_back(getSharedStateType()->getPointerTo());
        }
        FunctionType * const terminateType = FunctionType::get(resultType, params, false);
        terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, funcName, m);
        terminateFunc->setCallingConv(CallingConv::C);
        terminateFunc->setDoesNotRecurse();

        auto args = terminateFunc->arg_begin();
        if (LLVM_LIKELY(isStateful())) {
            (args++)->setName("handle");
        }
        assert (args == terminateFunc->arg_end());
    }
    return terminateFunc;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOrDeclareMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addOrDeclareMainFunction(BuilderRef b, const MainMethodGenerationType method) {

    addKernelDeclarations(b);

    unsigned suppliedArgs = 1;
    if (LLVM_LIKELY(isStateful())) {
        ++suppliedArgs;
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        ++suppliedArgs;
    }

    Module * const m = b->getModule();
    Function * const doSegment = getDoSegmentFunction(b);
    assert (doSegment->arg_size() >= suppliedArgs);
    const auto numOfDoSegArgs = doSegment->arg_size() - suppliedArgs;
    Function * const terminate = getFinalizeFunction(b);

    // maintain consistency with the Kernel interface by passing first the stream sets
    // and then the scalars.
    SmallVector<Type *, 32> params;
    params.reserve(numOfDoSegArgs + getNumOfScalarInputs());

    // The initial params of doSegment are its shared handle, thread-local handle and numOfStrides.
    // (assuming the kernel has both handles). The remaining are the stream set params
    auto doSegParam = doSegment->arg_begin();
    std::advance(doSegParam, suppliedArgs);
    const auto doSegEnd = doSegment->arg_end();
    while (doSegParam != doSegEnd) {
        params.push_back(doSegParam->getType());
        std::advance(doSegParam, 1);
    }
    for (const auto & input : getInputScalarBindings()) {
        params.push_back(input.getType());
    }

    // get the finalize method output type and set its return type as this function's return type
    FunctionType * const mainFunctionType = FunctionType::get(terminate->getReturnType(), params, false);

    const auto linkageType = (method == AddInternal) ? Function::InternalLinkage : Function::ExternalLinkage;

    Function * const main = Function::Create(mainFunctionType, linkageType, getName() + "_main", m);
    main->setCallingConv(CallingConv::C);

    // declaration only; exit
    if (method == DeclareExternal) {
        return main;
    }

    if (LLVM_UNLIKELY(hasAttribute(AttrId::InternallySynchronized))) {
        report_fatal_error("main cannot be externally synchronized");
    }

    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", main));
    auto arg = main->arg_begin();
    auto nextArg = [&]() {
        assert (arg != main->arg_end());
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };

    SmallVector<Value *, 16> segmentArgs(doSegment->arg_size());
    segmentArgs[suppliedArgs - 1] = b->getSize(1); // numOfStrides
    for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
        segmentArgs[suppliedArgs + i] = nextArg();
    }

    Value * sharedHandle = nullptr;
    BEGIN_SCOPED_REGION
    ParamMap paramMap;
    for (const auto & input : getInputScalarBindings()) {
        paramMap.insert(std::make_pair(cast<Scalar>(input.getRelationship()), nextArg()));
    }
    assert (arg == main->arg_end());
    InitArgs args;
    sharedHandle = constructFamilyKernels(b, args, paramMap);
    END_SCOPED_REGION

    Value * threadLocalHandle = nullptr;
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        threadLocalHandle = initializeThreadLocalInstance(b, sharedHandle);
    }
    if (LLVM_LIKELY(isStateful())) {
        segmentArgs[0] = sharedHandle;
    }
    if (hasThreadLocal()) {
        segmentArgs[suppliedArgs - 2] = threadLocalHandle;
    }


    #ifdef NDEBUG
    const bool ea = true;
    #else
    const bool ea = codegen::DebugOptionIsSet(codegen::EnableAsserts);
    #endif

    PHINode * successPhi = nullptr;
    if (LLVM_UNLIKELY(ea)) {
        BasicBlock * const handleCatch = b->CreateBasicBlock("");
        BasicBlock * const handleDeallocation = b->CreateBasicBlock("");

        IntegerType * const int32Ty = b->getInt32Ty();
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        LLVMContext & C = b->getContext();
        StructType * const caughtResultType = StructType::get(C, { int8PtrTy, int32Ty });
        Function * const personalityFn = b->getDefaultPersonalityFunction();
        main->setPersonalityFn(personalityFn);

        BasicBlock * const beforeInvoke = b->GetInsertBlock();
        b->CreateInvoke(doSegment, handleDeallocation, handleCatch, segmentArgs);

        b->SetInsertPoint(handleCatch);
        LandingPadInst * const caughtResult = b->CreateLandingPad(caughtResultType, 0);
        caughtResult->addClause(ConstantPointerNull::get(int8PtrTy));
        b->CreateCall(b->getBeginCatch(), {b->CreateExtractValue(caughtResult, 0)});
        b->CreateCall(b->getEndCatch());
        BasicBlock * const afterCatch = b->GetInsertBlock();
        b->CreateBr(handleDeallocation);

        b->SetInsertPoint(handleDeallocation);
        successPhi = b->CreatePHI(b->getInt1Ty(), 2);
        successPhi->addIncoming(b->getTrue(), beforeInvoke);
        successPhi->addIncoming(b->getFalse(), afterCatch);
    } else {
        b->CreateCall(doSegment, segmentArgs);
    }
    if (hasThreadLocal()) {
        SmallVector<Value *, 2> args;
        if (LLVM_LIKELY(isStateful())) {
            args.push_back(sharedHandle);
        }
        args.push_back(threadLocalHandle);
        finalizeThreadLocalInstance(b, args);
    }
    if (successPhi) {
        BasicBlock * const exitProgram = b->CreateBasicBlock("exitProgram");
        BasicBlock * const resumeProgram = b->CreateBasicBlock("return");
        b->CreateLikelyCondBr(successPhi, resumeProgram, exitProgram);
        // if we have any error, just exit(-1)
        b->SetInsertPoint(exitProgram);
        b->CreateExit(-1);
        b->CreateBr(resumeProgram);
        b->SetInsertPoint(resumeProgram);
    }
    if (isStateful()) {
        // call and return the final output value(s)
        Value * const retVal = finalizeInstance(b, sharedHandle);
        b->CreateRet(retVal);
    } else {
        b->CreateRetVoid();
    }
    return main;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::createInstance(BuilderRef b) const {
    if (isStateful()) {
        Constant * const size = ConstantExpr::getSizeOf(getSharedStateType());
        Value * handle = nullptr;
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            handle = b->CreateAlignedMalloc(size, b->getPageSize());
            b->CreateMProtect(handle, size, CBuilder::Protect::READ);
        } else {
            handle = b->CreateAlignedMalloc(size, b->getCacheAlignment());
        }
        return b->CreatePointerCast(handle, getSharedStateType()->getPointerTo());
    }
    llvm_unreachable("createInstance should not be called on stateless kernels");
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeInstance(BuilderRef b, llvm::ArrayRef<Value *> args) {
    assert (args.size() == getNumOfScalarInputs() + 1);
    assert (args[0] && "cannot initialize before creation");
    assert (args[0]->getType()->getPointerElementType() == getSharedStateType());
    Function * const init = getInitializeFunction(b);
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeThreadLocalInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::initializeThreadLocalInstance(BuilderRef b, Value * const handle) {
    Value * instance = nullptr;
    if (hasThreadLocal()) {
        Function * const init = getInitializeThreadLocalFunction(b);
        if (handle) {
            instance = b->CreateCall(init, handle);
        } else {
            instance = b->CreateCall(init);
        }
    }
    return instance;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeThreadLocalInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::finalizeThreadLocalInstance(BuilderRef b, llvm::ArrayRef<Value *> args) const {
    assert (args.size() == (isStateful() ? 2 : 1));
    Function * const init = getFinalizeThreadLocalFunction(b); assert (init);
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::finalizeInstance(BuilderRef b, Value * const handle) const {
    Value * result = nullptr;
    Function * const termFunc = getFinalizeFunction(b);
    if (LLVM_LIKELY(isStateful())) {
        result = b->CreateCall(termFunc, { handle });
    } else {
        result = b->CreateCall(termFunc);
    }
    if (mOutputScalars.empty()) {
        assert (!result || result->getType()->isVoidTy());
        result = nullptr;
    }
    return result;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructFamilyKernels
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::constructFamilyKernels(BuilderRef b, InitArgs & hostArgs, const ParamMap & params) const {

    #warning TODO: need to test for termination on init call

    PointerType * const voidPtrTy = b->getVoidPtrTy();
    auto addHostArg = [&](Value * ptr) {
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(ptr, "constructFamilyKernels cannot pass a null value to pipeline");
        }
        hostArgs.push_back(b->CreatePointerCast(ptr, voidPtrTy));
    };

    Value * handle = nullptr;

    BEGIN_SCOPED_REGION
    InitArgs initArgs;
    if (LLVM_LIKELY(isStateful())) {
        handle = createInstance(b);
        initArgs.push_back(handle);
        addHostArg(handle);
    }
    for (const Binding & input : mInputScalars) {
        const auto f = params.find(cast<Scalar>(input.getRelationship()));
        if (LLVM_UNLIKELY(f == params.end())) {
            SmallVector<char, 512> tmp;
            raw_svector_ostream out(tmp);
            out << "Could not find paramater for " << getName() << ':' << input.getName()
                << " from the provided program parameters (i.e., unknown input scalar.)";
            report_fatal_error(out.str());
        }
        initArgs.push_back(f->second); assert (initArgs.back());
    }
    recursivelyConstructFamilyKernels(b, initArgs, params);
    b->CreateCall(getInitializeFunction(b), initArgs);
    END_SCOPED_REGION

    if (LLVM_LIKELY(hasFamilyName())) {
        if (hasThreadLocal()) {
            addHostArg(getInitializeThreadLocalFunction(b));
        }
        addHostArg(getDoSegmentFunction(b));
        if (hasThreadLocal()) {
            addHostArg(getFinalizeThreadLocalFunction(b));
        }
        addHostArg(getFinalizeFunction(b));
    }
    return handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recursivelyConstructFamilyKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::recursivelyConstructFamilyKernels(BuilderRef /* b */, InitArgs & /* args */, const ParamMap & /* params */) const {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeSignature
 *
 * Default kernel signature: generate the IR and emit as byte code.
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::makeSignature(BuilderRef /* b */) const {
    if (LLVM_UNLIKELY(hasSignature())) {
        llvm::report_fatal_error(getName() + " should have overridden makeSignature");
    } else {
        return getModule()->getModuleIdentifier();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStringHash
 *
 * Create a fixed length string hash of the given str
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getStringHash(const StringRef str) {

    uint32_t digest[5]; // 160 bits in total
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.data(), str.size());
    sha1.get_digest(digest);

    std::string buffer;
    buffer.reserve((5 * 8) + 1);
    raw_string_ostream out(buffer);
    for (unsigned i = 0; i < 5; ++i) {
        out << format_hex_no_prefix(digest[i], 8);
    }
    out.flush();

    return buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief supportsInternalSynchronization
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::supportsInternalSynchronization() const {
    if (LLVM_UNLIKELY(hasAttribute(AttrId::InternallySynchronized))) {
        if (LLVM_UNLIKELY(canSetTerminateSignal())) {
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << getName() <<
                   " cannot be internally synchronized and be permitted to terminate early";
            report_fatal_error(out.str());
        }

        auto checkValidRate = [this](const Binding & binding) {
            if (LLVM_LIKELY(isCountable(binding) && !isLocalBuffer(binding))) return;
            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << getName() << ":" << binding.getName() <<
                   " must be a non-local non-deferred countable rate"
                   " to support internal synchronization";
            report_fatal_error(out.str());
        };

        for (const Binding & input : mInputStreamSets) {
            checkValidRate(input);
        }

        for (const Binding & output : mOutputStreamSets) {
            checkValidRate(output);
        }
        return true;
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setInputStreamSetAt(const unsigned i, StreamSet * const value) {
    mInputStreamSets[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setOutputStreamSetAt(const unsigned i, StreamSet * const value) {
    mOutputStreamSets[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setInputScalarAt(const unsigned i, Scalar * const value) {
    mInputScalars[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setOutputScalarAt(const unsigned i, Scalar * const value) {
    mOutputScalars[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void SegmentOrientedKernel::generateKernelMethod(BuilderRef b) {
    generateDoSegmentMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief annotateKernelNameWithDebugFlags
 ** ------------------------------------------------------------------------------------------------------------- */
inline std::string annotateKernelNameWithDebugFlags(std::string && name) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        name += "_EA";
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        name += "+MP";
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::DisableIndirectBranch))) {
        name += "-Ibranch";
    }
    if (LLVM_UNLIKELY(codegen::FreeCallBisectLimit >= 0)) {
        name += "+FreeLimit";
    }
//    name += "_O" + std::to_string((int)codegen::OptLevel);
    return std::move(name);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDefaultFamilyName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getDefaultFamilyName() const {
    std::string tmp;
    raw_string_ostream out(tmp);
    if (LLVM_LIKELY(isStateful())) {
        out << "F";
    } else {
        out << "L";
    }
    out << getStride();
    AttributeSet::print(out);
    for (const Binding & input : mInputScalars) {
        out << ",IV("; input.print(this, out); out << ')';
    }
    for (const Binding & input : mInputStreamSets) {
        out << ",IS("; input.print(this, out); out << ')';
    }
    for (const Binding & output : mOutputStreamSets) {
        out << ",OS("; output.print(this, out); out << ')';
    }
    for (const Binding & output : mOutputScalars) {
        out << ",OV("; output.print(this, out); out << ')';
    }
    out.flush();
    return tmp;
}

// CONSTRUCTOR
Kernel::Kernel(BuilderRef b,
               const TypeId typeId,
               std::string && kernelName,
               Bindings &&stream_inputs,
               Bindings &&stream_outputs,
               Bindings &&scalar_inputs,
               Bindings &&scalar_outputs,
               InternalScalars && internal_scalars)
: mTypeId(typeId)
, mStride(b->getBitBlockWidth())
, mInputStreamSets(std::move(stream_inputs))
, mOutputStreamSets(std::move(stream_outputs))
, mInputScalars(std::move(scalar_inputs))
, mOutputScalars(std::move(scalar_outputs))
, mInternalScalars( std::move(internal_scalars))
, mKernelName(annotateKernelNameWithDebugFlags(std::move(kernelName))) {


}

Kernel::~Kernel() { }

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(BuilderRef b,
                                             std::string && kernelName,
                                             Bindings &&stream_inputs,
                                             Bindings &&stream_outputs,
                                             Bindings &&scalar_parameters,
                                             Bindings &&scalar_outputs,
                                             InternalScalars && internal_scalars)
: Kernel(b,
TypeId::SegmentOriented, std::move(kernelName),
std::move(stream_inputs), std::move(stream_outputs),
std::move(scalar_parameters), std::move(scalar_outputs),
std::move(internal_scalars)) {

}

}
