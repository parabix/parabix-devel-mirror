/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/core/kernel.h>
#include <toolchain/toolchain.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/DerivedTypes.h>
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
#include <llvm/Support/Debug.h>
#include <util/sha1.hpp>
#include <llvm/Support/Format.h>
#include <sstream>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
// #include <kernel/pipeline/pipeline_kernel.h>

using namespace llvm;
using namespace boost;

namespace kernel {

using AttrId = Attribute::KindId;
using RateValue = ProcessingRate::RateValue;
using RateId = ProcessingRate::KindId;
using StreamPort = Kernel::StreamSetPort;
using PortType = Kernel::PortType;

// TODO: make "namespaced" internal scalars that are automatically grouped into cache-aligned structs
// within the kernel state to hide the complexity from the user?

// TODO: create a kernel compiler class, similar to the pipeline compiler, to avoid having any state
// associated with a cached kernel in memory?

// TODO: pass item counts using thread local space?

const static auto INITIALIZE_SUFFIX = "_Initialize";
const static auto INITIALIZE_THREAD_LOCAL_SUFFIX = "_InitializeThreadLocal";
const static auto DO_SEGMENT_SUFFIX = "_DoSegment";
const static auto FINALIZE_THREAD_LOCAL_SUFFIX = "_FinalizeThreadLocal";
const static auto FINALIZE_SUFFIX = "_Finalize";

const static auto SHARED_SUFFIX = "_shared_state";
const static auto THREAD_LOCAL_SUFFIX = "_thread_local";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief nullIfEmpty
 ** ------------------------------------------------------------------------------------------------------------- */
inline StructType * nullIfEmpty(StructType * type) {
    return (type && type->isEmptyTy()) ? nullptr : type;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setHandle(BuilderRef b, Value * const handle) const {
    assert ("handle cannot be null!" && handle);
    assert ("handle must be a pointer!" && handle->getType()->isPointerTy());
    assert ("handle must be a kernel state object!" && (handle->getType()->getPointerElementType() == mSharedStateType));
    #ifndef NDEBUG
    const Function * const handleFunction = isa<Argument>(handle) ? cast<Argument>(handle)->getParent() : cast<Instruction>(handle)->getParent()->getParent();
    const Function * const builderFunction = b->GetInsertBlock()->getParent();
    assert ("handle is not from the current function." && (handleFunction == builderFunction));
    #endif
    mSharedHandle = handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isLocalBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isLocalBuffer(const Binding & output) {
    return output.getRate().isUnknown() || output.hasAttribute(AttrId::ManagedBuffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const unsigned n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
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
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::generateKernel(BuilderRef b) {
    if (LLVM_UNLIKELY(mIsGenerated)) return;
    b->setKernel(this);
    b->setModule(mModule);
    addKernelDeclarations(b);
    callGenerateInitializeMethod(b);
    callGenerateInitializeThreadLocalMethod(b);
    callGenerateDoSegmentMethod(b);
    callGenerateFinalizeThreadLocalMethod(b);
    callGenerateFinalizeMethod(b);
    addAdditionalFunctions(b);
    mIsGenerated = true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addInitializeDeclaration(BuilderRef b) const {
    const auto name = getName() + INITIALIZE_SUFFIX;
    Module * const m = b->getModule();
    Function * initFunc = m->getFunction(name);
    if (LLVM_LIKELY(initFunc == nullptr)) {

        InitArgTypes params;
        if (LLVM_LIKELY(isStateful())) {
            params.push_back(mSharedStateType->getPointerTo());
        }
        for (const Binding & binding : mInputScalars) {
            params.push_back(binding.getType());
        }
        addFamilyInitializationArgTypes(b, params);
        FunctionType * const initType = FunctionType::get(b->getSizeTy(), params, false);
        initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, name, m);
        initFunc->setCallingConv(CallingConv::C);
//        if (LLVM_LIKELY(!codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
//            initFunc->setDoesNotThrow();
//        }
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
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeMethod(BuilderRef b) {
    mCurrentMethod = getInitializeFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto arg = mCurrentMethod->arg_begin();
    const auto arg_end = mCurrentMethod->arg_end();
    auto nextArg = [&]() {
        assert (arg != arg_end);
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };
    if (LLVM_LIKELY(isStateful())) {
        setHandle(b, nextArg());
    }
    if (LLVM_LIKELY(isStateful())) {
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            b->CreateMProtect(mSharedHandle, CBuilder::Protect::WRITE);
        }
    }
    initializeScalarMap(b, InitializeScalarMapOptions::SkipThreadLocal);
    for (const auto & binding : mInputScalars) {
        b->setScalarField(binding.getName(), nextArg());
    }
    bindFamilyInitializationArguments(b, arg, arg_end);
    assert (arg == arg_end);
    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(b, handle);
        }
    }
    // any kernel can set termination on initialization
    mTerminationSignalPtr = b->CreateAlloca(b->getSizeTy(), nullptr, "terminationSignal");
    b->CreateStore(b->getSize(KernelBuilder::TerminationCode::None), mTerminationSignalPtr);
    generateInitializeMethod(b);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect) && isStateful())) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::READ);
    }
    b->CreateRet(b->CreateLoad(mTerminationSignalPtr));
    mTerminationSignalPtr = nullptr;
    mSharedHandle = nullptr;
    mThreadLocalHandle = nullptr;
    mCurrentMethod = nullptr;
    mScalarValueMap.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeThreadLocalDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addInitializeThreadLocalDeclaration(BuilderRef b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {
        const auto name = getName() + INITIALIZE_THREAD_LOCAL_SUFFIX;
        Module * const m = b->getModule();
        func = m->getFunction(name);
        if (LLVM_LIKELY(func == nullptr)) {

            SmallVector<Type *, 1> params;
            if (LLVM_LIKELY(isStateful())) {
                params.push_back(mSharedStateType->getPointerTo());
            }
            PointerType * const retTy = mThreadLocalStateType->getPointerTo();
            FunctionType * const funcType = FunctionType::get(retTy, params, false);
            func = Function::Create(funcType, GlobalValue::ExternalLinkage, name, m);
            func->setCallingConv(CallingConv::C);
//            if (LLVM_LIKELY(!codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
//                func->setDoesNotThrow();
//            }
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
 * @brief callGenerateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeThreadLocalMethod(BuilderRef b) {
    if (hasThreadLocal()) {
        assert (mSharedHandle == nullptr && mThreadLocalHandle == nullptr);
        mCurrentMethod = getInitializeThreadLocalFunction(b);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
        auto arg = mCurrentMethod->arg_begin();
        auto nextArg = [&]() {
            assert (arg != mCurrentMethod->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };
        if (LLVM_LIKELY(isStateful())) {
            setHandle(b, nextArg());
        }
        Value * instance = nullptr;
        Constant * const size = ConstantExpr::getSizeOf(mThreadLocalStateType);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            instance = b->CreateAlignedMalloc(size, b->getPageSize());
            b->CreateMProtect(instance, size, CBuilder::Protect::READ);
        } else {
            instance = b->CreateAlignedMalloc(size, b->getCacheAlignment());
        }
        mThreadLocalHandle = b->CreatePointerCast(instance, mThreadLocalStateType->getPointerTo());
        initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);
        generateInitializeThreadLocalMethod(b);
        Value * const retVal = mThreadLocalHandle;
        b->CreateRet(retVal);
        mSharedHandle = nullptr;
        mThreadLocalHandle = nullptr;
        mCurrentMethod = nullptr;
        mScalarValueMap.clear();
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addBaseKernelProperties
 *
 * Base kernel properties are those that the pipeline requires access to and must be in a fixed memory location.
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addBaseKernelProperties(BuilderRef b) {

    // TODO: if a stream has an Expandable or ManagedBuffer attribute or is produced at an Unknown rate,
    // the pipeline ought to pass the stream as a DynamicBuffer. This will require some coordination between
    // the pipeline and kernel to ensure both have a consistent view of the buffer and that if either expands,
    // any other kernel that is (simultaneously) reading from the buffer is unaffected.

    mStreamSetInputBuffers.clear();
    const auto numOfInputStreams = mInputStreamSets.size();
    mStreamSetInputBuffers.reserve(numOfInputStreams);
    for (unsigned i = 0; i < numOfInputStreams; ++i) {
        const Binding & input = mInputStreamSets[i];
        mStreamSetInputBuffers.emplace_back(new ExternalBuffer(b, input.getType(), true, 0));
    }

    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = mOutputStreamSets.size();
    mStreamSetOutputBuffers.reserve(numOfOutputStreams);
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        mStreamSetOutputBuffers.emplace_back(new ExternalBuffer(b, output.getType(), true, 0));
    }

    // If an output is a managed buffer, store its handle.
    for (unsigned i = 0; i < getNumOfStreamOutputs(); ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Type * const handleTy = mStreamSetOutputBuffers[i]->getHandleType(b);
            addInternalScalar(handleTy, output.getName() + BUFFER_HANDLE_SUFFIX);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasFixedRate
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE bool hasFixedRate(const Kernel * const kernel) {
    const auto n = kernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = kernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            return true;
        }
    }
    const auto m = kernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
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
RateValue getFixedRateLCM(const Kernel * const kernel) {
    RateValue rateLCM(1);
    bool hasFixedRate = false;
    const auto n = kernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = kernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRate = true;
        }
    }
    const auto m = kernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (LLVM_LIKELY(rate.isFixed())) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRate = true;
        }
    }
    return hasFixedRate ? rateLCM : RateValue{0};
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
 * @brief addDoSegmentDeclaration
 ** ------------------------------------------------------------------------------------------------------------ */
Function * Kernel::addDoSegmentDeclaration(BuilderRef b) const {

    const auto name = getName() + DO_SEGMENT_SUFFIX;
    Module * const m = b->getModule();
    Function * doSegment = m->getFunction(name);
    if (LLVM_LIKELY(doSegment == nullptr)) {

        Type * const retTy = canSetTerminateSignal() ? b->getSizeTy() : b->getVoidTy();
        FunctionType * const doSegmentType = FunctionType::get(retTy, getDoSegmentFields(b), false);
        doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, name, m);
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
        if (hasAttribute(AttrId::InternallySynchronized)) {
            setNextArgName("externalSegNo");
        }
        setNextArgName("numOfStrides");
        if (LLVM_LIKELY(hasFixedRate(this))) {
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
//            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
//                setNextArgName(input.getName() + "_popCountArray");
//            }
//            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
//                setNextArgName(input.getName() + "_negatedPopCountArray");
//            }
        }

        const auto canTerminate = canSetTerminateSignal();

        for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
            const Binding & output = mOutputStreamSets[i];
            const auto isLocal = isLocalBuffer(output);
            if (LLVM_LIKELY(!isLocal)) {
                setNextArgName(output.getName());
            }
            if (LLVM_LIKELY(canTerminate || isAddressable(output) || isCountable(output))) {
                setNextArgName(output.getName() + "_produced");
            }
            if (LLVM_LIKELY(isLocal)) {
                setNextArgName(output.getName() + "_consumed");
            } else if (requiresItemCount(output)) {
                setNextArgName(output.getName() + "_writable");
            }
        }
        assert (arg == doSegment->arg_end());
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFields
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Type *> Kernel::getDoSegmentFields(BuilderRef b) const {

    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();
    const auto n = mInputStreamSets.size();
    const auto m = mOutputStreamSets.size();

    std::vector<Type *> fields;
    fields.reserve(2 + n + m);

    if (LLVM_LIKELY(isStateful())) {
        fields.push_back(mSharedStateType->getPointerTo());  // handle
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        fields.push_back(mThreadLocalStateType->getPointerTo());  // handle
    }
    if (hasAttribute(AttrId::InternallySynchronized)) {
        fields.push_back(sizeTy);
    }
    fields.push_back(sizeTy); // numOfStrides
    if (LLVM_LIKELY(hasFixedRate(this))) {
        fields.push_back(sizeTy); // fixedRateFactor
    }
    for (unsigned i = 0; i < n; ++i) {
        Type * const bufferType = mStreamSetInputBuffers[i]->getType();
        // virtual base input address
        fields.push_back(bufferType->getPointerTo());
        // processed input items
        const Binding & input = mInputStreamSets[i];
        if (isAddressable(input)) {
            fields.push_back(sizePtrTy); // updatable
        } else if (isCountable(input)) {
            fields.push_back(sizeTy); // constant
        }
        // accessible input items
        if (requiresItemCount(input)) {
            fields.push_back(sizeTy);
        }

//        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
//            fields.push_back(sizePtrTy);
//        }
//        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
//            fields.push_back(sizePtrTy);
//        }
    }

    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = mOutputStreamSets[i];
        // virtual base output address
        const auto isLocal = isLocalBuffer(output);
        if (LLVM_LIKELY(!isLocal)) {
            Type * const bufferType = mStreamSetOutputBuffers[i]->getType();
            fields.push_back(bufferType->getPointerTo());
        }
        // produced output items
        if (canTerminate || isAddressable(output)) {
            fields.push_back(sizePtrTy); // updatable
        } else if (isCountable(output)) {
            fields.push_back(sizeTy); // constant
        }
        // If this is a local buffer, the next param is its consumed item count;
        // otherwise it'll hold its writable output items.
        if (isLocal || requiresItemCount(output)) {
            fields.push_back(sizeTy);
        }
    }
    return fields;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateDoSegmentMethod(BuilderRef b) {

    assert (mInputStreamSets.size() == mStreamSetInputBuffers.size());
    assert (mOutputStreamSets.size() == mStreamSetOutputBuffers.size());

    b->setKernel(this);
    mCurrentMethod = getDoSegmentFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));

    std::vector<Value *> args;
    args.reserve(mCurrentMethod->arg_size());
    for (auto ArgI = mCurrentMethod->arg_begin(); ArgI != mCurrentMethod->arg_end(); ++ArgI) {
        args.push_back(&(*ArgI));
    }
    setDoSegmentProperties(b, args);

    generateKernelMethod(b);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::READ);
    }

    const auto numOfInputs = getNumOfStreamInputs();

    for (unsigned i = 0; i < numOfInputs; i++) {
        if (mUpdatableProcessedInputItemPtr[i]) {
            Value * const items = b->CreateLoad(mProcessedInputItemPtr[i]);
            b->CreateStore(items, mUpdatableProcessedInputItemPtr[i]);
        }
    }

    const auto numOfOutputs = getNumOfStreamOutputs();

    for (unsigned i = 0; i < numOfOutputs; i++) {
        if (mUpdatableProducedOutputItemPtr[i]) {
            Value * const items = b->CreateLoad(mProducedOutputItemPtr[i]);
            b->CreateStore(items, mUpdatableProducedOutputItemPtr[i]);
        }
    }

    // return the termination signal (if one exists)
    if (mTerminationSignalPtr) {
        b->CreateRet(b->CreateLoad(mTerminationSignalPtr));
        mTerminationSignalPtr = nullptr;
    } else {
        b->CreateRetVoid();
    }

    // Clean up all of the constructed buffers.
    mSharedHandle = nullptr;
    mThreadLocalHandle = nullptr;
    mExternalSegNo = nullptr;
    mCurrentMethod = nullptr;
    mIsFinal = nullptr;
    mNumOfStrides = nullptr;
    mScalarValueMap.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setDoSegmentProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setDoSegmentProperties(BuilderRef b, const ArrayRef<Value *> args) {

    auto arg = args.begin();
    auto nextArg = [&]() {
        assert (arg != args.end());
        Value * const v = *arg; assert (v);
        std::advance(arg, 1);
        return v;
    };
    if (LLVM_LIKELY(isStateful())) {
        setHandle(b, nextArg());
        assert (mSharedHandle->getType()->getPointerElementType() == mSharedStateType);
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        mThreadLocalHandle = nextArg();
        assert (mThreadLocalHandle->getType()->getPointerElementType() == mThreadLocalStateType);
    }
    mExternalSegNo = nullptr;
    if (hasAttribute(AttrId::InternallySynchronized)) {
        mExternalSegNo = nextArg();
    }
    mNumOfStrides = nextArg();
    const auto fixedRateLCM = getFixedRateLCM(this);
    mFixedRateFactor = nullptr;
    if (LLVM_LIKELY(fixedRateLCM.numerator() != 0)) {
        mFixedRateFactor = nextArg();
    }
    assert (!mFixedRateFactor ^ hasFixedRate(this));

    mIsFinal = b->CreateIsNull(mNumOfStrides);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::WRITE);
    }
    initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);

    // NOTE: the disadvantage of passing the stream pointers as a parameter is that it becomes more difficult
    // to access a stream set from a LLVM function call. We could create a stream-set aware function creation
    // and call system here but that is not an ideal way of handling this.

    const auto numOfInputs = getNumOfStreamInputs();

    reset(mProcessedInputItemPtr, numOfInputs);
    reset(mAccessibleInputItems, numOfInputs);
    reset(mAvailableInputItems, numOfInputs);
    reset(mPopCountRateArray, numOfInputs);
    reset(mNegatedPopCountRateArray, numOfInputs);
    reset(mUpdatableProcessedInputItemPtr, numOfInputs);

    IntegerType * const sizeTy = b->getSizeTy();

    for (unsigned i = 0; i < numOfInputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------
        auto & buffer = mStreamSetInputBuffers[i];
        const Binding & input = mInputStreamSets[i];
        Value * const virtualBaseAddress = nextArg();
        assert (virtualBaseAddress->getType() == buffer->getPointerType());
        Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
        buffer->setHandle(b, localHandle);
        buffer->setBaseAddress(b, virtualBaseAddress);
        /// ----------------------------------------------------
        /// processed item count
        /// ----------------------------------------------------

        // NOTE: we create a redundant alloca to store the input param so that
        // Mem2Reg can convert it into a PHINode if the item count is updated in
        // a loop; otherwise, it will be discarded in favor of the param itself.

        const ProcessingRate & rate = input.getRate();
        Value * processed = nullptr;
        if (isAddressable(input)) {
            mUpdatableProcessedInputItemPtr[i] = nextArg();
            processed = b->CreateLoad(mUpdatableProcessedInputItemPtr[i]);
        } else if (LLVM_LIKELY(isCountable(input))) {
            processed = nextArg();
        } else { // isRelative
            const auto port = getStreamPort(rate.getReference());
            assert (port.Type == PortType::Input && port.Number < i);
            assert (mProcessedInputItemPtr[port.Number]);
            Value * const ref = b->CreateLoad(mProcessedInputItemPtr[port.Number]);
            processed = b->CreateMulRate(ref, rate.getRate());
        }
        assert (processed);
        assert (processed->getType() == sizeTy);
        AllocaInst * const processedItems = b->CreateAlloca(sizeTy);
        b->CreateStore(processed, processedItems);
        mProcessedInputItemPtr[i] = processedItems;
        /// ----------------------------------------------------
        /// accessible item count
        /// ----------------------------------------------------
        Value * accessible = nullptr;
        if (LLVM_UNLIKELY(requiresItemCount(input))) {
            accessible = nextArg();
        } else {
            accessible = b->CreateCeilUMulRate(mFixedRateFactor, rate.getRate() / fixedRateLCM);
        }
        assert (accessible->getType() == sizeTy);
        mAccessibleInputItems[i] = accessible;
        Value * capacity = b->CreateAdd(processed, accessible);
        mAvailableInputItems[i] = capacity;
        if (input.hasLookahead()) {
            capacity = b->CreateAdd(capacity, b->getSize(input.getLookahead()));
        }
        buffer->setCapacity(b, capacity);
//        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
//            mPopCountRateArray[i] = nextArg();
//        }

//        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
//            mNegatedPopCountRateArray[i] = nextArg();
//        }
    }

    // set all of the output buffers
    const auto numOfOutputs = getNumOfStreamOutputs();
    reset(mProducedOutputItemPtr, numOfOutputs);
    reset(mWritableOutputItems, numOfOutputs);
    reset(mConsumedOutputItems, numOfOutputs);
    reset(mUpdatableProducedOutputItemPtr, numOfOutputs);

    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < numOfOutputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------

        auto & buffer = mStreamSetOutputBuffers[i];
        const Binding & output = mOutputStreamSets[i];
        const auto isLocal = isLocalBuffer(output);
        if (LLVM_UNLIKELY(isLocal)) {
            // If an output is a managed buffer, the address is stored within the state instead
            // of being passed in through the function call.
            Value * const handle = getScalarValuePtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
            buffer->setHandle(b, handle);
        } else {
            Value * const virtualBaseAddress = nextArg();
            assert (virtualBaseAddress->getType() == buffer->getPointerType());
            Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
            buffer->setHandle(b, localHandle);
            buffer->setBaseAddress(b, virtualBaseAddress);
        }

        /// ----------------------------------------------------
        /// produced item count
        /// ----------------------------------------------------
        const ProcessingRate & rate = output.getRate();
        Value * produced = nullptr;
        if (LLVM_LIKELY(canTerminate || isAddressable(output))) {
            mUpdatableProducedOutputItemPtr[i] = nextArg();
            produced = b->CreateLoad(mUpdatableProducedOutputItemPtr[i]);
        } else if (LLVM_LIKELY(isCountable(output))) {
            produced = nextArg();
        } else { // isRelative
            // For now, if something is produced at a relative rate to another stream in a kernel that
            // may terminate, its final item count is inherited from its reference stream and cannot
            // be set independently. Should they be independent at early termination?
            const auto port = getStreamPort(rate.getReference());
            assert (port.Type == PortType::Input || (port.Type == PortType::Output && port.Number < i));
            const auto & items = (port.Type == PortType::Input) ? mProcessedInputItemPtr : mProducedOutputItemPtr;
            Value * const ref = b->CreateLoad(items[port.Number]);
            produced = b->CreateMulRate(ref, rate.getRate());
        }
        assert (produced);
        assert (produced->getType() == sizeTy);
        AllocaInst * const producedItems = b->CreateAlloca(sizeTy);
        b->CreateStore(produced, producedItems);
        mProducedOutputItemPtr[i] = producedItems;
        /// ----------------------------------------------------
        /// consumed or writable item count
        /// ----------------------------------------------------
        if (LLVM_UNLIKELY(isLocal)) {
            Value * const consumed = nextArg();
            assert (consumed->getType() == sizeTy);
            mConsumedOutputItems[i] = consumed;
        } else {
            Value * writable = nullptr;
            if (requiresItemCount(output)) {
                writable = nextArg();
            } else {
                writable = b->CreateCeilUMulRate(mFixedRateFactor, rate.getRate() / fixedRateLCM);
            }
            assert (writable->getType() == sizeTy);
            mWritableOutputItems[i] = writable;
            Value * const capacity = b->CreateAdd(produced, writable);
            buffer->setCapacity(b, capacity);
        }
    }
    assert (arg == args.end());

    // initialize the termination signal if this kernel can set it
    mTerminationSignalPtr = nullptr;
    if (canTerminate) {
        mTerminationSignalPtr = b->CreateAlloca(b->getSizeTy(), nullptr, "terminationSignal");
        b->CreateStore(b->getSize(KernelBuilder::TerminationCode::None), mTerminationSignalPtr);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentProperties
 *
 * Reverse of the setDoSegmentProperties operation; used by the PipelineKernel when constructing internal threads
 * to simplify passing of the state data.
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> Kernel::getDoSegmentProperties(BuilderRef b) const {

    std::vector<Value *> props;
    if (LLVM_LIKELY(isStateful())) {
        props.push_back(mSharedHandle); assert (mSharedHandle);
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        props.push_back(mThreadLocalHandle); assert (mThreadLocalHandle);
    }
    props.push_back(mNumOfStrides); assert (mNumOfStrides);
    if (LLVM_LIKELY(hasFixedRate(this))) {
        props.push_back(mFixedRateFactor); // fixedRateFactor
    }

    const auto numOfInputs = getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------
        const auto & buffer = mStreamSetInputBuffers[i];
        props.push_back(buffer->getBaseAddress(b));
        /// ----------------------------------------------------
        /// processed item count
        /// ----------------------------------------------------
        const Binding & input = mInputStreamSets[i];
        if (isAddressable(input)) {
            props.push_back(mProcessedInputItemPtr[i]);
        } else if (LLVM_LIKELY(isCountable(input))) {
            props.push_back(b->CreateLoad(mProcessedInputItemPtr[i]));
        }
        /// ----------------------------------------------------
        /// accessible item count
        /// ----------------------------------------------------
        if (requiresItemCount(input)) {
            props.push_back(mAccessibleInputItems[i]);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            props.push_back(mPopCountRateArray[i]);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            props.push_back(mNegatedPopCountRateArray[i]);
        }
    }

    // set all of the output buffers
    const auto numOfOutputs = getNumOfStreamOutputs();
    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < numOfOutputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------
        const auto & buffer = mStreamSetOutputBuffers[i];
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            // If an output is a managed buffer, the address is stored within the state instead
            // of being passed in through the function call.
            Value * const handle = getScalarValuePtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
            props.push_back(handle);
        } else {
            props.push_back(buffer->getBaseAddress(b));
        }
        /// ----------------------------------------------------
        /// produced item count
        /// ----------------------------------------------------
        if (LLVM_LIKELY(canTerminate || isAddressable(output))) {
            props.push_back(mProducedOutputItemPtr[i]);
        } else if (LLVM_LIKELY(isCountable(output))) {
            props.push_back(b->CreateLoad(mProducedOutputItemPtr[i]));
        }
        /// ----------------------------------------------------
        /// consumed or writable item count
        /// ----------------------------------------------------
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            props.push_back(mConsumedOutputItems[i]);
        } else if (requiresItemCount(output)) {
            props.push_back(mWritableOutputItems[i]);
        }
    }

    if (hasAttribute(AttrId::InternallySynchronized)) {
        props.push_back(mExternalSegNo);
    }

    return props;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeThreadLocalDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addFinalizeThreadLocalDeclaration(BuilderRef b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {

        const auto name = getName() + FINALIZE_THREAD_LOCAL_SUFFIX;
        Module * const m = b->getModule();
        func = m->getFunction(name);
        if (LLVM_LIKELY(func == nullptr)) {
            SmallVector<Type *, 2> params;
            if (LLVM_LIKELY(isStateful())) {
                params.push_back(mSharedStateType->getPointerTo());
            }
            params.push_back(mThreadLocalStateType->getPointerTo());

            FunctionType * const funcType = FunctionType::get(b->getVoidTy(), params, false);
            func = Function::Create(funcType, GlobalValue::ExternalLinkage, name, m);
            func->setCallingConv(CallingConv::C);
//            if (LLVM_LIKELY(!codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
//                func->setDoesNotThrow();
//            }
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
 * @brief callGenerateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeThreadLocalMethod(BuilderRef b) {
    if (hasThreadLocal()) {
        mCurrentMethod = getFinalizeThreadLocalFunction(b);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
        auto arg = mCurrentMethod->arg_begin();
        auto nextArg = [&]() {
            assert (arg != mCurrentMethod->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };
        if (LLVM_LIKELY(isStateful())) {
            setHandle(b, nextArg());
        }
        mThreadLocalHandle = nextArg();
        initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);
        generateFinalizeThreadLocalMethod(b);
        b->CreateRetVoid();
        mSharedHandle = nullptr;
        mThreadLocalHandle = nullptr;
        mCurrentMethod = nullptr;
        mScalarValueMap.clear();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addFinalizeDeclaration(BuilderRef b) const {

    const auto name = getName() + FINALIZE_SUFFIX;
    Module * const m = b->getModule();
    Function * terminateFunc = m->getFunction(name);
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
            params.push_back(mSharedStateType->getPointerTo());
        }
        FunctionType * const terminateType = FunctionType::get(resultType, params, false);
        terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, name, m);
        terminateFunc->setCallingConv(CallingConv::C);
//        if (LLVM_LIKELY(!codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
//            terminateFunc->setDoesNotThrow();
//        }
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
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeMethod(BuilderRef b) {

    b->setKernel(this);
    mCurrentMethod = getFinalizeFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    if (LLVM_LIKELY(isStateful())) {
        auto args = mCurrentMethod->arg_begin();
        setHandle(b, &*(args++));
        assert (args == mCurrentMethod->arg_end());
    }
    initializeScalarMap(b, InitializeScalarMapOptions::SkipThreadLocal);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle,CBuilder::Protect::WRITE);
    }
    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = getScalarValuePtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(b, handle);
        }
    }
    generateFinalizeMethod(b); // may be overridden by the Kernel subtype
    const auto outputs = getFinalOutputScalars(b);
    if (LLVM_LIKELY(isStateful())) {
        b->CreateFree(mSharedHandle);
    }
    mSharedHandle = nullptr;
    if (outputs.empty()) {
        b->CreateRetVoid();
    } else {
        const auto n = outputs.size();
        if (n == 1) {
            b->CreateRet(outputs[0]);
        } else {
            b->CreateAggregateRet(outputs.data(), n);
        }
    }
    mCurrentMethod = nullptr;
    mScalarValueMap.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> Kernel::getFinalOutputScalars(BuilderRef b) {
    const auto n = mOutputScalars.size();
    std::vector<Value *> outputs(n);
    for (unsigned i = 0; i < n; ++i) {
        outputs[i] = b->CreateLoad(getScalarValuePtr(b.get(), mOutputScalars[i].getName()));
    }
    return outputs;
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
 * @brief setModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * Kernel::setModule(Module * const module) {
    assert (mModule == nullptr || mModule == module);
    mModule = module;
    return mModule;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * Kernel::makeModule(BuilderRef b) {
    Module * m = new Module(getCacheName(b), b->getContext());
    m->setTargetTriple(b->getModule()->getTargetTriple());
    m->setDataLayout(b->getModule()->getDataLayout());
    return setModule(m);
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
 * @brief getTerminateFunction
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
 * @brief prepareKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareKernel(BuilderRef b) {
    if (LLVM_UNLIKELY(mStride == 0)) {
        report_fatal_error(getName() + ": stride cannot be 0");
    }
    addBaseKernelProperties(b);
    addInternalProperties(b);
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        makeModule(b);
    }
    constructStateTypes(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareCachedKernel(BuilderRef b) {
    if (LLVM_UNLIKELY(mSharedStateType != nullptr)) {
        llvm_unreachable("Cannot call prepareCachedKernel after constructing kernel state type");
    }
    Module * const m = getModule();
    addBaseKernelProperties(b);
    mSharedStateType = nullIfEmpty(m->getTypeByName(getName() + SHARED_SUFFIX));
    mThreadLocalStateType = nullIfEmpty(m->getTypeByName(getName() + THREAD_LOCAL_SUFFIX));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeSignature
 *
 * Default kernel signature: generate the IR and emit as byte code.
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::makeSignature(BuilderRef b) const {
    if (LLVM_UNLIKELY(hasSignature())) {
        // TODO: make this branch report an error instead.
        const_cast<Kernel *>(this)->generateKernel(b);
        std::string tmp;
        raw_string_ostream signature(tmp);
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(7, 0, 0)
        WriteBitcodeToFile(getModule(), signature);
        #else
        WriteBitcodeToFile(*(getModule()), signature);
        #endif
        return signature.str();
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
 * @brief createInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::createInstance(BuilderRef b) const {
    if (isStateful()) {
        Constant * const size = ConstantExpr::getSizeOf(mSharedStateType);
        Value * handle = nullptr;
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            handle = b->CreateAlignedMalloc(size, b->getPageSize());
            b->CreateMProtect(handle, size, CBuilder::Protect::READ);
        } else {
            handle = b->CreateAlignedMalloc(size, b->getCacheAlignment());
        }
        return b->CreatePointerCast(handle, mSharedStateType->getPointerTo());
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
    assert (args[0]->getType()->getPointerElementType() == mSharedStateType);
    b->setKernel(this);
    Function * const init = getInitializeFunction(b);
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeThreadLocalInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::initializeThreadLocalInstance(BuilderRef b, Value * const handle) {
    Value * instance = nullptr;
    if (hasThreadLocal()) {
        b->setKernel(this);
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
    b->setKernel(this);
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
 * @brief addOrDeclareMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::addOrDeclareMainFunction(BuilderRef, const MainMethodGenerationType) {
    report_fatal_error("Cannot generate a main method for " + mKernelName + ".");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyInitializationArgTypes
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addFamilyInitializationArgTypes(BuilderRef /* b */, InitArgTypes & /* argTypes */) const {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recursivelyConstructFamilyKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::recursivelyConstructFamilyKernels(BuilderRef /* b */, InitArgs & /* args */, const ParamMap & /* params */) const {
    if (LLVM_UNLIKELY(containsKernelFamilies())) {
        report_fatal_error("recursivelyConstructFamilyKernels should have been overridden for " + mKernelName + ".");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindFamilyInitializationArguments
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::bindFamilyInitializationArguments(BuilderRef /* b */, ArgIterator & /* arg */, const ArgIterator & /* arg_end */) const {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructFamilyKernels
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::constructFamilyKernels(BuilderRef b, InitArgs & hostArgs, const ParamMap & params) const {

    #warning TODO: need to test for termination on init call

    Value * handle = nullptr;
    InitArgs initArgs;
    if (LLVM_LIKELY(isStateful())) {
        handle = createInstance(b);
        initArgs.push_back(handle);
        hostArgs.push_back(handle);
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
    Function * const init = getInitializeFunction(b);
    b->CreateCall(init, initArgs);
    if (hasThreadLocal()) {
        hostArgs.push_back(getInitializeThreadLocalFunction(b));
    }
    hostArgs.push_back(getDoSegmentFunction(b));
    if (hasThreadLocal()) {
        hostArgs.push_back(getFinalizeThreadLocalFunction(b));
    }
    hostArgs.push_back(getFinalizeFunction(b));
    return handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLowerBound
 ** ------------------------------------------------------------------------------------------------------------- */
RateValue Kernel::getLowerBound(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.hasReference()) {
        return rate.getLowerBound() * getLowerBound(getStreamBinding(rate.getReference()));
    } else {
        return rate.getLowerBound();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getUpperBound
 ** ------------------------------------------------------------------------------------------------------------- */
RateValue Kernel::getUpperBound(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.hasReference()) {
        return rate.getUpperBound() * getUpperBound(getStreamBinding(rate.getReference()));
    } else {
        return rate.getUpperBound();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief requiresOverflow
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::requiresOverflow(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || binding.hasAttribute(AttrId::BlockSize)) {
        return false;
    } else if (rate.isRelative()) {
        return requiresOverflow(getStreamBinding(rate.getReference()));
    } else {
        return true;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructStateTypes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::constructStateTypes(BuilderRef b) {
    mSharedStateType = mModule->getTypeByName(getName() + SHARED_SUFFIX);
    mThreadLocalStateType = mModule->getTypeByName(getName() + THREAD_LOCAL_SUFFIX);
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
        mSharedStateType = StructType::create(b->getContext(), shared, getName() + SHARED_SUFFIX);
        mThreadLocalStateType = StructType::create(b->getContext(), threadLocal, getName() + THREAD_LOCAL_SUFFIX);
    }
    mSharedStateType = nullIfEmpty(mSharedStateType);
    mThreadLocalStateType = nullIfEmpty(mThreadLocalStateType);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeScalarMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeScalarMap(BuilderRef b, const InitializeScalarMapOptions options) const {
    mScalarValueMap.clear();
    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    unsigned sharedIndex = 0;
    bool noDuplicateFieldNames = true;

    for (const auto & binding : mInputScalars) {
        indices[1] = b->getInt32(sharedIndex++);
        assert (mSharedHandle);
        Value * scalar = b->CreateGEP(mSharedHandle, indices);
        assert (scalar->getType()->getPointerElementType() == binding.getType());
        const auto inserted = mScalarValueMap.insert(std::make_pair(binding.getName(), scalar)).second;
        noDuplicateFieldNames &= inserted;
    }
    for (const auto & binding : mOutputScalars) {
        indices[1] = b->getInt32(sharedIndex++);
        assert (mSharedHandle);
        Value * scalar = b->CreateGEP(mSharedHandle, indices);
        assert (scalar->getType()->getPointerElementType() == binding.getType());
        const auto inserted = mScalarValueMap.insert(std::make_pair(binding.getName(), scalar)).second;
        noDuplicateFieldNames &= inserted;
    }
    unsigned threadLocalIndex = 0;
    for (const auto & binding : mInternalScalars) {
        Value * scalar = nullptr;
        switch (binding.getScalarType()) {
            case ScalarType::Internal:
                indices[1] = b->getInt32(sharedIndex++);
                assert (mSharedHandle);
                scalar = b->CreateGEP(mSharedHandle, indices);
                break;
            case ScalarType::ThreadLocal:
                if (options == InitializeScalarMapOptions::SkipThreadLocal) continue;
                assert (mThreadLocalHandle);
                indices[1] = b->getInt32(threadLocalIndex++);
                scalar = b->CreateGEP(mThreadLocalHandle, indices);
                break;
            case ScalarType::NonPersistent:
                scalar = b->CreateAllocaAtEntryPoint(binding.getValueType());
                b->CreateStore(Constant::getNullValue(binding.getValueType()), scalar);
                break;
            default: llvm_unreachable("I/O scalars cannot be internal");
        }
        assert (scalar->getType()->getPointerElementType() == binding.getValueType());
        const auto inserted = mScalarValueMap.insert(std::make_pair(binding.getName(), scalar)).second;
        noDuplicateFieldNames &= inserted;
    }
    assert (mSharedHandle == nullptr || sharedIndex == mSharedStateType->getStructNumElements());
    assert (mThreadLocalHandle == nullptr || threadLocalIndex == mThreadLocalStateType->getStructNumElements());
    assert (noDuplicateFieldNames);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeScalarValuePtr
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned Kernel::getSharedScalarIndex(KernelBuilder * /* b */, const llvm::StringRef name) const {
    unsigned sharedIndex = 0;
    for (const auto & binding : mInputScalars) {
        if (name.equals(binding.getName())) {
            assert (mSharedHandle && sharedIndex < mSharedStateType->getStructNumElements());
            return sharedIndex;
        }
        ++sharedIndex;
    }
    for (const auto & binding : mOutputScalars) {
        if (name.equals(binding.getName())) {
            assert (mSharedHandle && sharedIndex < mSharedStateType->getStructNumElements());
            return sharedIndex;
        }
        ++sharedIndex;
    }
    for (const auto & binding : mInternalScalars) {
        switch (binding.getScalarType()) {
            case ScalarType::Internal:
                if (name.equals(binding.getName())) {
                    assert (mSharedHandle && sharedIndex < mSharedStateType->getStructNumElements());
                    return sharedIndex;
                }
                ++sharedIndex;
                break;
            case ScalarType::ThreadLocal:
                if (name.equals(binding.getName())) {
                    break;
                }
                break;
            case ScalarType::NonPersistent:
                if (name.equals(binding.getName())) {
                    break;
                }
                break;
            default: llvm_unreachable("I/O scalars cannot be internal");
        }
    }


    SmallVector<char, 1024> tmp;
    raw_svector_ostream out(tmp);
    out << "Kernel " << getName() <<
           " does not contain an input, output or "
           "internal scalar  named \"" << name << "\"";
    report_fatal_error(out.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarValuePtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::getScalarValuePtr(KernelBuilder * /* b */, const StringRef name) const {
    if (LLVM_UNLIKELY(mScalarValueMap.empty())) {
        return nullptr;
    } else {
        const auto f = mScalarValueMap.find(name);
        if (LLVM_UNLIKELY(f == mScalarValueMap.end())) {
            assert (false);
            SmallVector<char, 1024> tmp;
            raw_svector_ostream out(tmp);
            out << getName() << " does not contain scalar: " << name << "\n"
                "currently contains:";
            char spacer = ' ';
            for (const auto & entry : mScalarValueMap) {
                out << spacer << entry.getKey();
                spacer = ',';
            }
            report_fatal_error(out.str());
        }
        return f->second;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const BindingMapEntry & Kernel::getBinding(const BindingType type, const llvm::StringRef name) const {

    const auto f = mBindingMap.find(name);
    if (f != mBindingMap.end()) {
        const BindingMapEntry & entry = f->second;
        assert (entry.Type == type);
        return entry;
    }

    const Bindings & bindings = [&](const BindingType type) -> const Bindings & {
        switch (type) {
            case BindingType::ScalarInput:
                return mInputScalars;
            case BindingType::ScalarOutput:
                return mOutputScalars;
            case BindingType::StreamInput:
                return mInputStreamSets;
            case BindingType::StreamOutput:
                return mOutputStreamSets;
        }
        llvm_unreachable("unknown binding type");
    } (type);

    const auto n = bindings.size();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & binding = bindings[i];
        if (name.equals(binding.getName())) {
            auto f = mBindingMap.insert(std::make_pair(name, BindingMapEntry{type, i})).first;
            return f->getValue();
        }
    }

    std::string tmp;
    raw_string_ostream out(tmp);
    out << "Kernel " << getName() << " does not contain an ";
    switch (type) {
        case BindingType::ScalarInput:
            out << "input scalar"; break;
        case BindingType::ScalarOutput:
            out << "output scalar"; break;
        case BindingType::StreamInput:
            out << "input streamset"; break;
        case BindingType::StreamOutput:
            out << "output streamset"; break;
    }
    out << " named " << name;
    report_fatal_error(out.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStreamPort
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel::StreamSetPort Kernel::getStreamPort(const StringRef name) const {

    // NOTE: temporary refactoring step to limit changes outside of the kernel class

    static_assert(static_cast<unsigned>(BindingType::StreamInput) == static_cast<unsigned>(PortType::Input), "");
    static_assert(static_cast<unsigned>(BindingType::StreamOutput) == static_cast<unsigned>(PortType::Output), "");

    const auto f = mBindingMap.find(name);
    if (f != mBindingMap.end()) {

        const BindingMapEntry & entry = f->second;
        switch (entry.Type) {
            case BindingType::StreamInput:
            case BindingType::StreamOutput:
                return StreamSetPort(static_cast<PortType>(entry.Type), entry.Index);
            default: break;
        }

    } else {

        auto findStreamPort = [&](const BindingType type, const Bindings & bindings) {
            const auto n = bindings.size();
            for (unsigned i = 0; i < n; ++i) {
                const Binding & binding = bindings[i];
                if (name.equals(binding.getName())) {
                    mBindingMap.insert(std::make_pair(name, BindingMapEntry{type, i}));
                    return i;
                }
            }
            return -1U;
        };

        const auto inputPort = findStreamPort(BindingType::StreamInput, mInputStreamSets);
        if (inputPort != -1U) {
            return StreamSetPort(PortType::Input, inputPort);
        }

        const auto outputPort = findStreamPort(BindingType::StreamOutput, mOutputStreamSets);
        if (outputPort != -1U) {
            return StreamSetPort(PortType::Output, outputPort);
        }
    }

    assert (false);

    std::string tmp;
    raw_string_ostream out(tmp);
    out << "Kernel " << getName() << " does not contain a streamset named " << name;
    report_fatal_error(out.str());
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
: mInputStreamSets(std::move(stream_inputs))
, mOutputStreamSets(std::move(stream_outputs))
, mInputScalars(std::move(scalar_inputs))
, mOutputScalars(std::move(scalar_outputs))
, mInternalScalars( std::move(internal_scalars))
, mStride(b->getBitBlockWidth())
, mKernelName(annotateKernelNameWithDebugFlags(std::move(kernelName)))
, mTypeId(typeId) {

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
