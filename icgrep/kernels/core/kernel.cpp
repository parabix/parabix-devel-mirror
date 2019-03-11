/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <toolchain/driver.h>
#include <kernels/kernel_builder.h>
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
#include <kernels/pipeline_kernel.h>

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
void Kernel::setHandle(const std::unique_ptr<KernelBuilder> & b, Value * const handle) const {
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
void Kernel::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
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
void Kernel::generateKernel(const std::unique_ptr<KernelBuilder> & b) {
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
Function * Kernel::addInitializeDeclaration(const std::unique_ptr<KernelBuilder> & b) const {
    SmallVector<Type *, 32> params;
    if (LLVM_LIKELY(isStateful())) {
        params.push_back(mSharedStateType->getPointerTo());
    }
    for (const Binding & binding : mInputScalars) {
        params.push_back(binding.getType());
    }

    FunctionType * const initType = FunctionType::get(b->getInt1Ty(), params, false);
    Function * const initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INITIALIZE_SUFFIX, b->getModule());
    initFunc->setCallingConv(CallingConv::C);
    initFunc->setDoesNotThrow();

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
    assert (arg == initFunc->arg_end());
    return initFunc;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCurrentMethod = getInitializeFunction(b);
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
    if (LLVM_LIKELY(isStateful())) {
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            b->CreateMProtect(mSharedHandle, CBuilder::Protect::WRITE);
        }
    }
    initializeScalarMap(b, true);
    for (const auto & binding : mInputScalars) {
        b->setScalarField(binding.getName(), nextArg());
    }
    assert (arg == mCurrentMethod->arg_end());

    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(b, handle);
        }
    }
    // any kernel can set termination on initialization
    mTerminationSignalPtr = b->CreateAlloca(b->getInt1Ty(), nullptr, "terminationSignal");
    b->CreateStore(b->getFalse(), mTerminationSignalPtr);
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
Function * Kernel::addInitializeThreadLocalDeclaration(const std::unique_ptr<KernelBuilder> & b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {
        SmallVector<Type *, 2> params;
        if (LLVM_LIKELY(isStateful())) {
            params.push_back(mSharedStateType->getPointerTo());
        }
        params.push_back(mThreadLocalStateType->getPointerTo());

        FunctionType * const funcType = FunctionType::get(b->getVoidTy(), params, false);
        func = Function::Create(funcType, GlobalValue::ExternalLinkage, getName() + INITIALIZE_THREAD_LOCAL_SUFFIX, b->getModule());
        func->setCallingConv(CallingConv::C);
        func->setDoesNotThrow();

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
    return func;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b) {
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
        mThreadLocalHandle = nextArg();
        initializeScalarMap(b);
        generateInitializeThreadLocalMethod(b);
        b->CreateRetVoid();
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
void Kernel::addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & b) {

    // TODO: if a stream has an Expandable or ManagedBuffer attribute or is produced at an Unknown rate,
    // the pipeline ought to pass the stream as a DynamicBuffer. This will require some coordination between
    // the pipeline and kernel to ensure both have a consistent view of the buffer and that if either expands,
    // any other kernel that is (simultaneously) reading from the buffer is unaffected.

    mStreamSetInputBuffers.clear();
    const auto numOfInputStreams = mInputStreamSets.size();
    mStreamSetInputBuffers.reserve(numOfInputStreams);
    for (unsigned i = 0; i < numOfInputStreams; ++i) {
        const Binding & input = mInputStreamSets[i];
        mStreamSetInputBuffers.emplace_back(new ExternalBuffer(b, input.getType()));
    }

    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = mOutputStreamSets.size();
    mStreamSetOutputBuffers.reserve(numOfOutputStreams);
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        mStreamSetOutputBuffers.emplace_back(new ExternalBuffer(b, output.getType()));
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
 * @brief addDoSegmentDeclaration
 ** ------------------------------------------------------------------------------------------------------------ */
Function * Kernel::addDoSegmentDeclaration(const std::unique_ptr<KernelBuilder> & b) const {

    Type * const retTy = canSetTerminateSignal() ? b->getInt1Ty() : b->getVoidTy();
    FunctionType * const doSegmentType = FunctionType::get(retTy, getDoSegmentFields(b), false);
    Function * const doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, getName() + DO_SEGMENT_SUFFIX, b->getModule());
    doSegment->setCallingConv(CallingConv::C);
    doSegment->setDoesNotThrow();

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
    for (unsigned i = 0; i < mInputStreamSets.size(); ++i) {
        const Binding & input = mInputStreamSets[i];
        setNextArgName(input.getName());
        if (LLVM_LIKELY(isAddressable(input) || isCountable(input))) {
            setNextArgName(input.getName() + "_processed");
        }
        setNextArgName(input.getName() + "_accessible");
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            setNextArgName(input.getName() + "_popCountArray");
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            setNextArgName(input.getName() + "_negatedPopCountArray");
        }
    }

    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_LIKELY(!isLocalBuffer(output))) {
            setNextArgName(output.getName());
        }
        if (LLVM_LIKELY(canTerminate || isAddressable(output) || isCountable(output))) {
            setNextArgName(output.getName() + "_produced");
        }
        if (LLVM_LIKELY(isLocalBuffer(output))) {
            setNextArgName(output.getName() + "_consumed");
        } else {
            setNextArgName(output.getName() + "_writable");
        }
    }
    assert (arg == doSegment->arg_end());
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFields
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Type *> Kernel::getDoSegmentFields(const std::unique_ptr<KernelBuilder> & b) const {

    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();

    std::vector<Type *> fields;
    fields.reserve(2 + mInputStreamSets.size() + mOutputStreamSets.size());

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

    for (unsigned i = 0; i < mInputStreamSets.size(); ++i) {
        Type * const bufferType = mStreamSetInputBuffers[i]->getType();
        // logical base input address
        fields.push_back(bufferType->getPointerTo());
        // processed input items
        const Binding & input = mInputStreamSets[i];
        if (isAddressable(input)) {
            fields.push_back(sizePtrTy); // updatable
        } else if (isCountable(input)) {
            fields.push_back(sizeTy); // constant
        }
        // accessible input items (after non-deferred processed item count)
        fields.push_back(sizeTy);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            fields.push_back(sizePtrTy);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            fields.push_back(sizePtrTy);
        }
    }

    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
        const Binding & output = mOutputStreamSets[i];
        // logical base output address
        if (LLVM_LIKELY(!isLocalBuffer(output))) {
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
        fields.push_back(sizeTy);
    }
    return fields;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) {

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
void Kernel::setDoSegmentProperties(const std::unique_ptr<KernelBuilder> & b, const ArrayRef<Value *> args) {

    auto arg = args.begin();
    auto nextArg = [&]() {
        assert (arg != args.end());
        Value * const v = *arg; assert (v);
        std::advance(arg, 1);
        return v;
    };
    if (LLVM_LIKELY(isStateful())) {
        setHandle(b, nextArg());
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        mThreadLocalHandle = nextArg();
    }
    mExternalSegNo = nullptr;
    if (hasAttribute(AttrId::InternallySynchronized)) {
        mExternalSegNo = nextArg();
    }
    mNumOfStrides = nextArg();
    mIsFinal = b->CreateIsNull(mNumOfStrides);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::WRITE);
    }
    initializeScalarMap(b);

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
        const Binding & input = mInputStreamSets[i];
        Value * const addr = nextArg();
        auto & buffer = mStreamSetInputBuffers[i];
        Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
        buffer->setHandle(b, localHandle);
        buffer->setBaseAddress(b.get(), addr);
        /// ----------------------------------------------------
        /// processed item count
        /// ----------------------------------------------------

        // NOTE: we create a redundant alloca to store the input param so that
        // Mem2Reg can convert it into a PHINode if the item count is updated in
        // a loop; otherwise, it will be discarded in favor of the param itself.

        Value * processed = nullptr;
        if (isAddressable(input)) {
            mUpdatableProcessedInputItemPtr[i] = nextArg();
            processed = b->CreateLoad(mUpdatableProcessedInputItemPtr[i]);
        } else if (LLVM_LIKELY(isCountable(input))) {
            processed = nextArg();
        } else { // isRelative
            const ProcessingRate & rate = input.getRate();
            const auto port = getStreamPort(rate.getReference());
            assert (port.Type == PortType::Input && port.Number < i);
            assert (mProcessedInputItemPtr[port.Number]);
            Value * const ref = b->CreateLoad(mProcessedInputItemPtr[port.Number]);
            processed = b->CreateMul2(ref, rate.getRate());
        }
        AllocaInst * const processedItems = b->CreateAlloca(sizeTy);
        b->CreateStore(processed, processedItems);
        mProcessedInputItemPtr[i] = processedItems;
        /// ----------------------------------------------------
        /// accessible item count
        /// ----------------------------------------------------
        Value * const accessible = nextArg();
        mAccessibleInputItems[i] = accessible;
        Value * capacity = b->CreateAdd(processed, accessible);
        mAvailableInputItems[i] = capacity;
        if (input.hasLookahead()) {
            capacity = b->CreateAdd(capacity, b->getSize(input.getLookahead()));
        }
        buffer->setCapacity(b.get(), capacity);

        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            mPopCountRateArray[i] = nextArg();
        }

        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            mNegatedPopCountRateArray[i] = nextArg();
        }
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
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            // If an output is a managed buffer, the address is stored within the state instead
            // of being passed in through the function call.
            Value * const handle = getScalarValuePtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            buffer->setHandle(b, handle);
        } else {
            Value * const logicalBaseAddress = nextArg();
            Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
            buffer->setHandle(b, localHandle);
            buffer->setBaseAddress(b.get(), logicalBaseAddress);
        }
        /// ----------------------------------------------------
        /// produced item count
        /// ----------------------------------------------------
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

            const ProcessingRate & rate = output.getRate();
            const auto port = getStreamPort(rate.getReference());
            assert (port.Type == PortType::Input || (port.Type == PortType::Output && port.Number < i));
            const auto & items = (port.Type == PortType::Input) ? mProcessedInputItemPtr : mProducedOutputItemPtr;
            Value * const ref = b->CreateLoad(items[port.Number]);
            produced = b->CreateMul2(ref, rate.getRate());
        }

        AllocaInst * const producedItems = b->CreateAlloca(sizeTy);
        b->CreateStore(produced, producedItems);
        mProducedOutputItemPtr[i] = producedItems;
        /// ----------------------------------------------------
        /// consumed or writable item count
        /// ----------------------------------------------------
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const consumed = nextArg();
            mConsumedOutputItems[i] = consumed;
        } else {
            Value * writable = nextArg();
            mWritableOutputItems[i] = writable;
            Value * const capacity = b->CreateAdd(produced, writable);
            buffer->setCapacity(b.get(), capacity);
        }
    }
    assert (arg == args.end());

    // initialize the termination signal if this kernel can set it
    mTerminationSignalPtr = nullptr;
    if (canTerminate) {
        mTerminationSignalPtr = b->CreateAlloca(b->getInt1Ty(), nullptr, "terminationSignal");
        b->CreateStore(b->getFalse(), mTerminationSignalPtr);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentProperties
 *
 * Reverse of the setDoSegmentProperties operation; used by the PipelineKernel when constructing internal threads
 * to simplify passing of the state data.
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> Kernel::getDoSegmentProperties(const std::unique_ptr<KernelBuilder> & b) const {

    std::vector<Value *> props;
    if (LLVM_LIKELY(isStateful())) {
        props.push_back(mSharedHandle); assert (mSharedHandle);
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        props.push_back(mThreadLocalHandle); assert (mThreadLocalHandle);
    }
    props.push_back(mNumOfStrides); assert (mNumOfStrides);

    const auto numOfInputs = getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------
        const auto & buffer = mStreamSetInputBuffers[i];
        props.push_back(buffer->getBaseAddress(b.get()));
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
        props.push_back(mAccessibleInputItems[i]);
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
            Value * const handle = getScalarValuePtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            props.push_back(handle);
        } else {
            props.push_back(buffer->getBaseAddress(b.get()));
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
        } else {
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
Function * Kernel::addFinalizeThreadLocalDeclaration(const std::unique_ptr<KernelBuilder> & b) const {
    Function * func = nullptr;
    if (hasThreadLocal()) {
        SmallVector<Type *, 2> params;
        if (LLVM_LIKELY(isStateful())) {
            params.push_back(mSharedStateType->getPointerTo());
        }
        params.push_back(mThreadLocalStateType->getPointerTo());

        FunctionType * const funcType = FunctionType::get(b->getVoidTy(), params, false);
        func = Function::Create(funcType, GlobalValue::ExternalLinkage, getName() + FINALIZE_THREAD_LOCAL_SUFFIX, b->getModule());
        func->setCallingConv(CallingConv::C);
        func->setDoesNotThrow();

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
    return func;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeThreadLocalMethod(const std::unique_ptr<KernelBuilder> & b) {
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
        initializeScalarMap(b);
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
Function * Kernel::addFinalizeDeclaration(const std::unique_ptr<KernelBuilder> & b) const {
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
    Function * const terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, getName() + FINALIZE_SUFFIX, b->getModule());
    terminateFunc->setCallingConv(CallingConv::C);
    terminateFunc->setDoesNotThrow();
    auto args = terminateFunc->arg_begin();
    if (LLVM_LIKELY(isStateful())) {
        (args++)->setName("handle");
    }
    assert (args == terminateFunc->arg_end());
    return terminateFunc;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {

    b->setKernel(this);
    mCurrentMethod = getFinalizeFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    if (LLVM_LIKELY(isStateful())) {
        auto args = mCurrentMethod->arg_begin();
        setHandle(b, &*(args++));
        assert (args == mCurrentMethod->arg_end());
    }
    initializeScalarMap(b, true);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle,CBuilder::Protect::WRITE);
    }
    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = getScalarValuePtr(output.getName() + BUFFER_HANDLE_SUFFIX);
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
std::vector<Value *> Kernel::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    const auto n = mOutputScalars.size();
    std::vector<Value *> outputs(n);
    for (unsigned i = 0; i < n; ++i) {
        outputs[i] = b->CreateLoad(getScalarValuePtr(mOutputScalars[i].getName()));
    }
    return outputs;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getCacheName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getCacheName(const std::unique_ptr<KernelBuilder> & b) const {
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
Module * Kernel::makeModule(const std::unique_ptr<KernelBuilder> & b) {
    Module * m = new Module(getCacheName(b), b->getContext());
    m->setTargetTriple(b->getModule()->getTargetTriple());
    m->setDataLayout(b->getModule()->getDataLayout());
    return setModule(m);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getInitializeFunction(const std::unique_ptr<KernelBuilder> & b) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + INITIALIZE_SUFFIX);
//    if (LLVM_UNLIKELY(f == nullptr)) {
//        f = addInitializeDeclaration(b);
//    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializeThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getInitializeThreadLocalFunction(const std::unique_ptr<KernelBuilder> & b) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + INITIALIZE_THREAD_LOCAL_SUFFIX);
//    if (LLVM_UNLIKELY(f == nullptr)) {
//        f = addInitializeThreadLocalDeclaration(b);
//    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getDoSegmentFunction(const std::unique_ptr<KernelBuilder> & b) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + DO_SEGMENT_SUFFIX);
//    if (LLVM_UNLIKELY(f == nullptr)) {
//        f = addDoSegmentDeclaration(b);
//    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getFinalizeThreadLocalFunction(const std::unique_ptr<KernelBuilder> & b) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + FINALIZE_THREAD_LOCAL_SUFFIX);
//    if (LLVM_UNLIKELY(f == nullptr)) {
//        f = addFinalizeThreadLocalDeclaration(b);
//    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminateFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getFinalizeFunction(const std::unique_ptr<KernelBuilder> & b) const {
    const Module * const module = b->getModule();
    Function * f = module->getFunction(getName() + FINALIZE_SUFFIX);
//    if (LLVM_UNLIKELY(f == nullptr)) {
//        f = addFinalizeDeclaration(b);
//    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & b) {
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
void Kernel::prepareCachedKernel(const std::unique_ptr<KernelBuilder> & b) {
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
std::string Kernel::makeSignature(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(hasSignature())) {
        generateKernel(b);
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
Value * Kernel::createInstance(const std::unique_ptr<KernelBuilder> & b) const {
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
 * @brief createThreadLocalInstance
 ** ------------------------------------------------------------------------------------------------------------- */
llvm::Value * Kernel::createThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b) const {
    if (hasThreadLocal()) {
        Constant * const size = ConstantExpr::getSizeOf(mThreadLocalStateType);
        Value * handle = nullptr;
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            handle = b->CreateAlignedMalloc(size, b->getPageSize());
            b->CreateMProtect(handle, size, CBuilder::Protect::READ);
        } else {
            handle = b->CreateAlignedMalloc(size, b->getCacheAlignment());
        }
        return b->CreatePointerCast(handle, mThreadLocalStateType->getPointerTo());
    }
    llvm_unreachable("createThreadLocalInstance should not be called a kernel without thread-local space");
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b, llvm::ArrayRef<Value *> args) {
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
void Kernel::initializeThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b, ArrayRef<Value *> args) {
    assert (args.size() == isStateful() ? 2 : 1);
    b->setKernel(this);
    Function * const init = getInitializeThreadLocalFunction(b);
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeThreadLocalInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::finalizeThreadLocalInstance(const std::unique_ptr<KernelBuilder> & b, llvm::ArrayRef<Value *> args) const {
    assert (args.size() == isStateful() ? 2 : 1);
    b->setKernel(this);
    Function * const init = getFinalizeThreadLocalFunction(b);
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & b, Value * const handle) const {
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
Function * Kernel::addOrDeclareMainFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const MainMethodGenerationType method) {

    b->setKernel(this);

    addKernelDeclarations(b);

    unsigned suppliedArgs = 1;
    if (LLVM_LIKELY(isStateful())) {
        ++suppliedArgs;
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        ++suppliedArgs;
    }
    if (LLVM_UNLIKELY(hasAttribute(AttrId::InternallySynchronized))) {
        ++suppliedArgs;
    }

    Module * const m = b->getModule();
    Function * const doSegment = getDoSegmentFunction(b);
    assert (doSegment->arg_size() >= suppliedArgs);
    const auto numOfDoSegArgs = doSegment->arg_size() - suppliedArgs;
    Function * const terminate = getFinalizeFunction(b);

    // maintain consistency with the Kernel interface by passing first the stream sets
    // and then the scalars.
    std::vector<Type *> params;
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
        if (!input.hasAttribute(AttrId::Family)) {
            params.push_back(input.getType());
        }
    }

    const auto numOfInitArgs = params.size() - numOfDoSegArgs;

    // get the finalize method output type and set its return type as this function's return type
    FunctionType * const mainFunctionType = FunctionType::get(terminate->getReturnType(), params, false);

    const auto linkageType = (method == AddInternal) ? Function::InternalLinkage : Function::ExternalLinkage;

    Function * const main = Function::Create(mainFunctionType, linkageType, getName() + "_main", m);
    main->setCallingConv(CallingConv::C);

    if (method != DeclareExternal) {

        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", main));
        auto arg = main->arg_begin();
        auto nextArg = [&]() {
            assert (arg != main->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };


        Value * sharedHandle = nullptr;
        Value * threadLocalHandle = nullptr;
        if (isStateful()) {
            sharedHandle = createInstance(b);
        }
        if (hasThreadLocal()) {
            threadLocalHandle = createThreadLocalInstance(b);
        }
        SmallVector<Value *, 16> segmentArgs(doSegment->arg_size());
        unsigned index = 0;
        if (isStateful()) {
            segmentArgs[index++] = sharedHandle;
        }
        if (hasThreadLocal()) {
            segmentArgs[index++] = threadLocalHandle;
        }
        if (hasAttribute(AttrId::InternallySynchronized)) {
            segmentArgs[index++] = b->getSize(0);
        }
        segmentArgs[index++] = b->getSize(0);  // numOfStrides -> isFinal = True
        for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
            segmentArgs[index + i] = nextArg();
        }
        if (isStateful()) {
            SmallVector<Value *, 16> args(numOfInitArgs + 1);
            args[0] = sharedHandle;
            for (unsigned i = 0; i < numOfInitArgs; ++i) {
                args[i + 1] = nextArg();
            }
            initializeInstance(b, args);
        }
        assert (arg == main->arg_end());
        if (hasThreadLocal()) {
            SmallVector<Value *, 2> initArgs;
            if (LLVM_LIKELY(isStateful())) {
                initArgs.push_back(sharedHandle);
            }
            initArgs.push_back(threadLocalHandle);
            initializeThreadLocalInstance(b, initArgs);
        }

        b->CreateCall(doSegment, segmentArgs);
        if (hasThreadLocal()) {
            SmallVector<Value *, 2> args;
            if (LLVM_LIKELY(isStateful())) {
                args.push_back(sharedHandle);
            }
            args.push_back(threadLocalHandle);
            finalizeThreadLocalInstance(b, args);
        }
        if (isStateful()) {
            // call and return the final output value(s)
            b->CreateRet(finalizeInstance(b, sharedHandle));
        }
    }

    return main;
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
inline void Kernel::constructStateTypes(const std::unique_ptr<KernelBuilder> & b) {
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
void Kernel::initializeScalarMap(const std::unique_ptr<KernelBuilder> & b, const bool skipThreadLocal) const {
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
                if (skipThreadLocal) continue;
                assert (hasThreadLocal());
                assert (mThreadLocalHandle);
                indices[1] = b->getInt32(threadLocalIndex++);
                scalar = b->CreateGEP(mThreadLocalHandle, indices);
                break;
            case ScalarType::NonPersistent:
                // TODO: make CreateAllocaAtEntry function?
                scalar = b->CreateAlloca(binding.getValueType());
                b->CreateStore(ConstantAggregateZero::get(binding.getValueType()), scalar);
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
unsigned Kernel::getSharedScalarIndex(const llvm::StringRef name) const {
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
    std::string tmp;
    raw_string_ostream out(tmp);
    out << "Kernel " << getName() <<
           " does not contain an input, output or "
           "internal scalar  named " << name;
    report_fatal_error(out.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarValuePtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::getScalarValuePtr(const StringRef name) const {
    if (LLVM_UNLIKELY(mScalarValueMap.empty())) {
        return nullptr;
    } else {
        const auto f = mScalarValueMap.find(name);
        if (LLVM_UNLIKELY(f == mScalarValueMap.end())) {
            report_fatal_error(getName() + " does not contain scalar: " + name);
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

    std::string tmp;
    raw_string_ostream out(tmp);
    out << "Kernel " << getName() << " does not contain a streamset named " << name;
    report_fatal_error(out.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & Kernel::getStreamBinding(const StringRef name) const {
    const auto port = getStreamPort(name);
    if (port.Type == PortType::Input) {
        return getInputStreamSetBinding(port.Number);
    } else {
        return getOutputStreamSetBinding(port.Number);
    }
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
void SegmentOrientedKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
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
//    name += "_O" + std::to_string((int)codegen::OptLevel);
    return name;
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
Kernel::Kernel(const std::unique_ptr<KernelBuilder> & b,
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
SegmentOrientedKernel::SegmentOrientedKernel(const std::unique_ptr<KernelBuilder> & b,
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
