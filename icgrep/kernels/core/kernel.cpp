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

const static auto INIT_SUFFIX = "_Init";
const static auto DO_SEGMENT_SUFFIX = "_DoSegment";
const static auto TERMINATE_SUFFIX = "_Terminate";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setHandle(const std::unique_ptr<KernelBuilder> & b, Value * const handle) const {
    assert ("handle cannot be null!" && handle);
    assert ("handle must be a pointer!" && handle->getType()->isPointerTy());
    assert ("handle must be a kernel state object!" && (handle->getType()->getPointerElementType() == mKernelStateType));
    #ifndef NDEBUG
    const Function * const handleFunction = isa<Argument>(handle) ? cast<Argument>(handle)->getParent() : cast<Instruction>(handle)->getParent()->getParent();
    const Function * const builderFunction = b->GetInsertBlock()->getParent();
    assert ("handle is not from the current function." && (handleFunction == builderFunction));
    #endif
    mHandle = handle;
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
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Type * const handleTy = mStreamSetOutputBuffers[i]->getHandleType(b);
            addInternalScalar(handleTy, output.getName() + BUFFER_HANDLE_SUFFIX);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addScalarToMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addScalarToMap(const StringRef name, const ScalarType scalarType, const unsigned index) {
    const auto r = mScalarMap.insert(std::make_pair(name, ScalarField{scalarType, index}));
    if (LLVM_UNLIKELY(!r.second)) {
        const ScalarField & sf = r.first->second;
        if (LLVM_UNLIKELY(sf.Type != scalarType || sf.Index != index)) {
            report_fatal_error(getName() + " already contains scalar " + name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addScalarToMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addStreamToMap(const StringRef name, const PortType type, const unsigned number) {
    const auto r = mStreamSetMap.insert(std::make_pair(name, StreamSetPort{type, number}));
    if (LLVM_UNLIKELY(!r.second)) {
        const StreamPort & sf = r.first->second;
        if (LLVM_UNLIKELY(sf.Type != type || sf.Number != number)) {
            report_fatal_error(getName() + " already contains stream " + name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm_unreachable("Kernel state must be constructed prior to calling addKernelDeclarations");
    }
    addInitializeDeclaration(b);
    addDoSegmentDeclaration(b);
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
    callGenerateDoSegmentMethod(b);
    callGenerateFinalizeMethod(b);
    addAdditionalFunctions(b);
    mIsGenerated = true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::addInitializeDeclaration(const std::unique_ptr<KernelBuilder> & b) {

    std::vector<Type *> params;
    if (LLVM_LIKELY(isStateful())) {
        params.push_back(mKernelStateType->getPointerTo());
    }
    for (const Binding & binding : mInputScalars) {
        params.push_back(binding.getType());
    }

    FunctionType * const initType = FunctionType::get(b->getInt1Ty(), params, false);
    Function * const initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INIT_SUFFIX, b->getModule());
    initFunc->setCallingConv(CallingConv::C);
    initFunc->setDoesNotThrow();
    auto args = initFunc->arg_begin();
    if (LLVM_LIKELY(isStateful())) {
        (args++)->setName("handle");
    }
    for (const Binding & binding : mInputScalars) {
        (args++)->setName(binding.getName());
    }

    assert (args == initFunc->arg_end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    Value * const storedHandle = getHandle();
    mCurrentMethod = getInitFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    if (LLVM_LIKELY(isStateful())) {
        setHandle(b, &*(args++));
    }
    if (LLVM_LIKELY(isStateful())) {
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            b->CreateMProtect(mHandle, CBuilder::Protect::WRITE);
        }
        b->CreateStore(ConstantAggregateZero::get(mKernelStateType), mHandle);
    }
    for (const auto & binding : mInputScalars) {
        b->setScalarField(binding.getName(), &*(args++));
    }
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
    initializeLocalScalarValues(b);
    generateInitializeMethod(b);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect) && isStateful())) {
        b->CreateMProtect(mHandle, CBuilder::Protect::READ);
    }
    b->CreateRet(b->CreateLoad(mTerminationSignalPtr));
    mTerminationSignalPtr = nullptr;

    b->setKernel(storedKernel);
    mHandle = storedHandle;
    mCurrentMethod = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addDoSegmentDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::addDoSegmentDeclaration(const std::unique_ptr<KernelBuilder> & b) {

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
        setNextArgName("handle");
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
        fields.push_back(mKernelStateType->getPointerTo());  // handle
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

    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    Value * const storedHandle = getHandle();
    mCurrentMethod = getDoSegmentFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));

    std::vector<Value *> args;
    args.reserve(mCurrentMethod->arg_size());
    for (auto ArgI = mCurrentMethod->arg_begin(); ArgI != mCurrentMethod->arg_end(); ++ArgI) {
        args.push_back(&(*ArgI));
    }
    setDoSegmentProperties(b, args);

    generateKernelMethod(b);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle, CBuilder::Protect::READ);
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
    b->setKernel(storedKernel);
    mHandle = storedHandle;
    mCurrentMethod = nullptr;
    mIsFinal = nullptr;
    mNumOfStrides = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setDoSegmentProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::setDoSegmentProperties(const std::unique_ptr<KernelBuilder> & b, const std::vector<Value *> & args) {

    initializeLocalScalarValues(b);

    auto arg = args.begin();
    auto nextArg = [&]() {
        assert (arg != args.end());
        Value * const v = *arg;
        std::advance(arg, 1);
        return v;
    };

    if (LLVM_LIKELY(isStateful())) {
        setHandle(b, nextArg());
    }
    mNumOfStrides = nextArg();
    mIsFinal = b->CreateIsNull(mNumOfStrides);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle, CBuilder::Protect::WRITE);
    }

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
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
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
        props.push_back(mHandle); assert (mHandle);
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
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
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

    return props;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::addFinalizeDeclaration(const std::unique_ptr<KernelBuilder> & b) {
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
        params.push_back(mKernelStateType->getPointerTo());
    }
    FunctionType * const terminateType = FunctionType::get(resultType, params, false);
    Function * const terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, getName() + TERMINATE_SUFFIX, b->getModule());
    terminateFunc->setCallingConv(CallingConv::C);
    terminateFunc->setDoesNotThrow();
    auto args = terminateFunc->arg_begin();
    if (LLVM_LIKELY(isStateful())) {
        (args++)->setName("handle");
    }
    assert (args == terminateFunc->arg_end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {

    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    mCurrentMethod = getTerminateFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    if (LLVM_LIKELY(isStateful())) {
        auto args = mCurrentMethod->arg_begin();
        setHandle(b, &*(args++));
        assert (args == mCurrentMethod->arg_end());
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle,CBuilder::Protect::WRITE);
    }
    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(b, handle);
        }
    }
    initializeLocalScalarValues(b);
    generateFinalizeMethod(b); // may be overridden by the Kernel subtype
    const auto outputs = getFinalOutputScalars(b);
    if (LLVM_LIKELY(isStateful())) {
        b->CreateFree(mHandle);
    }
    mHandle = nullptr;
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

    b->setKernel(storedKernel);
    mCurrentMethod = nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> Kernel::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    const auto n = mOutputScalars.size();
    std::vector<Value *> outputs(n);
    for (unsigned i = 0; i < n; ++i) {
        outputs[i] = b->getScalarField(mOutputScalars[i].getName());
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
 * @brief getInitFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getInitFunction(Module * const module) const {
    const auto name = getName() + INIT_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm_unreachable("cannot find Initialize function");
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getDoSegmentFunction(Module * const module) const {
    const auto name = getName() + DO_SEGMENT_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm_unreachable("cannot find DoSegment function");
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getTerminateFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * Kernel::getTerminateFunction(Module * const module) const {
    const auto name = getName() + TERMINATE_SUFFIX;
    Function * f = module->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        llvm_unreachable("cannot find Terminate function");
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        llvm_unreachable("Cannot call prepareKernel after constructing kernel state type");
    }
    if (LLVM_UNLIKELY(mStride == 0)) {
        report_fatal_error(getName() + ": stride cannot be 0");
    }
    addBaseKernelProperties(b);
    addInternalKernelProperties(b);
    // NOTE: StructType::create always creates a new type even if an identical one exists.
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        makeModule(b);
    }
    mKernelStateType = mModule->getTypeByName(getName());
    if (LLVM_LIKELY(mKernelStateType == nullptr)) {
        std::vector<Type *> fields;
        fields.reserve(mInputScalars.size() + mOutputScalars.size() + mInternalScalars.size());
        for (const Binding & scalar : mInputScalars) {
            assert (scalar.getType());
            fields.push_back(scalar.getType());
        }
        for (const Binding & scalar : mOutputScalars) {
            assert (scalar.getType());
            fields.push_back(scalar.getType());
        }
        for (const Binding & scalar : mInternalScalars) {
            assert (scalar.getType());
            fields.push_back(scalar.getType());
        }
        mKernelStateType = StructType::create(b->getContext(), fields, getName());
    }
    assert (isa<StructType>(mKernelStateType));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalScalar
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addInternalScalar(Type * type, const StringRef name) {
    const auto index = mInternalScalars.size();
    mInternalScalars.emplace_back(type, name);
    addScalarToMap(name, ScalarType::Internal, index);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addLocalScalar
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addLocalScalar(Type * type, const StringRef name) {
    const auto index = mLocalScalars.size();
    mLocalScalars.emplace_back(type, name);
    addScalarToMap(name, ScalarType::Local, index);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareCachedKernel(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        llvm_unreachable("Cannot call prepareCachedKernel after constructing kernel state type");
    }
    addBaseKernelProperties(b);
    mKernelStateType = getModule()->getTypeByName(getName());
    // If we have a stateless object, the type would be optimized out of the
    // cached IR. Consequently, we create a dummy "empty struct" to simplify
    // the assumptions of the other Kernel functions.
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        mKernelStateType = StructType::get(b->getContext());
    }
    assert (isa<StructType>(mKernelStateType));
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
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm_unreachable("Kernel state must be constructed prior to calling createInstance");
    }
    if (LLVM_LIKELY(isStateful())) {
        Constant * const size = ConstantExpr::getSizeOf(mKernelStateType);
        Value * handle = nullptr;
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
            handle = b->CreateAlignedMalloc(size, b->getPageSize());
            b->CreateMProtect(handle, size, CBuilder::Protect::READ);
        } else {
            handle = b->CreateAlignedMalloc(size, b->getCacheAlignment());
        }
        return b->CreatePointerCast(handle, mKernelStateType->getPointerTo());
    }
    llvm_unreachable("createInstance should not be called on stateless kernels");
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> &args) {
    assert (args.size() == getNumOfScalarInputs() + 1);
    assert (args[0] && "cannot initialize before creation");
    assert (args[0]->getType()->getPointerElementType() == mKernelStateType);
    b->setKernel(this);
    Function * const init = getInitFunction(b->getModule());
    b->CreateCall(init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & b, Value * const handle) const {
    Value * result = nullptr;
    Function * const termFunc = getTerminateFunction(b->getModule());
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

    constexpr auto INTERNAL_VARIABLES = 2;

    b->setKernel(this);

    addKernelDeclarations(b);

    Module * const m = b->getModule();
    Function * const doSegment = getDoSegmentFunction(m);
    assert (doSegment->arg_size() >= INTERNAL_VARIABLES);
    const auto numOfDoSegArgs = doSegment->arg_size() - INTERNAL_VARIABLES;
    Function * const terminate = getTerminateFunction(m);

    // maintain consistency with the Kernel interface by passing first the stream sets
    // and then the scalars.
    std::vector<Type *> params;
    params.reserve(numOfDoSegArgs + getNumOfScalarInputs());

    // The first three params of doSegment are its handle, currentStrideNum and numOfStrides.
    // The remaining are the stream set params
    auto doSegParam = doSegment->arg_begin();
    std::advance(doSegParam, INTERNAL_VARIABLES);
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

        // TODO: Even if a kernel is in a family, it may have a different kernel struct. Make sure this works here.
        Value * const handle = createInstance(b);
        setHandle(b, handle);

        std::vector<Value *> segmentArgs(doSegment->arg_size());
        segmentArgs[0] = handle;
        segmentArgs[1] = b->getSize(0); // numOfStrides -> isFinal = True
        for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
            assert (arg != main->arg_end());
            segmentArgs[i + INTERNAL_VARIABLES] = &*arg++;
        }

        std::vector<Value *> initArgs(numOfInitArgs + 1);
        initArgs[0] = handle;
        for (unsigned i = 0; i < numOfInitArgs; ++i) {
            assert (arg != main->arg_end());
            initArgs[i + 1] = &*arg++;
        }
        assert (arg == main->arg_end());
        // initialize the kernel
        initializeInstance(b, initArgs);
        // call the pipeline kernel
        b->CreateCall(doSegment, segmentArgs);
        // call and return the final output value(s)
        b->CreateRet(finalizeInstance(b, handle));
    }

    return main;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarField
 ** ------------------------------------------------------------------------------------------------------------- */
const Kernel::ScalarField & Kernel::getScalarField(const StringRef name) const {
    assert (!mScalarMap.empty());
    const auto f = mScalarMap.find(name);
    if (LLVM_UNLIKELY(f == mScalarMap.end())) {
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarFieldPtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::getScalarFieldPtr(KernelBuilder & b, const StringRef name) const {
    const auto & field = getScalarField(name);
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm_unreachable("Kernel state must be constructed prior to calling getScalarFieldPtr");
    }
    unsigned index = field.Index;
    switch (field.Type) {
        case ScalarType::Local:
            return mLocalScalarPtr[index];
        case ScalarType::Internal:
            index += mOutputScalars.size();
        case ScalarType::Output:
            index += mInputScalars.size();
        case ScalarType::Input:
            break;
    }
    assert (index < mKernelStateType->getStructNumElements());
    return b.CreateGEP(getHandle(), {b.getInt32(0), b.getInt32(index)});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeLocalScalarValues
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeLocalScalarValues(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_LIKELY(mLocalScalars.empty())) {
        return;
    }
    mLocalScalarPtr.resize(mLocalScalars.size());
    const auto end = mScalarMap.end();
    for (auto i = mScalarMap.begin(); i != end; ++i) {
        ScalarField & field = i->getValue();
        if (LLVM_UNLIKELY(field.Type == ScalarType::Local)) {
            const auto index = field.Index;
            const Binding & local = mLocalScalars[index];
            Value * const scalar = b->CreateAlloca(local.getType());
            b->CreateStore(ConstantAggregateZero::get(local.getType()), scalar);
            mLocalScalarPtr[index] = scalar;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputScalarBinding
 ** ------------------------------------------------------------------------------------------------------------- */
Binding & Kernel::getInputScalarBinding(const StringRef name) {
    const ScalarField & field = getScalarField(name);
    if (LLVM_UNLIKELY(field.Type != ScalarType::Input)) {
        report_fatal_error(getName() + "." + name + "is not an input scalar");
    }
    return mInputScalars[field.Index];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalarBinding
 ** ------------------------------------------------------------------------------------------------------------- */
Binding & Kernel::getOutputScalarBinding(const StringRef name) {
    const ScalarField & field = getScalarField(name);
    if (LLVM_UNLIKELY(field.Type != ScalarType::Output)) {
        report_fatal_error(getName() + "." + name + "is not an output scalar");
    }
    return mOutputScalars[field.Index];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStreamPort
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel::StreamSetPort Kernel::getStreamPort(const StringRef name) const {
    const auto f = mStreamSetMap.find(name);
    if (LLVM_UNLIKELY(f == mStreamSetMap.end())) {
        assert (!"could not find stream set!");
        report_fatal_error(getName() + " does not contain stream set " + name);
    }
    return f->second;
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
 * @brief isStateful
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE bool Kernel::isStateful() const {
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        llvm_unreachable("kernel state must be constructed prior to calling isStateful");
    }
    return !mKernelStateType->isEmptyTy();
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
 * @brief initializeBindings
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::initializeBindings(BaseDriver & driver) {

    for (unsigned i = 0; i < mInputScalars.size(); i++) {
        Binding & input = mInputScalars[i];
        addScalarToMap(input.getName(), ScalarType::Input, i);
        if (input.getRelationship() == nullptr) {
            input.setRelationship(driver.CreateScalar(input.getType()));
        }
    }
    for (unsigned i = 0; i < mInputStreamSets.size(); i++) {
        Binding & input = mInputStreamSets[i];
        if (LLVM_UNLIKELY(input.getRelationship() == nullptr)) {
            report_fatal_error(getName()+ "." + input.getName() + " must be set upon construction");
        }
        addStreamToMap(input.getName(), PortType::Input, i);
    }
    for (unsigned i = 0; i < mOutputStreamSets.size(); i++) {
        Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(output.getRelationship() == nullptr)) {
            report_fatal_error(getName()+ "." + output.getName() + " must be set upon construction");
        }
        addStreamToMap(output.getName(), PortType::Output, i);
    }
    for (unsigned i = 0; i < mInternalScalars.size(); i++) {
        const Binding & internal = mInternalScalars[i];
        addScalarToMap(internal.getName(), ScalarType::Internal, i);
    }
    for (unsigned i = 0; i < mOutputScalars.size(); i++) {
        Binding & output = mOutputScalars[i];
        addScalarToMap(output.getName(), ScalarType::Output, i);
        if (output.getRelationship() == nullptr) {
            output.setRelationship(driver.CreateScalar(output.getType()));
        }
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
        name += "_MP";
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
               Bindings && stream_inputs,
               Bindings && stream_outputs,
               Bindings && scalar_inputs,
               Bindings && scalar_outputs,
               Bindings && internal_scalars)
: mIsGenerated(false)
, mHandle(nullptr)
, mModule(nullptr)
, mKernelStateType(nullptr)
, mInputStreamSets(std::move(stream_inputs))
, mOutputStreamSets(std::move(stream_outputs))
, mInputScalars(std::move(scalar_inputs))
, mOutputScalars(std::move(scalar_outputs))
, mInternalScalars( std::move(internal_scalars))
, mCurrentMethod(nullptr)
, mStride(b->getBitBlockWidth())
, mTerminationSignalPtr(nullptr)
, mIsFinal(nullptr)
, mNumOfStrides(nullptr)
, mKernelName(annotateKernelNameWithDebugFlags(std::move(kernelName)))
, mTypeId(typeId) {

}

Kernel::~Kernel() { }

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(const std::unique_ptr<KernelBuilder> & b,
                                             std::string && kernelName,
                                             Bindings && stream_inputs,
                                             Bindings && stream_outputs,
                                             Bindings && scalar_parameters,
                                             Bindings && scalar_outputs,
                                             Bindings && internal_scalars)
: Kernel(b,
TypeId::SegmentOriented, std::move(kernelName),
std::move(stream_inputs), std::move(stream_outputs),
std::move(scalar_parameters), std::move(scalar_outputs),
std::move(internal_scalars)) {

}


}