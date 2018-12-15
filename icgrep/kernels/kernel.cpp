/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "kernel.h"
#include <toolchain/toolchain.h>
#include <toolchain/driver.h>
#include <kernels/relationship.h>
#include <kernels/streamset.h>
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
#include <boost/uuid/sha1.hpp>
#include <llvm/Support/Format.h>
#include <sstream>

using namespace llvm;
using namespace boost;

namespace kernel {

using AttrId = Attribute::KindId;
using RateValue = ProcessingRate::RateValue;
using RateId = ProcessingRate::KindId;
using StreamPort = Kernel::StreamSetPort;
using Port = Kernel::Port;

// TODO: make "namespaced" internal scalars that are automatically grouped into cache-aligned structs
// within the kernel state to hide the complexity from the user?

const static auto INIT_SUFFIX = "_Init";
const static auto DO_SEGMENT_SUFFIX = "_DoSegment";
const static auto TERMINATE_SUFFIX = "_Terminate";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void  Kernel::setHandle(const std::unique_ptr<KernelBuilder> & b, Value * const handle) {
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
 * @brief addBaseKernelProperties
 *
 * Base kernel properties are those that the pipeline requires access to and must be in a fixed memory location.
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & b) {

    // Set the default kernel stride.
    if (mStride == 0) {
        mStride = b->getBitBlockWidth();
    }

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

    IntegerType * const sizeTy = b->getSizeTy();

    addInternalScalar(sizeTy, LOGICAL_SEGMENT_NO_SCALAR);

    // TODO: if we had a way of easily calculating the number of processed/produced items of the
    // final stride of a non-deferred fixed rate stream, we could avoid storing the item counts.
    for (unsigned i = 0; i < numOfInputStreams; ++i) {
        const Binding & input = mInputStreamSets[i];
        addInternalScalar(sizeTy, input.getName() + PROCESSED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(input.isDeferred())) {
            addInternalScalar(sizeTy, input.getName() + NON_DEFERRED_ITEM_COUNT_SUFFIX);
        }
    }

    // If an output is a managed buffer, we need to store both the buffer and a set of consumers.
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        addInternalScalar(sizeTy, output.getName() + PRODUCED_ITEM_COUNT_SUFFIX);
        if (LLVM_UNLIKELY(output.isDeferred())) {
            addInternalScalar(sizeTy, output.getName() + NON_DEFERRED_ITEM_COUNT_SUFFIX);
        }
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Type * const handleTy = mStreamSetOutputBuffers[i]->getHandleType(b);
            addInternalScalar(handleTy, output.getName() + BUFFER_HANDLE_SUFFIX);
            addInternalScalar(sizeTy, output.getName() + CONSUMED_ITEM_COUNT_SUFFIX);
        }
    }

    // We compile in a 64-bit CPU cycle counter into every kernel.   It will remain unused
    // in normal execution, but when codegen::EnableCycleCounter is specified, pipelines
    // will be able to add instrumentation to cached modules without recompilation.
    addInternalScalar(b->getInt64Ty(), CYCLECOUNT_SCALAR);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addScalarToMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addScalarToMap(const llvm::StringRef name, const ScalarType scalarType, const unsigned index) {
    const auto r = mScalarMap.insert(std::make_pair(name, ScalarField{scalarType, index}));
    if (LLVM_UNLIKELY(!r.second)) {
        const ScalarField & sf = r.first->second;
        if (LLVM_UNLIKELY(sf.type != scalarType || sf.index != index)) {
            report_fatal_error(getName() + " already contains scalar " + name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addScalarToMap
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addStreamToMap(const llvm::StringRef name, const Port port, const unsigned index) {
    const auto r = mStreamSetMap.insert(std::make_pair(name, std::make_pair(port, index)));
    if (LLVM_UNLIKELY(!r.second)) {
        const StreamPort & sf = r.first->second;
        if (LLVM_UNLIKELY(sf.first != port || sf.second != index)) {
            report_fatal_error(getName() + " already contains stream " + name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    if (mKernelStateType == nullptr) {
        throw std::runtime_error("Kernel state definition " + getName() + " has not been finalized.");
    }
    addInitializeDeclaration(b);
    addDoSegmentDeclaration(b);
    addFinalizeDeclaration(b);
    linkExternalMethods(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInitializeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addInitializeDeclaration(const std::unique_ptr<KernelBuilder> & b) {

    std::vector<Type *> params;
    params.push_back(mKernelStateType->getPointerTo());
    for (const Binding & binding : mInputScalars) {
        params.push_back(binding.getType());
    }

    FunctionType * const initType = FunctionType::get(b->getInt1Ty(), params, false);
    Function * const initFunc = Function::Create(initType, GlobalValue::ExternalLinkage, getName() + INIT_SUFFIX, b->getModule());
    initFunc->setCallingConv(CallingConv::C);
    initFunc->setDoesNotThrow();
    auto args = initFunc->arg_begin();
    args->setName("self");
    for (const Binding & binding : mInputScalars) {
        (++args)->setName(binding.getName());
    }

    assert (std::next(args) == initFunc->arg_end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::callGenerateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    Value * const storedHandle = getHandle();
    mCurrentMethod = getInitFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setHandle(b, &*args);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle, CBuilder::Protect::WRITE);
    }
    b->CreateStore(ConstantAggregateZero::get(mKernelStateType), getHandle());
    for (const auto & binding : mInputScalars) {
        b->setScalarField(binding.getName(), &*(++args));
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
    generateInitializeMethod(b);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
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
void Kernel::addDoSegmentDeclaration(const std::unique_ptr<KernelBuilder> & b) {

    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const sizePtrTy = sizeTy->getPointerTo();

    std::vector<Type *> params;
    params.reserve(2 + mInputStreamSets.size() + mOutputStreamSets.size());
    params.push_back(mKernelStateType->getPointerTo());  // self
    params.push_back(sizeTy); // numOfStrides
    for (unsigned i = 0; i < mInputStreamSets.size(); ++i) {
        Type * const bufferType = mStreamSetInputBuffers[i]->getType();
        params.push_back(bufferType->getPointerTo()); // logical "base" input address
        params.push_back(sizeTy);  // accessible input items (after non-deferred processed item count)
        const Binding & input = mInputStreamSets[i];
        unsigned numOfPopCountArrays = 0;
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            ++numOfPopCountArrays;
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            ++numOfPopCountArrays;
        }
        if (numOfPopCountArrays) {
            params.insert(params.end(), numOfPopCountArrays, sizePtrTy); // popCountRef array (length is numOfStrides)
        }
    }
    for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_LIKELY(!isLocalBuffer(output))) {
            Type * const bufferType = mStreamSetOutputBuffers[i]->getType();
            params.push_back(bufferType->getPointerTo()); // logical "base" output address
            params.push_back(sizeTy); // writable output items (after non-deferred produced item count)
        }
    }

    Type * const retTy = canSetTerminateSignal() ? b->getInt1Ty() : b->getVoidTy();
    FunctionType * const doSegmentType = FunctionType::get(retTy, params, false);
    Function * const doSegment = Function::Create(doSegmentType, GlobalValue::ExternalLinkage, getName() + DO_SEGMENT_SUFFIX, b->getModule());
    doSegment->setCallingConv(CallingConv::C);
    doSegment->setDoesNotThrow();
    auto args = doSegment->arg_begin();
    args->setName("self");
    (++args)->setName("numOfStrides");
    for (unsigned i = 0; i < mInputStreamSets.size(); ++i) {
        const Binding & input = mInputStreamSets[i];
        (++args)->setName(input.getName());
        (++args)->setName(input.getName() + "_accessible");
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            (++args)->setName(input.getName() + "_popCountArray");
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            (++args)->setName(input.getName() + "_negatedPopCountArray");
        }
    }
    for (unsigned i = 0; i < mOutputStreamSets.size(); ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_LIKELY(!isLocalBuffer(output))) {
            (++args)->setName(output.getName());
            (++args)->setName(output.getName() + "_writable");
        }
    }
    assert (std::next(args) == doSegment->arg_end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::callGenerateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    assert (mInputStreamSets.size() == mStreamSetInputBuffers.size());
    assert (mOutputStreamSets.size() == mStreamSetOutputBuffers.size());

    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    Value * const storedHandle = getHandle();
    mCurrentMethod = getDoSegmentFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setHandle(b, &*(args++));
    mNumOfStrides = &*(args++);
    mIsFinal = b->CreateIsNull(mNumOfStrides);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle,CBuilder::Protect::WRITE);
    }
    // NOTE: the disadvantage of passing the stream pointers as a parameter is that it becomes more difficult
    // to access a stream set from a LLVM function call. We could create a stream-set aware function creation
    // and call system here but that is not an ideal way of handling this.

    // TODO: use a graph to depict relations between binding? It would be better to first move to a model
    // where inputs and outputs are contained in a single parameter vector.

    const auto numOfInputs = getNumOfStreamInputs();
    mAccessibleInputItems.resize(numOfInputs, nullptr);
    mAvailableInputItems.resize(numOfInputs, nullptr);
    mPopCountRateArray.resize(numOfInputs, nullptr);
    mNegatedPopCountRateArray.resize(numOfInputs, nullptr);
    for (unsigned i = 0; i < numOfInputs; i++) {
        const Binding & input = mInputStreamSets[i];
        assert (args != mCurrentMethod->arg_end());
        Value * const addr = &*(args++);
        auto & buffer = mStreamSetInputBuffers[i];
        Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
        buffer->setHandle(b, localHandle);
        buffer->setBaseAddress(b.get(), addr);
        assert (args != mCurrentMethod->arg_end());
        Value * const accessible = &*(args++);
        mAccessibleInputItems[i] = accessible;
        Value * const processed = b->getNonDeferredProcessedItemCount(input);
        Value * capacity = b->CreateAdd(processed, accessible);
        mAvailableInputItems[i] = capacity;
        if (input.hasLookahead()) {
            capacity = b->CreateAdd(capacity, b->getSize(input.getLookahead()));
        }
        buffer->setCapacity(b.get(), capacity);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            assert (args != mCurrentMethod->arg_end());
            mPopCountRateArray[i] = &*(args++);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            assert (args != mCurrentMethod->arg_end());
            mNegatedPopCountRateArray[i] = &*(args++);
        }
    }

    // set all of the output buffers
    const auto numOfOutputs = getNumOfStreamOutputs();
    mWritableOutputItems.resize(numOfOutputs, nullptr);
    for (unsigned i = 0; i < numOfOutputs; i++) {
        // If an output is a managed buffer, the address is stored within the state instead
        // of being passed in through the function call.
        auto & buffer = mStreamSetOutputBuffers[i];
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(isLocalBuffer(output))) {
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            buffer->setHandle(b, handle);
        } else {
            assert (args != mCurrentMethod->arg_end());
            Value * const logicalBaseAddress = &*(args++);
            Value * const localHandle = b->CreateAlloca(buffer->getHandleType(b));
            buffer->setHandle(b, localHandle);
            buffer->setBaseAddress(b.get(), logicalBaseAddress);
            assert (args != mCurrentMethod->arg_end());
            Value * const writable = &*(args++);
            mWritableOutputItems[i] = writable;
            Value * const produced = b->getNonDeferredProducedItemCount(output);
            Value * const capacity = b->CreateAdd(produced, writable);
            buffer->setCapacity(b.get(), capacity);
        }
    }
    assert (args == mCurrentMethod->arg_end());

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const terminated = b->getTerminationSignal();
        b->CreateAssert(b->CreateNot(terminated), getName() + " was called after termination");
    }

    // initialize the termination signal if this kernel can set it
    if (canSetTerminateSignal()) {
        mTerminationSignalPtr = b->CreateAlloca(b->getInt1Ty(), nullptr, "terminationSignal");
        b->CreateStore(b->getFalse(), mTerminationSignalPtr);
    }

    // Calculate and/or load the accessible and writable item counts. If they are unneeded,
    // LLVM ought to recognize them as dead code and remove them.
    generateKernelMethod(b); // must be overridden by the Kernel subtype
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mHandle, CBuilder::Protect::READ);
    }

    // return the termination signal (if one exists)
    if (canSetTerminateSignal()) {
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
    mAccessibleInputItems.clear();
    mPopCountRateArray.clear();
    mNegatedPopCountRateArray.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFinalizeDeclaration
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::addFinalizeDeclaration(const std::unique_ptr<KernelBuilder> & b) {
    Type * resultType = nullptr;
    if (mOutputScalars.empty()) {
        resultType = b->getVoidTy();
    } else {
        const auto n = mOutputScalars.size();
        Type * outputType[n];
        for (unsigned i = 0; i < n; ++i) {
            outputType[i] = mOutputScalars[i].getType();
        }
        if (n == 1) {
            resultType = outputType[0];
        } else {
            resultType = StructType::get(b->getContext(), ArrayRef<Type *>(outputType, n));
        }
    }
    PointerType * const selfType = mKernelStateType->getPointerTo();
    FunctionType * const terminateType = FunctionType::get(resultType, {selfType}, false);
    Function * const terminateFunc = Function::Create(terminateType, GlobalValue::ExternalLinkage, getName() + TERMINATE_SUFFIX, b->getModule());
    terminateFunc->setCallingConv(CallingConv::C);
    terminateFunc->setDoesNotThrow();
    auto args = terminateFunc->arg_begin();
    args->setName("self");
    assert (std::next(args) == terminateFunc->arg_end());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {

    const Kernel * const storedKernel = b->getKernel();
    b->setKernel(this);
    mCurrentMethod = getTerminateFunction(b->getModule());
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto args = mCurrentMethod->arg_begin();
    setHandle(b, &*(args++));
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

    generateFinalizeMethod(b); // may be overridden by the Kernel subtype
    const auto outputs = getFinalOutputScalars(b);
    b->CreateFree(mHandle);
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
    assert (module != nullptr);
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
        report_fatal_error("Cannot find " + name);
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
        report_fatal_error("Cannot find " + name);
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
        report_fatal_error("Cannot find " + name);
    }
    return f;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareKernel(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    addBaseKernelProperties(b);
    addInternalKernelProperties(b);
    // NOTE: StructType::create always creates a new type even if an identical one exists.
    if (LLVM_UNLIKELY(mModule == nullptr)) {
        makeModule(b);
    }
    mKernelStateType = mModule->getTypeByName(getName());


    if (LLVM_LIKELY(mKernelStateType == nullptr)) {
        std::vector<llvm::Type *> fields;
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
void Kernel::addInternalScalar(llvm::Type * type, const llvm::StringRef name) {
    const auto index = mInternalScalars.size();
    mInternalScalars.emplace_back(type, name);
    addScalarToMap(name, ScalarType::Internal, index);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarIndex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned Kernel::getScalarIndex(const llvm::StringRef fieldName) const {
    const auto & field = getScalarField(fieldName);
    assert (mKernelStateType);
    unsigned index = field.index;
    switch (field.type) {
        case ScalarType::Internal:
            index += mOutputScalars.size();
        case ScalarType::Output:
            index += mInputScalars.size();
        case ScalarType::Input:
            break;
    }
    assert (index < mKernelStateType->getStructNumElements());
    return index;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief prepareCachedKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::prepareCachedKernel(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mKernelStateType != nullptr)) {
        report_fatal_error(getName() + ": cannot prepare kernel after kernel state finalized");
    }
    addBaseKernelProperties(b);
    mKernelStateType = getModule()->getTypeByName(getName());
    if (LLVM_UNLIKELY(mKernelStateType == nullptr)) {
        report_fatal_error("Kernel definition for " + getName() + " was not found in the cache!");
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
        WriteBitcodeToFile(getModule(), signature);
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
std::string Kernel::getStringHash(const llvm::StringRef str) {

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
Value * Kernel::createInstance(const std::unique_ptr<KernelBuilder> & b) {
    assert (mKernelStateType && "cannot create instance before calling prepareKernel() or prepareCachedKernel()");
    Constant * const size = ConstantExpr::getSizeOf(mKernelStateType);
    Value * handle = nullptr;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        handle = b->CreateAlignedMalloc(size, b->getPageSize());
        b->CreateMProtect(handle, size, CBuilder::Protect::READ);
    } else {
        handle = b->CreateAlignedMalloc(size, b->getCacheAlignment());
    }
//    mHandle = b->CreatePointerCast(handle, mKernelStateType->getPointerTo());
//    return mHandle;
    return b->CreatePointerCast(handle, mKernelStateType->getPointerTo());
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
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void Kernel::generateKernel(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(mIsGenerated)) return;
    b->setKernel(this);
    b->setModule(mModule);
    addKernelDeclarations(b);
    callGenerateInitializeMethod(b);
    callGenerateKernelMethod(b);
    callGenerateFinalizeMethod(b);
    addAdditionalFunctions(b);
    mIsGenerated = true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & b) {
    assert (mHandle && "was not set");
    Value * result = b->CreateCall(getTerminateFunction(b->getModule()), { mHandle });
    mHandle = nullptr;
    if (mOutputScalars.empty()) {
        assert (!result || result->getType()->isVoidTy());
        result = nullptr;
    }
    return result;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarField
 ** ------------------------------------------------------------------------------------------------------------- */
const Kernel::ScalarField & Kernel::getScalarField(const llvm::StringRef name) const {
    assert (!mScalarMap.empty());
    const auto f = mScalarMap.find(name);
    if (LLVM_UNLIKELY(f == mScalarMap.end())) {
        assert (!"could not find scalar!");
        report_fatal_error(getName() + " does not contain scalar: " + name);
    }
    return f->second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputScalarBinding
 ** ------------------------------------------------------------------------------------------------------------- */
Binding & Kernel::getInputScalarBinding(const llvm::StringRef name) {
    const ScalarField & field = getScalarField(name);
    if (LLVM_UNLIKELY(field.type != ScalarType::Input)) {
        report_fatal_error(getName() + "." + name + "is not an input scalar");
    }
    return mInputScalars[field.index];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputScalarBinding
 ** ------------------------------------------------------------------------------------------------------------- */
Binding & Kernel::getOutputScalarBinding(const llvm::StringRef name) {
    const ScalarField & field = getScalarField(name);
    if (LLVM_UNLIKELY(field.type != ScalarType::Output)) {
        report_fatal_error(getName() + "." + name + "is not an output scalar");
    }
    return mOutputScalars[field.index];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStreamPort
 ** ------------------------------------------------------------------------------------------------------------- */
Kernel::StreamSetPort Kernel::getStreamPort(const llvm::StringRef name) const {
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
const Binding & Kernel::getStreamBinding(const llvm::StringRef name) const {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    return (port == Port::Input) ? getInputStreamSetBinding(index) : getOutputStreamSetBinding(index);
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
 * @brief isCountable
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::isCountable(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount()) {
        return true;
    } else if (rate.isRelative()) {
        return isCountable(getStreamBinding(rate.getReference()));
    } else {
        return false;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isCalculable
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::isCalculable(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return true;
    } else if (rate.isRelative()) {
        return isCalculable(getStreamBinding(rate.getReference()));
    } else {
        return false;
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
 * @brief isUnknownRate
 ** ------------------------------------------------------------------------------------------------------------- */
bool Kernel::isUnknownRate(const Binding & binding) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isUnknown()) {
        return true;
    } else if (rate.isRelative()) {
        return isUnknownRate(getStreamBinding(rate.getReference()));
    } else {
        return false;
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
        addStreamToMap(input.getName(), Port::Input, i);
    }
    for (unsigned i = 0; i < mOutputStreamSets.size(); i++) {
        Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(output.getRelationship() == nullptr)) {
            report_fatal_error(getName()+ "." + output.getName() + " must be set upon construction");
        }
        addStreamToMap(output.getName(), Port::Output, i);
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
 * @brief getPopCountRateItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Kernel::getPopCountRateItemCount(const std::unique_ptr<KernelBuilder> & b, const ProcessingRate & rate, Value * const strideIndex) {
    assert (rate.isPopCount() || rate.isNegatedPopCount());
    Port refPort;
    unsigned refIndex = 0;
    std::tie(refPort, refIndex) = getStreamPort(rate.getReference());
    assert (refPort == Port::Input);
    Value * array = nullptr;
    if (rate.isNegatedPopCount()) {
        array = mNegatedPopCountRateArray[refIndex];
    } else {
        array = mPopCountRateArray[refIndex];
    }
    assert (array && "missing pop count array attribute");
    return b->CreateLoad(b->CreateGEP(array, strideIndex));
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
    name += "_O" + std::to_string((int)codegen::OptLevel);
    return name;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDefaultFamilyName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string Kernel::getDefaultFamilyName() const {
    std::string tmp;
    llvm::raw_string_ostream out(tmp);
    out << "F";
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
Kernel::Kernel(std::string && kernelName,
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
, mStride(0)
, mTerminationSignalPtr(nullptr)
, mIsFinal(nullptr)
, mNumOfStrides(nullptr)
, mKernelName(annotateKernelNameWithDebugFlags(std::move(kernelName))) {

}

Kernel::~Kernel() { }

// CONSTRUCTOR
SegmentOrientedKernel::SegmentOrientedKernel(std::string && kernelName,
                                             Bindings && stream_inputs,
                                             Bindings && stream_outputs,
                                             Bindings && scalar_parameters,
                                             Bindings && scalar_outputs,
                                             Bindings && internal_scalars)
: Kernel(std::move(kernelName),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_parameters), std::move(scalar_outputs),
         std::move(internal_scalars))  {

}


}
