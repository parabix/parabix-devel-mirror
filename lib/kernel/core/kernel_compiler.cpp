#include <kernel/core/kernel_compiler.h>
#include <kernel/core/kernel_builder.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Constants.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;
using namespace boost;

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return ((sizeof(unsigned) * CHAR_BIT) - 1U) - __builtin_clz(v);
}

namespace kernel {

using AttrId = Attribute::KindId;
using Rational = ProcessingRate::Rational;
using RateId = ProcessingRate::KindId;
using StreamSetPort = Kernel::StreamSetPort;
using PortType = Kernel::PortType;

const static std::string BUFFER_HANDLE_SUFFIX = "_buffer";
const static std::string TERMINATION_SIGNAL = "__termination_signal";

#define BEGIN_SCOPED_REGION {
#define END_SCOPED_REGION }

// TODO: this check is a bit too strict in general; if the pipeline could request data/
// EOF padding from the MemorySource kernel, it would be possible to re-enable.

// #define CHECK_IO_ADDRESS_RANGE

// TODO: split the init/final into two methods each, one to do allocation/init, and the
// other final/deallocate? Would potentially allow us to reuse the kernel/stream set
// memory in the nested engine if each init method memzero'ed them. Would need to change
// the "main" method.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::generateKernel(BuilderRef b) {
    // NOTE: make sure to keep and reset the original compiler here. A kernel could generate new kernels and
    // reuse the same KernelBuilder to do so; this could result in unexpected behaviour if the this function
    // exits without restoring the original compiler state.
    auto const oc = b->getCompiler();
    b->setCompiler(this);
    constructStreamSetBuffers(b);
    addBaseInternalProperties(b);
    mTarget->addInternalProperties(b);
    mTarget->constructStateTypes(b);
    mTarget->addKernelDeclarations(b);
    callGenerateInitializeMethod(b);
    callGenerateInitializeThreadLocalMethod(b);
    callGenerateDoSegmentMethod(b);
    callGenerateFinalizeThreadLocalMethod(b);
    callGenerateFinalizeMethod(b);
    mTarget->addAdditionalFunctions(b);
    b->setCompiler(oc);

    // TODO: we could create a LLVM optimization pass manager here and execute it on this kernel;
    // it would allow the programmer to define a set of optimizations they want executed on the
    // kernel code. However, if compilers are intended to be short lived, we wouldn't be able to
    // easily share it amongst the same type of kernel compiler.

    // What is the cost of generating a pass manager instance for each compiled kernel vs.
    // the complexity of using a factory?
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructStreamSetBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::constructStreamSetBuffers(BuilderRef b) {
    mStreamSetInputBuffers.clear();
    const auto numOfInputStreams = mInputStreamSets.size();
    mStreamSetInputBuffers.resize(numOfInputStreams);
    for (unsigned i = 0; i < numOfInputStreams; ++i) {
        const Binding & input = mInputStreamSets[i];
        mStreamSetInputBuffers[i].reset(new ExternalBuffer(b, input.getType(), true, 0));
    }
    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = mOutputStreamSets.size();
    mStreamSetOutputBuffers.resize(numOfOutputStreams);
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        mStreamSetOutputBuffers[i].reset(new ExternalBuffer(b, output.getType(), true, 0));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addBaseInternalProperties
  ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::addBaseInternalProperties(BuilderRef b) {
     // If an output is a managed buffer, store its handle.
    const auto n = mOutputStreamSets.size();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) {
            Type * const handleTy = mStreamSetOutputBuffers[i]->getHandleType(b);
            mTarget->addInternalScalar(handleTy, output.getName() + BUFFER_HANDLE_SUFFIX);
        }
    }
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        mTarget->addInternalScalar(b->getSizeTy(), TERMINATION_SIGNAL);
    } else {
        mTarget->addNonPersistentScalar(b->getSizeTy(), TERMINATION_SIGNAL);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const size_t n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelCompiler::callGenerateInitializeMethod(BuilderRef b) {
    b->setCompiler(this);
    mCurrentMethod = mTarget->getInitializeFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    auto arg = mCurrentMethod->arg_begin();
    const auto arg_end = mCurrentMethod->arg_end();
    auto nextArg = [&]() {
        assert (arg != arg_end);
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };
    if (LLVM_LIKELY(mTarget->isStateful())) {
        setHandle(nextArg());
    }
    if (LLVM_LIKELY(mTarget->isStateful())) {
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
    loadHandlesOfLocalOutputStreamSets(b);
    // any kernel can set termination on initialization
    mTerminationSignalPtr = b->getScalarFieldPtr(TERMINATION_SIGNAL);
    b->CreateStore(b->getSize(KernelBuilder::TerminationCode::None), mTerminationSignalPtr);
    mTarget->generateInitializeMethod(b);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect) && mTarget->isStateful())) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::READ);
    }

    b->CreateRet(b->CreateLoad(mTerminationSignalPtr));
    clearInternalStateAfterCodeGen();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindFamilyInitializationArguments
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::bindFamilyInitializationArguments(BuilderRef /* b */, ArgIterator & /* arg */, const ArgIterator & /* arg_end */) const {

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelCompiler::callGenerateInitializeThreadLocalMethod(BuilderRef b) {
    if (mTarget->hasThreadLocal()) {
        b->setCompiler(this);
        assert (mSharedHandle == nullptr && mThreadLocalHandle == nullptr);
        mCurrentMethod = mTarget->getInitializeThreadLocalFunction(b);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
        auto arg = mCurrentMethod->arg_begin();
        auto nextArg = [&]() {
            assert (arg != mCurrentMethod->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };
        if (LLVM_LIKELY(mTarget->isStateful())) {
            setHandle(nextArg());
        }
        mThreadLocalHandle = b->CreateCacheAlignedMalloc(mTarget->getThreadLocalStateType());
        initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);
        mTarget->generateInitializeThreadLocalMethod(b);
        Value * const retVal = mThreadLocalHandle;
        b->CreateRet(retVal);
        clearInternalStateAfterCodeGen();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setDoSegmentProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::setDoSegmentProperties(BuilderRef b, const ArrayRef<Value *> args) {

    // WARNING: any change to this must be reflected in Kernel::addDoSegmentDeclaration,
    // Kernel::getDoSegmentFields, KernelCompiler::getDoSegmentProperties,
    // and PipelineCompiler::writeKernelCall

    auto arg = args.begin();
    auto nextArg = [&]() {
        assert (arg != args.end());
        Value * const v = *arg; assert (v);
        std::advance(arg, 1);
        return v;
    };
    if (LLVM_LIKELY(mTarget->isStateful())) {
        setHandle(nextArg());
        assert (mSharedHandle->getType()->getPointerElementType() == mTarget->getSharedStateType());
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(getHandle(), "%s: shared handle cannot be null", b->GetString(getName()));
        }
    }
    if (LLVM_UNLIKELY(mTarget->hasThreadLocal())) {
        setThreadLocalHandle(nextArg());
        assert (mThreadLocalHandle->getType()->getPointerElementType() == mTarget->getThreadLocalStateType());
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            b->CreateAssert(getThreadLocalHandle(), "%s: thread local handle cannot be null", b->GetString(getName()));
        }
    }
    mNumOfStrides = nextArg();
    if (LLVM_UNLIKELY(Kernel::requiresExplicitPartialFinalStride(mTarget))) {
        mIsFinal = b->CreateIsNull(mNumOfStrides);
    } else {
        mIsFinal = nextArg();
    }
    const auto fixedRateLCM = mTarget->getFixedRateLCM();
    mExternalSegNo = nullptr;
    if (LLVM_UNLIKELY(mTarget->hasAttribute(AttrId::InternallySynchronized))) {
        mExternalSegNo = nextArg();
    }
    mFixedRateFactor = nullptr;
    if (LLVM_LIKELY(fixedRateLCM.numerator() != 0)) {
        mFixedRateFactor = nextArg();
    }

    initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);

    // NOTE: the disadvantage of passing the stream pointers as a parameter is that it becomes more difficult
    // to access a stream set from a LLVM function call. We could create a stream-set aware function creation
    // and call system here but that is not an ideal way of handling this.

    const auto numOfInputs = getNumOfStreamInputs();

    reset(mProcessedInputItemPtr, numOfInputs);
    reset(mAccessibleInputItems, numOfInputs);
    reset(mAvailableInputItems, numOfInputs);
    reset(mUpdatableProcessedInputItemPtr, numOfInputs);

    #ifdef CHECK_IO_ADDRESS_RANGE
    auto checkStreamRange = [&](const std::unique_ptr<StreamSetBuffer> & buffer, const Binding & binding, Value * const startItemCount) {

        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << "StreamSet " << getName() << ":" << binding.getName();

        PointerType * const int8PtrTy = b->getInt8PtrTy();

        ConstantInt * const BLOCK_WIDTH = b->getSize(b->getBitBlockWidth());

        Value * const baseAddress = buffer->getBaseAddress(b);

        ConstantInt * const ZERO = b->getSize(0);
        Value * const startIndex = b->CreateUDiv(startItemCount, BLOCK_WIDTH);
        Value * const startPtr = buffer->getStreamBlockPtr(b, baseAddress, ZERO, startIndex);
        Value * const start = b->CreatePointerCast(startPtr, int8PtrTy);

        Value * const MAX = b->CreateSub(buffer->getStreamSetCount(b), b->getSize(1));
        Value * const endIndex = b->CreateCeilUDiv(buffer->getCapacity(b), BLOCK_WIDTH);
        Value * const endPtr = buffer->getStreamBlockPtr(b, baseAddress, MAX, endIndex);
        Value * const end = b->CreatePointerCast(endPtr, int8PtrTy);

        Value * const length = b->CreatePtrDiff(end, start);
        b->CheckAddress(start, length, out.str());

    };
    #endif

    IntegerType * const sizeTy = b->getSizeTy();
    for (unsigned i = 0; i < numOfInputs; i++) {
        /// ----------------------------------------------------
        /// virtual base address
        /// ----------------------------------------------------
        auto & buffer = mStreamSetInputBuffers[i]; assert (buffer.get());
        const Binding & input = mInputStreamSets[i];
        Value * const virtualBaseAddress = nextArg();
        assert (virtualBaseAddress->getType() == buffer->getPointerType());
        Value * const localHandle = b->CreateAllocaAtEntryPoint(buffer->getHandleType(b));
        buffer->setHandle(localHandle);
        assert (isa<ExternalBuffer>(buffer));
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
        AllocaInst * const processedItems = b->CreateAllocaAtEntryPoint(sizeTy);
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
        assert (accessible);
        assert (accessible->getType() == sizeTy);
        mAccessibleInputItems[i] = accessible;
        Value * capacity = b->CreateAdd(processed, accessible);
        mAvailableInputItems[i] = capacity;
        if (input.hasLookahead()) {
            capacity = b->CreateAdd(capacity, b->getSize(input.getLookahead()));
        }
        buffer->setCapacity(b, capacity);
        #ifdef CHECK_IO_ADDRESS_RANGE
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            checkStreamRange(buffer, input, processed);
        }
        #endif
    }

    // set all of the output buffers
    const auto numOfOutputs = getNumOfStreamOutputs();
    reset(mProducedOutputItemPtr, numOfOutputs);
    reset(mInitiallyProducedOutputItems, numOfOutputs);
    reset(mWritableOutputItems, numOfOutputs);
    reset(mConsumedOutputItems, numOfOutputs);
    reset(mUpdatableProducedOutputItemPtr, numOfOutputs);
    reset(mUpdatableOutputBaseVirtualAddressPtr, numOfOutputs);

    const auto canTerminate = canSetTerminateSignal();

    for (unsigned i = 0; i < numOfOutputs; i++) {
        /// ----------------------------------------------------
        /// logical buffer base address
        /// ----------------------------------------------------

        const std::unique_ptr<StreamSetBuffer> & buffer = mStreamSetOutputBuffers[i]; assert (buffer.get());
        const Binding & output = mOutputStreamSets[i];
        const auto isLocal = Kernel::isLocalBuffer(output);

        if (LLVM_UNLIKELY(isLocal)) {
            // If an output is a managed buffer, the address is stored within the state instead
            // of being passed in through the function call.
            mUpdatableOutputBaseVirtualAddressPtr[i] = nextArg();
            Value * const handle = getScalarFieldPtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
            buffer->setHandle(handle);
        } else {
            Value * const virtualBaseAddress = nextArg();
            assert (virtualBaseAddress->getType() == buffer->getPointerType());
            Value * const localHandle = b->CreateAllocaAtEntryPoint(buffer->getHandleType(b));
            buffer->setHandle(localHandle);
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
        mInitiallyProducedOutputItems[i] = produced;
        AllocaInst * const producedItems = b->CreateAllocaAtEntryPoint(sizeTy);
        b->CreateStore(produced, producedItems);
        mProducedOutputItemPtr[i] = producedItems;
        /// ----------------------------------------------------
        /// writable / consumed item count
        /// ----------------------------------------------------
        Value * writable = nullptr;
        if (requiresItemCount(output)) {
            writable = nextArg(); assert (writable);
        } else {
            assert (mFixedRateFactor);
            writable = b->CreateCeilUMulRate(mFixedRateFactor, rate.getRate() / fixedRateLCM);
        }
        assert (writable && writable->getType() == sizeTy);
        mWritableOutputItems[i] = writable;
        if (LLVM_UNLIKELY(isLocal)) {
            Value * const consumed = nextArg();
            assert (consumed->getType() == sizeTy);
            mConsumedOutputItems[i] = consumed;
        } else {
            Value * const capacity = b->CreateAdd(produced, writable);
            buffer->setCapacity(b, capacity);
        }
        #ifdef CHECK_IO_ADDRESS_RANGE
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            checkStreamRange(buffer, output, produced);
        }
        #endif
    }
    assert (arg == args.end());

    // initialize the termination signal if this kernel can set it
    mTerminationSignalPtr = nullptr;
    if (canTerminate) {
        mTerminationSignalPtr = b->getScalarFieldPtr(TERMINATION_SIGNAL);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            Value * const unterminated =
                b->CreateICmpEQ(b->CreateLoad(mTerminationSignalPtr), b->getSize(KernelBuilder::TerminationCode::None));
            b->CreateAssert(unterminated, getName() + ".doSegment was called after termination?");
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelCompiler::callGenerateDoSegmentMethod(BuilderRef b) {

    assert (mInputStreamSets.size() == mStreamSetInputBuffers.size());
    assert (mOutputStreamSets.size() == mStreamSetOutputBuffers.size());

    b->setCompiler(this);
    mCurrentMethod = mTarget->getDoSegmentFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));

    BEGIN_SCOPED_REGION
    Vec<Value *, 64> args;
    args.reserve(mCurrentMethod->arg_size());
    for (auto ArgI = mCurrentMethod->arg_begin(); ArgI != mCurrentMethod->arg_end(); ++ArgI) {
        args.push_back(&(*ArgI));
    }
    setDoSegmentProperties(b, args);
    END_SCOPED_REGION

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle, CBuilder::Protect::WRITE);
    }

    mTarget->generateKernelMethod(b);

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
        // Write the virtual base address out to inform the pipeline of any changes
        const auto & buffer = mStreamSetOutputBuffers[i];
        if (mUpdatableOutputBaseVirtualAddressPtr[i]) {
            Value * const baseAddress = buffer->getBaseAddress(b);
            if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                const Binding & output = mOutputStreamSets[i];
                out << getName() << ":%s is returning a virtual base address "
                                    "computed from a null base address.";
                b->CreateAssert(baseAddress, out.str(), b->GetString(output.getName()));
            }
            Constant * const LOG_2_BLOCK_WIDTH = b->getSize(floor_log2(b->getBitBlockWidth()));
            Constant * const ZERO = b->getSize(0);
            Value * produced = mInitiallyProducedOutputItems[i];
            // TODO: will LLVM optimizations replace the following with the already loaded value?
            // If not, re-loading it here may reduce register pressure / compilation time.
            if (mUpdatableProducedOutputItemPtr[i]) {
                produced = b->CreateLoad(mUpdatableProducedOutputItemPtr[i]);
            }
            Value * const blockIndex = b->CreateLShr(produced, LOG_2_BLOCK_WIDTH);
            Value * vba = buffer->getStreamLogicalBasePtr(b.get(), baseAddress, ZERO, blockIndex);
            vba = b->CreatePointerCast(vba, buffer->getPointerType());
            b->CreateStore(vba, mUpdatableOutputBaseVirtualAddressPtr[i]);
        }
        if (mUpdatableProducedOutputItemPtr[i]) {
            Value * const items= b->CreateLoad(mProducedOutputItemPtr[i]);
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
    clearInternalStateAfterCodeGen();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentProperties
 *
 * Reverse of the setDoSegmentProperties operation; used by the PipelineKernel when constructing internal threads
 * to simplify passing of the state data.
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> KernelCompiler::getDoSegmentProperties(BuilderRef b) const {

    // WARNING: any change to this must be reflected in addDoSegmentDeclaration, getDoSegmentFields,
    // setDoSegmentProperties, and PipelineCompiler::writeKernelCall

    std::vector<Value *> props;
    if (LLVM_LIKELY(mTarget->isStateful())) {
        props.push_back(mSharedHandle); assert (mSharedHandle);
    }
    if (LLVM_UNLIKELY(mTarget->hasThreadLocal())) {
        props.push_back(mThreadLocalHandle); assert (mThreadLocalHandle);
    }
    props.push_back(mNumOfStrides); assert (mNumOfStrides);
    if (LLVM_UNLIKELY(!Kernel::requiresExplicitPartialFinalStride(mTarget))) {
        props.push_back(mIsFinal);
    }
    if (LLVM_UNLIKELY(mTarget->hasAttribute(AttrId::InternallySynchronized))) {
        props.push_back(mExternalSegNo);
    }
    if (LLVM_LIKELY(mTarget->hasFixedRate())) {
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
        const auto isLocal = Kernel::isLocalBuffer(output);
        if (LLVM_UNLIKELY(isLocal)) {
            // If an output is a managed buffer, the address is stored within the state instead
            // of being passed in through the function call.
            Value * const handle = getScalarFieldPtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
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
        /// writable / consumed item count
        /// ----------------------------------------------------
        if (requiresItemCount(output)) {
            props.push_back(mWritableOutputItems[i]);
        }
        if (LLVM_UNLIKELY(isLocal)) {
            props.push_back(mConsumedOutputItems[i]);
        }
    }
    return props;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelCompiler::callGenerateFinalizeThreadLocalMethod(BuilderRef b) {
    if (mTarget->hasThreadLocal()) {
        b->setCompiler(this);
        mCurrentMethod = mTarget->getFinalizeThreadLocalFunction(b);
        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
        auto arg = mCurrentMethod->arg_begin();
        auto nextArg = [&]() {
            assert (arg != mCurrentMethod->arg_end());
            Value * const v = &*arg;
            std::advance(arg, 1);
            return v;
        };
        if (LLVM_LIKELY(mTarget->isStateful())) {
            setHandle(nextArg());
        }
        mThreadLocalHandle = nextArg();
        initializeScalarMap(b, InitializeScalarMapOptions::IncludeThreadLocal);
        mTarget->generateFinalizeThreadLocalMethod(b);
        b->CreateRetVoid();
        clearInternalStateAfterCodeGen();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callGenerateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void KernelCompiler::callGenerateFinalizeMethod(BuilderRef b) {

    b->setCompiler(this);
    mCurrentMethod = mTarget->getFinalizeFunction(b);
    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", mCurrentMethod));
    if (LLVM_LIKELY(mTarget->isStateful())) {
        auto args = mCurrentMethod->arg_begin();
        setHandle(&*(args++));
        assert (args == mCurrentMethod->arg_end());
    }
    initializeScalarMap(b, InitializeScalarMapOptions::SkipThreadLocal);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableMProtect))) {
        b->CreateMProtect(mSharedHandle,CBuilder::Protect::WRITE);
    }
    loadHandlesOfLocalOutputStreamSets(b);
    mTarget->generateFinalizeMethod(b); // may be overridden by the Kernel subtype
    const auto outputs = getFinalOutputScalars(b);
    if (LLVM_LIKELY(mTarget->isStateful())) {
        b->CreateFree(mSharedHandle);
    }
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
    clearInternalStateAfterCodeGen();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> KernelCompiler::getFinalOutputScalars(BuilderRef b) {
    const auto n = mOutputScalars.size();
    std::vector<Value *> outputs(n);
    for (unsigned i = 0; i < n; ++i) {
        outputs[i] = b->CreateLoad(getScalarFieldPtr(b.get(), mOutputScalars[i].getName()));
    }
    return outputs;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeScalarMap
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::initializeScalarMap(BuilderRef b, const InitializeScalarMapOptions options) {

    FixedArray<Value *, 2> indices;
    indices[0] = b->getInt32(0);
    unsigned sharedIndex = 0;
    #ifndef NDEBUG
    const auto sharedTy = mTarget->getSharedStateType();
    #define CHECK_TYPES(H,T) ((H == nullptr && T == nullptr) || (H && T && H->getType() == T->getPointerTo()))
    assert ("incorrect shared handle/type!" && CHECK_TYPES(mSharedHandle, sharedTy));
    const auto sharedCount = sharedTy ? sharedTy->getStructNumElements() : 0U;
    const auto threadLocalTy = mTarget->getThreadLocalStateType();
    if (options == InitializeScalarMapOptions::IncludeThreadLocal) {
        assert ("incorrect thread local handle/type!" && CHECK_TYPES(mThreadLocalHandle, threadLocalTy));
    }
    #undef CHECK_TYPES
    const auto threadLocalCount = threadLocalTy ? threadLocalTy->getStructNumElements() : 0U;
    #endif

    mScalarFieldMap.clear();

    #ifndef NDEBUG
    auto noDuplicateFileName = [&](const bool valid, const StringRef bindingName) {
        if (LLVM_LIKELY(valid)) return true;
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << "Kernel " << getName() << " contains two scalar fields named " << bindingName;
        report_fatal_error(out.str());
    };

    auto isMatchingType = [&](Value * const scalar, Type * const expectedType, const StringRef bindingName) {
        Type * const type = scalar->getType()->getPointerElementType();
        if (LLVM_LIKELY(type == expectedType)) return true;
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << "Scalar " << getName() << '.' << bindingName << " was expected to be ";
        expectedType->print(out);
        out << " but was stored as ";
        type->print(out);
        report_fatal_error(out.str());
    };
    #endif

    auto enumerate = [&](const Bindings & bindings) {
        for (const auto & binding : bindings) {
            assert (sharedIndex < sharedCount);
            indices[1] = b->getInt32(sharedIndex++);
            Value * scalar = b->CreateInBoundsGEP(mSharedHandle, indices);
            assert (isMatchingType(scalar, binding.getType(), binding.getName()));
            const auto i = mScalarFieldMap.insert(std::make_pair(binding.getName(), scalar));
            assert(noDuplicateFileName(i.second, binding.getName()));
        }
    };

    enumerate(mInputScalars);
    enumerate(mOutputScalars);

    unsigned threadLocalIndex = 0;
    for (const auto & binding : mInternalScalars) {
        Value * scalar = nullptr;
        switch (binding.getScalarType()) {
            case ScalarType::Internal:
                assert (sharedIndex < sharedCount);
                indices[1] = b->getInt32(sharedIndex++);
                scalar = b->CreateInBoundsGEP(mSharedHandle, indices);
                break;
            case ScalarType::ThreadLocal:
                if (options == InitializeScalarMapOptions::SkipThreadLocal) continue;
                assert (threadLocalIndex < threadLocalCount);
                indices[1] = b->getInt32(threadLocalIndex++);
                scalar = b->CreateInBoundsGEP(mThreadLocalHandle, indices);
                break;
            case ScalarType::NonPersistent:
                scalar = b->CreateAllocaAtEntryPoint(binding.getValueType());
                b->CreateStore(Constant::getNullValue(binding.getValueType()), scalar);
                break;
            default: llvm_unreachable("I/O scalars cannot be internal");
        }
        assert (isMatchingType(scalar, binding.getValueType(), binding.getName()));
        const auto i = mScalarFieldMap.insert(std::make_pair(binding.getName(), scalar));
        assert(noDuplicateFileName(i.second, binding.getName()));
    }
    assert (mSharedHandle == nullptr || sharedIndex == sharedCount);
    assert (mThreadLocalHandle == nullptr || threadLocalIndex == threadLocalCount);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeBindingMap
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::initializeIOBindingMap() {

    auto enumerate = [&](const Bindings & bindings, const BindingType type) {
        const auto n = bindings.size();
        for (unsigned i = 0; i < n; ++i) {
            const auto & binding = bindings[i];
            mBindingMap.insert(std::make_pair(binding.getName(), BindingMapEntry{type, i}));
        }
    };

    enumerate(mInputScalars, BindingType::ScalarInput);
    enumerate(mOutputScalars, BindingType::ScalarOutput);
    enumerate(mInputStreamSets, BindingType::StreamInput);
    enumerate(mOutputStreamSets, BindingType::StreamOutput);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const BindingMapEntry & KernelCompiler::getBinding(const BindingType type, const llvm::StringRef name) const {

    const auto f = mBindingMap.find(name);
    if (f != mBindingMap.end()) {
        const BindingMapEntry & entry = f->second;
        assert (entry.Type == type);
        return entry;
    }

    SmallVector<char, 256> tmp;
    raw_svector_ostream out(tmp);
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
StreamSetPort KernelCompiler::getStreamPort(const StringRef name) const {

    // NOTE: temporary refactoring step to limit changes outside of the kernel class

    static_assert(static_cast<unsigned>(BindingType::StreamInput) == static_cast<unsigned>(PortType::Input), "");
    static_assert(static_cast<unsigned>(BindingType::StreamOutput) == static_cast<unsigned>(PortType::Output), "");

    const auto f = mBindingMap.find(name);
    if (LLVM_LIKELY(f != mBindingMap.end())) {

        const BindingMapEntry & entry = f->second;
        switch (entry.Type) {
            case BindingType::StreamInput:
            case BindingType::StreamOutput:
                return StreamSetPort(static_cast<PortType>(entry.Type), entry.Index);
            default: break;
        }
    }

    SmallVector<char, 256> tmp;
    raw_svector_ostream out(tmp);
    out << "Kernel " << getName() << " does not contain a streamset named " << name;
    report_fatal_error(out.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarValuePtr
 ** ------------------------------------------------------------------------------------------------------------- */
Value * KernelCompiler::getScalarFieldPtr(KernelBuilder * /* b */, const StringRef name) const {
    if (LLVM_UNLIKELY(mScalarFieldMap.empty())) {
        SmallVector<char, 256> tmp;
        raw_svector_ostream out(tmp);
        out << "Scalar map for " << getName() << " was not initialized prior to calling getScalarValuePtr";
        report_fatal_error(out.str());
    } else {
        const auto f = mScalarFieldMap.find(name);
        if (LLVM_UNLIKELY(f == mScalarFieldMap.end())) {
            SmallVector<char, 1024> tmp;
            raw_svector_ostream out(tmp);
            out << "Scalar map for " << getName() << " does not contain " << name << "\n\n"
                "Currently contains:";
            char spacer = ' ';
            for (const auto & entry : mScalarFieldMap) {
                out << spacer << entry.getKey();
                spacer = ',';
            }
            report_fatal_error(out.str());
        }
        return f->second;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalarValuePtr
 ** ------------------------------------------------------------------------------------------------------------- */
bool KernelCompiler::hasScalarField(const llvm::StringRef name) const {
    return mScalarFieldMap.count(name) == 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLowerBound
 ** ------------------------------------------------------------------------------------------------------------- */
Rational KernelCompiler::getLowerBound(const Binding & binding) const {
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
Rational KernelCompiler::getUpperBound(const Binding & binding) const {
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
bool KernelCompiler::requiresOverflow(const Binding & binding) const {
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
 * @brief loadHandlesOfLocalOutputStreamSets
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::loadHandlesOfLocalOutputStreamSets(BuilderRef b) const {
    const auto numOfOutputs = mOutputStreamSets.size();
    for (unsigned i = 0; i < numOfOutputs; i++) {
        const Binding & output = mOutputStreamSets[i];
        if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) {
            Value * const handle = getScalarFieldPtr(b.get(), output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(handle);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearInternalStateAfterCodeGen
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelCompiler::clearInternalStateAfterCodeGen() {
    mSharedHandle = nullptr;
    mThreadLocalHandle = nullptr;
    mExternalSegNo = nullptr;
    mCurrentMethod = nullptr;
    mIsFinal = nullptr;
    mNumOfStrides = nullptr;
    mTerminationSignalPtr = nullptr;
    for (const auto & buffer : mStreamSetInputBuffers) {
        buffer->setHandle(nullptr);
    }
    for (const auto & buffer : mStreamSetOutputBuffers) {
        buffer->setHandle(nullptr);
    }
//    mUpdatableProcessedInputItemPtr.clear();
//    mProcessedInputItemPtr.clear();
//    mAccessibleInputItems.clear();
//    mAvailableInputItems.clear();
//    mPopCountRateArray.clear();
//    mNegatedPopCountRateArray.clear();
//    mUpdatableOutputBaseVirtualAddressPtr.clear();
//    mUpdatableProducedOutputItemPtr.clear();
//    mProducedOutputItemPtr.clear();
//    mInitiallyProducedOutputItems.clear();
//    mWritableOutputItems.clear();
//    mConsumedOutputItems.clear();
    mScalarFieldMap.clear();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
KernelCompiler::KernelCompiler(not_null<Kernel *> kernel) noexcept
: mTarget(kernel)
, mInputStreamSets(kernel->mInputStreamSets)
, mOutputStreamSets(kernel->mOutputStreamSets)
, mInputScalars(kernel->mInputScalars)
, mOutputScalars(kernel->mOutputScalars)
, mInternalScalars(kernel->mInternalScalars) {
    initializeIOBindingMap();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief destructor
 ** ------------------------------------------------------------------------------------------------------------- */
KernelCompiler::~KernelCompiler() {

}

}
