#include <kernels/optimizationbranch.h>
#include "optimizationbranch_compiler.hpp"
#include <kernels/kernel_builder.h>
#include <boost/graph/adjacency_list.hpp>
#include <boost/container/flat_map.hpp>
#include <llvm/Support/raw_ostream.h>
#include <toolchain/toolchain.h>

#warning at compilation, this must verify that the I/O rates of the branch permits the rates of the branches

#warning move most of this logic this into the optimizationbranch compiler

using namespace llvm;
using namespace boost;
using namespace boost::container;

namespace kernel {

using AttrId = Attribute::KindId;

using ScalarDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, Value *, unsigned>;
using ScalarVertex = ScalarDependencyGraph::vertex_descriptor;
using ScalarDependencyMap = flat_map<const Relationship *, ScalarVertex>;

const std::string OptimizationBranch::CONDITION_TAG = "@condition";

const static std::string BRANCH_PREFIX = "@B";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isParamConstant
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isParamConstant(const Binding & binding) {
    if (binding.isDeferred()) {
        return false;
    }
    const ProcessingRate & rate = binding.getRate();
    return rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isParamAddressable
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isParamAddressable(const Binding & binding) {
    if (binding.isDeferred()) {
        return true;
    }
    const ProcessingRate & rate = binding.getRate();
    return (rate.isBounded() || rate.isUnknown());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isLocalBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isLocalBuffer(const Binding & output) {
    return output.getRate().isUnknown() || output.hasAttribute(AttrId::ManagedBuffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadKernelHandle
 ** ------------------------------------------------------------------------------------------------------------- */
void loadHandle(const std::unique_ptr<KernelBuilder> & b, Kernel * const kernel, const std::string suffix) {
    if (LLVM_LIKELY(kernel->isStateful())) {
        Value * handle = b->getScalarField(BRANCH_PREFIX + suffix);
        if (kernel->hasFamilyName()) {
            handle = b->CreatePointerCast(handle, kernel->getKernelType()->getPointerTo());
        }
        kernel->setHandle(b, handle);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {


    BasicBlock * const nonZeroPath = b->CreateBasicBlock("nonZeroPath");
    BasicBlock * const allZeroPath = b->CreateBasicBlock("allZeroPath");
    BasicBlock * const exit = b->CreateBasicBlock("exit");



    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    loadHandle(b, mAllZeroKernel, "0");
    loadHandle(b, mNonZeroKernel, "1");


    if (LLVM_LIKELY(isa<StreamSet>(mCondition))) {

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const loopCond = b->CreateBasicBlock("cond", nonZeroPath);
        BasicBlock * const summarizeOneStride = b->CreateBasicBlock("summarizeOneStride", nonZeroPath);
        BasicBlock * const checkStride = b->CreateBasicBlock("checkStride", nonZeroPath);
        BasicBlock * const processStrides = b->CreateBasicBlock("processStrides", nonZeroPath);
        BasicBlock * const mergePaths = b->CreateBasicBlock("mergePaths", nonZeroPath);

        b->CreateBr(loopCond);

        b->SetInsertPoint(loopCond);
        IntegerType * const sizeTy = b->getSizeTy();
        IntegerType * const boolTy = b->getInt1Ty();
        PHINode * const currentFirstIndex = b->CreatePHI(sizeTy, 3, "firstStride");
        currentFirstIndex->addIncoming(ZERO, entry);
        PHINode * const currentLastIndex = b->CreatePHI(sizeTy, 3, "lastStride");
        currentLastIndex->addIncoming(ZERO, entry);
        PHINode * const currentState = b->CreatePHI(boolTy, 3);
        currentState->addIncoming(UndefValue::get(boolTy), entry);


        Constant * const blocksPerStride = b->getSize(getStride() / b->getBitBlockWidth());
        Value * const numOfConditionStreams = b->getInputStreamSetCount(CONDITION_TAG);
        Value * const numOfConditionBlocks = b->CreateMul(numOfConditionStreams, blocksPerStride);

        Value * const offset = b->CreateMul(currentLastIndex, blocksPerStride);
        Value * basePtr = b->getInputStreamBlockPtr(CONDITION_TAG, ZERO, offset);
        Type * const BitBlockTy = b->getBitBlockType();
        basePtr = b->CreatePointerCast(basePtr, BitBlockTy->getPointerTo());
        b->CreateBr(summarizeOneStride);

        // Predeclare some phi nodes

        b->SetInsertPoint(nonZeroPath);
        PHINode * const firstNonZeroIndex = b->CreatePHI(sizeTy, 2);
        PHINode * const lastNonZeroIndex = b->CreatePHI(sizeTy, 2);
        PHINode * const allZeroAfterNonZero = b->CreatePHI(boolTy, 2);

        b->SetInsertPoint(allZeroPath);
        PHINode * const firstAllZeroIndex = b->CreatePHI(sizeTy, 2);
        PHINode * const lastAllZeroIndex = b->CreatePHI(sizeTy, 2);
        PHINode * const nonZeroAfterAllZero = b->CreatePHI(boolTy, 2);

        PHINode * terminatedPhi = nullptr;
        if (canSetTerminateSignal()) {
            b->SetInsertPoint(mergePaths);
            terminatedPhi = b->CreatePHI(b->getInt1Ty(), 2);
        }

        // OR together every condition block in this stride
        b->SetInsertPoint(summarizeOneStride);
        PHINode * const iteration = b->CreatePHI(b->getSizeTy(), 2);
        iteration->addIncoming(ZERO, loopCond);
        PHINode * const merged = b->CreatePHI(BitBlockTy, 2);
        merged->addIncoming(Constant::getNullValue(BitBlockTy), loopCond);
        Value * value = b->CreateBlockAlignedLoad(basePtr, iteration);
        value = b->CreateOr(value, merged);
        merged->addIncoming(value, summarizeOneStride);
        Value * const nextIteration = b->CreateAdd(iteration, ONE);
        Value * const more = b->CreateICmpNE(nextIteration, numOfConditionBlocks);
        iteration->addIncoming(nextIteration, b->GetInsertBlock());
        b->CreateCondBr(more, summarizeOneStride, checkStride);

        // Check the merged value of our condition block(s); if it differs from
        // the prior value or this is our last stride, then process the strides.
        // Note, however, initially state is "indeterminate" so we silently
        // ignore the first stride unless it is also our last.
        b->SetInsertPoint(checkStride);
        Value * const nextState = b->bitblock_any(value);
        Value * const sameState = b->CreateICmpEQ(nextState, currentState);
        Value * const firstStride = b->CreateICmpEQ(currentLastIndex, ZERO);
        Value * const continuation = b->CreateOr(sameState, firstStride);
        Value * const nextIndex = b->CreateAdd(currentLastIndex, ONE);
//        Value * const lastStrideIndex = b->CreateUMin(mNumOfStrides, ONE);
        Value * const notLastStride = b->CreateICmpULT(nextIndex, mNumOfStrides);
        Value * const checkNextStride = b->CreateAnd(continuation, notLastStride);
        currentLastIndex->addIncoming(nextIndex, checkStride);
        currentFirstIndex->addIncoming(currentFirstIndex, checkStride);
        currentState->addIncoming(nextState, checkStride);
        b->CreateLikelyCondBr(checkNextStride, loopCond, processStrides);

        // Process every stride between [first, last)
        b->SetInsertPoint(processStrides);

        // state is implicitly "indeterminate" during our first stride
        Value * const selectedPath = b->CreateSelect(firstStride, nextState, currentState);
        firstNonZeroIndex->addIncoming(currentFirstIndex, processStrides);
        firstAllZeroIndex->addIncoming(currentFirstIndex, processStrides);
        // When we reach the last (but not necessarily final) stride of this kernel,
        // we will either "append" the final stride to the current run or complete
        // the current run then perform one more iteration for the final stride, depending
        // whether it flips the branch selection state.

//        b->CallPrintInt(" &&& nextState", nextState);
//        b->CallPrintInt(" &&& firstStride", firstStride);
//        b->CallPrintInt(" &&& continuation", continuation);
//        b->CallPrintInt(" &&& notLastStride", notLastStride);

//        b->CallPrintInt(" &&& nextIndex", nextIndex);
//        b->CallPrintInt(" &&& currentLastIndex", currentLastIndex);

//        b->CallPrintInt(" &&& mNumOfStrides", mNumOfStrides);
//        b->CallPrintInt(" &&& nextIndex", nextIndex);

        Value * const nextLast = b->CreateSelect(continuation, mNumOfStrides, nextIndex);

//        b->CallPrintInt(" &&& nextLast", nextLast);

        Value * const nextFirst = b->CreateSelect(continuation, mNumOfStrides, currentLastIndex);

//        b->CallPrintInt(" &&& nextFirst", nextFirst);


        lastNonZeroIndex->addIncoming(nextFirst, processStrides);
        lastAllZeroIndex->addIncoming(nextFirst, processStrides);
        Value * finished = b->CreateNot(notLastStride);
        Value * const flipLastState = b->CreateAnd(finished, b->CreateNot(continuation));
        nonZeroAfterAllZero->addIncoming(flipLastState, processStrides);
        allZeroAfterNonZero->addIncoming(flipLastState, processStrides);
        b->CreateCondBr(selectedPath, nonZeroPath, allZeroPath);

        // make the actual calls and take any potential termination signal
        b->SetInsertPoint(nonZeroPath);
        callKernel(b, mNonZeroKernel, firstNonZeroIndex, lastNonZeroIndex, terminatedPhi);
        BasicBlock * const nonZeroPathExit = b->GetInsertBlock();
        firstAllZeroIndex->addIncoming(nextFirst, nonZeroPathExit);
        lastAllZeroIndex->addIncoming(nextLast, nonZeroPathExit);
        nonZeroAfterAllZero->addIncoming(b->getFalse(), nonZeroPathExit);
        b->CreateUnlikelyCondBr(allZeroAfterNonZero, allZeroPath, mergePaths);

        b->SetInsertPoint(allZeroPath);
        callKernel(b, mAllZeroKernel, firstAllZeroIndex, lastAllZeroIndex, terminatedPhi);
        BasicBlock * const allZeroPathExit = b->GetInsertBlock();
        firstNonZeroIndex->addIncoming(nextFirst, allZeroPathExit);
        lastNonZeroIndex->addIncoming(nextLast, allZeroPathExit);
        allZeroAfterNonZero->addIncoming(b->getFalse(), allZeroPathExit);
        b->CreateUnlikelyCondBr(nonZeroAfterAllZero, nonZeroPath, mergePaths);

        b->SetInsertPoint(mergePaths);
        currentFirstIndex->addIncoming(nextFirst, mergePaths);
        currentLastIndex->addIncoming(nextLast, mergePaths);
        currentState->addIncoming(nextState, mergePaths);
        if (terminatedPhi) {
            finished = b->CreateOr(finished, terminatedPhi);
        }
        b->CreateLikelyCondBr(finished, exit, loopCond);

    } else {

//        Value * const cond = b->getScalarField(CONDITION_TAG);
//        b->CreateCondBr(b->CreateIsNotNull(cond), nonZeroPath, allZeroPath);

//        // make the actual calls and take any potential termination signal
//        b->SetInsertPoint(nonZeroPath);
//        callKernel(b, mNonZeroKernel, ZERO, mNumOfStrides, b->getTrue(), nullptr);
//        b->CreateBr(exit);

//        b->SetInsertPoint(allZeroPath);
//        callKernel(b, mAllZeroKernel, ZERO, mNumOfStrides, b->getTrue(), nullptr);
//        b->CreateBr(exit);
    }

    b->SetInsertPoint(exit);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::callKernel(const std::unique_ptr<KernelBuilder> & b,
                                    const Kernel * const kernel,
                                    Value * const first, Value * const last,
                                    PHINode * const terminatedPhi) {

//    b->CallPrintInt(" &&& " + kernel->getName() + "_first", first);
//    b->CallPrintInt(" &&& " + kernel->getName() + "_last", last);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        Value * const nonZeroLength = b->CreateICmpULT(first, last);
        Value * valid = b->CreateOr(nonZeroLength, b->CreateIsNull(last));
        b->CreateAssert(valid,
            "Branch cannot execute 0 strides unless this is the final stride");
    }

    Function * const doSegment = kernel->getDoSegmentFunction(b->getModule());

    BasicBlock * incrementItemCounts = nullptr;
    BasicBlock * kernelExit = nullptr;
    if (kernel->canSetTerminateSignal()) {
        incrementItemCounts = b->CreateBasicBlock("incrementItemCounts");
        kernelExit = b->CreateBasicBlock("kernelExit");
    }

    std::vector<Value *> args;
    args.reserve(doSegment->arg_size());
    if (kernel->isStateful()) {
        args.push_back(kernel->getHandle()); // handle
    }
    Value * const numOfStrides = b->CreateSub(last, first);
    args.push_back(numOfStrides); // numOfStrides
    const auto numOfInputs = kernel->getNumOfStreamInputs();

    Value * const isFinal = b->CreateIsNull(last);

    for (unsigned i = 0; i < numOfInputs; i++) {

        const Binding & input = kernel->getInputStreamSetBinding(i);
        const auto & buffer = mStreamSetInputBuffers[i];
        // logical base input address
        args.push_back(buffer->getBaseAddress(b.get()));
        // processed input items
        Value * processed = mProcessedInputItemPtr[i];
        if (isParamConstant(input)) {
            processed = b->CreateLoad(processed);
        }
        args.push_back(processed);
        // accessible input items (after non-deferred processed item count)
        Value * accessible = getItemCountIncrement(b, input, first, last, mAccessibleInputItems[i]);
        accessible = b->CreateSelect(isFinal, mAccessibleInputItems[i], accessible);
        args.push_back(accessible);
        // TODO: What if one of the branches requires this but the other doesn't?
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            args.push_back(b->CreateGEP(mPopCountRateArray[i], first));
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            args.push_back(b->CreateGEP(mNegatedPopCountRateArray[i], first));
        }
    }

    const auto numOfOutputs = kernel->getNumOfStreamOutputs();
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        const auto & buffer = mStreamSetOutputBuffers[i];
        args.push_back(buffer->getBaseAddress(b.get()));
        // produced
        Value * produced = mProducedOutputItemPtr[i];
        if (isParamConstant(output)) {
            produced = b->CreateLoad(produced);
        }
        args.push_back(produced);
        Value * writable = getItemCountIncrement(b, output, first, last, mWritableOutputItems[i]);
        writable = b->CreateSelect(isFinal, mWritableOutputItems[i], writable);
        args.push_back(writable);
    }

    Value * const terminated = b->CreateCall(doSegment, args);
    if (incrementItemCounts) {
        b->CreateUnlikelyCondBr(terminated, kernelExit, incrementItemCounts);

        b->SetInsertPoint(incrementItemCounts);
    }

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = kernel->getInputStreamSetBinding(i);
        if (isParamConstant(input)) {
            Value * const processed = b->CreateLoad(mProcessedInputItemPtr[i]);
            Value * const itemCount = getItemCountIncrement(b, input, first, last);
            Value * const updatedInputCount = b->CreateAdd(processed, itemCount);
            b->CreateStore(updatedInputCount, mProcessedInputItemPtr[i]);
        }
//        Value * const processed = b->CreateLoad(mProcessedInputItemPtr[i]);
//        b->CallPrintInt(" &&& " + input.getName() + "_processed'", processed);
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        if (isParamConstant(output)) {
            Value * const produced = b->CreateLoad(mProducedOutputItemPtr[i]);
            Value * const itemCount = getItemCountIncrement(b, output, first, last);
            Value * const updatedOutputCount = b->CreateAdd(produced, itemCount);
            b->CreateStore(updatedOutputCount, mProducedOutputItemPtr[i]);
        }
//        Value * const processed = b->CreateLoad(mProducedOutputItemPtr[i]);
//        b->CallPrintInt(" &&& " + output.getName() + "_produced'", processed);
    }

    if (incrementItemCounts) {
        terminatedPhi->addIncoming(terminated, b->GetInsertBlock());
        b->CreateBr(kernelExit);
        b->SetInsertPoint(kernelExit);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemCountIncrement
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranch::getItemCountIncrement(const std::unique_ptr<KernelBuilder> & b, const Binding & binding,
                                                  Value * const first, Value * const last, Value * const defaultValue) const {
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        Constant * const strideLength = b->getSize(ceiling(getUpperBound(binding) * getStride()));
        Value * const numOfStrides = b->CreateSub(last, first);
        return b->CreateMul(numOfStrides, strideLength);
    } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
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
        Constant * const ONE = b->getSize(1);
        Value * const currentIndex = b->CreateSub(last, ONE);
        Value * const currentSum = b->CreateLoad(b->CreateGEP(array, currentIndex));
        Value * const priorIndex = b->CreateSub(first, ONE);
        Value * const priorSum = b->CreateLoad(b->CreateGEP(array, priorIndex));
        return b->CreateSub(currentSum, priorSum);
    }
    return defaultValue;
}

// TODO: abstract this. it's a near copy of the pipeline kernel logic

void enumerateScalarProducerBindings(const std::unique_ptr<KernelBuilder> & b,
                                     const ScalarVertex producer,
                                     const Bindings & bindings,
                                     ScalarDependencyGraph & G,
                                     ScalarDependencyMap & M) {
    const auto n = bindings.size();
    for (unsigned i = 0; i < n; ++i) {
        const Binding & binding = bindings[i];
        const Relationship * const rel = binding.getRelationship();
        assert (M.count(rel) == 0);
        Value * const value = b->getScalarField(binding.getName());
        const auto buffer = add_vertex(value, G);
        add_edge(producer, buffer, i, G);
        M.emplace(rel, buffer);
    }
}

ScalarVertex makeIfConstant(const Binding & binding,
                            ScalarDependencyGraph & G,
                            ScalarDependencyMap & M) {
    const Relationship * const rel = binding.getRelationship();
    const auto f = M.find(rel);
    if (LLVM_LIKELY(f != M.end())) {
        return f->second;
    } else if (LLVM_LIKELY(isa<ScalarConstant>(rel))) {
        const auto bufferVertex = add_vertex(cast<ScalarConstant>(rel)->value(), G);
        M.emplace(rel, bufferVertex);
        return bufferVertex;
    } else {
        report_fatal_error("unknown scalar value");
    }
}

void enumerateScalarConsumerBindings(const ScalarVertex consumer,
                                     const Bindings & bindings,
                                     ScalarDependencyGraph & G,
                                     ScalarDependencyMap & M) {
    const auto n = bindings.size();
    for (unsigned i = 0; i < n; ++i) {
        const auto buffer = makeIfConstant(bindings[i], G, M);
        assert (buffer < num_vertices(G));
        add_edge(buffer, consumer, i, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initKernel
 ** ------------------------------------------------------------------------------------------------------------- */
Value * initKernel(const std::unique_ptr<KernelBuilder> & b,
                   const unsigned index,
                   Kernel * const kernel,
                   Function * const initializer,
                   const ScalarDependencyGraph & G) {
    std::vector<Value *> args;
    const auto hasHandle = kernel->isStateful() ? 1U : 0U;
    args.resize(hasHandle + in_degree(index, G));
    if (LLVM_LIKELY(hasHandle)) {
        Value * handle = kernel->createInstance(b);
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            handle = b->CreatePointerCast(handle, b->getVoidPtrTy());
        }
        b->setScalarField(BRANCH_PREFIX + std::to_string(index - 1), handle);
        args[0] = handle;
    }
    for (const auto e : make_iterator_range(in_edges(index, G))) {
        const auto j = hasHandle + G[e];
        const auto scalar = source(e, G);
        args[j] = G[scalar];
    }
    return b->CreateCall(initializer, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {

    ScalarDependencyGraph G(3);
    ScalarDependencyMap M;

    enumerateScalarProducerBindings(b, 0, getInputScalarBindings(), G, M);
    enumerateScalarConsumerBindings(1, mAllZeroKernel->getInputScalarBindings(), G, M);
    enumerateScalarConsumerBindings(2, mNonZeroKernel->getInputScalarBindings(), G, M);

    Module * const m = b->getModule();
    Value * const term2 = initKernel(b, 1, mAllZeroKernel, mAllZeroKernel->getInitFunction(m), G);
    Value * const term1 = initKernel(b, 2, mNonZeroKernel, mNonZeroKernel->getInitFunction(m), G);
    b->CreateStore(b->CreateOr(term1, term2), mTerminationSignalPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * callTerminate(const std::unique_ptr<KernelBuilder> & b, Kernel * kernel, const std::string suffix) {
    loadHandle(b, kernel, suffix);
    return kernel->finalizeInstance(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * allZeroResult = callTerminate(b, mAllZeroKernel, "0");
    Value * nonZeroResult = callTerminate(b, mNonZeroKernel, "1");
    if (LLVM_UNLIKELY(nonZeroResult || allZeroResult)) {
        report_fatal_error("OptimizationBranch does not support output scalars yet");
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    mNonZeroKernel->addKernelDeclarations(b);
    mAllZeroKernel->addKernelDeclarations(b);
    Kernel::addKernelDeclarations(b);
}

void addHandle(const std::unique_ptr<KernelBuilder> & b, const Kernel * const kernel, Bindings & scalars, const std::string suffix) {
    if (LLVM_LIKELY(kernel->isStateful())) {
        Type * handleType = nullptr;
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            handleType = b->getVoidPtrTy();
        } else {
            handleType = kernel->getKernelType()->getPointerTo();
        }
        scalars.emplace_back(handleType, BRANCH_PREFIX + suffix);
    }
}

OptimizationBranch::OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
    std::string && signature,
    not_null<Relationship *> condition,
    not_null<Kernel *> nonZeroKernel,
    not_null<Kernel *> allZeroKernel,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_inputs,
    Bindings && scalar_outputs)
: Kernel(b, TypeId::OptimizationBranch, std::move(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs),
         {})
, mCondition(condition.get())
, mNonZeroKernel(nonZeroKernel.get())
, mAllZeroKernel(allZeroKernel.get()) {
    addHandle(b, mAllZeroKernel, mInternalScalars, "0");
    addHandle(b, mNonZeroKernel, mInternalScalars, "1");
}

OptimizationBranch::~OptimizationBranch() {

}

}
