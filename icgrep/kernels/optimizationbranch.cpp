#include "optimizationbranch.h"
#include <kernels/kernel_builder.h>
#include <boost/scoped_ptr.hpp>

#warning at compilation, this must verify that the I/O rates of the branch permits the rates of the branches

using namespace llvm;

namespace kernel {

using AttrId = Attribute::KindId;

const std::string OptimizationBranch::CONDITION_TAG = "@condition";

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkExternalMethods
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->linkExternalMethods(b);
    mFalseKernel->linkExternalMethods(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->generateInitializeMethod(b);
    mFalseKernel->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<llvm::Value *> & args) {
    mTrueKernel->initializeInstance(b, args);
    mFalseKernel->initializeInstance(b, args);
}

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
 * @brief isLocalBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isLocalBuffer(const Binding & output) {
    return output.getRate().isUnknown() || output.hasAttribute(AttrId::ManagedBuffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {

    BasicBlock * const loopCond = b->CreateBasicBlock("cond");
    BasicBlock * const nonZeroPath = b->CreateBasicBlock("nonZeroPath");
    BasicBlock * const allZeroPath = b->CreateBasicBlock("allZeroPath");
    BasicBlock * const mergePaths = b->CreateBasicBlock("mergePaths");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    const auto numOfConditionInputs = isa<StreamSet>(mCondition) ? 1 : 0;
    const auto numOfInputs = getNumOfStreamInputs() - numOfConditionInputs;
    std::vector<llvm::Value *> initialProcessedInputItems(numOfInputs, nullptr);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        if (isParamConstant(mInputStreamSets[i])) {
            initialProcessedInputItems[i] = b->CreateLoad(mProcessedInputItemPtr[i]);
        }
    }

    const auto numOfOutputs = getNumOfStreamOutputs();
    std::vector<llvm::Value *> initialProducedOutputItems(numOfOutputs, nullptr);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (isParamConstant(mOutputStreamSets[i])) {
            initialProducedOutputItems[i] = b->CreateLoad(mProducedOutputItemPtr[i]);
        }
    }

    BasicBlock * const entry = b->GetInsertBlock();
    b->CreateBr(loopCond);

    PHINode * terminatedPhi = nullptr;
    if (canSetTerminateSignal()) {
        b->SetInsertPoint(mergePaths);
        terminatedPhi = b->CreatePHI(b->getInt1Ty(), 2);
    }

    b->SetInsertPoint(loopCond);
    IntegerType * const sizeTy = b->getSizeTy();
    PHINode * const first = b->CreatePHI(sizeTy, 3);
    first->addIncoming(ZERO, entry);
    PHINode * const last = b->CreatePHI(sizeTy, 3);
    PHINode * const state = b->CreatePHI(b->getInt1Ty(), 3);
    state->addIncoming(b->getFalse(), entry);

    mProcessedInputItems.resize(numOfInputs);
    mAccessibleInputItemPhi.resize(numOfInputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        if (initialProcessedInputItems[i]) {
            PHINode * const inputPhi = b->CreatePHI(sizeTy, 2);
            inputPhi->addIncoming(initialProcessedInputItems[i], entry);
            mProcessedInputItems[i] = inputPhi;
        } else {
            mProcessedInputItems[i] = mProcessedInputItemPtr[i];
        }
        PHINode * const accessiblePhi = b->CreatePHI(sizeTy, 2);
        accessiblePhi->addIncoming(mAccessibleInputItems[i], entry);
        mAccessibleInputItemPhi[i] = accessiblePhi;
    }

    mProducedOutputItems.resize(numOfOutputs);
    mWritableOrConsumedOutputItemPhi.resize(numOfOutputs);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (initialProducedOutputItems[i]) {
            PHINode * const outputPhi = b->CreatePHI(sizeTy, 2);
            outputPhi->addIncoming(initialProducedOutputItems[i], entry);
            mProducedOutputItems[i] = outputPhi;
        } else {
            mProducedOutputItems[i] = mProducedOutputItemPtr[i];
        }
        PHINode * const writablePhi = b->CreatePHI(sizeTy, 2);
        if (isLocalBuffer(mOutputStreamSets[i])) {
            writablePhi->addIncoming(mConsumedOutputItems[i], entry);
        } else {
            writablePhi->addIncoming(mWritableOutputItems[i], entry);
        }
        mWritableOrConsumedOutputItemPhi[i] = writablePhi;
    }

    if (LLVM_LIKELY(isa<StreamSet>(mCondition))) {

        last->addIncoming(ZERO, entry);

        BasicBlock * const summarizeOneStride = b->CreateBasicBlock("summarizeOneStride", nonZeroPath);
        BasicBlock * const checkStride = b->CreateBasicBlock("checkStride", nonZeroPath);
        BasicBlock * const processStrides = b->CreateBasicBlock("processStrides", nonZeroPath);

        Constant * const strideCount = b->getSize(getStride() / b->getBitBlockWidth());

        Value * const streamCount = b->getInputStreamSetCount(CONDITION_TAG);
        Value * const blocksPerStride = b->CreateMul(streamCount, strideCount);

        Value * const offset = b->CreateMul(last, strideCount);
        Value * basePtr = b->getInputStreamBlockPtr(CONDITION_TAG, ZERO, offset);
        Type * const BitBlockTy = b->getBitBlockType();
        basePtr = b->CreatePointerCast(basePtr, BitBlockTy->getPointerTo());
        b->CreateBr(summarizeOneStride);

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
        Value * const more = b->CreateICmpNE(nextIteration, blocksPerStride);
        b->CreateCondBr(more, summarizeOneStride, checkStride);

        // Check the merged value of our condition block(s); if it differs from
        // the prior value or this is our last stride, then process the strides.
        // Note, however, initially state is "indeterminate" so we silently
        // ignore the first stride unless it is also our last.
        b->SetInsertPoint(checkStride);
        Value * const nextState = b->bitblock_any(merged);
        Value * const sameState = b->CreateICmpEQ(nextState, state);
        Value * const firstStride = b->CreateICmpEQ(last, ZERO);
        Value * const continuation = b->CreateOr(sameState, firstStride);
        Value * const nextIndex = b->CreateAdd(last, ONE);
        Value * const notLastStride = b->CreateICmpNE(nextIndex, mNumOfStrides);
        Value * const checkNextStride = b->CreateAnd(continuation, notLastStride);
        last->addIncoming(nextIndex, checkStride);
        first->addIncoming(first, checkStride);
        state->addIncoming(nextState, checkStride);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            if (initialProcessedInputItems[i]) {
                PHINode * const inputPhi = cast<PHINode>(mProcessedInputItems[i]);
                inputPhi->addIncoming(inputPhi, checkStride);
            }
            PHINode * const accessiblePhi = mAccessibleInputItemPhi[i];
            accessiblePhi->addIncoming(accessiblePhi, checkStride);
        }

        for (unsigned i = 0; i < numOfOutputs; ++i) {
            if (initialProducedOutputItems[i]) {
                PHINode * const outputPhi = cast<PHINode>(mProducedOutputItems[i]);
                outputPhi->addIncoming(outputPhi, checkStride);
            }
            PHINode * const writablePhi = mWritableOrConsumedOutputItemPhi[i];
            writablePhi->addIncoming(writablePhi, checkStride);
        }

        b->CreateLikelyCondBr(checkNextStride, loopCond, processStrides);

        // Process every stride between [first, index)
        b->SetInsertPoint(processStrides);
        // state is implicitly "indeterminate" during our first stride
        Value * const selectedPath = b->CreateSelect(firstStride, nextState, state);
        b->CreateCondBr(selectedPath, nonZeroPath, allZeroPath);

    } else {
        last->addIncoming(mNumOfStrides, entry);

        Value * const cond = b->getScalarField(CONDITION_TAG);
        b->CreateCondBr(b->CreateIsNotNull(cond), nonZeroPath, allZeroPath);
    }

    // make the actual calls and take any potential termination signal
    b->SetInsertPoint(nonZeroPath);
    callKernel(b, mTrueKernel, first, last, terminatedPhi);
    b->CreateBr(mergePaths);

    b->SetInsertPoint(allZeroPath);
    callKernel(b, mFalseKernel, first, last, terminatedPhi);
    b->CreateBr(mergePaths);

    b->SetInsertPoint(mergePaths);
    last->addIncoming(last, mergePaths);
    first->addIncoming(last, mergePaths);
    state->addIncoming(b->getFalse(), mergePaths);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mInputStreamSets[i];
        Value * updatedInputCount = nullptr;
        if (isParamConstant(input)) {
            Value * const itemCount = getItemCountIncrement(b, input, first, last);
            PHINode * const inputPhi = cast<PHINode>(mProcessedInputItems[i]);
            updatedInputCount = b->CreateAdd(inputPhi, itemCount);
            inputPhi->addIncoming(updatedInputCount, mergePaths);
        }
        PHINode * const accessiblePhi = mAccessibleInputItemPhi[i];
        if (updatedInputCount == nullptr) {
            updatedInputCount = b->CreateLoad(mProducedOutputItems[i]);
        }
        Value * const remaining = b->CreateSub(accessiblePhi, updatedInputCount);
        accessiblePhi->addIncoming(remaining, mergePaths);
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mOutputStreamSets[i];
        Value * updatedOutputCount = nullptr;
        if (isParamConstant(output)) {
            Value * const itemCount = getItemCountIncrement(b, output, first, last);
            PHINode * const outputPhi = cast<PHINode>(mProducedOutputItems[i]);
            updatedOutputCount = b->CreateAdd(outputPhi, itemCount);
            outputPhi->addIncoming(updatedOutputCount, mergePaths);
        }
        PHINode * const writablePhi = mWritableOrConsumedOutputItemPhi[i];
        if (isLocalBuffer(output)) {
            writablePhi->addIncoming(writablePhi, mergePaths);
        } else {
            if (updatedOutputCount == nullptr) {
                updatedOutputCount = b->CreateLoad(mProducedOutputItems[i]);
            }
            Value * const remaining = b->CreateSub(writablePhi, updatedOutputCount);
            writablePhi->addIncoming(remaining, mergePaths);
        }
    }

    Value * const lastStride = b->CreateICmpNE(last, mNumOfStrides);
    Value * const finished = b->CreateOr(lastStride, terminatedPhi);
    b->CreateLikelyCondBr(finished, exit, loopCond);

    b->SetInsertPoint(exit);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::callKernel(const std::unique_ptr<KernelBuilder> & b,
                                    const Kernel * const kernel,
                                    Value * const first, Value * const last,
                                    PHINode * const terminatedPhi) {

    std::vector<Value *> args;
    args.reserve(mCurrentMethod->arg_size());
    args.push_back(kernel->getHandle()); // handle
    args.push_back(b->CreateSub(last, first)); // numOfStrides
    const auto numOfInputs = kernel->getNumOfStreamInputs();
    for (unsigned i = 0; i < numOfInputs; i++) {

        const Binding & input = kernel->getInputStreamSetBinding(i);
        const auto & buffer = mStreamSetInputBuffers[i];
        // logical base input address
        args.push_back(buffer->getBaseAddress(b.get()));
        // processed input items
        args.push_back(mProcessedInputItems[i]);
        // accessible input items (after non-deferred processed item count)
        args.push_back(mAccessibleInputItemPhi[i]);
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
        if (!isLocalBuffer(output)) {
            const auto & buffer = mStreamSetOutputBuffers[i];
            args.push_back(buffer->getBaseAddress(b.get()));
        }
        args.push_back(mProducedOutputItems[i]);
        args.push_back(mWritableOrConsumedOutputItemPhi[i]);
    }

    Value * terminated = b->CreateCall(kernel->getDoSegmentFunction(b->getModule()), args);
    if (terminatedPhi) {
        if (!kernel->canSetTerminateSignal()) {
            terminated = b->getFalse();
        }
        terminatedPhi->addIncoming(terminated, b->GetInsertBlock());
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemCountIncrement
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranch::getItemCountIncrement(const std::unique_ptr<KernelBuilder> & b, const Binding & binding,
                                                  Value * const first, Value * const last) const {

    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed()) {
        Constant * const strideLength = b->getSize(ceiling(getUpperBound(binding) * getStride()));
        Value * const numOfStrides = b->CreateSub(last, first);
        return b->CreateMul(numOfStrides, strideLength);
    } else { assert (rate.isPopCount() || rate.isNegatedPopCount());
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

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->generateFinalizeMethod(b);
    mFalseKernel->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addAdditionalFunctions
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addAdditionalFunctions(const std::unique_ptr<KernelBuilder> & b) {
    mTrueKernel->addAdditionalFunctions(b);
    mFalseKernel->addAdditionalFunctions(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranch::finalizeInstance(const std::unique_ptr<KernelBuilder> & b) {

    // TODO: to have a returnable result here, we need to store the
    // scalars in this kernel or the pipeline.

//    Value * trueResult = mTrueKernel->finalizeInstance(b);
//    Value * falseResult = mFalseKernel->finalizeInstance(b);
    return nullptr;
}

void OptimizationBranch::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mTrueKernel->addInternalKernelProperties(b);
    mFalseKernel->addInternalKernelProperties(b);
}

std::vector<Value *> OptimizationBranch::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    return std::vector<Value *>{};
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
         std::move(scalar_inputs), std::move(scalar_outputs), {})
, mCondition(condition.get())
, mTrueKernel(nonZeroKernel.get())
, mFalseKernel(allZeroKernel.get()) {

}

OptimizationBranch::~OptimizationBranch() {

}

}
