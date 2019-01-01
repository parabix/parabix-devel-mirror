#include "optimizationbranch.h"
#include <kernels/kernel_builder.h>

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
 * @brief isParamConstant
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isParamConstant(const Binding & binding) {
    assert (!binding.isDeferred());
    const ProcessingRate & rate = binding.getRate();
    return rate.isFixed() || rate.isPopCount() || rate.isNegatedPopCount();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasParam
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool hasParam(const Binding & binding) {
    return !binding.getRate().isRelative();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::callKernel(const std::unique_ptr<KernelBuilder> & b,
                                    const Kernel * const kernel, std::vector<Value *> & args,
                                    PHINode * const terminatedPhi) {
    args[0] = kernel->getHandle();
    Value * terminated = b->CreateCall(kernel->getDoSegmentFunction(b->getModule()), args);
    if (terminatedPhi) {
        if (LLVM_UNLIKELY(kernel->canSetTerminateSignal())) {
            terminated = b->getFalse();
        }
        terminatedPhi->addIncoming(terminated, b->GetInsertBlock());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
#if 0

    BasicBlock * const loopCond = b->CreateBasicBlock("cond");
    BasicBlock * const nonZeroPath = b->CreateBasicBlock("nonZeroPath");
    BasicBlock * const allZeroPath = b->CreateBasicBlock("allZeroPath");
    BasicBlock * const mergePaths = b->CreateBasicBlock("mergePaths");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    const auto numOfInputs = getNumOfStreamInputs();
    std::vector<Value *> initialInputItems(numOfInputs, nullptr);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        if (isParamConstant(mInputStreamSets[i])) {
            initialInputItems[i] = b->CreateLoad(mProcessedInputItemPtr[i]);
        }
    }

    const auto numOfOutputs = getNumOfStreamOutputs();
    std::vector<Value *> initialOutputItems(numOfOutputs, nullptr);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (isParamConstant(mOutputStreamSets[i])) {
            initialOutputItems[i] = b->CreateLoad(mProducedOutputItemPtr[i]);
        }
    }

    BasicBlock * const entry = b->GetInsertBlock();
    b->CreateBr(loopCond);

    std::vector<Value *> args;
    PHINode * terminatedPhi = nullptr;
    if (canSetTerminateSignal()) {
        b->SetInsertPoint(mergePaths);
        terminatedPhi = b->CreatePHINode(b->getInt1Ty(), 2);
    }

    b->SetInsertPoint(loopCond);

    if (LLVM_LIKELY(isa<StreamSet>(mCondition))) {

        Type * const BitBlockTy = b->getBitBlockType();
        IntegerType * const sizeTy = b->getSizeTy();

        PHINode * const index = b->CreatePHI(sizeTy, 3);
        index->addIncoming(ZERO, entry);
        PHINode * const first = b->CreatePHI(sizeTy, 3);
        first->addIncoming(ZERO, entry);
        PHINode * const state = b->CreatePHI(b->getInt1Ty(), 3);
        state->addIncoming(b->getFalse(), entry);

        const auto numOfInputs = getNumOfStreamInputs() - 1; // the final input is our condition stream
        std::vector<PHINode *> inputPhis(numOfInputs);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            PHINode * const inputPhi = b->CreatePHI(sizeTy, 3);
            inputPhi->addIncoming(getAccessibleInputItems(i), entry);
            inputPhis[i] = inputPhi;
        }

        const auto numOfOutputs = getNumOfStreamOutputs();
        std::vector<PHINode *> outputPhis(numOfOutputs);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            PHINode * const outputPhi = b->CreatePHI(sizeTy, 3);
            outputPhi->addIncoming(getWritableInputItems(i), entry);
            outputPhis[i] = outputPhi;
        }


        BasicBlock * const summarizeOneStride = b->CreateBasicBlock("summarizeOneStride", nonZeroPath);
        BasicBlock * const checkStride = b->CreateBasicBlock("checkStride", nonZeroPath);
        BasicBlock * const processStrides = b->CreateBasicBlock("processStrides", nonZeroPath);

        Constant * const strideCount = b->getSize(getStride() / b->getBitBlockWidth());

        Value * const streamCount = b->getInputStreamSetCount(CONDITION_TAG);
        Value * const blocksPerStride = b->CreateMul(streamCount, strideCount);


        Value * const offset = b->CreateMul(index, strideCount);
        Value * basePtr = b->getInputStreamBlockPtr(CONDITION_TAG, ZERO, offset);
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
        Value * const firstStride = b->CreateICmpEQ(index, ZERO);
        Value * const continuation = b->CreateOr(sameState, firstStride);
        Value * const nextIndex = b->CreateAdd(index, ONE);
        Value * const notLastStride = b->CreateICmpNE(nextIndex, mNumOfStrides);
        Value * const checkNextStride = b->CreateAnd(continuation, notLastStride);
        index->addIncoming(nextIndex, checkStride);
        first->addIncoming(first, checkStride);
        state->addIncoming(nextState, checkStride);
        b->CreateLikelyCondBr(checkNextStride, loopCond, processStrides);

        // Process every stride between [first, index)
        b->SetInsertPoint(processStrides);

        // build our kernel call
        args.reserve(mCurrentMethod->arg_size());
        args.push_back(nullptr); // handle
        args.push_back(b->CreateSub(index, first)); // numOfStrides
        for (unsigned i = 0; i < numOfInputs; i++) {
            const StreamSetBuffer * const buffer = mStreamSetInputBuffers[i];
            // logical base input address
            args.push_back(buffer->getBaseAddress(b.get()));

            // processed input items
            const Binding & input = mInputStreamSets[i];
            if (isParamAddressable(input)) {
                args.push_back(mProcessedInputItemPtr[i]); // updatable
            }  else if (isParamConstant(input)) {
                args.push_back(b->CreateLoad(mProcessedInputItemPtr[i]));  // constant
            }

            // accessible input items (after non-deferred processed item count)
            args.push_back(sizeTy);

            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
                args.push_back(b->CreateGEP(mPopCountRateArray[i], first));
            }
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                args.push_back(b->CreateGEP(mNegatedPopCountRateArray[i], first));
            }
        }

        // state is implicitly "indeterminate" during our first stride
        Value * const currentState = b->CreateSelect(firstStride, nextState, state);
        b->CreateCondBr(currentState, nonZeroPath, allZeroPath);

    } else {

        Value * const cond = b->getScalarField(CONDITION_TAG);
        const auto n = mCurrentMethod->arg_size();
        args.resize(n);
        auto arg = mCurrentMethod->arg_begin();
        for (unsigned i = 1; i != n; ++i) {
            assert (arg != mCurrentMethod->arg_end());
            args[i] = *(++arg);
        }
        assert (args[0] == nullptr);
        b->CreateCondBr(b->CreateIsNotNull(cond), nonZeroPath, allZeroPath);
    }

    // make the actual calls and take any potential termination signal
    b->SetInsertPoint(nonZeroPath);
    callKernel(b, mTrueKernel, args, terminatedPhi);
    b->CreateBr(mergePaths);

    b->SetInsertPoint(allZeroPath);
    callKernel(b, mFalseKernel, args, terminatedPhi);
    b->CreateBr(mergePaths);


#endif
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
    addAttributesFrom({mTrueKernel, mFalseKernel});
}

OptimizationBranch::~OptimizationBranch() {

}

}
