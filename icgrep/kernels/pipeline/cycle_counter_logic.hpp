#include "pipeline_compiler.hpp"

#warning TODO: Print Total CPU Cycles

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelCycleCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addInternalKernelCycleCountProperties(BuilderRef b, const unsigned kernel) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        const auto prefix = makeKernelName(kernel);
        mPipelineKernel->addInternalScalar(b->getInt64Ty(), prefix + CYCLE_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief startOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::startOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        mCycleCountStart = b->CreateReadCycleCounter();
    } else {
        mCycleCountStart = nullptr;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        const auto prefix = makeKernelName(mKernelIndex);
        b->setKernel(mPipelineKernel);
        Value * const counterPtr = b->getScalarFieldPtr(prefix + CYCLE_COUNT_SUFFIX);
        Value * runningCount = b->CreateLoad(counterPtr, true);
        Value * const cycleCountEnd = b->CreateReadCycleCounter();
        Value * const segmentCycleCount = b->CreateSub(cycleCountEnd, mCycleCountStart);
        runningCount = b->CreateAdd(runningCount, segmentCycleCount);
        b->CreateStore(runningCount, counterPtr);
        b->setKernel(mKernel);
        mCycleCountStart = cycleCountEnd;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectPrincipleCycleCountBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & PipelineCompiler::selectPrincipleCycleCountBinding(const unsigned i) const {
    const Kernel * const kernel = mPipeline[i];
    if (LLVM_UNLIKELY(kernel->getNumOfStreamInputs() == 0)) {
        return kernel->getOutputStreamSetBinding(0);
    } else {
        return kernel->getInputStreamSetBinding(0);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        Value* FP_100 = ConstantFP::get(b->getDoubleTy(), 100.0);
        Value* totalCycles = b->getSize(0);
        b->setKernel(mPipelineKernel);
        for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
            const auto prefix = makeKernelName(i);
            Value * const counterPtr = b->getScalarFieldPtr(prefix + CYCLE_COUNT_SUFFIX);
            Value * const cycles = b->CreateLoad(counterPtr);
            totalCycles = b->CreateAdd(totalCycles, cycles);
        }
        Value* fTotalCycle = b->CreateUIToFP(totalCycles, b->getDoubleTy());

        for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {

            const Binding & binding = selectPrincipleCycleCountBinding(i);
            Value * const items = b->getScalarField(makeBufferName(i, binding) + ITEM_COUNT_SUFFIX);

            const auto numOfInputs = mKernel->getNumOfStreamInputs();
            for (unsigned i = 0; i < numOfInputs; ++i) {
                const Binding & input = mKernel->getInputStreamSetBinding(i);
                const auto prefix = makeBufferName(mKernelIndex, input);
                mInitiallyProcessedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
                if (input.isDeferred()) {
                    mInitiallyProcessedDeferredItemCount[i] = b->getScalarField(prefix + DEFERRED_ITEM_COUNT_SUFFIX);
                }
            }
            const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
            for (unsigned i = 0; i < numOfOutputs; ++i) {
                const Binding & output = mKernel->getOutputStreamSetBinding(i);
                const auto prefix = makeBufferName(mKernelIndex, output);
                mInitiallyProducedItemCount[i] = b->getScalarField(prefix + ITEM_COUNT_SUFFIX);
            }

            Value * fItems = b->CreateUIToFP(items, b->getDoubleTy());
            const auto name = makeKernelName(i);
            Value * const counterPtr = b->getScalarFieldPtr(name + CYCLE_COUNT_SUFFIX);
            Value * cycles = b->CreateLoad(counterPtr);
            Value * fCycles = b->CreateUIToFP(cycles, b->getDoubleTy());
            Value * percentage = b->CreateFDiv(b->CreateFMul(fCycles, FP_100), fTotalCycle);

            const Kernel * const kernel = mPipeline[i];
            const auto formatString = kernel->getName() + ": %7.2e items processed;"
                                                          "  %7.2e CPU cycles,"
                                                          "  %6.2f cycles per item,"
                                                          "  %2.2f%% of Total CPU Cycles.\n";
            Value * stringPtr = b->CreatePointerCast(b->GetString(formatString), b->getInt8PtrTy());
            b->CreateCall(b->GetDprintf(), {b->getInt32(2), stringPtr, fItems, fCycles, b->CreateFDiv(fCycles, fItems), percentage});
        }
    }
}

}
