#include "pipeline_compiler.hpp"

// TODO: Print Total CPU Cycles

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelCycleCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addCycleCounterProperties(BuilderRef b, const unsigned kernel) {
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
inline const Binding & PipelineCompiler::selectPrincipleCycleCountBinding(const unsigned kernel) const {
    const auto numOfInputs = in_degree(kernel, mBufferGraph);
    assert (degree(kernel, mBufferGraph));
    if (numOfInputs == 0) {
        return getOutputBinding(kernel, 0);
    } else {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = getInputBinding(kernel, i);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                return input;
            }
        }
        return getInputBinding(kernel, 0);
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
        for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
            const auto prefix = makeKernelName(i);
            Value * const counterPtr = b->getScalarFieldPtr(prefix + CYCLE_COUNT_SUFFIX);
            Value * const cycles = b->CreateLoad(counterPtr);
            totalCycles = b->CreateAdd(totalCycles, cycles);
        }
        Value* fTotalCycle = b->CreateUIToFP(totalCycles, b->getDoubleTy());

        for (unsigned k = FirstKernel; k <= LastKernel; ++k) {

            const auto & binding = selectPrincipleCycleCountBinding(k);
            Value * const items = b->getScalarField(makeBufferName(k, binding) + ITEM_COUNT_SUFFIX);

            Value * fItems = b->CreateUIToFP(items, b->getDoubleTy());
            const auto name = makeKernelName(k);
            Value * const counterPtr = b->getScalarFieldPtr(name + CYCLE_COUNT_SUFFIX);
            Value * cycles = b->CreateLoad(counterPtr);
            Value * fCycles = b->CreateUIToFP(cycles, b->getDoubleTy());
            Value * percentage = b->CreateFDiv(b->CreateFMul(fCycles, FP_100), fTotalCycle);

            const Kernel * const kernel = getKernel(k);
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
