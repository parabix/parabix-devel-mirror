#include "pipeline_compiler.hpp"

#warning TODO: Print Total CPU Cycles

namespace kernel {

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
        Value * const cycleCountEnd = b->CreateReadCycleCounter();
        Value * const counterPtr = b->getCycleCountPtr();
        Value * runningCount = b->CreateLoad(counterPtr);
        Value * const segmentCycleCount = b->CreateSub(cycleCountEnd, mCycleCountStart);
        runningCount = b->CreateAdd(runningCount, segmentCycleCount);
        b->CreateStore(runningCount, counterPtr);
        mCycleCountStart = cycleCountEnd;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        Value* FP_100 = ConstantFP::get(b->getDoubleTy(), 100.0);
        Value* totalCycles = b->getSize(0);
        for (const auto & kernel : mPipeline) {
            b->setKernel(kernel);
            Value * cycles = b->CreateLoad(b->getCycleCountPtr());
            totalCycles = b->CreateAdd(totalCycles, cycles);
        }
        Value* fTotalCycle = b->CreateUIToFP(totalCycles, b->getDoubleTy());

        for (const auto & kernel : mPipeline) {
            b->setKernel(kernel);
            const auto & inputs = kernel->getInputStreamSetBindings();
            const auto & outputs = kernel->getOutputStreamSetBindings();
            Value * items = nullptr;
            if (inputs.empty()) {
                items = b->getProducedItemCount(outputs[0].getName());
            } else {
                items = b->getProcessedItemCount(inputs[0].getName());
            }
            Value * fItems = b->CreateUIToFP(items, b->getDoubleTy());
            Value * cycles = b->CreateLoad(b->getCycleCountPtr());
            Value * fCycles = b->CreateUIToFP(cycles, b->getDoubleTy());
            Value * percentage = b->CreateFDiv(b->CreateFMul(fCycles, FP_100), fTotalCycle);

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
