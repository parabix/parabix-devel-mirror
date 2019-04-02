#include "pipeline_compiler.hpp"

// TODO: Print Total CPU Cycles

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelCycleCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addCycleCounterProperties(BuilderRef b, const unsigned kernel) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        // TODO: make these thread local to prevent false cache sharing
        const auto prefix = makeKernelName(kernel) + CYCLE_COUNT_SUFFIX;
        IntegerType * const int64Ty = b->getInt64Ty();
        mPipelineKernel->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::AFTER_SYNCHRONIZATION));
        mPipelineKernel->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::BUFFER_EXPANSION));
        mPipelineKernel->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::AFTER_KERNEL_CALL));
        mPipelineKernel->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::FINAL));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief startOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::startCycleCounter(BuilderRef b, const CycleCounter type) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        mCycleCounters[type] = b->CreateReadCycleCounter();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief startOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * PipelineCompiler::getBufferExpansionCycleCounter(BuilderRef b) const {
    Value * ptr = nullptr;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        b->setKernel(mPipelineKernel);
        const auto prefix = makeKernelName(mKernelIndex) + CYCLE_COUNT_SUFFIX;
        ptr = b->getScalarFieldPtr(prefix + std::to_string(BUFFER_EXPANSION));
        b->setKernel(mKernel);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateCycleCounter(BuilderRef b, const CycleCounter start, const CycleCounter end) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        Value * const endCount = b->CreateReadCycleCounter();
        Value * const duration = b->CreateSub(endCount, mCycleCounters[start]);
        const auto prefix = makeKernelName(mKernelIndex) + CYCLE_COUNT_SUFFIX;
        b->setKernel(mPipelineKernel);
        Value * const counterPtr = b->getScalarFieldPtr(prefix + std::to_string(end));
        Value * const runningCount = b->CreateLoad(counterPtr, true);
        Value * const updatedCount = b->CreateAdd(runningCount, duration);
        b->CreateStore(updatedCount, counterPtr);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectPrincipleCycleCountBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamPort PipelineCompiler::selectPrincipleCycleCountBinding(const unsigned kernel) const {
    const auto numOfInputs = in_degree(kernel, mBufferGraph);
    assert (degree(kernel, mBufferGraph));
    if (numOfInputs == 0) {
        return StreamPort{PortType::Output, 0};
    } else {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = getInputBinding(kernel, i);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                return StreamPort{PortType::Input, i};
            }
        }
        return StreamPort{PortType::Input, 0};
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {

        Value * totalCycles = b->getInt64(0);
        size_t maxLength = 0;
        b->setKernel(mPipelineKernel);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const auto prefix = makeKernelName(i) + CYCLE_COUNT_SUFFIX;
            Value * const cycles = b->getScalarField(prefix + std::to_string(CycleCounter::FINAL));
            totalCycles = b->CreateAdd(totalCycles, cycles);
            const Kernel * const kernel = getKernel(i);
            maxLength = std::max(maxLength, kernel->getName().length());
        }
        maxLength += 4;

        Type * const doubleTy = b->getDoubleTy();
        Constant * const fOneHundred = ConstantFP::get(doubleTy, 100.0);
        Value * const fTotalCycles = b->CreateUIToFP(totalCycles, doubleTy);

        // Print the title line

        SmallVector<char, 100> buffer;
        raw_svector_ostream title(buffer);

        title << "Name";
        title.indent(maxLength - 4);
        title << "     ITEMS " // items processed;
                 "    CYCLES " // CPU cycles,
                 "   RATE " // cycles per item,
                 " SYNC " // synchronization %,
                 " BUFF " // buffer expansion %,
                 " OVER " // overhead %,
                 "  EXEC " // execution %,
                 "    %\n"; // % of Total CPU Cycles.

        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = b->getInt32(STDERR_FILENO);
        titleArgs[1] = b->CreatePointerCast(b->GetString(title.str()), b->getInt8PtrTy());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Print each kernel line
        buffer.clear();

        raw_svector_ostream line(buffer);
        line << "%-" << maxLength << "s" // name
                "%10.2E " // items processed;
                "%10.2E " // CPU cycles,
                "%7.2f " // cycles per item,
                "%5.1f " // synchronization %,
                "%5.1f " // buffer expansion %,
                "%5.1f " // overhead %,
                "%6.1f " // execution %,
                "%5.1f\n"; // % of Total CPU Cycles.

        FixedArray<Value *, 11> args;
        args[0] = titleArgs[0];
        args[1] = b->CreatePointerCast(b->GetString(line.str()), b->getInt8PtrTy());

        for (auto i = FirstKernel; i <= LastKernel; ++i) {

            const auto binding = selectPrincipleCycleCountBinding(i);
            Value * const items = b->getScalarField(makeBufferName(i, binding) + ITEM_COUNT_SUFFIX);
            Value * const fItems = b->CreateUIToFP(items, doubleTy);

            const auto prefix = makeKernelName(i) + CYCLE_COUNT_SUFFIX;
            Value * const synchronizationCycles = b->getScalarField(prefix + std::to_string(CycleCounter::AFTER_SYNCHRONIZATION));
            Value * const fSynchronizationCycles = b->CreateUIToFP(synchronizationCycles, doubleTy);
            Value * const fSynchronizationCycles100 = b->CreateFMul(fSynchronizationCycles, fOneHundred);

            Value * const bufferExpandCycles = b->getScalarField(prefix + std::to_string(CycleCounter::BUFFER_EXPANSION));
            Value * const fBufferExpandCycles = b->CreateUIToFP(bufferExpandCycles, doubleTy);
            Value * const fBufferExpandCycles100 = b->CreateFMul(fBufferExpandCycles, fOneHundred);

            Value * const executionCycles = b->getScalarField(prefix + std::to_string(CycleCounter::AFTER_KERNEL_CALL));
            Value * const fExecutionCycles = b->CreateUIToFP(executionCycles, doubleTy);
            Value * const fExecutionCycles100 = b->CreateFMul(fExecutionCycles, fOneHundred);

            Value * const cycles = b->getScalarField(prefix + std::to_string(CycleCounter::FINAL));
            Value * const fCycles = b->CreateUIToFP(cycles, doubleTy);

            Value * const fSynchronizationPerc = b->CreateFDiv(fSynchronizationCycles100, fCycles);
            Value * const fBufferExpandPerc = b->CreateFDiv(fBufferExpandCycles100, fCycles);
            Value * const fExecutionPerc = b->CreateFDiv(fExecutionCycles100, fCycles);

            Value * const fKnownOverheads = b->CreateFAdd(b->CreateFAdd(fSynchronizationCycles100, fBufferExpandPerc), fExecutionCycles100);
            Value * const fCyclesPerItem = b->CreateFDiv(fCycles, fItems);
            Value * const fCycles100 = b->CreateFMul(fCycles, fOneHundred);

            Value * const fUnknownOverhead = b->CreateFDiv(b->CreateFSub(fCycles100, fKnownOverheads), fCycles);
            Value * const fPercentageOfTotal = b->CreateFDiv(fCycles100, fTotalCycles);

            const Kernel * const kernel = getKernel(i);
            args[2] = b->CreatePointerCast(b->GetString(kernel->getName()), b->getInt8PtrTy());
            args[3] = fItems;
            args[4] = fCycles;
            args[5] = fCyclesPerItem;
            args[6] = fSynchronizationPerc;
            args[7] = fBufferExpandPerc;
            args[8] = fUnknownOverhead;
            args[9] = fExecutionPerc;
            args[10] = fPercentageOfTotal;

            b->CreateCall(b->GetDprintf(), args);
        }
    }
}

}
