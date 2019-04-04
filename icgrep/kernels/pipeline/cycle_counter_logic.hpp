#include "pipeline_compiler.hpp"

// TODO: Print Total CPU Cycles

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelCycleCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addCycleCounterProperties(BuilderRef b, const unsigned kernel) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        // TODO: make these thread local to prevent false sharing and enable
        // analysis of thread distributions?
        const auto prefix = makeKernelName(kernel);
        IntegerType * const int64Ty = b->getInt64Ty();

        const auto prefix2 = prefix + STATISTICS_CYCLE_COUNT_SUFFIX;

        mPipelineKernel->addInternalScalar(int64Ty, prefix2 + std::to_string(CycleCounter::AFTER_SYNCHRONIZATION));
        mPipelineKernel->addInternalScalar(int64Ty, prefix2 + std::to_string(CycleCounter::BUFFER_EXPANSION));
        mPipelineKernel->addInternalScalar(int64Ty, prefix2 + std::to_string(CycleCounter::AFTER_KERNEL_CALL));
        mPipelineKernel->addInternalScalar(int64Ty, prefix2 + std::to_string(CycleCounter::FINAL));
    }

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {

        const auto prefix = makeKernelName(kernel);
        IntegerType * const int64Ty = b->getInt64Ty();

        // total # of segments processed by kernel
        mPipelineKernel->addInternalScalar(int64Ty, prefix + STATISTICS_SEGMENT_COUNT_SUFFIX);

        // # of blocked I/O channel attempts in which no strides
        // were possible (i.e., blocked on first iteration)
        const auto prefix3 = prefix + STATISTICS_BLOCKING_IO_SUFFIX + "I";
        const auto numOfInputs = getNumOfStreamInputs(kernel);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            mPipelineKernel->addInternalScalar(int64Ty, prefix3 + std::to_string(i));
        }
        const auto prefix4 = prefix + STATISTICS_BLOCKING_IO_SUFFIX + "O";
        const auto numOfOutputs = getNumOfStreamOutputs(kernel);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            // TODO: ignore dynamic buffers
            mPipelineKernel->addInternalScalar(int64Ty, prefix4 + std::to_string(i));
        }
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
        const auto prefix = makeKernelName(mKernelIndex) + STATISTICS_CYCLE_COUNT_SUFFIX;
        ptr = b->getScalarFieldPtr(prefix + std::to_string(BUFFER_EXPANSION));
        b->setKernel(mKernel);
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::updateCycleCounter(BuilderRef b, const CycleCounter start, const CycleCounter end) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        b->setKernel(mPipelineKernel);
        Value * const endCount = b->CreateReadCycleCounter();
        Value * const duration = b->CreateSub(endCount, mCycleCounters[start]);
        const auto prefix = makeKernelName(mKernelIndex) + STATISTICS_CYCLE_COUNT_SUFFIX;
        Value * const counterPtr = b->getScalarFieldPtr(prefix + std::to_string(end));
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, duration);
        b->CreateStore(updatedCount, counterPtr);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief incrementNumberOfSegmentsCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::incrementNumberOfSegmentsCounter(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {
        b->setKernel(mPipelineKernel);
        const auto fieldName =
            makeKernelName(mKernelIndex) + STATISTICS_SEGMENT_COUNT_SUFFIX;
        Value * const counterPtr = b->getScalarFieldPtr(fieldName);
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, b->getInt64(1));
        b->CreateStore(updatedCount, counterPtr);
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordBlockingIO
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::recordBlockingIO(BuilderRef b, const StreamPort port) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {
        b->setKernel(mPipelineKernel);
        const auto fieldName =
            makeKernelName(mKernelIndex) + STATISTICS_BLOCKING_IO_SUFFIX +
            (port.Type == PortType::Input ? "I" : "O") + std::to_string(port.Number);
        Value * const counterPtr = b->getScalarFieldPtr(fieldName);
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, mBlockedOnFirstStridePhi);
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
 * @brief printOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {

        // Print the title line

        size_t maxLength = 0;
        b->setKernel(mPipelineKernel);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            maxLength = std::max(maxLength, kernel->getName().length());
        }
        maxLength += 4;

        SmallVector<char, 100> buffer;
        raw_svector_ostream title(buffer);

        title << "CYCLE COUNTER:\n\n"

                 "  # "  // kernel #
                 "NAME"; // kernel Name
        title.indent(maxLength - 4);
        title << "     ITEMS " // items processed;
                 "    CYCLES " // CPU cycles,
                 "      RATE " // cycles per item,
                 " SYNC " // synchronization %,
                 " BUFF " // buffer expansion %,
                 " OVER " // overhead %,
                 "  EXEC " // execution %,
                 "    %%\n"; // % of Total CPU Cycles.

        Constant * const STDERR = b->getInt32(STDERR_FILENO);
        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = STDERR;
        titleArgs[1] = b->GetString(title.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Print each kernel line

        buffer.clear();

        raw_svector_ostream line(buffer);
        line << "%3" PRIu32 " " // kernel #
                "%-" << maxLength << "s" // name
                "%10.2E " // items processed;
                "%10.2E " // CPU cycles,
                "%10.2f " // cycles per item,
                "%5.1f " // synchronization %,
                "%5.1f " // buffer expansion %,
                "%5.1f " // overhead %,
                "%6.1f " // execution %,
                "%5.1f\n"; // % of Total CPU Cycles.

        FixedArray<Value *, 12> args;
        args[0] = STDERR;
        args[1] = b->GetString(line.str());

        Value * totalCycles = b->getInt64(0);
        b->setKernel(mPipelineKernel);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const auto prefix = makeKernelName(i) + STATISTICS_CYCLE_COUNT_SUFFIX;
            Value * const cycles = b->getScalarField(prefix + std::to_string(CycleCounter::FINAL));
            totalCycles = b->CreateAdd(totalCycles, cycles);
        }

        Type * const doubleTy = b->getDoubleTy();
        Constant * const fOneHundred = ConstantFP::get(doubleTy, 100.0);
        Value * const fTotalCycles = b->CreateUIToFP(totalCycles, doubleTy);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {

            const auto binding = selectPrincipleCycleCountBinding(i);
            Value * const items = b->getScalarField(makeBufferName(i, binding) + ITEM_COUNT_SUFFIX);
            Value * const fItems = b->CreateUIToFP(items, doubleTy);

            const auto prefix = makeKernelName(i) + STATISTICS_CYCLE_COUNT_SUFFIX;
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
            args[2] = b->getInt32(i);
            args[3] = b->GetString(kernel->getName());
            args[4] = fItems;
            args[5] = fCycles;
            args[6] = fCyclesPerItem;
            args[7] = fSynchronizationPerc;
            args[8] = fBufferExpandPerc;
            args[9] = fUnknownOverhead;
            args[10] = fExecutionPerc;
            args[11] = fPercentageOfTotal;

            b->CreateCall(b->GetDprintf(), args);
        }
        // print final new line
        FixedArray<Value *, 2> finalArgs;
        finalArgs[0] = STDERR;
        finalArgs[1] = b->GetString("\n");
        b->CreateCall(b->GetDprintf(), finalArgs);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalBlockingIOStatistics
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalBlockingIOStatistics(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {

        // Print the title line

        size_t maxKernelLength = 0;
        size_t maxBindingLength = 0;
        b->setKernel(mPipelineKernel);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            maxKernelLength = std::max(maxKernelLength, kernel->getName().length());
            for (const auto & e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
            for (const auto & e : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
        }
        maxKernelLength += 4;
        maxBindingLength += 4;

        SmallVector<char, 100> buffer;
        raw_svector_ostream line(buffer);

        line << "BLOCKING I/O:\n\n"
                "  # "  // kernel ID #  (only shown for first)
                 "KERNEL"; // kernel Name (only shown for first)
        line.indent(maxKernelLength - 6);
        line << "PORT";
        line.indent(5 + maxBindingLength - 4); // I/O Type (e.g., input port 3 = I3), Port Name
        line << " BUFFER " // buffer ID #
                "   BLOCKED "
                 "     %%\n"; // % of blocking attempts

        Constant * const STDERR = b->getInt32(STDERR_FILENO);

        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = STDERR;
        titleArgs[1] = b->GetString(line.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Print each kernel line

        // generate first line format string
        buffer.clear();
        line << "%3" PRIu32 " " // kernel #
                "%-" << maxKernelLength << "s" // kernel name
                "%c%-3" PRIu32 " " // I/O type
                "%-" << maxBindingLength << "s" // port name
                "%7" PRIu32 // buffer ID #
                "%11" PRIu64 " " // # of blocked attempts
                "%6.2f\n"; // % of blocking attempts
        Value * const firstLine = b->GetString(line.str());

        // generate remaining lines format string
        buffer.clear();
        line.indent(maxKernelLength + 4); // spaces for kernel #, kernel name
        line << "%c%-3" PRIu32 " " // I/O type
                "%-" << maxBindingLength << "s" // port name
                "%7" PRIu32 // buffer ID #
                "%11" PRIu64 " " // # of blocked attempts
                "%6.2f\n"; // % of blocking attempts
        Value * const remainingLines = b->GetString(line.str());

        SmallVector<Value *, 8> args;
        args[0] = STDERR;


        Type * const doubleTy = b->getDoubleTy();
        Constant * const fOneHundred = ConstantFP::get(doubleTy, 100.0);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {

            const auto prefix = makeKernelName(i);
            // total # of segments processed by kernel
            Value * const totalNumberOfSegments = b->getScalarField(prefix + STATISTICS_SEGMENT_COUNT_SUFFIX);
            Value * const fTotalNumberOfSegments = b->CreateUIToFP(totalNumberOfSegments, doubleTy);

            const Kernel * const kernel = getKernel(i);
            Constant * kernelName = b->GetString(kernel->getName());

            for (const auto & e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                args.push_back(STDERR);
                if (kernelName) {
                    args.push_back(firstLine);
                    args.push_back(b->getInt32(i));
                    args.push_back(kernelName);
                    kernelName = nullptr;
                } else {
                    args.push_back(remainingLines);
                }
                args.push_back(b->getInt8('I'));
                const auto inputPort = binding.inputPort();
                args.push_back(b->getInt32(inputPort));
                const Binding & ref = binding.Binding;

                args.push_back(b->GetString(ref.getName()));

                args.push_back(b->getInt32(source(e, mBufferGraph) - FirstStreamSet + 1));

                const auto prefix2 = prefix + STATISTICS_BLOCKING_IO_SUFFIX + "I";
                Value * const blockedCount = b->getScalarField(prefix2 + std::to_string(inputPort));
                args.push_back(blockedCount);

                Value * const fBlockedCount = b->CreateUIToFP(blockedCount, doubleTy);
                Value * const fBlockedCount100 = b->CreateFMul(fBlockedCount, fOneHundred);
                args.push_back(b->CreateFDiv(fBlockedCount100, fTotalNumberOfSegments));

                b->CreateCall(b->GetDprintf(), args);

                args.clear();
            }

            for (const auto & e : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                args.push_back(STDERR);
                if (kernelName) {
                    args.push_back(firstLine);
                    args.push_back(b->getInt32(i));
                    args.push_back(kernelName);
                    kernelName = nullptr;
                } else {
                    args.push_back(remainingLines);
                }
                args.push_back(b->getInt8('O'));
                const auto outputPort = binding.outputPort();
                args.push_back(b->getInt32(outputPort));
                const Binding & ref = binding.Binding;
                args.push_back(b->GetString(ref.getName()));

                args.push_back(b->getInt32(target(e, mBufferGraph) - FirstStreamSet + 1));

                const auto prefix2 = prefix + STATISTICS_BLOCKING_IO_SUFFIX + "O";
                Value * const blockedCount = b->getScalarField(prefix2 + std::to_string(outputPort));
                args.push_back(blockedCount);

                Value * const fBlockedCount = b->CreateUIToFP(blockedCount, doubleTy);
                Value * const fBlockedCount100 = b->CreateFMul(fBlockedCount, fOneHundred);
                args.push_back(b->CreateFDiv(fBlockedCount100, fTotalNumberOfSegments));

                b->CreateCall(b->GetDprintf(), args);

                args.clear();
            }

        }
        // print final new line
        FixedArray<Value *, 2> finalArgs;
        finalArgs[0] = STDERR;
        finalArgs[1] = b->GetString("\n");
        b->CreateCall(b->GetDprintf(), finalArgs);
    }
}

}
