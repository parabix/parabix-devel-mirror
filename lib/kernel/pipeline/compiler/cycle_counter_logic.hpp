#include "pipeline_compiler.hpp"

// TODO: Print Total CPU Cycles

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelCycleCountProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addCycleCounterProperties(BuilderRef b, const unsigned kernel) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        // TODO: make these thread local to prevent false sharing and enable
        // analysis of thread distributions?
        IntegerType * const int64Ty = b->getInt64Ty();

        const auto prefix = makeKernelName(kernel) + STATISTICS_CYCLE_COUNT_SUFFIX;
        mTarget->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::AFTER_SYNCHRONIZATION));
        mTarget->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::BUFFER_EXPANSION));
        mTarget->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::AFTER_COPY));
        mTarget->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::AFTER_KERNEL_CALL));
        mTarget->addInternalScalar(int64Ty, prefix + std::to_string(CycleCounter::FINAL));
    }

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {

        const auto prefix = makeKernelName(kernel);
        IntegerType * const int64Ty = b->getInt64Ty();

        // total # of segments processed by kernel
        mTarget->addInternalScalar(int64Ty, prefix + STATISTICS_SEGMENT_COUNT_SUFFIX);

        // # of blocked I/O channel attempts in which no strides
        // were possible (i.e., blocked on first iteration)
        const auto numOfInputs = getNumOfStreamInputs(kernel);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto prefix = makeBufferName(kernel, StreamSetPort{PortType::Input, i});
            mTarget->addInternalScalar(int64Ty, prefix + STATISTICS_BLOCKING_IO_SUFFIX);
        }
        const auto numOfOutputs = getNumOfStreamOutputs(kernel);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            // TODO: ignore dynamic buffers
            const auto prefix = makeBufferName(kernel, StreamSetPort{PortType::Output, i});
            mTarget->addInternalScalar(int64Ty, prefix + STATISTICS_BLOCKING_IO_SUFFIX);
        }
    }

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceBlockedIO))) {

        FixedArray<Type *, 3> fields;
        IntegerType * const sizeTy = b->getSizeTy();
        fields[0] = sizeTy->getPointerTo();
        fields[1] = sizeTy;
        fields[2] = sizeTy;
        StructType * const historyTy = StructType::get(b->getContext(), fields);

        // # of blocked I/O channel attempts in which no strides
        // were possible (i.e., blocked on first iteration)
        const auto numOfInputs = getNumOfStreamInputs(kernel);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto prefix = makeBufferName(kernel, StreamSetPort{PortType::Input, i});
            mTarget->addInternalScalar(historyTy, prefix + STATISTICS_BLOCKING_IO_HISTORY_SUFFIX);
        }
        const auto numOfOutputs = getNumOfStreamOutputs(kernel);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto prefix = makeBufferName(kernel, StreamSetPort{PortType::Output, i});
            mTarget->addInternalScalar(historyTy, prefix + STATISTICS_BLOCKING_IO_HISTORY_SUFFIX);
        }

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief startOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::startCycleCounter(BuilderRef b, const CycleCounter type) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        assert ((unsigned)type < mCycleCounters.size());
        mCycleCounters[(unsigned)type] = b->CreateReadCycleCounter();
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief startOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getBufferExpansionCycleCounter(BuilderRef b) const {
    Value * ptr = nullptr;
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        const auto prefix = makeKernelName(mKernelIndex) + STATISTICS_CYCLE_COUNT_SUFFIX;
        ptr = b->getScalarFieldPtr(prefix + std::to_string(BUFFER_EXPANSION));
    }
    return ptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updateOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updateCycleCounter(BuilderRef b, const CycleCounter start, const CycleCounter end) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {
        Value * const endCount = b->CreateReadCycleCounter();
        Value * const duration = b->CreateSub(endCount, mCycleCounters[start]);
        const auto prefix = makeKernelName(mKernelIndex) + STATISTICS_CYCLE_COUNT_SUFFIX;
        Value * const counterPtr = b->getScalarFieldPtr(prefix + std::to_string(end));
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, duration);
        b->CreateStore(updatedCount, counterPtr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief selectPrincipleCycleCountBinding
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSetPort PipelineCompiler::selectPrincipleCycleCountBinding(const unsigned kernel) const {
    const auto numOfInputs = in_degree(kernel, mBufferGraph);
    assert (degree(kernel, mBufferGraph));
    if (numOfInputs == 0) {
        return StreamSetPort{PortType::Output, 0};
    } else {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto inputPort = StreamSetPort{PortType::Input, i};
            const Binding & input = getInputBinding(kernel, inputPort);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                return inputPort;
            }
        }
        return StreamSetPort{PortType::Input, 0};
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalCycleCounter
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printOptionalCycleCounter(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableCycleCounter))) {

        // Print the title line

        size_t maxLength = 0;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            maxLength = std::max(maxLength, kernel->getName().size());
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
                 " COPY " // look ahead + copy back + look behind %,
                 " PIPE " // pipeline overhead %,
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
                "%5.1f " // look ahead + copy back + look behind %,
                "%5.1f " // overhead %,
                "%6.1f " // execution %,
                "%5.1f\n"; // % of Total CPU Cycles.

        FixedArray<Value *, 13> args;
        args[0] = STDERR;
        args[1] = b->GetString(line.str());

        Value * totalCycles = b->getInt64(0);

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

            Value * iKnownOverheads = b->CreateAdd(synchronizationCycles, bufferExpandCycles);

            Value * const executionCycles = b->getScalarField(prefix + std::to_string(CycleCounter::AFTER_KERNEL_CALL));
            Value * const fExecutionCycles = b->CreateUIToFP(executionCycles, doubleTy);
            Value * const fExecutionCycles100 = b->CreateFMul(fExecutionCycles, fOneHundred);

            iKnownOverheads = b->CreateAdd(iKnownOverheads, executionCycles);

            Value * const copyCycles = b->getScalarField(prefix + std::to_string(CycleCounter::AFTER_COPY));
            Value * const fCopyCycles = b->CreateUIToFP(copyCycles, doubleTy);
            Value * const fCopyCycles100 = b->CreateFMul(fCopyCycles, fOneHundred);

            iKnownOverheads = b->CreateAdd(iKnownOverheads, copyCycles);

            Value * const cycles = b->getScalarField(prefix + std::to_string(CycleCounter::FINAL));
            Value * const fCycles = b->CreateUIToFP(cycles, doubleTy);

            Value * const iInternalOverhead = b->CreateSub(cycles, iKnownOverheads);

            Value * const fSynchronizationPerc = b->CreateFDiv(fSynchronizationCycles100, fCycles);
            Value * const fBufferExpandPerc = b->CreateFDiv(fBufferExpandCycles100, fCycles);
            Value * const fCopyPerc = b->CreateFDiv(fCopyCycles100, fCycles);
            Value * const fExecutionPerc = b->CreateFDiv(fExecutionCycles100, fCycles);

            Value * const fCyclesPerItem = b->CreateFDiv(fCycles, fItems);
            Value * const fCycles100 = b->CreateFMul(fCycles, fOneHundred);

            Value * const fUnknownOverheads = b->CreateUIToFP(iInternalOverhead, doubleTy);
            Value * const fUnknownOverheads100 = b->CreateFMul(fUnknownOverheads, fOneHundred);
            Value * const fUnknownOverhead = b->CreateFDiv(fUnknownOverheads100, fCycles);

            Value * const fPercentageOfTotal = b->CreateFDiv(fCycles100, fTotalCycles);

            const Kernel * const kernel = getKernel(i);
            args[2] = b->getInt32(i);
            args[3] = b->GetString(kernel->getName());
            args[4] = fItems;
            args[5] = fCycles;
            args[6] = fCyclesPerItem;
            args[7] = fSynchronizationPerc;
            args[8] = fBufferExpandPerc;
            args[9] = fCopyPerc;
            args[10] = fUnknownOverhead;
            args[11] = fExecutionPerc;
            args[12] = fPercentageOfTotal;

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
 * @brief incrementNumberOfSegmentsCounter
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::incrementNumberOfSegmentsCounter(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {
        const auto fieldName =
            makeKernelName(mKernelIndex) + STATISTICS_SEGMENT_COUNT_SUFFIX;
        Value * const counterPtr = b->getScalarFieldPtr(fieldName);
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, b->getInt64(1));
        b->CreateStore(updatedCount, counterPtr);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordBlockingIO
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordBlockingIO(BuilderRef b, const StreamSetPort port) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {
        const auto prefix = makeBufferName(mKernelIndex, port);
        Value * const counterPtr = b->getScalarFieldPtr(prefix + STATISTICS_BLOCKING_IO_SUFFIX);
        Value * const runningCount = b->CreateLoad(counterPtr);
        Value * const updatedCount = b->CreateAdd(runningCount, b->getSize(1));
        b->CreateStore(updatedCount, counterPtr);
    }
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceBlockedIO))) {

        const auto prefix = makeBufferName(mKernelIndex, port);
        Value * const historyPtr = b->getScalarFieldPtr(prefix + STATISTICS_BLOCKING_IO_HISTORY_SUFFIX);

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);

        Value * const traceLogArrayField = b->CreateGEP(historyPtr, {ZERO, ZERO});
        Value * const traceLogArray = b->CreateLoad(traceLogArrayField);

        Value * const traceLogCountField = b->CreateGEP(historyPtr, {ZERO, ONE});
        Value * const traceLogCount = b->CreateLoad(traceLogCountField);

        Value * const traceLogCapacityField = b->CreateGEP(historyPtr, {ZERO, TWO});
        Value * const traceLogCapacity = b->CreateLoad(traceLogCapacityField);

        BasicBlock * const expandHistory = b->CreateBasicBlock(prefix + "_expandHistory", mKernelLoopCall);
        BasicBlock * const recordExpansion = b->CreateBasicBlock(prefix + "_recordExpansion", mKernelLoopCall);

        Value * const hasEnough = b->CreateICmpULT(traceLogCount, traceLogCapacity);

        BasicBlock * const entryBlock = b->GetInsertBlock();
        b->CreateLikelyCondBr(hasEnough, recordExpansion, expandHistory);

        b->SetInsertPoint(expandHistory);
        Constant * const SZ_ONE = b->getSize(1);
        Constant * const SZ_MIN_SIZE = b->getSize(32);
        Value * const newCapacity = b->CreateUMax(b->CreateShl(traceLogCapacity, SZ_ONE), SZ_MIN_SIZE);
        Value * const expandedLogArray = b->CreateRealloc(b->getSizeTy(), traceLogArray, newCapacity);
        assert (expandedLogArray->getType() == traceLogArray->getType());
        b->CreateStore(expandedLogArray, traceLogArrayField);
        b->CreateStore(newCapacity, traceLogCapacityField);
        BasicBlock * const branchExitBlock = b->GetInsertBlock();
        b->CreateBr(recordExpansion);

        b->SetInsertPoint(recordExpansion);
        PHINode * const logArray = b->CreatePHI(traceLogArray->getType(), 2);
        logArray->addIncoming(traceLogArray, entryBlock);
        logArray->addIncoming(expandedLogArray, branchExitBlock);
        b->CreateStore(b->CreateAdd(traceLogCount, b->getSize(1)), traceLogCountField);
        b->CreateStore(mSegNo, b->CreateGEP(logArray, traceLogCount));
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalBlockingIOStatistics
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printOptionalBlockingIOStatistics(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::EnableBlockingIOCounter))) {

        // Print the title line

        size_t maxKernelLength = 0;
        size_t maxBindingLength = 0;

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            maxKernelLength = std::max(maxKernelLength, kernel->getName().size());
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
            for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
        }
        maxKernelLength += 4;
        maxBindingLength += 4;

        SmallVector<char, 100> buffer;
        raw_svector_ostream line(buffer);

        line << "BLOCKING I/O STATISTICS:\n\n"
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

        Type * const doubleTy = b->getDoubleTy();
        Constant * const fOneHundred = ConstantFP::get(doubleTy, 100.0);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {

            const auto prefix = makeKernelName(i);
            // total # of segments processed by kernel
            Value * const totalNumberOfSegments = b->getScalarField(prefix + STATISTICS_SEGMENT_COUNT_SUFFIX);
            Value * const fTotalNumberOfSegments = b->CreateUIToFP(totalNumberOfSegments, doubleTy);

            const Kernel * const kernel = getKernel(i);
            Constant * kernelName = b->GetString(kernel->getName());

            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {

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

                args.push_back(b->getInt32(source(e, mBufferGraph)));

                const auto prefix = makeBufferName(i, binding.Port);
                Value * const blockedCount = b->getScalarField(prefix + STATISTICS_BLOCKING_IO_SUFFIX);
                args.push_back(blockedCount);

                Value * const fBlockedCount = b->CreateUIToFP(blockedCount, doubleTy);
                Value * const fBlockedCount100 = b->CreateFMul(fBlockedCount, fOneHundred);
                args.push_back(b->CreateFDiv(fBlockedCount100, fTotalNumberOfSegments));

                b->CreateCall(b->GetDprintf(), args);

                args.clear();
            }

            for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {

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

                args.push_back(b->getInt32(target(e, mBufferGraph)));

                const auto prefix = makeBufferName(i, binding.Port);
                Value * const blockedCount = b->getScalarField(prefix + STATISTICS_BLOCKING_IO_SUFFIX);
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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printOptionalBlockedIOPerSegment(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceBlockedIO))) {

        IntegerType * const sizeTy = b->getSizeTy();
        IntegerType * const boolTy = b->getInt1Ty();

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);

        ConstantInt * const SZ_ZERO = b->getSize(0);
        ConstantInt * const SZ_ONE = b->getSize(1);

        ConstantInt * const i1_FALSE = b->getFalse();
        ConstantInt * const i1_TRUE = b->getTrue();

        // Print the first title line
        SmallVector<char, 100> buffer;
        raw_svector_ostream format(buffer);
        format << "BLOCKED I/O PER SEGMENT:\n\n";
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            std::string temp = kernel->getName();
            boost::replace_all(temp, "\"", "\\\"");
            format << ",\"" << i << " " << temp << "\"";
            const auto numOfPorts = getNumOfStreamInputs(i) + getNumOfStreamOutputs(i);
            for (unsigned j = 1; j < numOfPorts; ++j) {
                format.write(',');
            }
        }
        format << "\n";

        Constant * const STDERR = b->getInt32(STDERR_FILENO);

        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = STDERR;
        titleArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Print the second title line
        buffer.clear();
        format << "SEG #";
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            unsigned j = 0;
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                format << ",I" << j << ':' << source(e, mBufferGraph);
                ++j;
            }
            j = 0;
            for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
                format << ",O" << j << ':' << target(e, mBufferGraph);
                ++j;
            }
        }
        format << '\n';
        titleArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Generate line format string
        buffer.clear();
        format << "%" PRIu64; // seg #
        unsigned fieldCount = 0;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const auto numOfPorts = getNumOfStreamInputs(i) + getNumOfStreamOutputs(i);
            fieldCount += numOfPorts;
            for (unsigned j = 0; j < numOfPorts; ++j) {
                format << ",%" PRIu64; // strides
            }
        }
        format << "\n";

        // Print each kernel line
        SmallVector<Value *, 64> args(fieldCount + 3);
        args[0] = STDERR;
        args[1] = b->GetString(format.str());

        // Pre load all of pointers to the the trace log out of the pipeline state.

        SmallVector<Value *, 64> traceLogArray(fieldCount);
        SmallVector<Value *, 64> traceLengthArray(fieldCount);
        SmallVector<PHINode *, 64> currentIndex(fieldCount);
        SmallVector<Value *, 64> updatedIndex(fieldCount);

        for (unsigned i = FirstKernel, j = 0; i <= LastKernel; ++i) {
            const auto numOfInputs = getNumOfStreamInputs(i);
            const auto numOfOutputs = getNumOfStreamOutputs(i);

            for (unsigned k = 0; k < numOfInputs; ++k, ++j) {
                const auto prefix = makeBufferName(i, StreamSetPort{PortType::Input, k});
                Value * const historyPtr = b->getScalarFieldPtr(prefix + STATISTICS_BLOCKING_IO_HISTORY_SUFFIX);
                traceLogArray[j] = b->CreateLoad(b->CreateGEP(historyPtr, {ZERO, ZERO}));
                traceLengthArray[j] = b->CreateLoad(b->CreateGEP(historyPtr, {ZERO, ONE}));
            }

            for (unsigned k = 0; k < numOfOutputs; ++k, ++j) {
                const auto prefix = makeBufferName(i, StreamSetPort{PortType::Output, k});
                Value * const historyPtr = b->getScalarFieldPtr(prefix + STATISTICS_BLOCKING_IO_HISTORY_SUFFIX);
                traceLogArray[j] = b->CreateLoad(b->CreateGEP(historyPtr, {ZERO, ZERO}));
                traceLengthArray[j] = b->CreateLoad(b->CreateGEP(historyPtr, {ZERO, ONE}));
            }
        }

        // Start printing the data lines

        BasicBlock * const loopEntry = b->GetInsertBlock();
        BasicBlock * const loopStart = b->CreateBasicBlock("reportStridesPerSegment");
        b->CreateBr(loopStart);

        b->SetInsertPoint(loopStart);
        PHINode * const segNo = b->CreatePHI(sizeTy, 2);
        segNo->addIncoming(SZ_ZERO, loopEntry);
        for (unsigned i = 0; i < fieldCount; ++i) {
            PHINode * const index = b->CreatePHI(sizeTy, 2);
            index->addIncoming(SZ_ZERO, loopEntry);
            currentIndex[i] = index;
        }

        args[2] = segNo;

        Value * writeLine = i1_FALSE;
        for (unsigned i = 0; i < fieldCount; ++i) {

            BasicBlock * const entry = b->GetInsertBlock();
            BasicBlock * const check = b->CreateBasicBlock();
            BasicBlock * const update = b->CreateBasicBlock();
            BasicBlock * const next = b->CreateBasicBlock();
            Value * const notEndOfTrace = b->CreateICmpNE(currentIndex[i], traceLengthArray[i]);
            b->CreateLikelyCondBr(notEndOfTrace, check, next);

            b->SetInsertPoint(check);
            Value * const nextSegNo = b->CreateLoad(b->CreateGEP(traceLogArray[i], currentIndex[i]));
            b->CreateCondBr(b->CreateICmpEQ(segNo, nextSegNo), update, next);

            b->SetInsertPoint(update);
            Value * const currentIndex1 = b->CreateAdd(currentIndex[i], SZ_ONE);
            b->CreateBr(next);

            b->SetInsertPoint(next);
            PHINode * const nextIndex = b->CreatePHI(sizeTy, 3);
            nextIndex->addIncoming(currentIndex[i], entry);
            nextIndex->addIncoming(currentIndex[i], check);
            nextIndex->addIncoming(currentIndex1, update);
            updatedIndex[i] = nextIndex;

            PHINode * const blockedValue = b->CreatePHI(sizeTy, 3);
            blockedValue->addIncoming(SZ_ZERO, entry);
            blockedValue->addIncoming(SZ_ZERO, check);
            blockedValue->addIncoming(SZ_ONE, update);

            PHINode * const nextWriteLine = b->CreatePHI(boolTy, 3);
            nextWriteLine->addIncoming(writeLine, entry);
            nextWriteLine->addIncoming(writeLine, check);
            nextWriteLine->addIncoming(i1_TRUE, update);
            writeLine = nextWriteLine;

            args[i + 3] = blockedValue;
        }

        BasicBlock * const doWriteLine = b->CreateBasicBlock("writeLine");
        BasicBlock * const checkNext = b->CreateBasicBlock("checkNext");

        b->CreateCondBr(writeLine, doWriteLine, checkNext);

        b->SetInsertPoint(doWriteLine);
        b->CreateCall(b->GetDprintf(), args);
        b->CreateBr(checkNext);

        b->SetInsertPoint(checkNext);
        BasicBlock * const loopExit = b->CreateBasicBlock("reportStridesPerSegmentExit");
        BasicBlock * const loopEnd = b->GetInsertBlock();
        segNo->addIncoming(b->CreateAdd(segNo, SZ_ONE), loopEnd);
        for (unsigned i = 0; i < fieldCount; ++i) {
            currentIndex[i]->addIncoming(updatedIndex[i], loopEnd);
        }
        Value * const notDone = b->CreateICmpNE(segNo, mSegNo);
        b->CreateLikelyCondBr(notDone, loopStart, loopExit);

        b->SetInsertPoint(loopExit);
        // print final new line
        FixedArray<Value *, 2> finalArgs;
        finalArgs[0] = STDERR;
        finalArgs[1] = b->GetString("\n");
        b->CreateCall(b->GetDprintf(), finalArgs);
        for (unsigned i = 0; i < fieldCount; ++i) {
            b->CreateFree(traceLogArray[i]);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeBufferExpansionHistory(BuilderRef b) const {

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {

        const auto firstBuffer = PipelineOutput + 1;
        const auto lastBuffer = num_vertices(mBufferGraph);

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);
        Constant * const SZ_ZERO = b->getSize(0);
        Constant * const SZ_ONE = b->getSize(1);

        for (unsigned i = firstBuffer; i < lastBuffer; ++i) {
            const BufferNode & bn = mBufferGraph[i];
            if (LLVM_LIKELY(bn.isOwned())) {
                const auto pe = in_edge(i, mBufferGraph);
                const auto p = source(pe, mBufferGraph);
                const BufferRateData & rd = mBufferGraph[pe];
                const auto prefix = makeBufferName(p, rd.Port);
                const StreamSetBuffer * const buffer = bn.Buffer; assert (buffer);

                if (isa<DynamicBuffer>(buffer)) {
                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
                    Type * const traceTy = traceData->getType()->getPointerElementType();
                    Type * const entryTy =  traceTy->getStructElementType(0)->getPointerElementType();
                    Value * const entryData = b->CreateCacheAlignedMalloc(entryTy, SZ_ONE);
                    // fill in the struct
                    b->CreateStore(entryData, b->CreateGEP(traceData, {ZERO, ZERO}));
                    b->CreateStore(SZ_ONE, b->CreateGEP(traceData, {ZERO, ONE}));
                    // then the initial record
                    b->CreateStore(SZ_ZERO, b->CreateGEP(entryData, {ZERO, ZERO}));
                    b->CreateStore(buffer->getCapacity(b), b->CreateGEP(entryData, {ZERO, ONE}));
                    const auto n = entryTy->getArrayNumElements(); assert (n > 3);
                    unsigned sizeTyWidth = b->getSizeTy()->getIntegerBitWidth() / 8;
                    Constant * const length = b->getSize(sizeTyWidth * (n - 2));
                    b->CreateMemZero(b->CreateGEP(entryData, {ZERO, TWO}), length, sizeTyWidth);
                }
            }
        }

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordBufferExpansionHistory(BuilderRef b, const StreamSetPort outputPort,
                                                    const StreamSetBuffer * const buffer) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        assert (isa<DynamicBuffer>(buffer));

        const auto prefix = makeBufferName(mKernelIndex, outputPort);

        Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
        Type * const traceDataTy = traceData->getType()->getPointerElementType();
        Type * const entryTy = traceDataTy->getStructElementType(0)->getPointerElementType();

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);
        Constant * const THREE = b->getInt32(3);

        Value * const traceLogArrayField = b->CreateGEP(traceData, {ZERO, ZERO});
        Value * entryArray = b->CreateLoad(traceLogArrayField);
        Value * const traceLogCountField = b->CreateGEP(traceData, {ZERO, ONE});
        Value * const traceIndex = b->CreateLoad(traceLogCountField);
        Value * const traceCount = b->CreateAdd(traceIndex, b->getSize(1));

        entryArray = b->CreateRealloc(entryTy, entryArray, traceCount);
        b->CreateStore(entryArray, traceLogArrayField);
        b->CreateStore(traceCount, traceLogCountField);

        // segment num  0
        b->CreateStore(mSegNo, b->CreateGEP(entryArray, {traceIndex, ZERO}));
        // new capacity 1
        b->CreateStore(buffer->getCapacity(b), b->CreateGEP(entryArray, {traceIndex, ONE}));
        // produced item count 2
        Value * const produced = mAlreadyProducedPhi[outputPort.Number];
        b->CreateStore(produced, b->CreateGEP(entryArray, {traceIndex, TWO}));

        // consumer processed item count [3,n)
        Value * const consumerDataPtr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);

        const auto n = entryTy->getArrayNumElements(); assert (n > 3);
        assert ((n - 3) == (consumerDataPtr->getType()->getPointerElementType()->getArrayNumElements() - 1));

        Value * const processedPtr = b->CreateGEP(consumerDataPtr, { ZERO, ONE });
        Value * const logPtr = b->CreateGEP(entryArray, {traceIndex, THREE});
        unsigned sizeTyWidth = b->getSizeTy()->getIntegerBitWidth() / 8;
        Constant * const length = b->getSize(sizeTyWidth * (n - 3));
        b->CreateMemCpy(logPtr, processedPtr, length, sizeTyWidth);

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printOptionalBufferExpansionHistory(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceDynamicBuffers))) {

        // Print the title line

        size_t maxKernelLength = 0;
        size_t maxBindingLength = 0;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            maxKernelLength = std::max(maxKernelLength, kernel->getName().size());
            for (const auto e : make_iterator_range(in_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
            for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & binding = mBufferGraph[e];
                const Binding & ref = binding.Binding;
                maxBindingLength = std::max(maxBindingLength, ref.getName().length());
            }
        }
        maxKernelLength += 4;
        maxBindingLength += 4;

        SmallVector<char, 160> buffer;
        raw_svector_ostream format(buffer);

        // TODO: if expanding buffers are supported again, we need another field here for streamset size

        format << "BUFFER EXPANSION HISTORY:\n\n"
                  "  # "  // kernel ID #  (only shown for first)
                  "KERNEL"; // kernel Name (only shown for first)
        format.indent(maxKernelLength - 6);
        format << "PORT";
        format.indent(5 + maxBindingLength - 4); // I/O Type (e.g., input port 3 = I3), Port Name
        format << " BUFFER " // buffer ID #
                  "        SEG # "
                  "     ITEM CAPACITY\n";

        Constant * const STDERR = b->getInt32(STDERR_FILENO);
        FixedArray<Value *, 2> constantArgs;
        constantArgs[0] = STDERR;
        constantArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), constantArgs);

        const auto totalLength = 4 + maxKernelLength + 4 + maxBindingLength + 7 + 15 + 18 + 2;

        // generate a single-line (-) bar
        buffer.clear();
        for (unsigned i = 0; i < totalLength; ++i) {
            format.write('-');
        }
        format.write('\n');
        Constant * const singleBar = b->GetString(format.str());
        constantArgs[1] = singleBar;
        b->CreateCall(b->GetDprintf(), constantArgs);


        // generate the produced/processed title line
        buffer.clear();
        format.indent(totalLength - 19);
        format << "PRODUCED/PROCESSED\n";
        constantArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), constantArgs);

        // generate a double-line (=) bar
        buffer.clear();
        for (unsigned i = 0; i < totalLength; ++i) {
            format.write('=');
        }
        format.write('\n');
        Constant * const doubleBar = b->GetString(format.str());
        constantArgs[1] = doubleBar;
        b->CreateCall(b->GetDprintf(), constantArgs);

        // Generate expansion line format string
        buffer.clear();
        format << "%3" PRIu32 " " // kernel #
                  "%-" << maxKernelLength << "s" // kernel name
                  "O%-3" PRIu32 " " // I/O type
                  "%-" << maxBindingLength << "s" // port name
                  "%7" PRIu32 // buffer ID #
                  "%14" PRIu64 " " // segment #
                  "%18" PRIu64 "\n"; // item capacity
        Constant * const expansionFormat = b->GetString(format.str());

        // Generate the item count history format string
        buffer.clear();
        format << "%3" PRIu32 " " // kernel #
                  "%-" << maxKernelLength << "s" // kernel name
                  "%c%-3" PRIu32 " " // I/O type
                  "%-" << maxBindingLength << "s" // port name
                  "%40" PRIu64 "\n"; // produced/processed item count
        Constant * const itemCountFormat = b->GetString(format.str());

        // Print each kernel line
        FixedArray<Value *, 9> expansionArgs;
        expansionArgs[0] = STDERR;
        expansionArgs[1] = expansionFormat;

        FixedArray<Value *, 8> itemCountArgs;
        itemCountArgs[0] = STDERR;
        itemCountArgs[1] = itemCountFormat;

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);

        Constant * const SZ_ZERO = b->getSize(0);
        Constant * const SZ_ONE = b->getSize(1);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {

            for (const auto output : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & br = mBufferGraph[output];
                const auto buffer = target(output, mBufferGraph);
                const BufferNode & bn = mBufferGraph[buffer];
                if (isa<DynamicBuffer>(bn.Buffer)) {

                    //  # KERNEL                      PORT                      BUFFER         SEG #      ITEM CAPACITY

                    expansionArgs[2] = b->getInt32(i);
                    expansionArgs[3] = b->GetString(getKernel(i)->getName());
                    const auto outputPort = br.outputPort();
                    expansionArgs[4] = b->getInt32(outputPort);
                    const Binding & binding = br.Binding;
                    expansionArgs[5] = b->GetString(binding.getName());
                    expansionArgs[6] = b->getInt32(buffer);

                    const auto prefix = makeBufferName(i, StreamSetPort{PortType::Output, outputPort});
                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);

                    Value * const traceArrayField = b->CreateGEP(traceData, {ZERO, ZERO});
                    Value * const entryArray = b->CreateLoad(traceArrayField);
                    Value * const traceCountField = b->CreateGEP(traceData, {ZERO, ONE});
                    Value * const traceCount = b->CreateLoad(traceCountField);

                    BasicBlock * const outputEntry = b->GetInsertBlock();
                    BasicBlock * const outputLoop = b->CreateBasicBlock(prefix + "_bufferExpansionReportLoop");
                    BasicBlock * const writeItemCount = b->CreateBasicBlock(prefix + "_bufferWriteItemCountLoop");

                    BasicBlock * const nextEntry = b->CreateBasicBlock(prefix + "_bufferWriteItemCountLoop");

                    BasicBlock * const outputExit = b->CreateBasicBlock(prefix + "_bufferExpansionReportExit");

                    b->CreateBr(outputLoop);

                    b->SetInsertPoint(outputLoop);
                    PHINode * const index = b->CreatePHI(b->getSizeTy(), 2);
                    index->addIncoming(SZ_ZERO, outputEntry);

                    Value * const isFirst = b->CreateICmpEQ(index, SZ_ZERO);
                    Value * const nextIndex = b->CreateAdd(index, SZ_ONE);
                    Value * const isLast = b->CreateICmpEQ(nextIndex, traceCount);
                    Value * const onlyEntry = b->CreateICmpEQ(traceCount, SZ_ONE);

                    Value * const openingBar = b->CreateSelect(onlyEntry, doubleBar, singleBar);

                    Value * const segmentNumField = b->CreateGEP(entryArray, {index, ZERO});
                    Value * const segmentNum = b->CreateLoad(segmentNumField);
                    expansionArgs[7] = segmentNum;

                    Value * const newBufferSizeField = b->CreateGEP(entryArray, {index, ONE});
                    Value * const newBufferSize = b->CreateLoad(newBufferSizeField);
                    expansionArgs[8] = newBufferSize;

                    b->CreateCall(b->GetDprintf(), expansionArgs);

                    constantArgs[1] = openingBar;

                    b->CreateCall(b->GetDprintf(), constantArgs);

                    b->CreateCondBr(isFirst, nextEntry, writeItemCount);

                    // Do not write processed/produced item counts for the first entry as they
                    // are guaranteed to be 0 and only add noise to the log output.
                    b->SetInsertPoint(writeItemCount);

                    // --------------------------------------------------------------------------------------------------

                    //  # KERNEL                      PORT                                           PRODUCED/PROCESSED

                    itemCountArgs[2] = expansionArgs[2];
                    itemCountArgs[3] = expansionArgs[3];
                    itemCountArgs[4] = b->getInt8('O');
                    itemCountArgs[5] = expansionArgs[4];
                    itemCountArgs[6] = expansionArgs[5];
                    Value * const producedField = b->CreateGEP(entryArray, {index, TWO});
                    itemCountArgs[7] = b->CreateLoad(producedField);

                    b->CreateCall(b->GetDprintf(), itemCountArgs);
                    itemCountArgs[4] = b->getInt8('I');
                    for (const auto e : make_iterator_range(out_edges(buffer, mConsumerGraph))) {
                        const ConsumerEdge & c = mConsumerGraph[e];
                        const auto consumer = target(e, mConsumerGraph);
                        itemCountArgs[2] = b->getInt32(consumer);
                        itemCountArgs[3] = b->GetString(getKernel(consumer)->getName());
                        itemCountArgs[5] = b->getInt32(c.Port);
                        const Binding & binding = getBinding(consumer, StreamSetPort{PortType::Input, c.Port});
                        itemCountArgs[6] = b->GetString(binding.getName());
                        const auto k = c.Index + 2; assert (k > 2);
                        Value * const processedField = b->CreateGEP(entryArray, {index, b->getInt32(k)});
                        itemCountArgs[7] = b->CreateLoad(processedField);
                        b->CreateCall(b->GetDprintf(), itemCountArgs);
                    }

                    Value * const closingBar = b->CreateSelect(isLast, doubleBar, singleBar);
                    constantArgs[1] = closingBar;
                    b->CreateCall(b->GetDprintf(), constantArgs);

                    b->CreateBr(nextEntry);

                    b->SetInsertPoint(nextEntry);
                    index->addIncoming(nextIndex, nextEntry);
                    b->CreateCondBr(isLast, outputExit, outputLoop);

                    b->SetInsertPoint(outputExit);
                }
            }
        }
        // print final new line
        constantArgs[1] = doubleBar;
        b->CreateCall(b->GetDprintf(), constantArgs);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeStridesPerSegment(BuilderRef b) const {

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {

        const auto prefix = makeKernelName(mKernelIndex);

        Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_STRIDES_PER_SEGMENT_SUFFIX);
        Type * const traceDataTy = traceData->getType()->getPointerElementType();
        Type * const traceLogTy =  traceDataTy->getStructElementType(1)->getPointerElementType();

        Constant * const SZ_ZERO = b->getSize(0);
        Constant * const SZ_DEFAULT_CAPACITY = b->getSize(32);
        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);
        Constant * const THREE = b->getInt32(3);

        Value * const traceDataArray = b->CreateCacheAlignedMalloc(traceLogTy, SZ_DEFAULT_CAPACITY);

        // fill in the struct
        b->CreateStore(SZ_ZERO, b->CreateGEP(traceData, {ZERO, ZERO})); // "last" num of strides
        b->CreateStore(traceDataArray, b->CreateGEP(traceData, {ZERO, ONE})); // trace log
        b->CreateStore(SZ_ZERO, b->CreateGEP(traceData, {ZERO, TWO})); // trace length
        b->CreateStore(SZ_DEFAULT_CAPACITY, b->CreateGEP(traceData, {ZERO, THREE})); // trace capacity
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordStridesPerSegment(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {
        // NOTE: this records only the change to attempt to reduce the memory usage of this log.

        const auto prefix = makeKernelName(mKernelIndex);
        Value * const kernelTraceLog = b->getScalarFieldPtr(prefix + STATISTICS_STRIDES_PER_SEGMENT_SUFFIX);

        Module * const m = b->getModule();
        Function * record = m->getFunction("$trace_strides_per_segment");
        if (LLVM_UNLIKELY(record == nullptr)) {
            auto ip = b->saveIP();
            LLVMContext & C = b->getContext();
            Type * const sizeTy = b->getSizeTy();
            FunctionType * fty = FunctionType::get(b->getVoidTy(), { kernelTraceLog->getType(), sizeTy, sizeTy }, false);
            record = Function::Create(fty, Function::PrivateLinkage, "$trace_strides_per_segment", m);

            BasicBlock * const entry = BasicBlock::Create(C, "entry", record);
            BasicBlock * const trace = BasicBlock::Create(C, "trace", record);
            BasicBlock * const expand = BasicBlock::Create(C, "expand", record);
            BasicBlock * const write = BasicBlock::Create(C, "write", record);
            BasicBlock * const exit = BasicBlock::Create(C, "exit", record);

            b->SetInsertPoint(entry);

            auto arg = record->arg_begin();
            arg->setName("traceData");
            Value * const traceData = &*arg++;
            arg->setName("segNo");
            Value * const segNo = &*arg++;
            arg->setName("numOfStrides");
            Value * const numOfStrides = &*arg++;
            assert (arg == record->arg_end());

            Constant * const ZERO = b->getInt32(0);
            Constant * const ONE = b->getInt32(1);
            Constant * const TWO = b->getInt32(2);
            Constant * const THREE = b->getInt32(3);

            Value * const lastNumOfStridesField = b->CreateGEP(traceData, {ZERO, ZERO});
            Value * const lastNumOfStrides = b->CreateLoad(lastNumOfStridesField);
            Value * const changed = b->CreateICmpNE(lastNumOfStrides, numOfStrides);
            b->CreateCondBr(changed, trace, exit);

            b->SetInsertPoint(trace);

            Value * const traceLogField = b->CreateGEP(traceData, {ZERO, ONE});
            Value * const traceLog = b->CreateLoad(traceLogField);
            Value * const traceLengthField = b->CreateGEP(traceData, {ZERO, TWO});
            Value * const traceLength = b->CreateLoad(traceLengthField);
            Value * const traceCapacityField = b->CreateGEP(traceData, {ZERO, THREE});
            Value * const traceCapacity = b->CreateLoad(traceCapacityField);

            Value * const hasSpace = b->CreateICmpNE(traceLength, traceCapacity);

            b->CreateLikelyCondBr(hasSpace, write, expand);

            b->SetInsertPoint(expand);
            Value * const nextTraceCapacity = b->CreateShl(traceCapacity, 1);
            Type * const traceDataTy = traceData->getType()->getPointerElementType();
            Type * const traceLogTy =  traceDataTy->getStructElementType(1)->getPointerElementType();
            Value * const expandedtraceLog = b->CreateRealloc(traceLogTy, traceLog, nextTraceCapacity);
            b->CreateStore(expandedtraceLog, traceLogField);
            b->CreateStore(nextTraceCapacity, traceCapacityField);
            b->CreateBr(write);

            b->SetInsertPoint(write);
            PHINode * const traceLogPhi = b->CreatePHI(traceLog->getType(), 2);
            traceLogPhi->addIncoming(traceLog, trace);
            traceLogPhi->addIncoming(expandedtraceLog, expand);
            b->CreateStore(segNo, b->CreateGEP(traceLogPhi, {traceLength , ZERO}));
            b->CreateStore(numOfStrides, b->CreateGEP(traceLogPhi, {traceLength , ONE}));
            b->CreateStore(numOfStrides, lastNumOfStridesField);
            b->CreateStore(b->CreateAdd(traceLength, b->getSize(1)), traceLengthField);
            b->CreateBr(exit);

            b->SetInsertPoint(exit);
            b->CreateRetVoid();

            b->restoreIP(ip);
        }

        b->CreateCall(record, { kernelTraceLog, mSegNo, mTotalNumOfStridesAtExitPhi } );
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printProducedItemCountDeltas
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printOptionalStridesPerSegment(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {

        IntegerType * const sizeTy = b->getSizeTy();

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);

        ConstantInt * const SZ_ZERO = b->getSize(0);
        ConstantInt * const SZ_ONE = b->getSize(1);

        // Print the title line
        SmallVector<char, 100> buffer;
        raw_svector_ostream format(buffer);
        format << "STRIDES PER SEGMENT:\n\n"
                  "SEG #";

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            std::string temp = kernel->getName();
            boost::replace_all(temp, "\"", "\\\"");
            format << ",\"" << i << " " << temp << "\"";
        }
        format << "\n";


        Constant * const STDERR = b->getInt32(STDERR_FILENO);

        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = STDERR;
        titleArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Generate line format string
        buffer.clear();

        format << "%" PRIu64; // seg #
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            format << ",%" PRIu64; // strides
        }
        format << "\n";

        // Print each kernel line
        SmallVector<Value *, 64> args(LastKernel + 3);
        args[0] = STDERR;
        args[1] = b->GetString(format.str());

        // Pre load all of pointers to the the trace log out of the pipeline state.

        SmallVector<Value *, 64> traceLogArray(LastKernel + 1);
        SmallVector<Value *, 64> traceLengthArray(LastKernel + 1);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const auto prefix = makeKernelName(i);
            Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_STRIDES_PER_SEGMENT_SUFFIX);
            traceLogArray[i] = b->CreateLoad(b->CreateGEP(traceData, {ZERO, ONE}));
            traceLengthArray[i] = b->CreateLoad(b->CreateGEP(traceData, {ZERO, TWO}));
        }

        // Start printing the data lines

        BasicBlock * const loopEntry = b->GetInsertBlock();
        BasicBlock * const loopStart = b->CreateBasicBlock("reportStridesPerSegment");
        b->CreateBr(loopStart);

        b->SetInsertPoint(loopStart);
        PHINode * const segNo = b->CreatePHI(sizeTy, 2);
        segNo->addIncoming(SZ_ZERO, loopEntry);
        SmallVector<PHINode *, 64> currentIndex(LastKernel + 1);
        SmallVector<PHINode *, 64> updatedIndex(LastKernel + 1);
        SmallVector<PHINode *, 64> currentValue(LastKernel + 1);
        SmallVector<PHINode *, 64> updatedValue(LastKernel + 1);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            currentIndex[i] = b->CreatePHI(sizeTy, 2);
            currentIndex[i]->addIncoming(SZ_ZERO, loopEntry);
            currentValue[i] = b->CreatePHI(sizeTy, 2);
            currentValue[i]->addIncoming(SZ_ZERO, loopEntry);
        }

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            BasicBlock * const entry = b->GetInsertBlock();
            BasicBlock * const check = b->CreateBasicBlock();
            BasicBlock * const update = b->CreateBasicBlock();
            BasicBlock * const next = b->CreateBasicBlock();
            Value * const notEndOfTrace = b->CreateICmpNE(currentIndex[i], traceLengthArray[i]);
            b->CreateLikelyCondBr(notEndOfTrace, check, next);

            b->SetInsertPoint(check);
            Value * const nextSegNo = b->CreateLoad(b->CreateGEP(traceLogArray[i], { currentIndex[i], ZERO }));
            b->CreateCondBr(b->CreateICmpEQ(segNo, nextSegNo), update, next);

            b->SetInsertPoint(update);
            Value * const numOfStrides = b->CreateLoad(b->CreateGEP(traceLogArray[i], { currentIndex[i], ONE }));
            Value * const currentIndex1 = b->CreateAdd(currentIndex[i], SZ_ONE);
            b->CreateBr(next);

            b->SetInsertPoint(next);
            PHINode * const nextValue = b->CreatePHI(sizeTy, 3);
            nextValue->addIncoming(SZ_ZERO, entry);
            nextValue->addIncoming(currentValue[i], check);
            nextValue->addIncoming(numOfStrides, update);
            updatedValue[i] = nextValue;

            PHINode * const nextIndex = b->CreatePHI(sizeTy, 3);
            nextIndex->addIncoming(currentIndex[i], entry);
            nextIndex->addIncoming(currentIndex[i], check);
            nextIndex->addIncoming(currentIndex1, update);
            updatedIndex[i] = nextIndex;
        }

        args[2] = segNo;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            args[i - FirstKernel + 3] = updatedValue[i];
        }

        b->CreateCall(b->GetDprintf(), args);

        BasicBlock * const loopExit = b->CreateBasicBlock("reportStridesPerSegmentExit");
        BasicBlock * const loopEnd = b->GetInsertBlock();
        segNo->addIncoming(b->CreateAdd(segNo, SZ_ONE), loopEnd);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            currentIndex[i]->addIncoming(updatedIndex[i], loopEnd);
            currentValue[i]->addIncoming(updatedValue[i], loopEnd);
        }

        Value * const notDone = b->CreateICmpNE(segNo, mSegNo);
        b->CreateLikelyCondBr(notDone, loopStart, loopExit);

        b->SetInsertPoint(loopExit);
        // print final new line
        FixedArray<Value *, 2> finalArgs;
        finalArgs[0] = STDERR;
        finalArgs[1] = b->GetString("\n");
        b->CreateCall(b->GetDprintf(), finalArgs);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            b->CreateFree(traceLogArray[i]);
        }
    }
}

#define ITEM_COUNT_DELTA_CHUNK_LENGTH (64 * 1024)

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addProducedItemCountDeltaProperties(BuilderRef b, unsigned kernel) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceProducedItemCounts))) {
        addItemCountDeltaProperties(b, kernel, STATISTICS_PRODUCED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordProducedItemCountDeltas(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceProducedItemCounts))) {
        recordItemCountDeltas(b, mFullyProducedItemCount, mInitiallyProducedItemCount, STATISTICS_PRODUCED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printProducedItemCountDeltas(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceProducedItemCounts))) {
        printItemCountDeltas(b, "PRODUCED ITEM COUNT DELTAS PER SEGMENT:", STATISTICS_PRODUCED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addUnconsumedItemCountProperties(BuilderRef b, unsigned kernel) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceUnconsumedItemCounts))) {
        addItemCountDeltaProperties(b, kernel, STATISTICS_UNCONSUMED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordUnconsumedItemCounts(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceUnconsumedItemCounts))) {
        recordItemCountDeltas(b, mInitiallyProducedItemCount, mConsumedItemCount, STATISTICS_UNCONSUMED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printUnconsumedItemCounts(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceUnconsumedItemCounts))) {
        printItemCountDeltas(b, "UNCONSUMED ITEM COUNTS PER SEGMENT:", STATISTICS_UNCONSUMED_ITEM_COUNT_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename VecA, typename VecB>
void PipelineCompiler::recordItemCountDeltas(BuilderRef b, const VecA & current, const VecB & prior, const StringRef suffix) const {

    const auto n = out_degree(mKernelIndex, mBufferGraph);
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }

    const auto fieldName = (makeKernelName(mKernelIndex) + suffix).str();
    Value * const trace = b->getScalarFieldPtr(fieldName);

    BasicBlock * const expand = b->CreateBasicBlock("");
    BasicBlock * const record = b->CreateBasicBlock("");

    Value * const offset = b->CreateURem(mSegNo, b->getSize(ITEM_COUNT_DELTA_CHUNK_LENGTH));

    Constant * const ZERO = b->getInt32(0);
    Constant * const ONE = b->getInt32(1);

    Value * const currentLog = b->CreateLoad(trace);
    BasicBlock * const entry = b->GetInsertBlock();
    b->CreateCondBr(b->CreateIsNull(offset), expand, record);

    b->SetInsertPoint(expand);
    Type * const logTy = currentLog->getType()->getPointerElementType();
    Value * const newLog = b->CreateCacheAlignedMalloc(logTy);
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    b->CreateStore(b->CreatePointerCast(currentLog, voidPtrTy), b->CreateGEP(newLog, { ZERO, ONE}));
    b->CreateStore(newLog, trace);
    BasicBlock * const expandExit = b->GetInsertBlock();
    b->CreateBr(record);

    b->SetInsertPoint(record);
    PHINode * const log = b->CreatePHI(currentLog->getType(), 2);
    log->addIncoming(currentLog, entry);
    log->addIncoming(newLog, expandExit);

    FixedArray<Value *, 4> indices;
    indices[0] = ZERO;
    indices[1] = ZERO;
    indices[2] = offset;

    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & out = mBufferGraph[e];
        const auto i = out.Port.Number;
        Value * const delta = b->CreateSub(current[i], prior[i]);
        indices[3] = b->getInt32(i);
        b->CreateStore(delta, b->CreateGEP(log, indices));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addItemCountDeltaProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addItemCountDeltaProperties(BuilderRef b, unsigned kernel, const StringRef suffix) const {
    const auto n = out_degree(kernel, mBufferGraph);
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    LLVMContext & C = b->getContext();
    IntegerType * const sizeTy = b->getSizeTy();
    PointerType * const voidPtrTy = b->getVoidPtrTy();
    ArrayType * const logTy = ArrayType::get(ArrayType::get(sizeTy, n), ITEM_COUNT_DELTA_CHUNK_LENGTH);
    StructType * const logChunkTy = StructType::get(C, { logTy, voidPtrTy } );
    PointerType * const traceTy = logChunkTy->getPointerTo();
    const auto fieldName = (makeKernelName(kernel) + suffix).str();
    mTarget->addInternalScalar(traceTy, fieldName);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printItemCountDeltas
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printItemCountDeltas(BuilderRef b, const StringRef title, const StringRef suffix) const {

    IntegerType * const sizeTy = b->getSizeTy();

    Constant * const ZERO = b->getInt32(0);
    Constant * const ONE = b->getInt32(1);

    ConstantInt * const SZ_ZERO = b->getSize(0);
    ConstantInt * const SZ_ONE = b->getSize(1);


    // Print the title line
    SmallVector<char, 100> buffer;
    raw_svector_ostream format(buffer);
    format << title << "\n\n"
              "SEG #";

    for (auto i = FirstKernel; i <= LastKernel; ++i) {

        const Kernel * const kernel = getKernel(i);
        std::string kernelName = kernel->getName();
        boost::replace_all(kernelName, "\"", "\\\"");

        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[e];
            const Binding & out = br.Binding;
            format << "," << target(e, mBufferGraph)
                   << " " << kernelName
                   << '.' << out.getName();
        }
    }

    format << "\n";


    Constant * const STDERR = b->getInt32(STDERR_FILENO);

    FixedArray<Value *, 2> titleArgs;
    titleArgs[0] = STDERR;
    titleArgs[1] = b->GetString(format.str());
    b->CreateCall(b->GetDprintf(), titleArgs);

    // Generate line format string
    buffer.clear();

    format << "%" PRIu64; // seg #
    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        format << ",%" PRIu64; // processed item count delta
    }
    format << "\n";

    // Print each kernel line
    SmallVector<Value *, 64> args(LastStreamSet - FirstStreamSet + 4);
    args[0] = STDERR;
    args[1] = b->GetString(format.str());

    SmallVector<Value *, 64> traceLogArray(LastKernel + 1);

    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        const auto n = out_degree(i, mBufferGraph);
        if (LLVM_UNLIKELY(n == 0)) {
            traceLogArray[i] = nullptr;
        } else {
            const auto fieldName = (makeKernelName(i) + suffix).str();
            traceLogArray[i] = b->getScalarFieldPtr(fieldName);
        }
    }

    Constant * const CHUNK_LENGTH = b->getSize(ITEM_COUNT_DELTA_CHUNK_LENGTH);

    Value * const chunkCount = b->CreateCeilUDiv(mSegNo,CHUNK_LENGTH);
    // Start printing the data lines
    BasicBlock * const loopEntry = b->GetInsertBlock();
    BasicBlock * const loopStart = b->CreateBasicBlock("reportStridesPerSegment");
    BasicBlock * const getNextLogChunk = b->CreateBasicBlock("getNextLogChunk");
    BasicBlock * const selectLogChunk = b->CreateBasicBlock("getNextLogChunk");
    BasicBlock * const printLogEntry = b->CreateBasicBlock("getNextLogChunk");


    b->CreateBr(loopStart);

    b->SetInsertPoint(loopStart);
    PHINode * const segNo = b->CreatePHI(sizeTy, 2);
    segNo->addIncoming(SZ_ZERO, loopEntry);
    PHINode * const chunkIndex = b->CreatePHI(sizeTy, 2);
    chunkIndex->addIncoming(chunkCount, loopEntry);
    SmallVector<PHINode *, 64> currentChunk(LastKernel + 1);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(traceLogArray[i] == nullptr)) continue;
        PointerType * const ty = cast<PointerType>(traceLogArray[i]->getType()->getPointerElementType());
        currentChunk[i] = b->CreatePHI(ty, 2);
        currentChunk[i]->addIncoming(ConstantPointerNull::get(ty), loopEntry);
    }

    args[2] = segNo;

    Value * const offset = b->CreateURem(segNo, CHUNK_LENGTH);
    b->CreateCondBr(b->CreateIsNull(offset), getNextLogChunk, printLogEntry);

    b->SetInsertPoint(getNextLogChunk);
    PHINode * const nextChunkIndexPhi = b->CreatePHI(sizeTy, 2);
    nextChunkIndexPhi->addIncoming(SZ_ZERO, loopStart);
    SmallVector<PHINode *, 64> subsequentChunk(LastKernel + 1);
    SmallVector<Value *, 64> nextChunk(LastKernel + 1);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(traceLogArray[i] == nullptr)) continue;
        Type * const ty = traceLogArray[i]->getType();
        subsequentChunk[i] = b->CreatePHI(ty, 2);
        subsequentChunk[i]->addIncoming(traceLogArray[i], loopStart);
    }
    FixedArray<Value *, 2> nextChunkIndices;
    nextChunkIndices[0] = ZERO;
    nextChunkIndices[1] = ONE;
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(subsequentChunk[i] == nullptr)) continue;
        nextChunk[i] = b->CreateLoad(subsequentChunk[i]);
        Value * subsequentChunk2 = b->CreateGEP(nextChunk[i], nextChunkIndices);
        subsequentChunk2 = b->CreatePointerCast(subsequentChunk2, subsequentChunk[i]->getType());
        subsequentChunk[i]->addIncoming(subsequentChunk2, getNextLogChunk);
    }
    Value * const nextChunkIndex = b->CreateAdd(nextChunkIndexPhi, SZ_ONE);
    nextChunkIndexPhi->addIncoming(nextChunkIndex, getNextLogChunk);

    Value * const foundCorrectChunk = b->CreateICmpEQ(nextChunkIndex, chunkIndex);
    b->CreateCondBr(foundCorrectChunk, selectLogChunk, getNextLogChunk);

    b->SetInsertPoint(selectLogChunk);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(nextChunk[i] == nullptr)) continue;
        Value * spentLog = b->CreateLoad(b->CreateGEP(nextChunk[i], nextChunkIndices));
        b->CreateFree(spentLog);
    }
    b->CreateBr(printLogEntry);

    b->SetInsertPoint(printLogEntry);
    PHINode * const nextChunkIndexPhi2 = b->CreatePHI(sizeTy, 2);
    nextChunkIndexPhi2->addIncoming(chunkIndex, loopStart);
    nextChunkIndexPhi2->addIncoming(nextChunkIndexPhi, selectLogChunk);
    SmallVector<PHINode *, 64> currentChunkPhi(LastKernel + 1);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(nextChunk[i] == nullptr)) continue;
        Type * const ty = nextChunk[i]->getType();
        currentChunkPhi[i] = b->CreatePHI(ty, 2);
        currentChunkPhi[i]->addIncoming(currentChunk[i], loopStart);
        currentChunkPhi[i]->addIncoming(nextChunk[i], selectLogChunk);
    }

    FixedArray<Value *, 4> indices;
    indices[0] = ZERO;
    indices[1] = ZERO;
    indices[2] = offset;
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(currentChunkPhi[i] == nullptr)) continue;
        for (const auto e : make_iterator_range(out_edges(i, mBufferGraph))) {
            const auto idx = target(e, mBufferGraph) - FirstStreamSet + 3;
            const BufferRateData & out = mBufferGraph[e];
            indices[3] = b->getInt32(out.Port.Number);
            args[idx] = b->CreateLoad(b->CreateGEP(currentChunkPhi[i], indices));
        }
    }

    b->CreateCall(b->GetDprintf(), args);

    BasicBlock * const loopExit = b->CreateBasicBlock("reportStridesPerSegmentExit");
    BasicBlock * const loopEnd = b->GetInsertBlock();
    Value * const nextSegNo = b->CreateAdd(segNo, SZ_ONE);
    segNo->addIncoming(nextSegNo, loopEnd);
    chunkIndex->addIncoming(nextChunkIndexPhi2, loopEnd);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(currentChunk[i] == nullptr)) continue;
        currentChunk[i]->addIncoming(currentChunkPhi[i], loopEnd);
    }

    Value * const notDone = b->CreateICmpNE(nextSegNo, mSegNo);
    b->CreateLikelyCondBr(notDone, loopStart, loopExit);

    b->SetInsertPoint(loopExit);
    // print final new line
    FixedArray<Value *, 2> finalArgs;
    finalArgs[0] = STDERR;
    finalArgs[1] = b->GetString("\n");
    b->CreateCall(b->GetDprintf(), finalArgs);
    for (auto i = FirstKernel; i <= LastKernel; ++i) {
        if (LLVM_UNLIKELY(currentChunkPhi[i] == nullptr)) continue;
        b->CreateFree(currentChunkPhi[i]);
    }
}


}
