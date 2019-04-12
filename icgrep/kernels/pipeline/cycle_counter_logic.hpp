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

                args.push_back(b->getInt32(getBufferIndex(target(e, mBufferGraph))));

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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeBufferExpansionHistory(BuilderRef b) const {

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {

        const auto firstBuffer = PipelineOutput + 1;
        const auto lastBuffer = num_vertices(mBufferGraph);

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const SZ_ZERO = b->getSize(0);
        Constant * const SZ_ONE = b->getSize(1);

        for (unsigned i = firstBuffer; i < lastBuffer; ++i) {
            const BufferNode & bn = mBufferGraph[i];
            if (LLVM_LIKELY(bn.Type == BufferType::Internal)) {
                const auto pe = in_edge(i, mBufferGraph);
                const auto p = source(pe, mBufferGraph);
                const BufferRateData & rd = mBufferGraph[pe];
                const auto prefix = makeBufferName(p, rd.Port);
                const StreamSetBuffer * const buffer = bn.Buffer;

                if (isa<DynamicBuffer>(buffer)) {
                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
                    Type * const traceDataTy = traceData->getType()->getPointerElementType();
                    Type * const traceLogTy =  traceDataTy->getStructElementType(0)->getPointerElementType();

                    Value * const traceDataArray = b->CreateCacheAlignedMalloc(traceLogTy, SZ_ONE);
                    // fill in the struct
                    b->CreateStore(traceDataArray, b->CreateGEP(traceData, {ZERO, ZERO}));
                    b->CreateStore(SZ_ONE, b->CreateGEP(traceData, {ZERO, ONE}));
                    // then the initial record
                    b->CreateStore(SZ_ZERO, b->CreateGEP(traceDataArray, {ZERO, ZERO}));
                    b->CreateStore(buffer->getCapacity(b.get()), b->CreateGEP(traceDataArray, {ZERO, ONE}));
                }
            }
        }

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::recordBufferExpansionHistory(BuilderRef b, const unsigned outputPort, const StreamSetBuffer * const buffer, Value * const expanded) const {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))) {
        if (isa<DynamicBuffer>(buffer)) {
            b->setKernel(mPipelineKernel);

            const auto prefix = makeBufferName(mKernelIndex, StreamPort{PortType::Output, outputPort});

            BasicBlock * const recordExpansion = b->CreateBasicBlock(prefix + "_recordExpansion", mKernelLoopExit);
            BasicBlock * const continueChecking = b->CreateBasicBlock(prefix + "_continueChecking", mKernelLoopExit);

            b->CreateUnlikelyCondBr(expanded, recordExpansion, continueChecking);

            b->SetInsertPoint(recordExpansion);

            Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);
            Type * const traceDataTy = traceData->getType()->getPointerElementType();
            Type * const traceLogTy =  traceDataTy->getStructElementType(0)->getPointerElementType();

            Constant * const ZERO = b->getInt32(0);
            Constant * const ONE = b->getInt32(1);

            Value * const traceLogArrayField = b->CreateGEP(traceData, {ZERO, ZERO});
            Value * traceLogArray = b->CreateLoad(traceLogArrayField);
            Value * const traceLogCountField = b->CreateGEP(traceData, {ZERO, ONE});
            Value * const traceIndex = b->CreateLoad(traceLogCountField);
            Value * const traceCount = b->CreateAdd(traceIndex, b->getSize(1));

            traceLogArray = b->CreateRealloc(traceLogTy, traceLogArray, traceCount);
            b->CreateStore(traceLogArray, traceLogArrayField);
            b->CreateStore(traceCount, traceLogCountField);

            b->CreateStore(mSegNo, b->CreateGEP(traceLogArray, {traceIndex, ZERO}));
            b->CreateStore(buffer->getCapacity(b.get()), b->CreateGEP(traceLogArray, {traceIndex, ONE}));
            b->CreateBr(continueChecking);

            b->SetInsertPoint(continueChecking);
            b->setKernel(mKernel);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalBufferExpansionHistory
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalBufferExpansionHistory(BuilderRef b) {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceDynamicBuffers))) {

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
                  "              SIZE\n"; // % of blocking attempts

        Constant * const STDERR = b->getInt32(STDERR_FILENO);

        FixedArray<Value *, 2> titleArgs;
        titleArgs[0] = STDERR;
        titleArgs[1] = b->GetString(format.str());
        b->CreateCall(b->GetDprintf(), titleArgs);

        // Generate line format string
        buffer.clear();
        format << "%3" PRIu32 " " // kernel #
                  "%-" << maxKernelLength << "s" // kernel name
                  "O%-3" PRIu32 " " // I/O type
                  "%-" << maxBindingLength << "s" // port name
                  "%7" PRIu32 // buffer ID #
                  "%14" PRIu64 " " // segment #
                  "%18" PRIu64 "\n"; // size (as multiple of streamset block size)

        // Print each kernel line

        FixedArray<Value *, 9> args;
        args[0] = STDERR;
        args[1] = b->GetString(format.str());

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            bool first = true;
            for (const auto & output : make_iterator_range(out_edges(i, mBufferGraph))) {
                const BufferRateData & br = mBufferGraph[output];
                const auto bufferVertex = target(output, mBufferGraph);
                const BufferNode & bn = mBufferGraph[bufferVertex];
                if (isa<DynamicBuffer>(bn.Buffer)) {

                    if (first) {
                        const Kernel * const kernel = getKernel(i);
                        args[2] = b->getInt32(i);
                        args[3] = b->GetString(kernel->getName());
                        first = false;
                    }

                    const auto outputPort = br.outputPort();
                    args[4] = b->getInt32(outputPort);
                    const Binding & binding = br.Binding;
                    args[5] = b->GetString(binding.getName());
                    args[6] = b->getInt32(getBufferIndex(bufferVertex));

                    const auto prefix = makeBufferName(i, StreamPort{PortType::Output, outputPort});
                    Value * const traceData = b->getScalarFieldPtr(prefix + STATISTICS_BUFFER_EXPANSION_SUFFIX);

                    Constant * const ZERO = b->getInt32(0);
                    Constant * const ONE = b->getInt32(1);

                    Value * const traceArrayField = b->CreateGEP(traceData, {ZERO, ZERO});
                    Value * const traceArray = b->CreateLoad(traceArrayField);
                    Value * const traceCountField = b->CreateGEP(traceData, {ZERO, ONE});
                    Value * const traceCount = b->CreateLoad(traceCountField);

                    BasicBlock * const outputEntry = b->GetInsertBlock();
                    BasicBlock * const outputLoop = b->CreateBasicBlock(prefix + "_bufferExpansionReportLoop");
                    BasicBlock * const outputExit = b->CreateBasicBlock(prefix + "_bufferExpansionReportExit");

                    b->CreateBr(outputLoop);

                    b->SetInsertPoint(outputLoop);
                    PHINode * const index = b->CreatePHI(b->getSizeTy(), 2);
                    index->addIncoming(b->getSize(0), outputEntry);

                    Value * const segmentNumField = b->CreateGEP(traceArray, {index, ZERO});
                    Value * const segmentNum = b->CreateLoad(segmentNumField);

                    args[7] = segmentNum;

                    Value * const newBufferSizeField = b->CreateGEP(traceArray, {index, ONE});
                    Value * const newBufferSize = b->CreateLoad(newBufferSizeField);

                    args[8] = newBufferSize;

                    b->CreateCall(b->GetDprintf(), args);

                    Value * const nextIndex = b->CreateAdd(index, b->getSize(1));
                    index->addIncoming(nextIndex, outputLoop);
                    Value * const notDone = b->CreateICmpULT(nextIndex, traceCount);
                    b->CreateCondBr(notDone, outputLoop, outputExit);

                    b->SetInsertPoint(outputExit);
                }
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
 * @brief initializeStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeStridesPerSegment(BuilderRef b) const {

    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {
        b->setKernel(mPipelineKernel);

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

        b->setKernel(mKernel);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recordStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::recordStridesPerSegment(BuilderRef b, Value * const numOfStrides) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {
        // NOTE: this records only the change to attempt to reduce the memory usage of this log.

        b->setKernel(mPipelineKernel);
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

        b->CreateCall(record, { kernelTraceLog, mSegNo, numOfStrides } );
        b->setKernel(mKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printOptionalStridesPerSegment
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::printOptionalStridesPerSegment(BuilderRef b) const {
    if (LLVM_UNLIKELY(DebugOptionIsSet(codegen::TraceStridesPerSegment))) {

        IntegerType * const sizeTy = b->getSizeTy();
        IntegerType * const boolTy = b->getInt1Ty();

        Constant * const ZERO = b->getInt32(0);
        Constant * const ONE = b->getInt32(1);
        Constant * const TWO = b->getInt32(2);

        ConstantInt * const SZ_ZERO = b->getSize(0);
        ConstantInt * const SZ_ONE = b->getSize(1);

        ConstantInt * const FALSE = b->getFalse();
        ConstantInt * const TRUE = b->getTrue();

        // Print the title line
        SmallVector<char, 100> buffer;
        raw_svector_ostream format(buffer);
        format << "STRIDES PER SEGMENT:\n\n"
                  "SEG #";
        SmallVector<Constant *, 64> strideSize(LastKernel + 1);

        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            const Kernel * const kernel = getKernel(i);
            strideSize[i] = b->getSize(kernel->getStride());
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

        Value * notDone = FALSE;
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
            PHINode * const nextNotDone = b->CreatePHI(boolTy, 3);
            nextNotDone->addIncoming(notDone, entry);
            nextNotDone->addIncoming(TRUE, check);
            nextNotDone->addIncoming(TRUE, update);
            notDone = nextNotDone;
        }

        args[2] = segNo;
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            args[i - FirstKernel + 3] = b->CreateMul(updatedValue[i], strideSize[i]);
        }

        b->CreateCall(b->GetDprintf(), args);

        BasicBlock * const loopExit = b->CreateBasicBlock("reportStridesPerSegmentExit");
        BasicBlock * const loopEnd = b->GetInsertBlock();
        segNo->addIncoming(b->CreateAdd(segNo, SZ_ONE), loopEnd);
        for (auto i = FirstKernel; i <= LastKernel; ++i) {
            currentIndex[i]->addIncoming(updatedIndex[i], loopEnd);
            currentValue[i]->addIncoming(updatedValue[i], loopEnd);
        }

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

}
