#ifndef IO_CALCULATION_LOGIC_HPP
#define IO_CALCULATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#include <llvm/Support/ErrorHandling.h>

// TODO: add in assertions to prove whether all countable rate pipeline I/O was satisfied in the single iteration
// Is it sufficient to verify symbolic rate of the pipeline matches the rate of the I/O?

// TODO: if a popcount ref stream is zero extended, the current partial sum replacement would not work correctly
// since the equivalent for it would be to repeat the final number infinitely.

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readPipelineIOItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readPipelineIOItemCounts(BuilderRef b) {

    // TODO: this needs to be considered more: if we have multiple consumers of a pipeline input and
    // they process the input data at differing rates, how do we ensure that we always resume processing
    // at the correct position? We can store the actual item counts / delta of the consumed count
    // internally but this would be problematic for optimization branches as we may have processed data
    // using the alternate path and any internally stored counts/deltas are irrelevant.

    // Would a simple "reset" be enough?

    mKernelId = PipelineInput;

    ConstantInt * const ZERO = b->getSize(0);

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        mLocallyAvailableItems[streamSet] = ZERO;
    }

    // NOTE: all outputs of PipelineInput node are inputs to the PipelineKernel
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const StreamSetPort inputPort = mBufferGraph[e].Port;
        assert (inputPort.Type == PortType::Output);
        Value * const available = getAvailableInputItems(inputPort.Number);
        setLocallyAvailableItemCount(b, inputPort, available);
        initializeConsumedItemCount(b, inputPort, available);
    }

    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {

        const auto buffer = target(e, mBufferGraph);
        const StreamSetPort inputPort = mBufferGraph[e].Port;
        assert (inputPort.Type == PortType::Output);

        Value * const inPtr = getProcessedInputItemsPtr(inputPort.Number);
        Value * const processed = b->CreateLoad(inPtr);
        for (const auto e : make_iterator_range(out_edges(buffer, mBufferGraph))) {
            const BufferPort & rd = mBufferGraph[e];
            const auto kernelIndex = target(e, mBufferGraph);
            const auto prefix = makeBufferName(kernelIndex, rd.Port);
            Value * const ptr = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
            b->CreateStore(processed, ptr);
        }
    }

    mKernelId = PipelineOutput;

    // NOTE: all inputs of PipelineOutput node are outputs of the PipelineKernel
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        const StreamSetPort outputPort = mBufferGraph[e].Port;
        assert (outputPort.Type == PortType::Input);
        Value * outPtr = getProducedOutputItemsPtr(outputPort.Number);
        Value * const produced = b->CreateLoad(outPtr);
        for (const auto e : make_iterator_range(in_edges(buffer, mBufferGraph))) {
            const BufferPort & rd = mBufferGraph[e];
            const auto kernelId = source(e, mBufferGraph);
            const auto prefix = makeBufferName(kernelId, rd.Port);
            Value * const ptr = b->getScalarFieldPtr(prefix + ITEM_COUNT_SUFFIX);
            b->CreateStore(produced, ptr);
        }
    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief detemineMaximumNumberOfStrides
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::detemineMaximumNumberOfStrides(BuilderRef b) {
    // The partition root kernel determines the amount of data processed based on the partition input.
    // During normal execution, it always performs max num of strides worth of work. However, if this
    // segment is the last segment, mPartitionSegmentLength is artificially raised to a value of ONE
    // even though we likely can only execute a partial segment worth of work. The root kernel
    // calculates how many strides can be performed and sets mNumOfPartitionStrides to that value.

    // To avoid having every kernel test their I/O during normal execution, the non-root kernels in
    // the same partition refer to the mNumOfPartitionStrides to determine how their segment length.

    if (mIsPartitionRoot) {
       mMaximumNumOfStrides = b->CreateMul(mExpectedNumOfStridesMultiplier, b->getSize(MaximumNumOfStrides[FirstKernelInPartition]));
    } else {
        const Rational strideRateFactor{MaximumNumOfStrides[mKernelId], MaximumNumOfStrides[FirstKernelInPartition]};
        mMaximumNumOfStrides = b->CreateMulRational(mNumOfPartitionStrides, strideRateFactor / mPartitionStrideRateScalingFactor);
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineNumOfLinearStrides
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determineNumOfLinearStrides(BuilderRef b) {

    // If this kernel does not have an explicit do final segment, then we want to know whether this stride will
    // be the final stride of the kernel. (i.e., that it will be flagged as terminated after executing the kernel
    // code.) For this to occur, at least one of the inputs must be closed and we must pass all of the data from
    // that closed input stream to the kernel. It is possible for a stream to be closed but to have more data
    // left to process (either due to some data being divided across a buffer boundary or because another stream
    // has less data (relatively speaking) than the closed stream.

    if (LLVM_UNLIKELY(TraceIO)) {
        mBranchToLoopExit = b->getFalse();
    }

    // If this kernel is the root of a partition, we'll use the available input to compute how many strides the
    // kernels within the partition will execute. Otherwise we begin by bounding the kernel by the expected number
    // of strides w.r.t. its partition's root.

    Value * numOfLinearStrides = nullptr;

    if (mMayLoopToEntry) {
        numOfLinearStrides = b->CreateSub(mMaximumNumOfStrides, mCurrentNumOfStridesAtLoopEntryPhi);
    } else {
        numOfLinearStrides = mMaximumNumOfStrides;
    }

    for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[input];
        if (port.CanModifySegmentLength) {
            const auto streamSet = source(input, mBufferGraph);
            checkForSufficientInputData(b, port, streamSet);
        } else {
            getAccessibleInputItems(b, port);
        }
    }


    if (LLVM_LIKELY(hasAtLeastOneNonGreedyInput())) {
        for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & port = mBufferGraph[input];
            if (port.CanModifySegmentLength) {
                Value * const strides = getNumOfAccessibleStrides(b, port, numOfLinearStrides);
                numOfLinearStrides = b->CreateUMin(numOfLinearStrides, strides);
            }
        }
    } else {
        Value * const exhausted = checkIfInputIsExhausted(b, InputExhaustionReturnType::Conjunction);
        numOfLinearStrides = b->CreateZExt(b->CreateNot(exhausted), b->getSizeTy());
    }

    Value * numOfOutputLinearStrides = numOfLinearStrides;
    for (const auto output : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[output];
        if (port.CanModifySegmentLength) {
            Value * const strides = getNumOfWritableStrides(b, port, numOfOutputLinearStrides);
            numOfOutputLinearStrides = b->CreateUMin(numOfOutputLinearStrides, strides);
        }
    }

    if (numOfOutputLinearStrides != numOfLinearStrides) {
        Value * const mustExpand = b->CreateIsNull(numOfOutputLinearStrides);
        numOfLinearStrides = b->CreateSelect(mustExpand, numOfLinearStrides, numOfOutputLinearStrides);
    }

    numOfLinearStrides = calculateTransferableItemCounts(b, numOfLinearStrides);

    mNumOfLinearStrides = numOfLinearStrides;

    if (mMayLoopToEntry) {
        mUpdatedNumOfStrides = b->CreateAdd(mCurrentNumOfStridesAtLoopEntryPhi, numOfLinearStrides);
    } else {
        mUpdatedNumOfStrides = numOfLinearStrides;
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[e];
        const auto streamSet = target(e, mBufferGraph);
        ensureSufficientOutputSpace(b, port, streamSet);
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateTransferableItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateTransferableItemCounts(BuilderRef b, Value * const numOfLinearStrides) {

    const auto numOfInputs = in_degree(mKernelId, mBufferGraph);
    const auto numOfOutputs = out_degree(mKernelId, mBufferGraph);

    // --- lambda function start
    auto phiOutItemCounts = [&](const Vec<Value *> & accessibleItems,
                               const Vec<Value *> & inputVirtualBaseAddress,
                               const Vec<Value *> & writableItems,
                               Value * const fixedRateFactor,
                               Value * const terminationSignal,
                               Value * const numOfLinearStrides,
                               Value * const fixedRatePartialStrideRemainder) {
        BasicBlock * const exitBlock = b->GetInsertBlock();
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto port = StreamSetPort{ PortType::Input, i };
            mLinearInputItemsPhi[port]->addIncoming(accessibleItems[i], exitBlock);
            mInputVirtualBaseAddressPhi[port]->addIncoming(inputVirtualBaseAddress[i], exitBlock);
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto port = StreamSetPort{ PortType::Output, i };
            mLinearOutputItemsPhi[port]->addIncoming(writableItems[i], exitBlock);
        }
        if (mFixedRateFactorPhi) { assert (fixedRateFactor);
            mFixedRateFactorPhi->addIncoming(fixedRateFactor, exitBlock);
        }
        mIsFinalInvocationPhi->addIncoming(terminationSignal, exitBlock);
        mNumOfLinearStridesPhi->addIncoming(numOfLinearStrides, exitBlock);
        if (mIsPartitionRoot) {
            mFinalPartialStrideFixedRateRemainderPhi->addIncoming(fixedRatePartialStrideRemainder, exitBlock);
        }
    };
    // --- lambda function end

    ConstantInt * const sz_ZERO = b->getSize(0);

    Vec<Value *> accessibleItems(numOfInputs);

    Vec<Value *> inputVirtualBaseAddress(numOfInputs, nullptr);

    Vec<Value *> writableItems(numOfOutputs);

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);

    getInputVirtualBaseAddresses(b, inputVirtualBaseAddress);

    Value * nonFinalNumOfLinearStrides = nullptr;
    if (LLVM_UNLIKELY(mIsPartitionRoot && StrideStepLength[mKernelId] > 1)) {
        nonFinalNumOfLinearStrides = b->CreateRoundDown(numOfLinearStrides, b->getSize(StrideStepLength[mKernelId]));
    } else {
        nonFinalNumOfLinearStrides = numOfLinearStrides;
    }

    if (LLVM_LIKELY(in_degree(mKernelId, mBufferGraph) > 0)) {

        const auto prefix = makeKernelName(mKernelId);
        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(prefix + "_nonFinalSegment", mKernelCheckOutputSpace);

        BasicBlock * const enteringFinalSegment = b->CreateBasicBlock(prefix + "_finalSegment", mKernelCheckOutputSpace);

        Vec<Value *> zeroExtendedInputVirtualBaseAddress(numOfInputs, nullptr);

        /// -------------------------------------------------------------------------------------
        /// HANDLE ZERO EXTENSION
        /// -------------------------------------------------------------------------------------

        Value * isFinalSegment = nullptr;
        if (mIsPartitionRoot) {
            isFinalSegment = mAnyClosed; // b->CreateICmpEQ(numOfLinearStrides, sz_ZERO);
        } else {
            isFinalSegment = mFinalPartitionSegment;
        }

        BasicBlock * const nonZeroExtendExit = b->GetInsertBlock();

        BasicBlock * afterNonFinalZeroExtendExit = nullptr;

        if (mHasZeroExtendedInput) {
            BasicBlock * const checkFinal =
                b->CreateBasicBlock(prefix + "_checkFinal", enteringNonFinalSegment);
            Value * const isFinalOrZeroExtended = b->CreateOr(mHasZeroExtendedInput, isFinalSegment); // isFinal);
            b->CreateUnlikelyCondBr(isFinalOrZeroExtended, checkFinal, enteringNonFinalSegment);

            b->SetInsertPoint(checkFinal);
            Value * const zeroExtendSpace = allocateLocalZeroExtensionSpace(b, enteringNonFinalSegment);
            getZeroExtendedInputVirtualBaseAddresses(b, inputVirtualBaseAddress, zeroExtendSpace, zeroExtendedInputVirtualBaseAddress);
            afterNonFinalZeroExtendExit = b->GetInsertBlock();
        }
        b->CreateUnlikelyCondBr(isFinalSegment, enteringFinalSegment, enteringNonFinalSegment);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL OR ZERO-EXTENDED SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringFinalSegment);

        // if we have a potentially zero-extended buffer, use that; otherwise select the normal buffer
        Vec<Value *> truncatedInputVirtualBaseAddress(numOfInputs);
        for (unsigned i = 0; i != numOfInputs; ++i) {
            Value * const ze = zeroExtendedInputVirtualBaseAddress[i];
            Value * const vba = inputVirtualBaseAddress[i];
            truncatedInputVirtualBaseAddress[i] = ze ? ze : vba;
        }

        Value * numOfFinalLinearStrides = numOfLinearStrides;

        if (!mIsPartitionRoot) {
            for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
                const BufferPort & port = mBufferGraph[input];
                if (!port.CanModifySegmentLength) {
                    Value * const strides = getNumOfAccessibleStrides(b, port, numOfLinearStrides);
                    numOfFinalLinearStrides = b->CreateUMin(numOfFinalLinearStrides, strides);
                }
            }
        }

        Value * const isFinal = b->CreateICmpEQ(numOfFinalLinearStrides, sz_ZERO);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL STRIDE
        /// -------------------------------------------------------------------------------------

        BasicBlock * penultimateSegmentExit = nullptr;
        if (LLVM_LIKELY(mMayLoopToEntry)) {
            BasicBlock * const enteringFinalStride = b->CreateBasicBlock(prefix + "_finalStride", mKernelCheckOutputSpace);
            penultimateSegmentExit = b->GetInsertBlock();
            b->CreateUnlikelyCondBr(isFinal, enteringFinalStride, enteringNonFinalSegment);

            b->SetInsertPoint(enteringFinalStride);
        }

        Value * fixedItemFactor = nullptr;
        Value * partialPartitionStride = nullptr;

        calculateFinalItemCounts(b, accessibleItems, writableItems, fixedItemFactor, partialPartitionStride);
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        zeroInputAfterFinalItemCount(b, accessibleItems, truncatedInputVirtualBaseAddress);
        phiOutItemCounts(accessibleItems, truncatedInputVirtualBaseAddress, writableItems,
                         fixedItemFactor, completed, numOfLinearStrides, partialPartitionStride);
        b->CreateBr(mKernelCheckOutputSpace);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING NON-FINAL SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringNonFinalSegment);
        if (afterNonFinalZeroExtendExit || penultimateSegmentExit) {
            PHINode * const nonFinalNumOfLinearStridesPhi = b->CreatePHI(numOfLinearStrides->getType(), 3);

            nonFinalNumOfLinearStridesPhi->addIncoming(nonFinalNumOfLinearStrides, nonZeroExtendExit);
            if (afterNonFinalZeroExtendExit) {
                nonFinalNumOfLinearStridesPhi->addIncoming(nonFinalNumOfLinearStrides, afterNonFinalZeroExtendExit);
            }
            if (penultimateSegmentExit) {
                nonFinalNumOfLinearStridesPhi->addIncoming(numOfLinearStrides, penultimateSegmentExit);
            }

            for (unsigned i = 0; i != numOfInputs; ++i) {
                Value * const ze = zeroExtendedInputVirtualBaseAddress[i];
                if (ze) {
                    PHINode * const phi = b->CreatePHI(ze->getType(), 3);
                    phi->addIncoming(inputVirtualBaseAddress[i], nonZeroExtendExit);
                    if (afterNonFinalZeroExtendExit) {
                        phi->addIncoming(ze, afterNonFinalZeroExtendExit);
                    }
                    if (penultimateSegmentExit) {
                        phi->addIncoming(ze, penultimateSegmentExit);
                    }
                    inputVirtualBaseAddress[i] = phi;
                }
            }
            nonFinalNumOfLinearStrides = nonFinalNumOfLinearStridesPhi;
        }
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALCULATE NON-FINAL INPUT COUNT
    /// -------------------------------------------------------------------------------------

    assert (nonFinalNumOfLinearStrides);

    Value * fixedRateFactor = nullptr;
    if (mFixedRateFactorPhi) {
        const Rational stride(mKernel->getStride());
        fixedRateFactor  = b->CreateMulRational(nonFinalNumOfLinearStrides, stride * mFixedRateLCM);
    }

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        accessibleItems[br.Port.Number] = calculateNumOfLinearItems(b, br, nonFinalNumOfLinearStrides);
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        writableItems[br.Port.Number] = calculateNumOfLinearItems(b, br, nonFinalNumOfLinearStrides);
    }

    phiOutItemCounts(accessibleItems, inputVirtualBaseAddress, writableItems,
                     fixedRateFactor, unterminated, nonFinalNumOfLinearStrides, sz_ZERO);

    b->CreateBr(mKernelCheckOutputSpace);

    b->SetInsertPoint(mKernelCheckOutputSpace);
    return mNumOfLinearStridesPhi;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForSufficientInputData(BuilderRef b, const BufferPort & port, const unsigned streamSet) {

    const auto inputPort = port.Port;
    assert (inputPort.Type == PortType::Input);

    Value * strideLength = getInputStrideLength(b, port);
    // Only the partition root dictates how many strides this kernel will end up doing. All other kernels
    // simply have to trust that the root determined the correct number or we'd be forced to have an
    // under/overflow capable of containing an entire segment rather than a single stride.
    if (mIsPartitionRoot && StrideStepLength[mKernelId] > 1) {
        ConstantInt * const stepSize = b->getSize(StrideStepLength[mKernelId]);
        strideLength = b->CreateMul(strideLength, stepSize);
    }

    Value * const required = addLookahead(b, port, strideLength); assert (required);
    const auto prefix = makeBufferName(mKernelId, inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_requiredInput (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), required);
    #endif

    Value * const accessible = getAccessibleInputItems(b, port); assert (accessible);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_accessible (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), accessible);
    #endif

    Value * const hasEnough = b->CreateICmpUGE(accessible, required);
    Value * const closed = isClosed(b, inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_closed = %" PRIu8, closed);
    #endif
    if (mIsPartitionRoot) {
        if (mAnyClosed) {
            mAnyClosed = b->CreateOr(mAnyClosed, closed);
        } else {
            mAnyClosed = closed;
        }
    }
    Value * const sufficientInput = b->CreateOr(hasEnough, closed);
    BasicBlock * const hasInputData = b->CreateBasicBlock(prefix + "_hasInputData", mKernelCheckOutputSpace);

    BasicBlock * recordBlockedIO = nullptr;
    BasicBlock * insufficentIO = mKernelInsufficientInput;
    assert (mKernelInsufficientInput);
    if (LLVM_UNLIKELY(TraceIO)) {
        recordBlockedIO = b->CreateBasicBlock(prefix + "_recordBlockedIO", hasInputData);
        insufficentIO = recordBlockedIO;
    }

    Value * test = sufficientInput;
    Value * insufficient = mBranchToLoopExit;
    if (LLVM_UNLIKELY(TraceIO)) {
        // do not record the block if this not the first execution of the
        // kernel but ensure that the system knows at least one failed.
        test = b->CreateOr(sufficientInput, mExecutedAtLeastOnceAtLoopEntryPhi);
        insufficient = b->CreateOr(mBranchToLoopExit, b->CreateNot(sufficientInput));
    }

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_hasInputData = %" PRIu8, test);
    #endif


    if (mExhaustedPipelineInputPhi && !TraceIO) {
        Value * exhausted = mExhaustedInput;
        if (LLVM_UNLIKELY(mHasPipelineInput.test(inputPort.Number))) {
            exhausted = b->getTrue();
        }
        BasicBlock * const exitBlock = b->GetInsertBlock();
        mExhaustedPipelineInputPhi->addIncoming(exhausted, exitBlock);
    }

    b->CreateLikelyCondBr(test, hasInputData, insufficentIO);

    // When tracing blocking I/O, test all I/O streams but do not execute
    // the kernel if any stream is insufficient.
    if (LLVM_UNLIKELY(TraceIO)) {
        BasicBlock * const entryBlock = b->GetInsertBlock();

        b->SetInsertPoint(recordBlockedIO);
        recordBlockingIO(b, inputPort);
        BasicBlock * const exitBlock = b->GetInsertBlock();
        b->CreateBr(hasInputData);

        b->SetInsertPoint(hasInputData);
        IntegerType * const boolTy = b->getInt1Ty();

        PHINode * const anyInsufficient = b->CreatePHI(boolTy, 2);
        anyInsufficient->addIncoming(insufficient, entryBlock);
        anyInsufficient->addIncoming(b->getTrue(), exitBlock);
        mBranchToLoopExit = anyInsufficient;
    }

    b->SetInsertPoint(hasInputData);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkIfInputIsExhausted
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::checkIfInputIsExhausted(BuilderRef b, InputExhaustionReturnType returnValType) {
    if (LLVM_UNLIKELY(in_degree(mKernelId, mBufferGraph) == 0)) {
        if (ExternallySynchronized) {
            return b->isFinal();
        } else {
            return b->getFalse();
        }
    }
    if (mIsPartitionRoot) {
        Value * resultVal = nullptr;
        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & br =  mBufferGraph[e];
            if (LLVM_UNLIKELY(br.IsZeroExtended)) {
                continue;
            }
            const auto streamSet = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            Value * const closed = isClosed(b, br.Port); assert (closed);
            Value * fullyConsumed = closed;
            if (!bn.IsLinear) {
                Value * const processed = mAlreadyProcessedPhi[br.Port];
                Value * const accessible = getAccessibleInputItems(b, br);
                Value * const total = b->CreateAdd(processed, accessible);
                Value * const avail = getLocallyAvailableItemCount(b, br.Port);
                Value * const fullyReadable = b->CreateICmpEQ(total, avail);
                fullyConsumed = b->CreateAnd(closed, fullyReadable);
            }
            if (resultVal) {
                if (returnValType == InputExhaustionReturnType::Conjunction) {
                    resultVal = b->CreateAnd(resultVal, closed);
                } else {
                    resultVal = b->CreateOr(resultVal, closed);
                }
            } else {
                resultVal = fullyConsumed;
            }
        }
        assert (resultVal && "non-zero-extended stream is required");
        return resultVal;
    } else {
        return mFinalPartitionSegment;
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasMoreInput
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasMoreInput(BuilderRef b) {
    assert (mMayLoopToEntry);
    assert (mMaximumNumOfStrides);

    Value * const notAtSegmentLimit = b->CreateICmpNE(mUpdatedNumOfStrides, mMaximumNumOfStrides);
    Value * const nonFinal = b->CreateIsNull(mIsFinalInvocation);

    if (mIsPartitionRoot) {

        ConstantInt * const i1_TRUE = b->getTrue();

        BasicBlock * const lastTestExit = b->CreateBasicBlock("", mKernelLoopExit);
        PHINode * const enoughInputPhi = PHINode::Create(b->getInt1Ty(), 4, "", lastTestExit);

        bool firstTest = true;

        graph_traits<BufferGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(mKernelId, mBufferGraph);

        Value * enoughInput = nonFinal;

        ConstantInt * const amount = b->getSize(StrideStepLength[mKernelId]);
        Value * const nextStrideIndex = b->CreateAdd(mNumOfLinearStrides, amount);

        while (ei != ei_end) {
            const auto e = *ei++;
            const BufferPort & port =  mBufferGraph[e];
            if (LLVM_UNLIKELY(port.IsZeroExtended)) {
                continue;
            }
            const auto streamSet = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            if (LLVM_UNLIKELY(bn.Locality == BufferLocality::ThreadLocal)) {
                continue;
            }

            Value * const processed = mAlreadyProcessedPhi[port.Port];
            // Value * const processed = mProcessedItemCount[port.Port];
            Value * avail = getLocallyAvailableItemCount(b, port.Port);

            Value * const closed = isClosed(b, port.Port);

            if (port.Add) {
                Constant * const ZERO = b->getSize(0);
                Constant * const ADD = b->getSize(port.Add);
                Value * const added = b->CreateSelect(closed, ADD, ZERO);
                avail = b->CreateAdd(avail, added);
            }

            Value * const remaining = b->CreateSub(avail, processed);
            Value * const nextStrideLength = calculateStrideLength(b, port, processed, nextStrideIndex);
            Value * const required = addLookahead(b, port, nextStrideLength); assert (required);

            Value * hasEnough = b->CreateOr(closed, b->CreateICmpUGE(remaining, required));

            // If the next rate we check is a PartialSum, always check it; otherwise we expect that
            // if this test passes the first check, it will pass the remaining ones so don't bother
            // creating a branch for the remaining checks.
            bool useBranch = firstTest;
            for (auto ej = ei; ej != ei_end; ++ej) {
                const BufferPort & next =  mBufferGraph[*ej];
                if (LLVM_UNLIKELY(next.IsZeroExtended)) {
                    continue;
                }
                const Binding & binding = next.Binding;
                const ProcessingRate & rate = binding.getRate();
                useBranch = rate.isPartialSum();
                break;
            }

            if (useBranch) {
                BasicBlock * const nextTest = b->CreateBasicBlock("", mKernelLoopExit);
                BasicBlock * const exitBlock = b->GetInsertBlock();
                enoughInputPhi->addIncoming(b->getFalse(), exitBlock);

                if (firstTest) {
                    enoughInput = b->CreateAnd(enoughInput, hasEnough);
                    Value * const supportsAnotherStride = b->CreateAnd(enoughInput, notAtSegmentLimit);
                    b->CreateUnlikelyCondBr(supportsAnotherStride, nextTest, lastTestExit);
                    firstTest = false;
                } else {
                    assert (enoughInput);
                    enoughInput = b->CreateAnd(enoughInput, hasEnough);
                    Value * const supportsAnotherStride = enoughInput;

                    b->CreateLikelyCondBr(supportsAnotherStride, nextTest, lastTestExit);
                }
                b->SetInsertPoint(nextTest);
                enoughInput = i1_TRUE;
            } else {
                enoughInput = b->CreateAnd(enoughInput, hasEnough);
            }
        }
        enoughInputPhi->addIncoming(enoughInput, b->GetInsertBlock());
        b->CreateBr(lastTestExit);

        b->SetInsertPoint(lastTestExit);
        Value * hasMore = nullptr;
        if (LLVM_UNLIKELY(firstTest)) {
            hasMore = notAtSegmentLimit;
        } else {
            hasMore = enoughInputPhi;
        }
        return hasMore;
    } else {
        //  (final segment OR up<max) AND NOT final stride
        return b->CreateAnd(b->CreateOr(mFinalPartitionSegment, notAtSegmentLimit), nonFinal);
   }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getAccessibleInputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getAccessibleInputItems(BuilderRef b, const BufferPort & port, const bool useOverflow) {

    const auto inputPort = port.Port;
    assert (inputPort.Type == PortType::Input);

    auto & A = mAccessibleInputItems[inputPort.Number];
    Value * const alreadyComputed = A[useOverflow ? WITH_OVERFLOW : WITHOUT_OVERFLOW];
    if (alreadyComputed) {
        return alreadyComputed;
    }

    const auto input = getInput(mKernelId, inputPort);
    const auto streamSet = source(input, mBufferGraph);

    const BufferNode & bn = mBufferGraph[streamSet];

    const StreamSetBuffer * const buffer = bn.Buffer;
    Value * const available = getLocallyAvailableItemCount(b, inputPort);

    Value * const processed = mAlreadyProcessedPhi[inputPort];

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelId, inputPort);
    debugPrint(b, prefix + "_available = %" PRIu64, available);
    debugPrint(b, prefix + "_processed (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), processed);
    #endif

    Value * overflow = nullptr;
    if (LLVM_LIKELY(useOverflow)) {
        if (bn.CopyForwards > 0 || port.Add > 0) {
            const auto A = port.Add;
            const auto L = bn.CopyForwards;
            if (A == L) {
                overflow = b->getSize(A);
            } else {
                Value * const closed = isClosed(b, inputPort);
                overflow = b->CreateSelect(closed, b->getSize(A), b->getSize(L));
            }
            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, prefix + "_overflow (add:%" PRIu64 ",la:%" PRIu64 ") = %" PRIu64,
                                    b->getSize(A),
                                    b->getSize(L),
                                    overflow);
            #endif
        }
    }

    Value * accessible = buffer->getLinearlyAccessibleItems(b, processed, available, overflow);
    #ifndef DISABLE_ZERO_EXTEND
    if (LLVM_UNLIKELY(port.IsZeroExtended)) {
        // To zero-extend an input stream, we must first exhaust all input for this stream before
        // switching to a "zeroed buffer". The size of the buffer will be determined by the final
        // number of non-zero-extended strides.

        // NOTE: the producer of this stream will zero out all data after its last produced item
        // that can be read by a single iteration of any consuming kernel.

        Constant * const MAX_INT = ConstantInt::getAllOnesValue(b->getSizeTy());
        Value * const closed = isClosed(b, inputPort);
        Value * const exhausted = b->CreateICmpUGE(processed, available);
        Value * const useZeroExtend = b->CreateAnd(closed, exhausted);
        mIsInputZeroExtended[inputPort] = useZeroExtend;
        if (LLVM_LIKELY(mHasZeroExtendedInput == nullptr)) {
            mHasZeroExtendedInput = useZeroExtend;
        } else {
            mHasZeroExtendedInput = b->CreateOr(mHasZeroExtendedInput, useZeroExtend);
        }
        accessible = b->CreateSelect(useZeroExtend, MAX_INT, accessible);
    }
    #endif

    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & inputBinding = port.Binding;
        Value * valid = b->CreateICmpULE(processed, available);
        Value * const zeroExtended = mIsInputZeroExtended[inputPort];
        if (zeroExtended) {
            valid = b->CreateOr(valid, zeroExtended);
        }
        b->CreateAssert(valid,
                        "%s.%s: processed count (%" PRIu64 ") exceeds total count (%" PRIu64 ")",
                        mCurrentKernelName,
                        b->GetString(inputBinding.getName()),
                        processed, available);
    }

    // cache the values for later use
    if (useOverflow) {
        A[WITH_OVERFLOW] = accessible;
    }
    if (overflow == nullptr) {
        A[WITHOUT_OVERFLOW] = accessible;
    }
    return accessible;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ensureSufficientOutputSpace
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::ensureSufficientOutputSpace(BuilderRef b, const BufferPort & port, const unsigned streamSet) {

    const BufferNode & bn = mBufferGraph[streamSet];

    if (bn.Locality == BufferLocality::ThreadLocal || bn.isUnowned()) {
        return;
    }

    const auto outputPort = port.Port;
    assert (outputPort.Type == PortType::Output);
    const auto prefix = makeBufferName(mKernelId, outputPort);
    const StreamSetBuffer * const buffer = bn.Buffer;

    getWritableOutputItems(b, port, true);

    Value * const required = mLinearOutputItemsPhi[outputPort];
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_required (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), required);
    debugPrint(b, prefix + "_addr [%" PRIx64 ",%" PRIx64 ")",
               buffer->getMallocAddress(b), buffer->getOverflowAddress(b));
    #endif

    BasicBlock * const expandBuffer = b->CreateBasicBlock(prefix + "_expandBuffer", mKernelLoopCall);
    BasicBlock * const expanded = b->CreateBasicBlock(prefix + "_expandedBuffer", mKernelLoopCall);
    const auto beforeExpansion = mWritableOutputItems[outputPort.Number];

    Value * const hasEnoughSpace = b->CreateICmpULE(required, beforeExpansion[WITH_OVERFLOW]);

    BasicBlock * const noExpansionExit = b->GetInsertBlock();
    b->CreateLikelyCondBr(hasEnoughSpace, expanded, expandBuffer);

    b->SetInsertPoint(expandBuffer);
    #ifdef ENABLE_PAPI
    readPAPIMeasurement(b, mKernelId, PAPIReadBeforeMeasurementArray);
    #endif
    Value * cycleCounterStart = nullptr;
    if (LLVM_UNLIKELY(EnableCycleCounter)) {
        cycleCounterStart = b->CreateReadCycleCounter();
    }

    // TODO: we need to calculate the total amount required assuming we process all input. This currently
    // has a flaw in which if the input buffers had been expanded sufficiently yet processing had been
    // held back by some input stream, we may end up expanding twice in the same iteration of this kernel,
    // which could result in free'ing the "old" buffer twice.

    Value * const produced = mAlreadyProducedPhi[outputPort]; assert (produced);
    Value * const consumed = mInitialConsumedItemCount[streamSet]; assert (consumed);

    buffer->reserveCapacity(b, produced, consumed, required);

    recordBufferExpansionHistory(b, outputPort, buffer);
    updateCycleCounter(b, mKernelId, cycleCounterStart, BUFFER_EXPANSION);
    #ifdef ENABLE_PAPI
    accumPAPIMeasurementWithoutReset(b, PAPIReadBeforeMeasurementArray, mKernelId, PAPI_BUFFER_EXPANSION);
    #endif

    auto & afterExpansion = mWritableOutputItems[outputPort.Number];
    afterExpansion[WITH_OVERFLOW] = nullptr;
    afterExpansion[WITHOUT_OVERFLOW] = nullptr;

    getWritableOutputItems(b, port, true);
    if (LLVM_UNLIKELY(beforeExpansion[WITHOUT_OVERFLOW] && (beforeExpansion[WITH_OVERFLOW] != beforeExpansion[WITHOUT_OVERFLOW]))) {
        getWritableOutputItems(b, port, false);
    }

    assert (beforeExpansion[WITH_OVERFLOW] == nullptr || (beforeExpansion[WITH_OVERFLOW] != afterExpansion[WITH_OVERFLOW]));
    assert ((beforeExpansion[WITH_OVERFLOW] != nullptr) && (afterExpansion[WITH_OVERFLOW] != nullptr));
    assert (beforeExpansion[WITHOUT_OVERFLOW] == nullptr || (beforeExpansion[WITHOUT_OVERFLOW] != afterExpansion[WITHOUT_OVERFLOW]));
    assert ((beforeExpansion[WITHOUT_OVERFLOW] == nullptr) ^ (afterExpansion[WITHOUT_OVERFLOW] != nullptr));
    assert ((beforeExpansion[WITH_OVERFLOW] != beforeExpansion[WITHOUT_OVERFLOW]) ^ (afterExpansion[WITH_OVERFLOW] == afterExpansion[WITHOUT_OVERFLOW]));

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_addr' [%" PRIx64 ",%" PRIx64 ")",
               buffer->getMallocAddress(b), buffer->getOverflowAddress(b));
    #endif

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_writable' = %" PRIu64, afterExpansion[WITH_OVERFLOW]);
    #endif

    BasicBlock * const expandBufferExit = b->GetInsertBlock();
    b->CreateBr(expanded);

    b->SetInsertPoint(expanded);

    IntegerType * const sizeTy = b->getSizeTy();
    if (afterExpansion[WITH_OVERFLOW]) {
        PHINode * const phi = b->CreatePHI(sizeTy, 2);
        phi->addIncoming(beforeExpansion[WITH_OVERFLOW], noExpansionExit);
        phi->addIncoming(afterExpansion[WITH_OVERFLOW], expandBufferExit);
        afterExpansion[WITH_OVERFLOW] = phi;
    }

    if (afterExpansion[WITHOUT_OVERFLOW]) {
        if (LLVM_LIKELY(beforeExpansion[WITH_OVERFLOW] == beforeExpansion[WITHOUT_OVERFLOW])) {
            afterExpansion[WITHOUT_OVERFLOW] = afterExpansion[WITH_OVERFLOW];
        } else {
            PHINode * const phi = b->CreatePHI(sizeTy, 2);
            phi->addIncoming(beforeExpansion[WITHOUT_OVERFLOW], noExpansionExit);
            phi->addIncoming(afterExpansion[WITHOUT_OVERFLOW], expandBufferExit);
            afterExpansion[WITHOUT_OVERFLOW] = phi;
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfWritableStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b,
                                                  const BufferPort & port,
                                                  Value * const numOfLinearStrides) {

    const auto outputPort = port.Port;
    assert (outputPort.Type == PortType::Output);
    const auto bufferVertex = getOutputBufferVertex(outputPort);
    const BufferNode & bn = mBufferGraph[bufferVertex];
    if (LLVM_UNLIKELY(bn.isUnowned())) {
        return nullptr;
    }
    const Binding & output = port.Binding;
    Value * numOfStrides = nullptr;
    if (LLVM_UNLIKELY(output.getRate().isPartialSum())) {
        numOfStrides = getMaximumNumOfPartialSumStrides(b, port, numOfLinearStrides);
    } else {
        Value * const writable = getWritableOutputItems(b, port);
        Value * const strideLength = getOutputStrideLength(b, port);
        numOfStrides = b->CreateUDiv(writable, strideLength);
    }
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelId, outputPort);
    debugPrint(b, "> " + prefix + "_numOfStrides = %" PRIu64, numOfStrides);
    #endif
    return numOfStrides;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getWritableOutputItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getWritableOutputItems(BuilderRef b, const BufferPort & port, const bool useOverflow) {

    const auto outputPort = port.Port;
    assert (outputPort.Type == PortType::Output);

    auto & W = mWritableOutputItems[outputPort.Number];
    Value * const alreadyComputed = W[useOverflow ? WITH_OVERFLOW : WITHOUT_OVERFLOW];
    if (alreadyComputed) {
        return alreadyComputed;
    }

    const auto output = getOutput(mKernelId, outputPort);
    const auto streamSet = target(output, mBufferGraph);
    const BufferNode & bn = mBufferGraph[streamSet];
    const StreamSetBuffer * const buffer = bn.Buffer;
    Value * const produced = mAlreadyProducedPhi[outputPort]; assert (produced);
    Value * const consumed = mInitialConsumedItemCount[streamSet]; assert (consumed);

    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelId, outputPort);
    debugPrint(b, prefix + "_produced = %" PRIu64, produced);
    debugPrint(b, prefix + "_consumed (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), consumed);
    #endif

    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & output = getOutputBinding(outputPort);
        Value * const sanityCheck = b->CreateICmpULE(consumed, produced);
        b->CreateAssert(sanityCheck,
                        "%s.%s: consumed count (%" PRIu64 ") exceeds produced count (%" PRIu64 ")",
                        mCurrentKernelName,
                        b->GetString(output.getName()),
                        consumed, produced);        
    }

    ConstantInt * overflow = nullptr;
    if (useOverflow && (bn.CopyBack || port.Add)) {
        const auto k = std::max<unsigned>(bn.CopyBack, port.Add);
        overflow = b->getSize(k);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, prefix + "_overflow = %" PRIu64, overflow);
        #endif
    }


    Value * const writable = buffer->getLinearlyWritableItems(b, produced, consumed, overflow);

    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_writable = (%" PRIu64 ") %" PRIu64, b->getSize(streamSet), writable);
    #endif

    // cache the values for later use
    if (useOverflow) {
        W[WITH_OVERFLOW] = writable;
    }
    if (overflow == nullptr) {
        W[WITHOUT_OVERFLOW] = writable;
    }
    return writable;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfAccessibleStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfAccessibleStrides(BuilderRef b,
                                                    const BufferPort & port,
                                                    Value * const numOfLinearStrides) {
    const auto inputPort = port.Port;
    assert (inputPort.Type == PortType::Input);
    const Binding & input = port.Binding;
    const ProcessingRate & rate = input.getRate();
    Value * numOfStrides = nullptr;
    #ifdef PRINT_DEBUG_MESSAGES
    const auto prefix = makeBufferName(mKernelId, inputPort);
    #endif
    if (LLVM_UNLIKELY(rate.isPartialSum())) {
        numOfStrides = getMaximumNumOfPartialSumStrides(b, port, numOfLinearStrides);
    } else if (LLVM_UNLIKELY(rate.isGreedy())) {
        return nullptr;
    } else {
        Value * const accessible = getAccessibleInputItems(b, port); assert (accessible);
        Value * const strideLength = getInputStrideLength(b, port); assert (strideLength);
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, "< " + prefix + "_accessible = %" PRIu64, accessible);
        debugPrint(b, "< " + prefix + "_strideLength = %" PRIu64, strideLength);
        #endif
        numOfStrides = b->CreateUDiv(subtractLookahead(b, port, accessible), strideLength);
    }
    Value * const ze = mIsInputZeroExtended[inputPort];
    if (ze) {
        numOfStrides = b->CreateSelect(ze, numOfLinearStrides, numOfStrides, "numOfZeroExtendedStrides");
    }
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, "< " + prefix + "_numOfStrides = %" PRIu64, numOfStrides);
    #endif
    return numOfStrides;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::calculateFinalItemCounts(BuilderRef b,
                                                Vec<Value *> & accessibleItems,
                                                Vec<Value *> & writableItems,
                                                Value *& minFixedRateFactor,
                                                Value *& finalStrideRemainder) {

    for (auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[e];
        assert (port.Port.Type == PortType::Input);
        Value * accessible = getAccessibleInputItems(b, port);
        const int k = port.Add - port.Truncate;
        if (LLVM_UNLIKELY(k != 0)) {
            Value * selected;
            if (LLVM_LIKELY(k > 0)) {
                selected = b->CreateAdd(accessible, b->getSize(k));
            } else  {
                selected = b->CreateSaturatingSub(accessible, b->getSize(-k));
            }
            accessible = b->CreateSelect(isClosedNormally(b, port.Port), selected, accessible, "accessible");
        }
        accessibleItems[port.Port.Number] = accessible;
    }

    Value * principalFixedRateFactor = nullptr;
    for (auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[e];
        assert (port.Port.Type == PortType::Input);
        const Binding & input = port.Binding;
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed() && LLVM_UNLIKELY(input.isPrincipal())) {
            Value * const accessible = accessibleItems[port.Port.Number];
            const auto factor = mFixedRateLCM / rate.getRate();
            principalFixedRateFactor = b->CreateMulRational(accessible, factor);
            break;
        }
    }

    for (auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[e];
        assert (port.Port.Type == PortType::Input);
        const auto inputPort = port.Port;
        Value * accessible = accessibleItems[inputPort.Number];
        if (LLVM_UNLIKELY(mIsInputZeroExtended[inputPort] != nullptr)) {
            // If this input stream is zero extended, the current input items will be MAX_INT.
            // However, since we're now in the final stride, so we can bound the stream to:
            const Binding & input = getInputBinding(inputPort);
            const ProcessingRate & rate = input.getRate();
            if (principalFixedRateFactor && rate.isFixed()) {
                const auto factor = rate.getRate() / mFixedRateLCM;
                accessible = b->CreateCeilUMulRational(principalFixedRateFactor, factor);
            } else {
                Value * maxItems = b->CreateAdd(mAlreadyProcessedPhi[inputPort], getInputStrideLength(b, port));
                // But since we may not necessarily be in our zero extension region, we must first
                // test whether we are:
                accessible = b->CreateSelect(mIsInputZeroExtended[inputPort], maxItems, accessible);
            }
        }
        accessibleItems[inputPort.Number] = accessible;
    }

    minFixedRateFactor = principalFixedRateFactor;

    if (principalFixedRateFactor == nullptr) {
        for (auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & port = mBufferGraph[e];
            assert (port.Port.Type == PortType::Input);
            const Binding & input = port.Binding;
            const ProcessingRate & rate = input.getRate();
            if (rate.isFixed()) {
                Value * const fixedRateFactor =
                    b->CreateMulRational(accessibleItems[port.Port.Number], mFixedRateLCM / rate.getRate());
                minFixedRateFactor =
                    b->CreateUMin(minFixedRateFactor, fixedRateFactor);
            }
        }
    }

//    Value * maxFixedRateFactor = minFixedRateFactor;

    if (minFixedRateFactor) {
        // truncate any fixed rate input down to the length of the shortest stream
        for (auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & port = mBufferGraph[e];
            const auto inputPort = port.Port;
            assert (inputPort.Type == PortType::Input);
            const Binding & input = port.Binding;
            const ProcessingRate & rate = input.getRate();

            if (rate.isFixed()) {
                const auto factor = rate.getRate() / mFixedRateLCM;
                Value * calculated = b->CreateCeilUMulRational(minFixedRateFactor, factor);
                const auto k = port.TransitiveAdd;

                // ... but ensure that it reflects whether it was produced with an
                // Add/Truncate attributed rate.
                if (k) {

                    const auto stride = mKernel->getStride();

                    // (x + (g/h)) * (c/d) = (xh + g) * c/hd
                    Constant * const h = b->getSize(stride);
                    Value * const xh = b->CreateMul(minFixedRateFactor, h);
                    Constant * const g = b->getSize(std::abs(k));
                    Value * y;
                    if (k > 0) {
                        y = b->CreateAdd(xh, g);
                    } else {
                        y = b->CreateSub(xh, g);
                    }

                    const Rational r{factor.numerator(), factor.denominator() * stride}; // := factor / Rational{stride};
                    Value * const z = b->CreateCeilUMulRational(y, r);
                    calculated = b->CreateSelect(isClosedNormally(b, inputPort), z, calculated);
                }

                accessibleItems[inputPort.Number] = calculated;
            }
            #ifdef PRINT_DEBUG_MESSAGES
            const auto prefix = makeBufferName(mKernelId, inputPort);
            debugPrint(b, prefix + ".accessible' = %" PRIu64, accessibleItems[inputPort.Number]);
            #endif
        }
    }

    finalStrideRemainder = nullptr;
    if (LLVM_LIKELY(mIsPartitionRoot)) {
        if (minFixedRateFactor == nullptr) {
            finalStrideRemainder = b->getSize(0);
        } else {
            finalStrideRemainder = minFixedRateFactor;
        }
        assert (finalStrideRemainder);
    }

    Constant * const sz_ONE = b->getSize(1);

    for (auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & port = mBufferGraph[e];
        const auto outputPort = port.Port;
        assert (outputPort.Type == PortType::Output);
        const Binding & output = port.Binding;
        const ProcessingRate & rate = output.getRate();

        Value * writable = nullptr;
        if (rate.isFixed() && minFixedRateFactor) {
            const auto factor = rate.getRate() / mFixedRateLCM;
            writable = b->CreateCeilUMulRational(minFixedRateFactor, factor);
        } else {
            writable = calculateNumOfLinearItems(b, port, sz_ONE);
        }

        // update the final item counts with any Add/RoundUp attributes
        for (const Attribute & attr : output.getAttributes()) {
            switch (attr.getKind()) {
                case AttrId::Add:
                    writable = b->CreateAdd(writable, b->getSize(attr.amount()));
                    break;
                case AttrId::Truncate:
                    writable = b->CreateSaturatingSub(writable, b->getSize(attr.amount()));
                    break;
                case AttrId::RoundUpTo:
                    writable = b->CreateRoundUp(writable, b->getSize(attr.amount()));
                    break;
                default: break;
            }
        }
        writableItems[outputPort.Number] = writable;
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelId, outputPort);
        debugPrint(b, prefix + ".writable' = %" PRIu64, writable);
        #endif
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getInputStrideLength(BuilderRef b, const BufferPort & inputPort) {
    if (mFirstInputStrideLength[inputPort.Port]) {
        return mFirstInputStrideLength[inputPort.Port];
    } else {
        Value * const strideLength = calculateStrideLength(b, inputPort, mAlreadyProcessedPhi[inputPort.Port], nullptr);
        mFirstInputStrideLength[inputPort.Port] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getOutputStrideLength(BuilderRef b, const BufferPort & outputPort) {
    if (mFirstOutputStrideLength[outputPort.Port]) {
        return mFirstOutputStrideLength[outputPort.Port];
    } else {
        Value * const strideLength = calculateStrideLength(b, outputPort, mAlreadyProducedPhi[outputPort.Port], nullptr);
        mFirstOutputStrideLength[outputPort.Port] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartialSumItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getPartialSumItemCount(BuilderRef b, const BufferPort & partialSumPort, Value * const previouslyTransferred, Value * const offset) const {
    const auto port = partialSumPort.Port;
    const auto ref = getReference(mKernelId, port);
    assert (ref.Type == PortType::Input);
    assert (previouslyTransferred);

    const StreamSetBuffer * const buffer = getInputBuffer(mKernelId, ref);

    Constant * const sz_ZERO = b->getSize(0);
    Value * position = mAlreadyProcessedPhi[ref];

    if (offset) {
        if (LLVM_UNLIKELY(CheckAssertions)) {
            const Binding & binding = partialSumPort.Binding;
            b->CreateAssert(b->CreateICmpNE(offset, sz_ZERO),
                            "%s.%s: partial sum offset must be non-zero",
                            mCurrentKernelName,
                            b->GetString(binding.getName()));
        }

        Constant * const sz_ONE = b->getSize(1);
        position = b->CreateAdd(position, b->CreateSub(offset, sz_ONE));
    }

    Value * const currentPtr = buffer->getRawItemPointer(b, sz_ZERO, position);
    Value * current = b->CreateLoad(currentPtr);

//    b->CreateDprintfCall(b->getInt32(STDERR_FILENO),
//                         "< pop[%" PRIu64 "] = %" PRIu64 " (0x%" PRIx64 ")\n",
//                         position, current, currentPtr);

    if (mBranchToLoopExit) {
        current = b->CreateSelect(mBranchToLoopExit, previouslyTransferred, current);
    }
    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & binding = partialSumPort.Binding;
        b->CreateAssert(b->CreateICmpULE(previouslyTransferred, current),
                        "%s.%s: partial sum is not non-decreasing at %" PRIu64
                        " (prior %" PRIu64 " > current %" PRIu64 ")",
                        mCurrentKernelName,
                        b->GetString(binding.getName()),
                        position, previouslyTransferred, current);
    }
    return b->CreateSub(current, previouslyTransferred);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPartialSumStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPartialSumStrides(BuilderRef b,
                                                           const BufferPort & partialSumPort,
                                                           Value * const numOfLinearStrides) {

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const sz_ZERO = b->getSize(0);
    Constant * const sz_ONE = b->getSize(1);
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(sizeTy);

    Value * initialItemCount = nullptr;
    Value * sourceItemCount = nullptr;
    Value * peekableItemCount = nullptr;
    Value * minimumItemCount = MAX_INT;
    Value * nonOverflowItems = nullptr;

    const auto port = partialSumPort.Port;

    const StreamSetBuffer * sourceBuffer = nullptr;

    unsigned numOfPeekableItems = 0;

    if (port.Type == PortType::Input) {
        initialItemCount = mAlreadyProcessedPhi[port];
        Value * const accessible = getAccessibleInputItems(b, partialSumPort, true);
        const auto streamSet = getInputBufferVertex(port);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyForwards) {
            sourceBuffer = bn.Buffer;
            numOfPeekableItems = bn.CopyForwards;

            nonOverflowItems = getAccessibleInputItems(b, partialSumPort, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = subtractLookahead(b, partialSumPort, b->CreateAdd(initialItemCount, accessible));
            minimumItemCount = getInputStrideLength(b, partialSumPort);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, accessible);
        }
        sourceItemCount = subtractLookahead(b, partialSumPort, sourceItemCount);
    } else { // if (port.Type == PortType::Output) {
        initialItemCount = mAlreadyProducedPhi[port];
        Value * const writable = getWritableOutputItems(b, partialSumPort, true);
        const auto streamSet = getOutputBufferVertex(port);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyBack) {
            sourceBuffer = bn.Buffer;
            numOfPeekableItems = bn.CopyBack;

            nonOverflowItems = getWritableOutputItems(b, partialSumPort, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = b->CreateAdd(initialItemCount, writable);
            minimumItemCount = getOutputStrideLength(b, partialSumPort);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, writable);
        }
    }

    const auto ref = getReference(port);
    assert (ref.Type == PortType::Input);
    const auto prefix = makeBufferName(mKernelId, ref) + "_readPartialSum";

    const auto refInput = getInput(mKernelId, ref);
    const BufferPort & refInputRate = mBufferGraph[refInput];
    const auto refBufferVertex = getInputBufferVertex(ref);
    const StreamSetBuffer * const popCountBuffer = mBufferGraph[refBufferVertex].Buffer;

    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelCheckOutputSpace);
    BasicBlock * const popCountLoopExit =
        b->CreateBasicBlock(prefix + "LoopExit", mKernelCheckOutputSpace);
    Value * const baseAddress = popCountBuffer->getRawItemPointer(b, sz_ZERO, mAlreadyProcessedPhi[ref]);
    BasicBlock * const popCountEntry = b->GetInsertBlock();

    Value * cond = b->CreateICmpNE(numOfLinearStrides, sz_ZERO);
    if (peekableItemCount) {
        cond = b->CreateAnd(cond, b->CreateICmpUGE(sourceItemCount, minimumItemCount));
    }
    b->CreateLikelyCondBr(cond, popCountLoop, popCountLoopExit);

    // TODO: replace this with a parallel icmp check and bitscan? binary search with initial
    // check on the rightmost entry?

    b->SetInsertPoint(popCountLoop);
    PHINode * const numOfStrides = b->CreatePHI(sizeTy, 2);
    numOfStrides->addIncoming(numOfLinearStrides, popCountEntry);
    PHINode * const nextRequiredItems = b->CreatePHI(sizeTy, 2);
    nextRequiredItems->addIncoming(MAX_INT, popCountEntry);

    Value * const strideIndex = b->CreateSub(numOfStrides, sz_ONE);

    Value * offset = strideIndex;

    // get the popcount kernel's input rate so we can calculate the
    // step factor for this kernel's usage of pop count partial sum
    // stream.
    const auto refOuput = in_edge(refBufferVertex, mBufferGraph);
    const BufferPort & refOutputRate = mBufferGraph[refOuput];
    const auto stepFactor = refInputRate.Maximum / refOutputRate.Maximum;

    assert (stepFactor.denominator() == 1);
    const auto step = stepFactor.numerator();
    if (LLVM_UNLIKELY(step > 1)) {
        offset = b->CreateMul(offset, b->getSize(step));
    }
    Value * const ptr = b->CreateInBoundsGEP(baseAddress, offset);
    Value * const requiredItems = b->CreateLoad(ptr);

    Value * const notEnough = b->CreateICmpUGT(requiredItems, sourceItemCount);
    Value * const notDone = b->CreateICmpNE(strideIndex, sz_ZERO);
    Value * const repeat = b->CreateAnd(notDone, notEnough);

    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & input = getInputBinding(ref);
        Value * const inputName = b->GetString(input.getName());
        b->CreateAssert(b->CreateICmpULE(initialItemCount, requiredItems),
                        "%s.%s: partial sum is not non-decreasing at %" PRIu64
                        " (prior %" PRIu64 " > current %" PRIu64 ")",
                        mCurrentKernelName, inputName,
                        strideIndex, initialItemCount, requiredItems);
    }

    nextRequiredItems->addIncoming(requiredItems, popCountLoop);
    numOfStrides->addIncoming(strideIndex, popCountLoop);
    b->CreateCondBr(repeat, popCountLoop, popCountLoopExit);

    b->SetInsertPoint(popCountLoopExit);
    PHINode * const numOfStridesPhi = b->CreatePHI(sizeTy, 2);
    numOfStridesPhi->addIncoming(sz_ZERO, popCountEntry);
    numOfStridesPhi->addIncoming(numOfStrides, popCountLoop);
    PHINode * const requiredItemsPhi = b->CreatePHI(sizeTy, 2);
    requiredItemsPhi->addIncoming(sz_ZERO, popCountEntry);
    requiredItemsPhi->addIncoming(requiredItems, popCountLoop);
    PHINode * const nextRequiredItemsPhi = b->CreatePHI(sizeTy, 2);
    nextRequiredItemsPhi->addIncoming(minimumItemCount, popCountEntry);
    nextRequiredItemsPhi->addIncoming(nextRequiredItems, popCountLoop);

    Value * finalNumOfStrides = numOfStridesPhi;
    if (peekableItemCount) {
        // Since we want to allow the stream to peek into the overflow but not start
        // in it, check to see if we can support one more stride by using it.
//        Value * const internalCapacity = sourceBuffer->getInternalCapacity(b);
//        Value * const pos = b->CreateURem(nextRequiredItemsPhi, internalCapacity);
//        ConstantInt * const overflowLimit = b->getSize(numOfPeekableItems);
//        Value * const hasOverwrittenData = b->CreateICmpUGE(nextRequiredItemsPhi, overflowLimit);
//        Value * const canPeekIntoOverflow = b->CreateAnd(hasOverwrittenData, b->CreateICmpULE(nextRequiredItemsPhi, overflowLimit));

        Value * const canPeekIntoOverflow = b->CreateICmpULE(nextRequiredItemsPhi, peekableItemCount);
        finalNumOfStrides = b->CreateAdd(finalNumOfStrides, b->CreateZExt(canPeekIntoOverflow, sizeTy));
    }
    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & binding = getInputBinding(ref);
        b->CreateAssert(b->CreateICmpNE(finalNumOfStrides, MAX_INT),
                        "%s.%s: attempting to use sentinal popcount entry",
                        mCurrentKernelName,
                        b->GetString(binding.getName()));
    }
    return finalNumOfStrides;
}


#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPartialSumStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPartialSumStrides(BuilderRef b,
                                                           const BufferPort & partialSumPort,
                                                           Value * const numOfLinearStrides) {

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const sz_ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(sizeTy);


    Value * initialItemCount = nullptr;
    Value * sourceItemCount = nullptr;
    Value * peekableItemCount = nullptr;
    Value * minimumItemCount = MAX_INT;
    Value * nonOverflowItems = nullptr;

    const auto port = partialSumPort.Port;

    if (port.Type == PortType::Input) {
        initialItemCount = mAlreadyProcessedPhi[port];
        Value * const accessible = getAccessibleInputItems(b, partialSumPort, true);
        const auto streamSet = getInputBufferVertex(port);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyForwards) {
            nonOverflowItems = getAccessibleInputItems(b, partialSumPort, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = b->CreateAdd(initialItemCount, accessible);
            minimumItemCount = getInputStrideLength(b, partialSumPort);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, accessible);
        }
        sourceItemCount = subtractLookahead(b, partialSumPort, sourceItemCount);
    } else { // if (port.Type == PortType::Output) {
        initialItemCount = mAlreadyProducedPhi[port];
        Value * const writable = getWritableOutputItems(b, partialSumPort, true);
        const auto streamSet = getOutputBufferVertex(port);
        const BufferNode & bn = mBufferGraph[streamSet];
        if (bn.CopyBack) {
            nonOverflowItems = getWritableOutputItems(b, partialSumPort, false);
            sourceItemCount = b->CreateAdd(initialItemCount, nonOverflowItems);
            peekableItemCount = b->CreateAdd(initialItemCount, writable);
            minimumItemCount = getOutputStrideLength(b, partialSumPort);
        } else {
            sourceItemCount = b->CreateAdd(initialItemCount, writable);
        }
    }

    const auto ref = getReference(port);
    assert (ref.Type == PortType::Input);
    const auto prefix = makeBufferName(mKernelId, ref) + "_readPartialSum";

    // get the popcount kernel's input rate so we can calculate the
    // step factor for this kernel's usage of pop count partial sum
    // stream.
    const auto refInput = getInput(mKernelId, ref);
    const BufferPort & refInputRate = mBufferGraph[refInput];
    const auto refBufferVertex = getInputBufferVertex(ref);
    const auto refOuput = in_edge(refBufferVertex, mBufferGraph);
    const BufferPort & refOutputRate = mBufferGraph[refOuput];
    const auto stepFactor = refInputRate.Maximum / refOutputRate.Maximum;

    assert (stepFactor.denominator() == 1);
    const auto step = stepFactor.numerator();
    Constant * const STEP = b->getSize(step);

    const StreamSetBuffer * const buffer = mBufferGraph[refBufferVertex].Buffer;
    BasicBlock * const popCountLoop =
        b->CreateBasicBlock(prefix + "Loop", mKernelCheckOutputSpace);
    BasicBlock * const popCountLoopExit =
        b->CreateBasicBlock(prefix + "LoopExit", mKernelCheckOutputSpace);
    Value * const baseOffset = mAlreadyProcessedPhi[ref];


    Value * const baseAddress = buffer->getRawItemPointer(b, sz_ZERO, baseOffset);
    BasicBlock * const popCountEntry = b->GetInsertBlock();
    Value * const initialStrideCount = b->CreateMul(numOfLinearStrides, STEP);

    Value * cond = b->CreateICmpNE(numOfLinearStrides, sz_ZERO);
    if (peekableItemCount) {
        cond = b->CreateAnd(cond, b->CreateICmpUGE(sourceItemCount, minimumItemCount));
    }
    b->CreateLikelyCondBr(cond, popCountLoop, popCountLoopExit);

    // TODO: replace this with a parallel icmp check and bitscan? binary search with initial
    // check on the rightmost entry?

    b->SetInsertPoint(popCountLoop);
    PHINode * const numOfStrides = b->CreatePHI(sizeTy, 2);
    numOfStrides->addIncoming(initialStrideCount, popCountEntry);
    PHINode * const nextRequiredItems = b->CreatePHI(sizeTy, 2);
    nextRequiredItems->addIncoming(MAX_INT, popCountEntry);

    Value * const strideIndex = b->CreateSub(numOfStrides, STEP);
    Value * const ptr = b->CreateInBoundsGEP(baseAddress, strideIndex);
    Value * const requiredItems = b->CreateLoad(ptr);
    Value * const hasEnough = b->CreateICmpULE(requiredItems, sourceItemCount);
    Value * const done = b->CreateOr(hasEnough, b->CreateICmpULE(strideIndex, STEP));

    // NOTE: popcount streams are produced with a 1 element lookbehind window.
    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & input = getInputBinding(ref);
        Value * const inputName = b->GetString(input.getName());
        b->CreateAssert(b->CreateOr(b->CreateICmpUGE(numOfStrides, STEP), hasEnough),
                        "%s.%s: attempting to read invalid popcount entry",
                        mCurrentKernelName, inputName);
        b->CreateAssert(b->CreateICmpULE(initialItemCount, requiredItems),
                        "%s.%s: partial sum is not non-decreasing at %" PRIu64
                        " (prior %" PRIu64 " > current %" PRIu64 ")",
                        mCurrentKernelName, inputName,
                        strideIndex, initialItemCount, requiredItems);
    }

    nextRequiredItems->addIncoming(requiredItems, popCountLoop);
    numOfStrides->addIncoming(strideIndex, popCountLoop);
    b->CreateCondBr(done, popCountLoopExit, popCountLoop);

    b->SetInsertPoint(popCountLoopExit);
    PHINode * const numOfStridesPhi = b->CreatePHI(sizeTy, 2);
    numOfStridesPhi->addIncoming(sz_ZERO, popCountEntry);
    numOfStridesPhi->addIncoming(numOfStrides, popCountLoop);
    PHINode * const requiredItemsPhi = b->CreatePHI(sizeTy, 2);
    requiredItemsPhi->addIncoming(sz_ZERO, popCountEntry);
    requiredItemsPhi->addIncoming(requiredItems, popCountLoop);
    PHINode * const nextRequiredItemsPhi = b->CreatePHI(sizeTy, 2);
    nextRequiredItemsPhi->addIncoming(minimumItemCount, popCountEntry);
    nextRequiredItemsPhi->addIncoming(nextRequiredItems, popCountLoop);
    Value * finalNumOfStrides = numOfStridesPhi;
    if (peekableItemCount) {
        // Since we want to allow the stream to peek into the overflow but not start
        // in it, check to see if we can support one more stride by using it.
        Value * const endedPriorToBufferEnd = b->CreateICmpNE(requiredItemsPhi, sourceItemCount);
        Value * const canPeekIntoOverflow = b->CreateICmpULE(nextRequiredItemsPhi, peekableItemCount);
        Value * const useOverflow = b->CreateAnd(endedPriorToBufferEnd, canPeekIntoOverflow);
        Value * const plusOne = b->CreateAdd(finalNumOfStrides, ONE);
        finalNumOfStrides = b->CreateSelect(useOverflow, plusOne, finalNumOfStrides);
    }
    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & binding = getInputBinding(ref);
        b->CreateAssert(b->CreateICmpNE(finalNumOfStrides, MAX_INT),
                        "%s.%s: attempting to use sentinal popcount entry",
                        mCurrentKernelName,
                        b->GetString(binding.getName()));
    }
    return finalNumOfStrides;
}

#endif



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateStrideLength(BuilderRef b, const BufferPort & port, Value * const previouslyTransferred, Value * const strideIndex) {
    const Binding & binding = port.Binding;
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        const auto baseStrideLength = rate.getUpperBound() * mKernel->getStride();
        assert (baseStrideLength.denominator() == 1);
        ConstantInt * baseStrideVal = b->getSize(baseStrideLength.numerator());
        if (strideIndex == nullptr) {
            return baseStrideVal;
        } else {
            return b->CreateMul(strideIndex, baseStrideVal);
        }
    } else if (rate.isGreedy()) {
        assert ("kernel cannot have a greedy output rate" && port.Port.Type != PortType::Output);
        return b->getSize(ceiling(rate.getLowerBound()));
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port, previouslyTransferred, strideIndex);
    } else if (rate.isRelative()) {
        const auto refPort = getReference(port.Port);
        const auto refInput = getInput(mKernelId, refPort);
        const BufferPort & ref = mBufferGraph[refInput];
        Value * const baseRate = calculateStrideLength(b, ref, previouslyTransferred, strideIndex);
        return b->CreateMulRational(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const BufferPort & port, Value * const linearStrides) {
    assert (linearStrides);
    const Binding & binding = port.Binding;
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMulRational(linearStrides, rate.getUpperBound() * mKernel->getStride());
    } else if (rate.isGreedy()) {
        return getAccessibleInputItems(b, port);
    } else if (rate.isPartialSum()) {
        Value * priorItemCount = nullptr;
        if (LLVM_LIKELY(port.Port.Type == PortType::Input)) {
            priorItemCount = mAlreadyProcessedPhi[port.Port];
        } else {
            priorItemCount = mAlreadyProducedPhi[port.Port];
        }
        return getPartialSumItemCount(b, port, priorItemCount, linearStrides);
    } else if (rate.isRelative()) {
        auto getRefPort = [&] () {
            const auto refPort = getReference(port.Port);
            if (LLVM_LIKELY(refPort.Type == PortType::Input)) {
                return getInput(mKernelId, refPort);
            } else {
                return getOutput(mKernelId, refPort);
            }
        };
        const BufferPort & ref = mBufferGraph[getRefPort()];
        Value * const baseCount = calculateNumOfLinearItems(b, ref, linearStrides);
        return b->CreateMulRational(baseCount, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief updatePHINodesForLoopExit
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::updatePHINodesForLoopExit(BuilderRef b) {

    BasicBlock * const exitBlock = b->GetInsertBlock();
    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mUpdatedProcessedPhi[port]->addIncoming(mAlreadyProcessedPhi[port], exitBlock);
        if (mUpdatedProcessedDeferredPhi[port]) {
            mUpdatedProcessedDeferredPhi[port]->addIncoming(mAlreadyProcessedDeferredPhi[port], exitBlock);
        }
    }
    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const auto port = mBufferGraph[e].Port;
        mUpdatedProducedPhi[port]->addIncoming(mAlreadyProducedPhi[port], exitBlock);
        if (mUpdatedProducedDeferredPhi[port]) {
            mUpdatedProducedDeferredPhi[port]->addIncoming(mAlreadyProducedDeferredPhi[port], exitBlock);
        }
    }

}

}

#endif // IO_CALCULATION_LOGIC_HPP
