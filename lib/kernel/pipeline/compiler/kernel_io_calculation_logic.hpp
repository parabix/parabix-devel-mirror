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
    const auto & max = MaximumNumOfStrides[mKernelId];
    if (mIsPartitionRoot) {
        if (ExternallySynchronized) {
            mMaximumNumOfStrides = nullptr;
        } else {
            mMaximumNumOfStrides = b->getSize(ceiling(max));
        }
    } else {
        const auto factor = (max / MaxPartitionStrideRate);
        assert (mNumOfPartitionStrides);
        mMaximumNumOfStrides = b->CreateMulRational(mNumOfPartitionStrides, factor);
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

    assert (mKernelIsFinal == nullptr);

    Value * numOfLinearStrides = nullptr;

    if (mMayLoopToEntry && !ExternallySynchronized) {
        numOfLinearStrides = b->CreateSub(mMaximumNumOfStrides, mCurrentNumOfStridesAtLoopEntryPhi);
    } else {
        numOfLinearStrides = mMaximumNumOfStrides;
    }

    SmallVector<unsigned, 8> testedPortIds;

    auto unchecked = [&](const unsigned portId) {
        for (const auto checked : testedPortIds) {
            if (checked == portId) {
                return false;
            }
        }
        testedPortIds.push_back(portId);
        return true;
    };

    // If two streams have the same global id then they are guaranteed to have an equivalent
    // number of items at the start and end of each kernel. When two streams share a local id,
    // they will have the exact same state within the kernel processing them.

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const auto streamSet = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[streamSet];
        const BufferPort & br = mBufferGraph[e];

        assert (bn.isInternal() || (bn.isNonThreadLocal() && mCheckIO));

        #ifdef CHECK_EVERY_IO_PORT
        const auto check = true;
        #else
        const auto check = mCheckIO && bn.isNonThreadLocal() && unchecked(br.LocalPortId);
        #endif

        if (check) {
            checkForSufficientInputData(b, br, streamSet);
        } else { // ensure the accessible input count dominates all uses
            getAccessibleInputItems(b, br);
        }
    }

    Value * numOfActualInputStrides = numOfLinearStrides;

    const auto prefix = makeKernelName(mKernelId);

     if (mCheckIO || CheckAssertions) {

        Value * numOfInputStrides = numOfLinearStrides;

        testedPortIds.clear();

        for (const auto input : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const auto streamSet = source(input, mBufferGraph);
            const BufferPort & br = mBufferGraph[input];
            const BufferNode & bn = mBufferGraph[streamSet];

            #ifdef CHECK_EVERY_IO_PORT
            const auto check = true;
            #else
            const auto check = (bn.isNonThreadLocal() || bn.NonLinear) && unchecked(br.LocalPortId);
            #endif

            if (LLVM_LIKELY(check)) {
                Value * const strides = getNumOfAccessibleStrides(b, br, numOfInputStrides);
                numOfInputStrides = b->CreateUMin(numOfInputStrides, strides);
            }
            if (LLVM_UNLIKELY(CheckAssertions)) {
                Value * const strides = getNumOfAccessibleStrides(b, br, numOfActualInputStrides);
                numOfActualInputStrides = b->CreateUMin(numOfActualInputStrides, strides);
            }
        }

        Value * numOfOutputStrides = nullptr;

        testedPortIds.clear();

        ConstantInt * const ZERO = b->getSize(0);
        for (const auto output : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
            const auto streamSet = target(output, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            const BufferPort & br = mBufferGraph[output];


            #ifdef CHECK_EVERY_IO_PORT
            const auto check = true;
            #else
            const auto check = (bn.isNonThreadLocal() && bn.NonLinear) && unchecked(br.LocalPortId);
            #endif

            if (LLVM_LIKELY(check)) {
                if (numOfOutputStrides == nullptr) {
                    ConstantInt * const ONE = b->getSize(1);
                    numOfOutputStrides = b->CreateUMax(numOfInputStrides, ONE);
                }
                Value * const strides = getNumOfWritableStrides(b, br, numOfOutputStrides);
                if (strides) {
                    Value * const minStrides = b->CreateUMin(numOfOutputStrides, strides);
                    Value * const isZero = b->CreateICmpEQ(strides, ZERO);
                    numOfOutputStrides = b->CreateSelect(isZero, numOfOutputStrides, minStrides, "numOfOutputStrides");
                }
            }
        }


        if (numOfOutputStrides) {
            numOfLinearStrides = b->CreateUMin(numOfInputStrides, numOfOutputStrides);
        } else {
            numOfLinearStrides = numOfInputStrides;
        }

    }

    mNumOfLinearStrides = numOfLinearStrides;

    determineIsFinal(b);
    calculateItemCounts(b);

    for (const auto output : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        ensureSufficientOutputSpace(b, mBufferGraph[output], target(output, mBufferGraph));
    }

    if (CheckAssertions) {

       Value * numOfActualOutputStrides = nullptr;

       ConstantInt * const ZERO = b->getSize(0);
       for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
           const BufferPort & br = mBufferGraph[e];
           if (numOfActualOutputStrides == nullptr) {
               ConstantInt * const ONE = b->getSize(1);
               numOfActualOutputStrides = b->CreateUMax(numOfActualInputStrides, ONE);
           }
           Value * const strides = getNumOfWritableStrides(b, br, numOfActualOutputStrides);
           if (strides) {
               Value * const minStrides = b->CreateUMin(numOfActualOutputStrides, strides);
               Value * const isZero = b->CreateICmpEQ(strides, ZERO);
               numOfActualOutputStrides = b->CreateSelect(isZero, numOfActualOutputStrides, minStrides, "numOfActualOutputStrides");
           }
       }

       Value * numOfActualLinearStrides = numOfActualInputStrides;
       if (numOfActualOutputStrides) {
           numOfActualLinearStrides = b->CreateUMin(numOfActualInputStrides, numOfActualOutputStrides);
       }
       Value * const match = b->CreateICmpEQ(numOfLinearStrides, numOfActualLinearStrides);
       b->CreateAssert(match, "%s was expected to execute %" PRIu64 " strides but can only execute %" PRIu64,
                       mCurrentKernelName, numOfLinearStrides, numOfActualLinearStrides);
    }

    if (mMayLoopToEntry) {
        mUpdatedNumOfStrides = b->CreateAdd(mCurrentNumOfStridesAtLoopEntryPhi, numOfLinearStrides);
    } else {
        mUpdatedNumOfStrides = numOfLinearStrides;
    }

    assert (mIsFinal);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::calculateItemCounts(BuilderRef b) {

    const auto numOfInputs = in_degree(mKernelId, mBufferGraph);
    const auto numOfOutputs = out_degree(mKernelId, mBufferGraph);

    // --- lambda function start
    auto phiOutItemCounts = [&](const Vec<Value *> & accessibleItems,
                               const Vec<Value *> & inputVirtualBaseAddress,
                               const Vec<Value *> & writableItems,
                               Value * const fixedRateFactor,
                               Constant * const terminationSignal,
                               Value * const partialPartitionStrides) {
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
        if (mIsPartitionRoot) {
            mPartialPartitionStridesPhi->addIncoming(partialPartitionStrides, exitBlock);
        }
    };
    // --- lambda function end


    Vec<Value *> accessibleItems(numOfInputs);

    Vec<Value *> inputVirtualBaseAddress(numOfInputs, nullptr);

    Vec<Value *> writableItems(numOfOutputs);

    Constant * const unterminated = getTerminationSignal(b, TerminationSignal::None);

    getInputVirtualBaseAddresses(b, inputVirtualBaseAddress);

//    assert ("unbounded zero extended input?" && ((mHasZeroExtendedInput == nullptr) || mIsBounded));

    assert (mKernelIsFinal);

    if (LLVM_LIKELY(in_degree(mKernelId, mBufferGraph) > 0)) {

        const auto prefix = makeKernelName(mKernelId);
        BasicBlock * const enteringNonFinalSegment = b->CreateBasicBlock(prefix + "_nonFinalSegment", mKernelCheckOutputSpace);
        BasicBlock * const enteringFinalStride = b->CreateBasicBlock(prefix + "_finalStride", mKernelCheckOutputSpace);

        Vec<Value *> zeroExtendedInputVirtualBaseAddress(numOfInputs, nullptr);

        BasicBlock * nonZeroExtendExit = nullptr;
        BasicBlock * zeroExtendExit = nullptr;

        /// -------------------------------------------------------------------------------------
        /// HANDLE ZERO EXTENSION
        /// -------------------------------------------------------------------------------------

        if (mHasZeroExtendedInput) {
            BasicBlock * const checkFinal =
                b->CreateBasicBlock(prefix + "_checkFinal", enteringNonFinalSegment);
            Value * const isFinalOrZeroExtended = b->CreateOr(mHasZeroExtendedInput, mKernelIsFinal);

            #ifdef PRINT_DEBUG_MESSAGES
            debugPrint(b, "* " + prefix + "_isFinal = %" PRIu64, mKernelIsFinal);
            #endif

            nonZeroExtendExit = b->GetInsertBlock();
            b->CreateUnlikelyCondBr(isFinalOrZeroExtended, checkFinal, enteringNonFinalSegment);

            b->SetInsertPoint(checkFinal);
            Value * const zeroExtendSpace = allocateLocalZeroExtensionSpace(b, enteringNonFinalSegment);
            getZeroExtendedInputVirtualBaseAddresses(b, inputVirtualBaseAddress, zeroExtendSpace, zeroExtendedInputVirtualBaseAddress);
            zeroExtendExit = b->GetInsertBlock();
            b->CreateCondBr(mKernelIsFinal, enteringFinalStride, enteringNonFinalSegment);

        } else {
            b->CreateUnlikelyCondBr(mKernelIsFinal, enteringFinalStride, enteringNonFinalSegment);
        }

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING FINAL STRIDE
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringFinalStride);
        Value * fixedItemFactor = nullptr;
        Value * partialPartitionStride = nullptr;
        calculateFinalItemCounts(b, accessibleItems, writableItems, fixedItemFactor, partialPartitionStride);
        // if we have a potentially zero-extended buffer, use that; otherwise select the normal buffer
        Vec<Value *> truncatedVirtualBaseAddress(numOfInputs);
        for (unsigned i = 0; i != numOfInputs; ++i) {
            Value * const ze = zeroExtendedInputVirtualBaseAddress[i];
            Value * const vba = inputVirtualBaseAddress[i];
            truncatedVirtualBaseAddress[i] = ze ? ze : vba;
        }
        zeroInputAfterFinalItemCount(b, accessibleItems, truncatedVirtualBaseAddress);
        Constant * const completed = getTerminationSignal(b, TerminationSignal::Completed);
        phiOutItemCounts(accessibleItems, truncatedVirtualBaseAddress, writableItems,
                         fixedItemFactor, completed, partialPartitionStride);
        b->CreateBr(mKernelCheckOutputSpace);

        /// -------------------------------------------------------------------------------------
        /// KERNEL ENTERING NON-FINAL SEGMENT
        /// -------------------------------------------------------------------------------------

        b->SetInsertPoint(enteringNonFinalSegment);
        if (mHasZeroExtendedInput) {
            for (unsigned i = 0; i != numOfInputs; ++i) {
                Value * const ze = zeroExtendedInputVirtualBaseAddress[i];
                if (ze) {
                    PHINode * const phi = b->CreatePHI(ze->getType(), 2);
                    phi->addIncoming(inputVirtualBaseAddress[i], nonZeroExtendExit);
                    phi->addIncoming(ze, zeroExtendExit);
                    inputVirtualBaseAddress[i] = phi;
                }
            }
        }
    }

    /// -------------------------------------------------------------------------------------
    /// KERNEL CALCULATE NON-FINAL INPUT COUNT
    /// -------------------------------------------------------------------------------------

    assert (mNumOfLinearStrides);
    Value * fixedRateFactor = nullptr;
    if (mFixedRateFactorPhi) {
        const Rational stride(mKernel->getStride());
        fixedRateFactor  = b->CreateMulRational(mNumOfLinearStrides, stride * mFixedRateLCM);
    }

    for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        accessibleItems[br.Port.Number] = calculateNumOfLinearItems(b, br, mNumOfLinearStrides);
    }

    for (const auto e : make_iterator_range(out_edges(mKernelId, mBufferGraph))) {
        const BufferPort & br = mBufferGraph[e];
        writableItems[br.Port.Number] = calculateNumOfLinearItems(b, br, mNumOfLinearStrides);
    }

    ConstantInt * const ZERO = b->getSize(0);

    phiOutItemCounts(accessibleItems, inputVirtualBaseAddress, writableItems,
                     fixedRateFactor, unterminated, ZERO);

    b->CreateBr(mKernelCheckOutputSpace);

    b->SetInsertPoint(mKernelCheckOutputSpace);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForSufficientInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::checkForSufficientInputData(BuilderRef b, const BufferPort & port, const unsigned streamSet) {

    const auto inputPort = port.Port;
    assert (inputPort.Type == PortType::Input);

    Value * const strideLength = getInputStrideLength(b, port);
    Value * const segmentLength = b->CreateMulRational(strideLength, MinimumNumOfStrides[mKernelId]);
    Value * const required = addLookahead(b, port, segmentLength); assert (required);
    const auto prefix = makeBufferName(mKernelId, inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_requiredInput (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), required);
    #endif

    Value * const accessible = getAccessibleInputItems(b, port); assert (accessible);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_accessible (%" PRIu64 ") = %" PRIu8, b->getSize(streamSet), accessible);
    #endif

    Value * const hasEnough = b->CreateICmpUGE(accessible, required);
    Value * const closed = isClosed(b, inputPort);
    #ifdef PRINT_DEBUG_MESSAGES
    debugPrint(b, prefix + "_closed = %" PRIu8, closed);
    #endif

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
 * @brief anyInputClosed
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::anyInputClosed(BuilderRef b) {
    if (LLVM_UNLIKELY(in_degree(mKernelId, mBufferGraph) == 0)) {
        if (ExternallySynchronized) {
            return b->isFinal();
        } else {
            return b->getFalse();
        }
    }
    if (mIsPartitionRoot) {
        Value * anyClosed = nullptr;
        for (const auto e : make_iterator_range(in_edges(mKernelId, mBufferGraph))) {
            const BufferPort & br =  mBufferGraph[e];
            if (LLVM_UNLIKELY(br.IsZeroExtended)) {
                continue;
            }
            const auto streamSet = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];
            Value * const closed = isClosed(b, br.Port); assert (closed);
            Value * fullyConsumed = closed;
            if (bn.NonLinear) {
                Value * const processed = mAlreadyProcessedPhi[br.Port];
                Value * const accessible = getAccessibleInputItems(b, br);
                Value * const total = b->CreateAdd(processed, accessible);
                Value * const avail = getLocallyAvailableItemCount(b, br.Port);
                Value * const fullyReadable = b->CreateICmpEQ(total, avail);
                fullyConsumed = b->CreateAnd(closed, fullyReadable);
            }
            if (anyClosed) {
                anyClosed = b->CreateOr(anyClosed, closed);
            } else {
                anyClosed = fullyConsumed;
            }
        }
        assert (anyClosed && "non-zero-extended stream is required");
        return anyClosed;
    }
    Value * const signal = getCurrentTerminationSignal(); assert (signal);
    return b->CreateIsNotNull(signal);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineIsFinal
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determineIsFinal(BuilderRef b) {
    if (mKernelIsFinal == nullptr) {
        Value * const anyClosed = anyInputClosed(b); assert (anyClosed);
        if (mMayLoopToEntry) {
            ConstantInt * const sz_ZERO = b->getSize(0);
            Value * const noMoreStrides = b->CreateICmpEQ(mNumOfLinearStrides, sz_ZERO);
            mKernelIsFinal = b->CreateAnd(anyClosed, noMoreStrides);
            Value * const hasMoreStrides = b->CreateICmpNE(mNumOfLinearStrides, sz_ZERO);
            mKernelIsPenultimate = b->CreateAnd(anyClosed, hasMoreStrides);
        } else {
            mKernelIsFinal = anyClosed;            
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasMoreInput
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::hasMoreInput(BuilderRef b) {
    assert (mMayLoopToEntry);
    assert (mKernelIsPenultimate);

    ConstantInt * const i1_TRUE = b->getTrue();
    Value * notAtSegmentLimit = i1_TRUE;

    b->CallPrintInt("mKernelIsPenultimate", mKernelIsPenultimate);

    if (mMaximumNumOfStrides) {
        assert (mUpdatedNumOfStrides);

        b->CallPrintInt("mUpdatedNumOfStrides", mUpdatedNumOfStrides);
        b->CallPrintInt("mMaximumNumOfStrides", mMaximumNumOfStrides);

        notAtSegmentLimit = b->CreateICmpNE(mUpdatedNumOfStrides, mMaximumNumOfStrides);

        b->CallPrintInt("notAtSegmentLimit", notAtSegmentLimit);
    }


    if (mIsPartitionRoot) {

        BasicBlock * const lastTestExit = b->CreateBasicBlock("", mKernelLoopExit);
        PHINode * const enoughInputPhi = PHINode::Create(b->getInt1Ty(), 4, "", lastTestExit);

        bool firstTest = true;

        graph_traits<BufferGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(mKernelId, mBufferGraph);

        Value * enoughInput = b->CreateNot(mKernelIsFinal);

        ConstantInt * const amount = b->getSize(1); // ceiling(PartitionStrideFactor));
        Value * const nextStrideIndex = b->CreateAdd(mNumOfLinearStrides, amount);

        while (ei != ei_end) {
            const auto e = *ei++;
            const BufferPort & br =  mBufferGraph[e];
            if (LLVM_UNLIKELY(br.IsZeroExtended)) {
                continue;
            }
            const auto streamSet = source(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[streamSet];

            if (LLVM_UNLIKELY(bn.Locality != BufferLocality::ThreadLocal)) {
                Value * const processed =  mProcessedItemCount[br.Port];
                Value * avail = getLocallyAvailableItemCount(b, br.Port);

                Value * const closed = isClosed(b, br.Port);

                if (br.Add) {
                    Constant * const ZERO = b->getSize(0);
                    Constant * const ADD = b->getSize(br.Add);
                    Value * const added = b->CreateSelect(closed, ADD, ZERO);
                    avail = b->CreateAdd(avail, added);
                }

                Value * const remaining = b->CreateSub(avail, processed);
                Value * const required = calculateNumOfLinearItems(b, br, nextStrideIndex);
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
                    const auto streamSet = source(e, mBufferGraph);
                    const BufferNode & bn = mBufferGraph[streamSet];

                    if (LLVM_UNLIKELY(bn.Locality != BufferLocality::ThreadLocal)) {
                        const Binding & binding = next.Binding;
                        const ProcessingRate & rate = binding.getRate();
                        useBranch = rate.isPartialSum();
                    }
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
        return b->CreateOr(mKernelIsPenultimate, notAtSegmentLimit);
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
//        if (bn.CopyBackReflection || rateData.LookAhead || rateData.Add) {
//            const auto A = rateData.Add; //std::max(bn.CopyBackReflection, rateData.Add); //
//            const auto B = std::max(bn.CopyBackReflection, rateData.LookAhead);
//            if (A == B) {
//                overflow = b->getSize(A);
//            } else {
//                Value * const closed = isClosed(b, inputPort);
//                overflow = b->CreateSelect(closed, b->getSize(A), b->getSize(B));
//            }
//            #ifdef PRINT_DEBUG_MESSAGES
//            debugPrint(b, prefix + "_overflow (add:%" PRIu64 ",la:%" PRIu64 ") = %" PRIu64,
//                                    b->getSize(A),
//                                    b->getSize(B),
//                                    overflow);
//            #endif
//        }
    }



    Value * accessible = buffer->getLinearlyAccessibleItems(b, processed, available, overflow);

//    if (LLVM_UNLIKELY(CheckAssertions)) {
//        Value * intCapacity = buffer->getInternalCapacity(b);
//        if (overflow) {
//            intCapacity = b->CreateAdd(intCapacity, overflow);
//        }
//        Value * const ok = b->CreateICmpULE(accessible, intCapacity);
//        const Binding & input = getInputBinding(inputPort);
//        b->CreateAssert(ok, "%s.%s reported %" PRIu64 " accessible items but only has capacity for %" PRIu64,
//                        mCurrentKernelName, b->GetString(input.getName()), accessible, intCapacity);
//    }

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

    const auto outputPort = port.Port;
    assert (outputPort.Type == PortType::Output);

    const BufferNode & bn = mBufferGraph[streamSet];

    #ifdef PERMIT_BUFFER_MEMORY_REUSE
    if (bn.Locality == BufferLocality::ThreadLocal) {
        return;
    }
    #endif

    if (LLVM_UNLIKELY(bn.isOwned())) {

        const auto prefix = makeBufferName(mKernelId, outputPort);
        const StreamSetBuffer * const buffer = bn.Buffer;

        getWritableOutputItems(b, port, true);

        Value * const required = mLinearOutputItemsPhi[outputPort];
        #ifdef PRINT_DEBUG_MESSAGES
        debugPrint(b, prefix + "_required (%" PRIu64 ") = %" PRIu64, b->getSize(streamSet), required);
        #endif

        BasicBlock * const expandBuffer = b->CreateBasicBlock(prefix + "_expandBuffer", mKernelCheckOutputSpace);
        BasicBlock * const expanded = b->CreateBasicBlock(prefix + "_expandedBuffer", mKernelCheckOutputSpace);
        const auto beforeExpansion = mWritableOutputItems[outputPort.Number];

        Value * const hasEnoughSpace = b->CreateICmpULE(required, beforeExpansion[WITH_OVERFLOW]);

        BasicBlock * const noExpansionExit = b->GetInsertBlock();
        b->CreateLikelyCondBr(hasEnoughSpace, expanded, expandBuffer);

        b->SetInsertPoint(expandBuffer);
        Value * const cycleCounterAccumulator = getBufferExpansionCycleCounter(b);
        Value * cycleCounterStart = nullptr;
        if (cycleCounterAccumulator) {
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
        if (cycleCounterAccumulator) {
            Value * const cycleCounterEnd = b->CreateReadCycleCounter();
            Value * const duration = b->CreateSub(cycleCounterEnd, cycleCounterStart);
            Value * const accum = b->CreateAdd(b->CreateLoad(cycleCounterAccumulator), duration);
            b->CreateStore(accum, cycleCounterAccumulator);
        }

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

    const BufferPort & rateData = mBufferGraph[output];

    ConstantInt * overflow = nullptr;
    if (useOverflow && (bn.CopyBack || rateData.Add)) {
        const auto k = std::max<unsigned>(bn.CopyBack, rateData.Add);
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
 * @brief getNumOfWritableStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getNumOfWritableStrides(BuilderRef b,
                                                  const BufferPort & port,
                                                  Value * const numOfLinearStrides) {

    const auto outputPort = port.Port;
    assert (outputPort.Type == PortType::Output);
    const auto bufferVertex = getOutputBufferVertex(outputPort);
    const BufferNode & bn = mBufferGraph[bufferVertex];
    #ifdef PERMIT_BUFFER_MEMORY_REUSE
    if (bn.Locality == BufferLocality::ThreadLocal) {
        return nullptr;
    }
    #endif
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
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::calculateFinalItemCounts(BuilderRef b,
                                                Vec<Value *> & accessibleItems,
                                                Vec<Value *> & writableItems,
                                                Value *& minFixedRateFactor,
                                                Value *& partialPartitionStrides) {

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
                    const Rational r = factor / Rational{stride};
                    Value * const z = b->CreateCeilUMulRational(y, r);
                    calculated = b->CreateSelect(isClosedNormally(b, inputPort), z, calculated);

//                    Value * const outputFactor = b->CreateCeilUDivRate(calculated, factor);
//                    maxFixedRateFactor = b->CreateUMax(maxFixedRateFactor, outputFactor);
                }

//                if (LLVM_UNLIKELY(mCheckAssertions)) {
//                    Value * const accessible = accessibleItems[i];
//                    Value * correctItemCount = b->CreateICmpULE(calculated, accessible);
//                    Value * const zeroExtended = mIsInputZeroExtended(mKernelId, inputPort);
//                    if (LLVM_UNLIKELY(zeroExtended != nullptr)) {
//                        correctItemCount = b->CreateOr(correctItemCount, zeroExtended);
//                    }
//                    b->CreateAssert(correctItemCount,
//                                    "%s.%s: final calculated rate item count (%" PRIu64 ") "
//                                    "exceeds accessible item count (%" PRIu64 ")",
//                                    mKernelAssertionName,
//                                    b->GetString(input.getName()),
//                                    calculated, accessible);
//                }

                accessibleItems[inputPort.Number] = calculated;
            }
            #ifdef PRINT_DEBUG_MESSAGES
            const auto prefix = makeBufferName(mKernelId, inputPort);
            debugPrint(b, prefix + ".accessible' = %" PRIu64, accessibleItems[inputPort.Number]);
            #endif
        }
    }

    partialPartitionStrides = nullptr;
    if (LLVM_LIKELY(mIsPartitionRoot)) {
        const auto scale = MaxPartitionStrideRate / MaximumNumOfStrides[mKernelId];
        if (minFixedRateFactor == nullptr || (scale.numerator() == 1 && scale.denominator() == 1)) {
            partialPartitionStrides = b->getSize(0);
        } else {
            const auto factor = scale / (mFixedRateLCM * mKernel->getStride());
            partialPartitionStrides = b->CreateMulRational(minFixedRateFactor, factor);
        }
        assert (partialPartitionStrides);
    }

    Constant * const ONE = b->getSize(1);

//    Value * numOfOutputStrides = nullptr;
//    if (minFixedRateFactor) {
//        const auto factor = Rational{mKernel->getStride()} * mFixedRateLCM;
//        numOfOutputStrides = b->CreateCeilUDivRate(minFixedRateFactor, factor);
//    } else {
//        numOfOutputStrides = b->getSize(1);
//    }

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
            writable = calculateNumOfLinearItems(b, port, ONE);
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
        Value * const strideLength = calculateStrideLength(b, inputPort);
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
        Value * const strideLength = calculateStrideLength(b, outputPort);
        mFirstOutputStrideLength[outputPort.Port] = strideLength;
        return strideLength;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartialSumItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getPartialSumItemCount(BuilderRef b, const BufferPort & partialSumPort, Value * const offset) const {
    const auto port = partialSumPort.Port;
    const auto ref = getReference(mKernelId, port);
    assert (ref.Type == PortType::Input);

    const StreamSetBuffer * const buffer = getInputBuffer(mKernelId, ref);
    Value * prior = nullptr;    
    if (port.Type == PortType::Input) {
        prior = mAlreadyProcessedPhi[port];
    } else { // if (port.Type == PortType::Output) {
        prior = mAlreadyProducedPhi[port];
    }
    assert (prior);

    Constant * const ZERO = b->getSize(0);
    Value * position = mAlreadyProcessedPhi[ref];

    if (offset) {
        if (LLVM_UNLIKELY(CheckAssertions)) {
            const Binding & binding = partialSumPort.Binding;
            b->CreateAssert(b->CreateICmpNE(offset, ZERO),
                            "%s.%s: partial sum offset must be non-zero",
                            mCurrentKernelName,
                            b->GetString(binding.getName()));
        }
        Constant * const ONE = b->getSize(1);
        position = b->CreateAdd(position, b->CreateSub(offset, ONE));
    }

    Value * const currentPtr = buffer->getRawItemPointer(b, ZERO, position);
    Value * current = b->CreateLoad(currentPtr);

//    b->CreateDprintfCall(b->getInt32(STDERR_FILENO),
//                         "< pop[%" PRIu64 "] = %" PRIu64 " (0x%" PRIx64 ")\n",
//                         position, current, currentPtr);

    if (mBranchToLoopExit) {
        current = b->CreateSelect(mBranchToLoopExit, prior, current);
    }
    if (LLVM_UNLIKELY(CheckAssertions)) {
        const Binding & binding = partialSumPort.Binding;
        b->CreateAssert(b->CreateICmpULE(prior, current),
                        "%s.%s: partial sum is not non-decreasing at %" PRIu64
                        " (prior %" PRIu64 " > current %" PRIu64 ")",
                        mCurrentKernelName,
                        b->GetString(binding.getName()),
                        position, prior, current);
    }
    return b->CreateSub(current, prior);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getMaximumNumOfPartialSumStrides
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getMaximumNumOfPartialSumStrides(BuilderRef b,
                                                           const BufferPort & partialSumPort,
                                                           Value * const numOfLinearStrides) {

    assert (numOfLinearStrides);

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const MAX_INT = ConstantInt::getAllOnesValue(sizeTy);


    Value * initialItemCount = nullptr;
    Value * sourceItemCount = nullptr;
    Value * peekableItemCount = nullptr;
    Value * minimumItemCount = MAX_INT;

    const auto port = partialSumPort.Port;

    if (port.Type == PortType::Input) {
        initialItemCount = mAlreadyProcessedPhi[port];
        Value * const accessible = getAccessibleInputItems(b, partialSumPort);
        const auto input = getInput(mKernelId, port);
        const BufferPort & br = mBufferGraph[input];
        if (br.LookAhead) {
            Value * const nonOverflowItems = getAccessibleInputItems(b, partialSumPort, false);
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
            Value * const nonOverflowItems = getWritableOutputItems(b, partialSumPort, false);
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
    Value * const baseAddress = buffer->getRawItemPointer(b, ZERO, baseOffset);
    BasicBlock * const popCountEntry = b->GetInsertBlock();
    Value * const initialStrideCount = b->CreateMul(numOfLinearStrides, STEP);

    if (peekableItemCount) {
        Value * const hasNonOverflowStride = b->CreateICmpUGE(sourceItemCount, minimumItemCount);
        b->CreateLikelyCondBr(hasNonOverflowStride, popCountLoop, popCountLoopExit);
    } else {
        b->CreateBr(popCountLoop);
    }

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
    b->CreateCondBr(hasEnough, popCountLoopExit, popCountLoop);

    b->SetInsertPoint(popCountLoopExit);
    Value * finalNumOfStrides = numOfStrides;
    if (peekableItemCount) {
        PHINode * const numOfStridesPhi = b->CreatePHI(sizeTy, 2);
        numOfStridesPhi->addIncoming(ZERO, popCountEntry);
        numOfStridesPhi->addIncoming(numOfStrides, popCountLoop);
        PHINode * const requiredItemsPhi = b->CreatePHI(sizeTy, 2);
        requiredItemsPhi->addIncoming(ZERO, popCountEntry);
        requiredItemsPhi->addIncoming(requiredItems, popCountLoop);
        PHINode * const nextRequiredItemsPhi = b->CreatePHI(sizeTy, 2);
        nextRequiredItemsPhi->addIncoming(minimumItemCount, popCountEntry);
        nextRequiredItemsPhi->addIncoming(nextRequiredItems, popCountLoop);
        // Since we want to allow the stream to peek into the overflow but not start
        // in it, check to see if we can support one more stride by using it.
        Value * const endedPriorToBufferEnd = b->CreateICmpNE(requiredItemsPhi, sourceItemCount);
        Value * const canPeekIntoOverflow = b->CreateICmpULE(nextRequiredItemsPhi, peekableItemCount);
        Value * const useOverflow = b->CreateAnd(endedPriorToBufferEnd, canPeekIntoOverflow);
        finalNumOfStrides = b->CreateSelect(useOverflow, b->CreateAdd(numOfStridesPhi, ONE), numOfStridesPhi);
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFirstStrideLength
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateStrideLength(BuilderRef b, const BufferPort & port) {
    const Binding & binding = port.Binding;
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        const Rational ub = rate.getUpperBound() * mKernel->getStride();
        if (LLVM_UNLIKELY(ub.denominator() != 1)) {
            std::string tmp;
            raw_string_ostream out(tmp);
            out << mKernel->getName() << "." << binding.getName()
                << ": rate upper-bound is not a multiple of kernel stride.";
            report_fatal_error(out.str());
        }
        return b->getSize(ub.numerator());
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port, nullptr);
    } else if (rate.isGreedy()) {
        assert ("kernel cannot have a greedy output rate" && port.Port.Type != PortType::Output);
        const Rational lb = rate.getLowerBound();
        const auto ilb = floor(lb);
        return b->getSize(ilb);
    } else if (rate.isRelative()) {
        const auto refPort = getReference(port.Port);
        const auto refInput = getInput(mKernelId, refPort);
        const BufferPort & ref = mBufferGraph[refInput];
        Value * const baseRate = calculateStrideLength(b, ref);
        return b->CreateMulRational(baseRate, rate.getRate());
    }
    llvm_unreachable("unexpected rate type");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateNumOfLinearItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::calculateNumOfLinearItems(BuilderRef b, const BufferPort & port, Value * const linearStrides) {
    const Binding & binding = port.Binding;
    const ProcessingRate & rate = binding.getRate();
    if (rate.isFixed() || rate.isBounded()) {
        return b->CreateMulRational(linearStrides, rate.getUpperBound() * mKernel->getStride());
    } else if (rate.isGreedy()) {
        return getAccessibleInputItems(b, port);
    } else if (rate.isPartialSum()) {
        return getPartialSumItemCount(b, port, linearStrides);
    } else if (rate.isRelative()) {
        const auto refPort = getReference(port.Port);
        const auto refInput = getInput(mKernelId, refPort);
        const BufferPort & ref = mBufferGraph[refInput];
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
