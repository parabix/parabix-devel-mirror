#ifndef SYNCHRONIZATION_LOGIC_HPP
#define SYNCHRONIZATION_LOGIC_HPP

#include "pipeline_compiler.hpp"

// Suppose T1 and T2 are two pipeline threads where all segment processing
// of kernel Ki in T1 logically happens before Ki in T2.

// Any stateless kernel (or kernel marked as internally synchronized) with
// countable input rates that is not a source, sink or side-effecting can
// be executed in parallel once we've calculated the "future" item count
// position(s). However, T2 may finish before T1 and a Kj>i in T2 could
// attempt to consume unfinished data from T1. So we must ensure that T1
// is always completely finished before T2 may execute Kj.

// For any such kernel, we require two counters. The first marks that T1
// has computed T2's initial processed item counts. The second informs T2
// when T1 has finished writing to its output streams. T2 may begin p
// rocessing once it acquires the first lock but cannot write its output
// until acquiring the second.

// If each stride of output of Ki cannot be guaranteed to be written on
// a cache-aligned boundary, regardless of input state, a temporary output
// buffer is required. After acquiring the second lock, the data
// must be copied to the shared stream set. To minimize redundant copies,
// if at the point of executing Ki,
// we require an additional lock that indicates whether some kernel "owns"
// the actual stream set.

// Even though T1 and T2 process a segment per call, a segment may require
// several iterations (due to buffer contraints, final stride processing,
// etc.) Thus to execute a stateful internally synchronized kernel, we must
// hold both buffer locks until reaching the last partial segment.

// TODO: Fix cycle counter and serialize option for nested pipelines

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireCurrentSegment
 *
 * Before the segment is processed, this loads the segment number of the kernel state and ensures the previous
 * segment is complete (by checking that the acquired segment number is equal to the desired segment number).
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::acquireSynchronizationLock(BuilderRef b, const LockType lockType, const CycleCounter start) {
    if (mNumOfThreads > 1 || ExternallySynchronized) {
        const auto prefix = makeKernelName(mKernelId);
        const auto serialize = codegen::DebugOptionIsSet(codegen::SerializeThreads);
        const unsigned waitingOnIdx = serialize ? LastKernel : mKernelId;
        const auto waitingOn = makeKernelName(waitingOnIdx);

        auto getLockName = [&]() -> const std::string & {
            switch (lockType) {
                case LockType::ItemCheck: return ITEM_COUNT_READ_GUARD_SUFFIX;
                case LockType::Segment: return LOGICAL_SEGMENT_SUFFIX;
            }
            llvm_unreachable("unknown lock type!");
        };

        const auto suffix = getLockName();
        Value * const waitingOnPtr = getScalarFieldPtr(b.get(), waitingOn + suffix);

//        #ifdef PRINT_DEBUG_MESSAGES
//        debugPrint(b, "* waiting " + prefix + suffix + " = %" PRIu64 " (PTR) %" PRIu64, mSegNo, waitingOnPtr);
//        debugPrint(b, "* init " + prefix + suffix + " = %" PRIu64, b->CreateLoad(waitingOnPtr));
//        #endif

        BasicBlock * const nextNode = b->GetInsertBlock()->getNextNode();
        BasicBlock * const acquired = b->CreateBasicBlock(prefix + "_acquired" + suffix, nextNode);
        BasicBlock * const acquire = b->CreateBasicBlock(prefix + "_acquire" + suffix, acquired);
        b->CreateBr(acquire);

        b->SetInsertPoint(acquire);
        Value * const currentSegNo = b->CreateAtomicLoadAcquire(waitingOnPtr);
        if (LLVM_UNLIKELY(mCheckAssertions)) {
            Value * const pendingOrReady = b->CreateICmpULE(currentSegNo, mSegNo);

            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "%s: ";
            switch (lockType) {
                case LockType::ItemCheck:
                    out << "item check guard";
                    break;
                case LockType::Segment:
                    out << "logical segment number";
                    break;
            }
            out << " is %" PRIu64 " but was expected to be [0,%" PRIu64 "]";
            b->CreateAssert(pendingOrReady, out.str(), mKernelAssertionName, currentSegNo, mSegNo);
        }
        Value * const ready = b->CreateICmpEQ(mSegNo, currentSegNo);
        b->CreateLikelyCondBr(ready, acquired, acquire);

        b->SetInsertPoint(acquired);
        updateCycleCounter(b, start, CycleCounter::AFTER_SYNCHRONIZATION);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 *
 * After executing the kernel, the segment number must be incremented to release the kernel for the next thread.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::releaseSynchronizationLock(BuilderRef b, const LockType lockType) {
    if (mNumOfThreads > 1 || ExternallySynchronized) {
        Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
        const auto prefix = makeKernelName(mKernelId);

        auto getLockName = [&]() -> const std::string & {
            switch (lockType) {
                case LockType::ItemCheck: return ITEM_COUNT_READ_GUARD_SUFFIX;
                case LockType::Segment: return LOGICAL_SEGMENT_SUFFIX;
            }
            llvm_unreachable("unknown lock type!");
        };

        const auto suffix = getLockName();

        Value * const waitingOnPtr = getScalarFieldPtr(b.get(), prefix + suffix);
        if (LLVM_UNLIKELY(mCheckAssertions)) {
            Value * const currentSegNo = b->CreateAtomicLoadAcquire(waitingOnPtr);
            Value * const unchanged = b->CreateICmpEQ(mSegNo, currentSegNo);

            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "%s: ";
            switch (lockType) {
                case LockType::ItemCheck:
                    out << "item check guard";
                    break;
                case LockType::Segment:
                    out << "logical segment number";
                    break;
            }
            out << " is %" PRIu64 " but was expected to be %" PRIu64;
            b->CreateAssert(unchanged, out.str(), mKernelAssertionName, currentSegNo, mSegNo);
        }
        b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);
//        #ifdef PRINT_DEBUG_MESSAGES
//        debugPrint(b, "* released " + prefix + suffix + " %" PRIu64 " (PTR) %" PRIu64, nextSegNo, waitingOnPtr);
//        #endif
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifySynchronizationLock
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifySynchronizationLock(BuilderRef b, const LockType lockType) {
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        if (mNumOfThreads > 1 || ExternallySynchronized) {
            const auto waitingOn = makeKernelName(mKernelId);

            auto getLockName = [&]() -> const std::string & {
                switch (lockType) {
                    case LockType::ItemCheck: return ITEM_COUNT_READ_GUARD_SUFFIX;
                    case LockType::Segment: return LOGICAL_SEGMENT_SUFFIX;
                }
                llvm_unreachable("unknown lock type!");
            };

            const auto suffix = getLockName();
            Value * const waitingOnPtr = getScalarFieldPtr(b.get(), waitingOn + suffix);
            Value * const currentSegNo = b->CreateAtomicLoadAcquire(waitingOnPtr);

            Value * const writtenOrPassed = b->CreateICmpUGT(currentSegNo, mSegNo);

            SmallVector<char, 256> tmp;
            raw_svector_ostream out(tmp);
            out << "%s: ";
            switch (lockType) {
                case LockType::ItemCheck:
                    out << "item check guard";
                    break;
                case LockType::Segment:
                    out << "logical segment number";
                    break;
            }
            out << " is %" PRIu64 " but was expected to be greater than %" PRIu64;
            b->CreateAssert(writtenOrPassed, out.str(), mKernelAssertionName, currentSegNo, mSegNo);
        }
    }
}


}

#endif // SYNCHRONIZATION_LOGIC_HPP
