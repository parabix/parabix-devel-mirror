#ifndef PARTITION_PROCESSING_LOGIC_HPP
#define PARTITION_PROCESSING_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

//const static std::string PARTITION_LOGICAL_SEGMENT_NUMBER = ".PLS";
//const static std::string PARTITION_ITEM_COUNT_SUFFIX = ".PIC";
//const static std::string PARTITION_FIXED_RATE_SUFFIX = ".PFR";

inline std::string getPartitionSegmentNumberName(const size_t partitionId) {
    return std::to_string(partitionId) + PARTITION_LOGICAL_SEGMENT_NUMBER;
}

inline std::string getPartitionFixedRateName(const size_t partitionId) {
    return std::to_string(partitionId) + PARTITION_FIXED_RATE_SUFFIX;
}

inline std::string getPartitionPortName(const size_t partitionId, const PartitioningGraphEdge & e) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << partitionId << '.'
        << e.Kernel <<
        ((e.Port.Type == PortType::Input) ? 'I' : 'O') <<
        e.Port.Number <<
        PARTITION_ITEM_COUNT_SUFFIX;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPartitionItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addPartitionInputItemCounts(BuilderRef b, const size_t partitionId) const {
    IntegerType * const int64Ty = b->getInt64Ty();
    mTarget->addInternalScalar(int64Ty, getPartitionSegmentNumberName(partitionId));

    bool hasFixedRate = false;
    for (unsigned i = FirstKernel, partitionSize = 0; i <= LastKernel; ++i) {
        if (KernelPartitionId[i] == partitionId) {
            ++partitionSize;
            if (partitionSize > 1) {
                hasFixedRate = true;
                break;
            }
        }
    }

    for (const auto e : make_iterator_range(out_edges(partitionId, mPartitioningGraph))) {
        const auto v = target(e, mPartitioningGraph);
        const PartitioningGraphNode & node = mPartitioningGraph[v];
        if (node.Type == PartitioningGraphNode::Fixed) {
            hasFixedRate = true;
            break;
        }
    }

    if (LLVM_LIKELY(hasFixedRate)) {
        mTarget->addInternalScalar(int64Ty, getPartitionFixedRateName(partitionId));
    }
    for (const auto e : make_iterator_range(in_edges(partitionId, mPartitioningGraph))) {
        mTarget->addInternalScalar(int64Ty, getPartitionPortName(partitionId, mPartitioningGraph[e]));
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionEntryPoints
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::makePartitionEntryPoints(BuilderRef b) {    
    mPipelineLoop = b->CreateBasicBlock("PipelineLoop");
    mPartitionEntryPoint.resize(PartitionCount + 1);
    for (unsigned i = 0; i <= PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock("Partition" + std::to_string(i));
    }
    mPipelineEnd = b->CreateBasicBlock("PipelineEnd");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief branchToInitialPartition
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::branchToInitialPartition(BuilderRef b) {
    assert (KernelPartitionId[FirstKernel] == 0);
    BasicBlock * const entry = mPartitionEntryPoint[0];
    b->CreateBr(entry);
    b->SetInsertPoint(entry);
    mCurrentPartitionId = -1U;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getPartitionExitPoint
 ** ------------------------------------------------------------------------------------------------------------- */
inline BasicBlock * PipelineCompiler::getPartitionExitPoint(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto partitionId = KernelPartitionId[mKernelId];
    return mPartitionEntryPoint[partitionId + 1];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkInputDataOnPartitionEntry
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool PipelineCompiler::isPartitionRoot() {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto partitionId = KernelPartitionId[mKernelId];
    if (partitionId == mCurrentPartitionId) {
        return false;
    }
    mPartitionRootKernelId = mKernelId;
    mCurrentPartitionId = partitionId;
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief checkForPartitionExit
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::checkForPartitionExit(BuilderRef b) {
    assert (mKernelId >= FirstKernel && mKernelId <= LastKernel);
    const auto nextPartitionId = KernelPartitionId[mKernelId + 1];
    if (nextPartitionId != mCurrentPartitionId) {
        BasicBlock * nextPartition;
        if (LLVM_UNLIKELY(nextPartitionId == -1U)) {
            nextPartition = mPartitionEntryPoint[PartitionCount];
        } else {
            assert (nextPartitionId < PartitionCount);
            nextPartition = mPartitionEntryPoint[nextPartitionId];
        }
        b->CreateBr(nextPartition);
        b->SetInsertPoint(nextPartition);
    }
}

}

#endif
