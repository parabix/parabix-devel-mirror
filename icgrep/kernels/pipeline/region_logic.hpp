#ifndef REGION_LOGIC_HPP
#define REGION_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

void PipelineCompiler::enterRegionSpan(BuilderRef b) {
/*
    mKernelRegionEntryLoop = nullptr;
    mKernelRegionExitLoopCheck = nullptr;

    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & br = mBufferGraph[e];
        if (LLVM_UNLIKELY(br.inputPort() == IMPLICIT_REGION_SELECTOR)) {
            const auto prefix = makeKernelName(mKernelIndex);
            BasicBlock * const regionEntry = b->CreateBasicBlock(prefix + "_regionEntry", mKernelLoopCall);
            BasicBlock * const regionCheck = b->CreateBasicBlock(prefix + "_regionCheck", mKernelLoopCall);
            BasicBlock * const regionExit = b->CreateBasicBlock(prefix + "_regionExit", mKernelTerminationCheck);







            mKernelRegionEntryLoop = regionEntry;
            mKernelRegionExitLoopCheck = regionExit;
        }
    }

*/
}

void PipelineCompiler::exitRegionSpan(BuilderRef b) {


}


}

#endif // REGION_LOGIC_HPP
