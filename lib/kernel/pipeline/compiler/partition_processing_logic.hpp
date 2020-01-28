#ifndef PARTITION_PROCESSING_LOGIC_HPP
#define PARTITION_PROCESSING_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePartitionEntryPoints
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::makePartitionEntryPoints(BuilderRef b) {

    // TODO: each partition will have a partition seg# to check against.

    // Make one basic block for each partition and one that will branch into
    // the final exit loop condition.
    mPartitionEntryPoint.resize(PartitionCount + 1);
    for (unsigned i = 0; i <= PartitionCount; ++i) {
        mPartitionEntryPoint[i] = b->CreateBasicBlock();
    }
    mCurrentPartitionIndex = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief testPartitionInputData
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::testPartitionInputData(BuilderRef b) {

    const auto eval = determinePartitionInputEvaluationOrder(mCurrentPartitionIndex);

    // ACQUIRE NEXT PARTITION SEG#



    for (const PartitionInput & input : eval) {

        Value * const accessible = mLocallyAvailableItems[input.StreamSet];





    }


}


}

#endif
