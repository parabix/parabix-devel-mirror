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

    // When testing partition inputs, we have a few hard guarantees: the first node in the partition is the only
    // kernel that may have a bounded *input* rate that cannot be transitively guaranteed to be satisfied by some
    // dominating kernel. We can avoid testing those inputs here. All other inputs must be countable. Since every
    // kernel in the partition will perform N * C_i strides (where C_i is a constant factor for the i-th kernel),
    // we can trivially calculate the Fixed rates by maintaining a next partition relative stride counter.
    // PartialSum rates, however, must be counted independently.

    // All inputs for the partition must be tested after acquiring the first partition node's lock. No other
    // kernel in the partition is required to test its input. Each kernel, however, is responsible for
    // assessing whether it has sufficient output space for the given input. Although this too could be determined
    // upon entering the partition, other threads may still be actively consuming data from the produced streams;
    // thus we may end up needlessly expanding buffers by performing the test too early.


    const auto eval = determinePartitionInputEvaluationOrder(mCurrentPartitionIndex);

    // ACQUIRE NEXT PARTITION SEG#



    for (const PartitionInput & input : eval) {

        Value * const accessible = mLocallyAvailableItems[input.StreamSet];





    }


}


}

#endif
