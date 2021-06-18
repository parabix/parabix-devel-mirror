/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef UNTIL_N_H
#define UNTIL_N_H

#include <kernel/core/kernel.h>

namespace kernel {

class UntilNkernel final : public MultiBlockKernel {
public:
    UntilNkernel(BuilderRef b, Scalar * maxCount, StreamSet * AllMatches, StreamSet * Matches);
private:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) final;

};

}
#endif
