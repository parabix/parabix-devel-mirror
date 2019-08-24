/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <kernel/core/kernel.h>
#include <kernel/pipeline/pipeline_builder.h>

namespace kernel {

/**
 * Kernel driving `AssertEQ` and `AssertNE` functions.
 * 
* Should NOT be used directly. Use either `AssertEQ` or `AssertNE` instead.
 */
class StreamEquivalenceKernel : public MultiBlockKernel {
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
public:
    enum class Mode { EQ, NE };

    StreamEquivalenceKernel(BuilderRef b, Mode mode, StreamSet * x, StreamSet * y, Scalar * outPtr);
    void generateInitializeMethod(BuilderRef b) override;
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
    void generateFinalizeMethod(BuilderRef b) override;
    bool hasSignature() const override { return false; }
    bool isCachable() const override { return true; }
private:
    Mode mMode;
};

}

namespace testing {

/**
 * Compares two `StreamsSets` to see if they are equal setting the test case's 
 * state to `failing` if they are not.
 * 
 * Preconditions:
 *  - `lhs` and `rhs` must have the same field width
 *  - `lhs` and `rhs` must have the same number of elements
 */
void AssertEQ(const std::unique_ptr<kernel::ProgramBuilder> & P, kernel::StreamSet * lhs, kernel::StreamSet * rhs);

/**
 * Compares two `StreamsSets` to see if they are equal setting the test case's 
 * state to `failing` if they are.
 * 
 * Preconditions:
 *  - `lhs` and `rhs` must have the same field width
 *  - `lhs` and `rhs` must have the same number of elements
 */
void AssertNE(const std::unique_ptr<kernel::ProgramBuilder> & P, kernel::StreamSet * lhs, kernel::StreamSet * rhs);

}
