/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <testing/common.h>
#include <kernel/core/kernel.h>

namespace kernel {

class StreamEquivalenceKernel : public MultiBlockKernel {
    using BuilderRef = const std::unique_ptr<KernelBuilder> &;
public:
    StreamEquivalenceKernel(BuilderRef b, StreamSet * x, StreamSet * y, Scalar * carry);
    void generateInitializeMethod(BuilderRef b) override;
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
    bool hasSignature() const override { return false; }
    bool isCachable() const override { return false; }
};

}

namespace testing {

Scalar * AssertEQ(TestEngine & T, StreamSet * lhs, StreamSet * rhs);

}
