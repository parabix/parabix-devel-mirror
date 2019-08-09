/*
 * Copyright (c) 2019 International Characters.
 * This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include <memory>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/relationship.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <kernel/pipeline/driver/cpudriver.h>

namespace testing {

using kernel::StreamSet;
using kernel::Scalar;

using UnitTestFunc = int32_t(*)(const void *, size_t, const void *, size_t);

class TestEngine {
public:
    using PipelineRef = const std::unique_ptr<kernel::ProgramBuilder> &;

    TestEngine()
    : mDriver("unit-test-framework")
    , mPipelineBuilder(nullptr)
    {}

    TestEngine(const TestEngine &) = delete;
    TestEngine(TestEngine &&) = delete;

    CPUDriver & driver() noexcept { return mDriver; }

    PipelineRef pipeline() noexcept {
        assert(mPipelineBuilder != nullptr);
        return mPipelineBuilder;
    }

    UnitTestFunc compile() {
        return reinterpret_cast<UnitTestFunc>(pipeline()->compile());
    }

    void setReturnValue(Scalar * val) {
        pipeline()->setOutputScalar("result", val);
    }

    void setReturnValue(uint32_t i) {
        pipeline()->setOutputScalar("result", driver().CreateConstant(driver().getBuilder()->getInt32(i)));
    }

    // Implicit cast to `ProgramBuilder` reference.
    operator PipelineRef () {
        return pipeline();
    }

    // Access `ProgramBuilder`'s methods via `->` operator.
    PipelineRef operator -> () {
        return pipeline();
    }

    // Constructs a new pipeline instance.
    void makePipeline(kernel::Bindings inputBindings) {
        using namespace kernel;
        assert(mPipelineBuilder == nullptr);
        auto const & b = mDriver.getBuilder();
        mPipelineBuilder = mDriver.makePipeline(
            std::move(inputBindings),
            { Binding{b->getInt32Ty(), "result"} }
        );
    }

private:
    CPUDriver                               mDriver;
    std::unique_ptr<kernel::ProgramBuilder> mPipelineBuilder;
};

}
