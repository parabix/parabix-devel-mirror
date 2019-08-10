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

using UnitTestFunc = void(*)(const void *, size_t, const void *, size_t, int32_t *);

/**
 * `TestEngine` is an all-in-one `CPUDriver` and `ProgramBuilder` used to
 * construct kernel-test pipelines. Its primary purpose is to serve as a
 * `ProgramBuilder` constructing kernel calls. As such it is implicitly castable
 * to a `const std::unique_ptr<kernel::ProgramBuilder> &` and implements the
 * `->` operator to provide access to `ProgramBuilder`'s methods.
 * 
 * Like a regular `ProgramBuilder`, `TestEngine` can construct stream sets
 * via `T->CreateStreamSet(...)` and create kernel calls via
 * `T->CreateKernelCall<...>(...)`.
 * 
 * The `driver()` method provides access to its internal `CPUDriver` if needed,
 * and the `builder()` method provides access to the driver's `KernelBuilder`.
 * 
 * Since `TestEngine` provides an implicit cast to a `ProgramBuilder` reference,
 * it can be directly passed to functions which accept a `ProgramBuilder`.
 * 
 *  Example:
 *  
 *      TestEngine & T = ...;
 *      //                           can directly pass TestEngine as a
 *      //                         v const std::unique_ptr<ProgramBuilder &
 *      kernel::util::DebugDisplay(T, "test", SomeStreamSet);
 */
class TestEngine {
public:
    using PipelineRef = const std::unique_ptr<kernel::ProgramBuilder> &;
    using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

    TestEngine()
    : mDriver("unit-test-framework")
    , mPipelineBuilder(nullptr)
    {}

    // `TestEngine` may not be copied or moved, must always be passed by reference.
    TestEngine(const TestEngine &) = delete;
    TestEngine(TestEngine &&) = delete;

    /**
     * Returns a reference to the engine's internal `CPUDriver`.
     */
    CPUDriver & driver() noexcept { return mDriver; }

    /**
     * Returns a reference to the engine's internal `ProgramBuilder`.
     * 
     * Preconditions:
     *  - `makePipeline(...)` must be called to initialize the internal program
     *    builder before this method may be called.
     */
    PipelineRef pipeline() noexcept {
        assert(mPipelineBuilder != nullptr);
        return mPipelineBuilder;
    }

    /**
     * Returns a reference to the internal driver's `KernelBuilder`.
     */
    BuilderRef builder() noexcept { return mDriver.getBuilder(); }

    /**
     * Compiles the pipeline and casts the result to the correct function type
     * before returning it.
     */
    UnitTestFunc compile() {
        return reinterpret_cast<UnitTestFunc>(pipeline()->compile());
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
        inputBindings.push_back({b->getInt32Ty()->getPointerTo(), "__ptr_out"});
        mPipelineBuilder = mDriver.makePipeline(
            std::move(inputBindings),
            {}
        );
    }

private:
    CPUDriver                               mDriver;
    std::unique_ptr<kernel::ProgramBuilder> mPipelineBuilder;
};

}
