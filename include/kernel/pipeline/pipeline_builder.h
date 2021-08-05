#ifndef PIPELINE_BUILDER_H
#define PIPELINE_BUILDER_H

#include <kernel/pipeline/pipeline_kernel.h>

class BaseDriver;

namespace kernel {

class OptimizationBranchBuilder;

class PipelineBuilder {
    friend class PipelineKernel;
    friend class OptimizationBranchBuilder;
public:

    using Kernels = PipelineKernel::Kernels;
    using CallBinding = PipelineKernel::CallBinding;
    using CallBindings = PipelineKernel::CallBindings;
    using NestedBuilders = std::vector<std::shared_ptr<PipelineBuilder>>;
    using LengthAssertion = PipelineKernel::LengthAssertion;
    using LengthAssertions = PipelineKernel::LengthAssertions;

    BaseDriver & getDriver() { return mDriver;}

    template<typename KernelType, typename... Args>
    Kernel * CreateKernelCall(Args &&... args) {
        return initializeKernel(new KernelType(mDriver.getBuilder(), std::forward<Args>(args) ...));
    }

    Kernel * AddKernelCall(Kernel * kernel) {
        return initializeKernel(kernel);
    }

    std::shared_ptr<OptimizationBranchBuilder>
        CreateOptimizationBranch(Relationship * const condition,
                                 Bindings && stream_inputs = {}, Bindings && stream_outputs = {},
                                 Bindings && scalar_inputs = {}, Bindings && scalar_outputs = {});

    StreamSet * CreateStreamSet(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return mDriver.CreateStreamSet(NumElements, FieldWidth);
    }

    Scalar * CreateConstant(llvm::Constant * value) {
        return mDriver.CreateConstant(value);
    }

    template <typename ExternalFunctionType>
    void CreateCall(llvm::StringRef name, ExternalFunctionType & functionPtr, std::initializer_list<Scalar *> args) {
        llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(mDriver.getContext());
        assert ("FunctionTypeBuilder did not resolve a function type." && type);
        assert ("Function was not provided the correct number of args" && type->getNumParams() == args.size());
        // Since the pipeline kernel module has not been made yet, just record the function info and its arguments.
        mCallBindings.emplace_back(name, type, reinterpret_cast<void *>(&functionPtr), std::move(args));
    }

    Scalar * getInputScalar(const unsigned i) {
        return llvm::cast<Scalar>(mInputScalars[i].getRelationship());
    }

    Scalar * getInputScalar(const llvm::StringRef name);

    void setInputScalar(const llvm::StringRef name, Scalar * value);

    Scalar * getOutputScalar(const unsigned i) {
        return llvm::cast<Scalar>(mOutputScalars[i].getRelationship());
    }

    Scalar * getOutputScalar(const llvm::StringRef name);

    void setOutputScalar(const llvm::StringRef name, Scalar * value);

    void AssertEqualLength(const StreamSet * A, const StreamSet * B) {
        mLengthAssertions.emplace_back(LengthAssertion{{A, B}});
    }

    PipelineBuilder(BaseDriver & driver,
                    Bindings && stream_inputs, Bindings && stream_outputs,
                    Bindings && scalar_inputs, Bindings && scalar_outputs,
                    const unsigned numOfThreads);

    virtual ~PipelineBuilder() {}

    virtual Kernel * makeKernel();

    void setExternallySynchronized(const bool value = true) {
        mExternallySynchronized = value;
    }

protected:


    // Internal pipeline constructor uses a zero-length tag struct to prevent
    // overloading errors. This parameter will be dropped by the compiler.
    struct Internal {};
    PipelineBuilder(Internal, BaseDriver & driver,
                    Bindings stream_inputs, Bindings stream_outputs,
                    Bindings scalar_inputs, Bindings scalar_outputs);

    Kernel * initializeKernel(Kernel * const kernel);

protected:

    BaseDriver &        mDriver;
    // eventual pipeline configuration
    unsigned            mNumOfThreads;
    bool                mExternallySynchronized = false;
    Bindings            mInputStreamSets;
    Bindings            mOutputStreamSets;
    Bindings            mInputScalars;
    Bindings            mOutputScalars;
    Bindings            mInternalScalars;
    Kernels             mKernels;
    CallBindings        mCallBindings;
    NestedBuilders      mNestedBuilders;
    LengthAssertions    mLengthAssertions;
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief ProgramBuilder
 ** ------------------------------------------------------------------------------------------------------------- */
class ProgramBuilder : public PipelineBuilder {
    friend class PipelineBuilder;
public:

    void * compile();

    void setNumOfThreads(const unsigned threads) {
        mNumOfThreads = threads;
    }

    ProgramBuilder(BaseDriver & driver,
                   Bindings && stream_inputs, Bindings && stream_outputs,
                   Bindings && scalar_inputs, Bindings && scalar_outputs);

private:

    void * compileKernel(Kernel * const kernel);
};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief PipelineBranchBuilder
 ** ------------------------------------------------------------------------------------------------------------- */
class OptimizationBranchBuilder final : public PipelineBuilder {
    friend class PipelineKernel;
    friend class PipelineBuilder;
public:

    const std::unique_ptr<PipelineBuilder> & getNonZeroBranch() const {
        return mNonZeroBranch;
    }

    const std::unique_ptr<PipelineBuilder> & getAllZeroBranch() const {
        return mAllZeroBranch;
    }

    ~OptimizationBranchBuilder();

protected:

    OptimizationBranchBuilder(BaseDriver & driver, Relationship * const condition,
                              Bindings && stream_inputs, Bindings && stream_outputs,
                              Bindings && scalar_inputs, Bindings && scalar_outputs);

    Kernel * makeKernel() override;

private:
    Relationship * const             mCondition;
    std::unique_ptr<PipelineBuilder> mNonZeroBranch;
    std::unique_ptr<PipelineBuilder> mAllZeroBranch;
};

inline std::shared_ptr<OptimizationBranchBuilder> PipelineBuilder::CreateOptimizationBranch (
        Relationship * const condition,
        Bindings && stream_inputs, Bindings && stream_outputs,
        Bindings && scalar_inputs, Bindings && scalar_outputs) {
    std::shared_ptr<OptimizationBranchBuilder> branch(
        new OptimizationBranchBuilder(mDriver, condition,
            std::move(stream_inputs), std::move(stream_outputs),
            std::move(scalar_inputs), std::move(scalar_outputs)));
    mNestedBuilders.emplace_back(std::static_pointer_cast<PipelineBuilder>(branch));
    return branch;
}

}

#endif // PIPELINE_BUILDER_H
