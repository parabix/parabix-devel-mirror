#ifndef PIPELINE_BUILDER_H
#define PIPELINE_BUILDER_H

#include <kernels/pipeline_kernel.h>

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

    template<typename KernelType, typename... Args>
    Kernel * CreateKernelCall(Args &&... args) {
        return initializeKernel(new KernelType(mDriver.getBuilder(), std::forward<Args>(args) ...));
    }

    std::shared_ptr<OptimizationBranchBuilder>
        CreateOptimizationBranch(StreamSet * const condition,
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

    Scalar * getInputScalar(const std::string & name);

    void setInputScalar(const std::string & name, Scalar * value);

    Scalar * getOutputScalar(const unsigned i) {
        return llvm::cast<Scalar>(mOutputScalars[i].getRelationship());
    }

    Scalar * getOutputScalar(const std::string & name);

    void setOutputScalar(const std::string & name, Scalar * value);

    PipelineBuilder(BaseDriver & driver,
                    Bindings && stream_inputs, Bindings && stream_outputs,
                    Bindings && scalar_inputs, Bindings && scalar_outputs,
                    const unsigned numOfThreads = 1);
    
    virtual ~PipelineBuilder() {}

protected:

    virtual Kernel * makeKernel();

    Kernel * initializeKernel(Kernel * const kernel);

    void addInputScalar(llvm::Type * type, std::string name);

    llvm::Function * addOrDeclareMainFunction(PipelineKernel * const pk);

protected:

    BaseDriver &        mDriver;
    // eventual pipeline configuration
    unsigned            mNumOfThreads;
    Bindings            mInputStreamSets;
    Bindings            mOutputStreamSets;
    Bindings            mInputScalars;
    Bindings            mOutputScalars;
    Bindings            mInternalScalars;
    Kernels             mKernels;
    CallBindings        mCallBindings;
    NestedBuilders      mNestedBuilders;
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

};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief PipelineBranchBuilder
 ** ------------------------------------------------------------------------------------------------------------- */
class OptimizationBranchBuilder final : public PipelineBuilder {
    friend class PipelineKernel;
    friend class PipelineBuilder;
public:

    const std::unique_ptr<PipelineBuilder> & getTrueBranch() const {
        return mTrueBranch;
    }

    const std::unique_ptr<PipelineBuilder> & getFalseBranch() const {
        return mFalseBranch;
    }

    ~OptimizationBranchBuilder();

protected:

    OptimizationBranchBuilder(BaseDriver & driver, StreamSet * const condition,
                              Bindings && stream_inputs, Bindings && stream_outputs,
                              Bindings && scalar_inputs, Bindings && scalar_outputs);

    Kernel * makeKernel() override;

private:
    StreamSet * const                mCondition;
    std::unique_ptr<PipelineBuilder> mTrueBranch;
    std::unique_ptr<PipelineBuilder> mFalseBranch;
};

inline std::shared_ptr<OptimizationBranchBuilder> PipelineBuilder::CreateOptimizationBranch (
        StreamSet * const condition,
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
