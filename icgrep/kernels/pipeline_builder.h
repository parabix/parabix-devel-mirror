#ifndef PIPELINE_BUILDER_H
#define PIPELINE_BUILDER_H

#include <kernels/pipeline_kernel.h>

class BaseDriver;

namespace kernel {

class PipelineBuilder {
public:

    using Kernels = PipelineKernel::Kernels;
    using CallBinding = PipelineKernel::CallBinding;
    using CallBindings = PipelineKernel::CallBindings;

    template<typename KernelType, typename... Args>
    Kernel * CreateKernelCall(Args &&... args) {
        return initializeKernel(new KernelType(mDriver.getBuilder(), std::forward<Args>(args) ...));
    }

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

    void * compile();

    void setNumOfThreads(const unsigned threads) {
        mNumOfThreads = threads;
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

protected:

    PipelineKernel * makePipelineKernel();

    Kernel * initializeKernel(Kernel * const kernel);

    void addInputScalar(llvm::Type * type, std::string name);

    llvm::Function * addOrDeclareMainFunction(PipelineKernel * const pk);

private:

    BaseDriver &       mDriver;

    // eventual pipeline configuration
    unsigned                        mNumOfThreads;
    Bindings                        mInputStreamSets;
    Bindings                        mOutputStreamSets;
    Bindings                        mInputScalars;
    Bindings                        mOutputScalars;
    Bindings                        mInternalScalars;
    Kernels                         mKernels;
    CallBindings                    mCallBindings;

};

}

#endif // PIPELINE_BUILDER_H
