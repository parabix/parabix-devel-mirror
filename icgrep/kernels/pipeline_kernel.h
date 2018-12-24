#ifndef PIPELINE_KERNEL_H
#define PIPELINE_KERNEL_H

#include "kernel.h"
#include <type_traits>
#include <functional>
#include <toolchain/driver.h>
#include <boost/container/flat_map.hpp>

namespace llvm { class Value; }

namespace kernel {

const static std::string INITIALIZE_FUNCTION_POINTER_SUFFIX = "_IFP";
const static std::string DO_SEGMENT_FUNCTION_POINTER_SUFFIX = "_SFP";
const static std::string FINALIZE_FUNCTION_POINTER_SUFFIX = "_FIP";

class PipelineCompiler;

class PipelineKernel : public Kernel {
    friend class PipelineCompiler;
    friend class PipelineBuilder;
public:

    static bool classof(const Kernel * const k) {
        return k->getTypeId() == TypeId::Pipeline;
    }

    static bool classof(const void *) { return false; }

public:

    using Scalars = std::vector<Scalar *>;
    using Kernels = std::vector<Kernel *>;

    struct CallBinding {
        const std::string Name;
        llvm::FunctionType * const Type;
        void * const FunctionPointer;
        const Scalars Args;

        llvm::Constant * Callee;

        CallBinding(const std::string Name, llvm::FunctionType * Type, void * FunctionPointer, std::initializer_list<Scalar *> && Args)
        : Name(Name), Type(Type), FunctionPointer(FunctionPointer), Args(Args), Callee(nullptr) { }
    };

    using CallBindings = std::vector<CallBinding>;

    template <typename Value>
    using StreamSetBufferMap = boost::container::flat_map<const StreamSetBuffer *, Value>;

    const std::string getName() const final;

    bool isCachable() const final { return true; }

    bool hasSignature() const final { return true; }

    std::string makeSignature(const std::unique_ptr<KernelBuilder> &) final {
        return mSignature;
    }

    const unsigned getNumOfThreads() const {
        return mNumOfThreads;
    }

    const Kernels & getKernels() const {
        return mKernels;
    }

    const CallBindings & getCallBindings() const {
        return mCallBindings;
    }

    virtual ~PipelineKernel();

    bool isProxy() const {
        return mKernels.size() == 1;
    }

protected:

    PipelineKernel(std::string && signature, const unsigned numOfThreads,
                   Kernels && kernels, CallBindings && callBindings,
                   Bindings && stream_inputs, Bindings && stream_outputs,
                   Bindings && scalar_inputs, Bindings && scalar_outputs);

    static LLVM_READNONE std::string makeKernelName(const Kernel * const kernel, const unsigned kernelIndex);

    static LLVM_READNONE std::string makeBufferName(const Kernel * const kernel, const unsigned kernelIndex, const Binding & binding);

    void linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) final;

    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<llvm::Value *> & args) final;

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) final;

    void addAdditionalFunctions(const std::unique_ptr<KernelBuilder> & b) final;

    llvm::Value * finalizeInstance(const std::unique_ptr<KernelBuilder> & b) final;

    void addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) final;

    void setInputStreamSetAt(const unsigned i, StreamSet * const value) final;

    void setOutputStreamSetAt(const unsigned i, StreamSet * const value) final;

    void setInputScalarAt(const unsigned i, Scalar * const value) final;

    void setOutputScalarAt(const unsigned i, Scalar * const value) final;

    std::vector<llvm::Value *> getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) final;

    enum MainMethodGenerationType {
        AddInternal
        , DeclareExternal
        , AddExternal
    };

    llvm::Function * addOrDeclareMainFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const MainMethodGenerationType method);

    LLVM_READNONE bool hasStaticMain() const;

protected:

    std::unique_ptr<PipelineCompiler>  mCompiler;
    const unsigned                     mNumOfThreads;
    const Kernels                      mKernels;
    CallBindings                       mCallBindings;
    const std::string                  mSignature;
};

}

#endif // PIPELINE_KERNEL_H
