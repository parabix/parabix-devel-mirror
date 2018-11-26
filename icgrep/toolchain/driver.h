#ifndef DRIVER_H
#define DRIVER_H

#include <IR_Gen/FunctionTypeBuilder.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <kernels/relationship.h>
#include <util/slab_allocator.h>
#include <string>
#include <vector>
#include <memory>

namespace llvm { class Function; }
namespace kernel { class KernelBuilder; }
namespace kernel { class PipelineBuilder; }
class CBuilder;

class BaseDriver {
    friend class CBuilder;
    friend class kernel::PipelineBuilder;
    using Kernel = kernel::Kernel;
    using Relationship = kernel::Relationship;
    using Bindings = kernel::Bindings;
    using OwnedKernels = std::vector<std::unique_ptr<Kernel>>;

public:

    std::unique_ptr<kernel::PipelineBuilder> makePipelineWithIO(Bindings stream_inputs = {}, Bindings stream_outputs = {}, Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    std::unique_ptr<kernel::PipelineBuilder> makePipeline(Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    const std::unique_ptr<kernel::KernelBuilder> & getBuilder() {
        return iBuilder;
    }

    kernel::StreamSet * CreateStreamSet(const unsigned NumElements = 1, const unsigned FieldWidth = 1);

    kernel::Scalar * CreateScalar(llvm::Type * scalarType);

    kernel::Scalar * CreateConstant(llvm::Constant * value);

    void addKernel(Kernel * const kernel);

    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(not_null<Kernel *> kb, llvm::StringRef name, ExternalFunctionType & functionPtr) const;

    virtual bool hasExternalFunction(const llvm::StringRef functionName) const = 0;

    virtual void generateUncachedKernels() = 0;

    virtual void * finalizeObject(llvm::Function * mainMethod) = 0;

    virtual ~BaseDriver();

    llvm::LLVMContext & getContext() const {
        return *mContext.get();
    }

    llvm::Module * getMainModule() const {
        return mMainModule;
    }

protected:

    BaseDriver(std::string && moduleName);

    virtual llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const = 0;

protected:

    std::unique_ptr<llvm::LLVMContext>                      mContext;
    llvm::Module * const                                    mMainModule;
    std::unique_ptr<kernel::KernelBuilder>                  iBuilder;
    OwnedKernels                                            mUncachedKernel;
    OwnedKernels                                            mCachedKernel;
    SlabAllocator<>                                         mAllocator;
};

template <typename ExternalFunctionType>
llvm::Function * BaseDriver::LinkFunction(not_null<Kernel *> kb, llvm::StringRef name, ExternalFunctionType & functionPtr) const {
    llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(getContext());
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    return addLinkFunction(kb->getModule(), name, type, reinterpret_cast<void *>(functionPtr));
}

#endif // DRIVER_H
