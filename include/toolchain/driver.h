#ifndef DRIVER_H
#define DRIVER_H

#include <codegen/FunctionTypeBuilder.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <kernel/core/kernel.h>
// #include <kernels/core/streamset.h>
#include <kernel/core/relationship.h>
#include <util/slab_allocator.h>
#include <string>
#include <vector>
#include <memory>

namespace llvm { class Function; }
namespace kernel { class KernelBuilder; }
namespace kernel { class ProgramBuilder; }
class CBuilder;

class BaseDriver {
    friend class CBuilder;
    friend class kernel::ProgramBuilder;

public:

    using Kernel = kernel::Kernel;
    using Relationship = kernel::Relationship;
    using Bindings = kernel::Bindings;

    std::unique_ptr<kernel::ProgramBuilder> makePipelineWithIO(Bindings stream_inputs = {}, Bindings stream_outputs = {}, Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    std::unique_ptr<kernel::ProgramBuilder> makePipeline(Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    const std::unique_ptr<kernel::KernelBuilder> & getBuilder() {
        return iBuilder;
    }

    kernel::StreamSet * CreateStreamSet(const unsigned NumElements = 1, const unsigned FieldWidth = 1);

    kernel::Scalar * CreateScalar(not_null<llvm::Type *> scalarType);

    kernel::Scalar * CreateConstant(not_null<llvm::Constant *> value);

    void addKernel(not_null<Kernel *> kernel);

    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(not_null<Kernel *> kb, llvm::StringRef name, ExternalFunctionType & functionPtr) const;

    virtual bool hasExternalFunction(const llvm::StringRef functionName) const = 0;

    virtual void generateUncachedKernels() = 0;

    virtual void * finalizeObject(kernel::Kernel * pipeline) = 0;

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
    std::vector<std::unique_ptr<Kernel>>                    mUncachedKernel;
    std::vector<std::unique_ptr<Kernel>>                    mCachedKernel;
    SlabAllocator<>                                         mAllocator;
};

template <typename ExternalFunctionType>
llvm::Function * BaseDriver::LinkFunction(not_null<Kernel *> kb, llvm::StringRef name, ExternalFunctionType & functionPtr) const {
    llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(getContext());
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    return addLinkFunction(kb->getModule(), name, type, reinterpret_cast<void *>(functionPtr));
}

#endif // DRIVER_H
