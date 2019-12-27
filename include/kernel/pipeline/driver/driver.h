#ifndef DRIVER_H
#define DRIVER_H

#include <codegen/FunctionTypeBuilder.h>
#include <codegen/virtual_driver.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <kernel/core/kernel.h>
#include <kernel/core/relationship.h>
#include <util/slab_allocator.h>
#include <boost/container/flat_set.hpp>
#include <string>
#include <vector>
#include <memory>

namespace llvm { class Function; }
namespace kernel { class KernelBuilder; }
namespace kernel { class ProgramBuilder; }
class CBuilder;

class BaseDriver : public codegen::VirtualDriver {
    friend class CBuilder;
    friend class kernel::ProgramBuilder;
    friend class kernel::Kernel;
public:

    using Kernel = kernel::Kernel;
    using Relationship = kernel::Relationship;
    using Bindings = kernel::Bindings;
    using BuilderRef = Kernel::BuilderRef;
    using KernelSet = std::vector<std::unique_ptr<Kernel>>;

    std::unique_ptr<kernel::ProgramBuilder> makePipelineWithIO(Bindings stream_inputs = {}, Bindings stream_outputs = {}, Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    std::unique_ptr<kernel::ProgramBuilder> makePipeline(Bindings scalar_inputs = {}, Bindings scalar_outputs = {});

    BuilderRef getBuilder() {
        return mBuilder;
    }

    kernel::StreamSet * CreateStreamSet(const unsigned NumElements = 1, const unsigned FieldWidth = 1);

    kernel::Scalar * CreateScalar(not_null<llvm::Type *> scalarType);

    kernel::Scalar * CreateConstant(not_null<llvm::Constant *> value);

    void addKernel(not_null<Kernel *> kernel);

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

    bool getPreservesKernels() const {
        return mPreservesKernels;
    }

    void setPreserveKernels(const bool value = true) {
        mPreservesKernels = value;
    }

protected:

    BaseDriver(std::string && moduleName);

    template <typename ExternalFunctionType>
    void LinkFunction(not_null<Kernel *> kernel, llvm::StringRef name, ExternalFunctionType & functionPtr) const;

    virtual llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const = 0;

protected:

    std::unique_ptr<llvm::LLVMContext>                      mContext;
    llvm::Module * const                                    mMainModule;
    std::unique_ptr<kernel::KernelBuilder>                  mBuilder;
    bool                                                    mPreservesKernels = false;
    KernelSet                                               mUncachedKernel;
    KernelSet                                               mCachedKernel;
    KernelSet                                               mPreservedKernel;
    SlabAllocator<>                                         mAllocator;
};

template <typename ExternalFunctionType>
void BaseDriver::LinkFunction(not_null<Kernel *> kernel, llvm::StringRef name, ExternalFunctionType & functionPtr) const {
    kernel->link<ExternalFunctionType>(name, functionPtr);
}

#endif // DRIVER_H
