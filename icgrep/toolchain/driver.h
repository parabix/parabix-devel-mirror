#ifndef DRIVER_H
#define DRIVER_H

#include <IR_Gen/FunctionTypeBuilder.h>
#include <llvm/ExecutionEngine/GenericValue.h>
#include <kernels/streamset.h>
#include <kernels/kernel.h>
#include <string>
#include <vector>
#include <memory>

namespace llvm { class Function; }
namespace kernel { class KernelBuilder; }

class Driver {
    friend class CBuilder;
public:
    Driver(std::string && moduleName);

    virtual ~Driver() = default;

    const std::unique_ptr<kernel::KernelBuilder> & getBuilder() {
        return iBuilder;
    }

    template<typename BufferType, typename... Args>
    parabix::StreamSetBuffer * addBuffer(Args &&... args) {
        BufferType * const b = new BufferType(std::forward<Args>(args) ...);
        mOwnedBuffers.emplace_back(b);
        if (!std::is_same<BufferType, parabix::ExternalBuffer>::value) {
            mOwnedBuffers.back()->allocateBuffer(iBuilder);
        }
        return b;
    }

    template<typename KernelType, typename... Args>
    kernel::Kernel * addKernelInstance(Args &&... args) {
        KernelType * const k = new KernelType(std::forward<Args>(args) ...);
        mOwnedKernels.emplace_back(k);
        return k;
    }

    virtual void makeKernelCall(kernel::Kernel * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs) = 0;

    virtual void generatePipelineIR() = 0;

    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(kernel::Kernel & kb, llvm::StringRef name, ExternalFunctionType * functionPtr) const;

    virtual bool hasExternalFunction(const llvm::StringRef functionName) const = 0;

    void deallocateBuffers();
    
    virtual void finalizeObject() = 0;
    
    virtual void * getMain() = 0; // "main" exists until the driver is deleted
    
    virtual void performIncrementalCacheCleanupStep() = 0;

protected:

    virtual llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const = 0;

protected:
    std::unique_ptr<llvm::LLVMContext>                      mContext;
    llvm::Module * const                                    mMainModule;
    std::unique_ptr<kernel::KernelBuilder>                  iBuilder;
    std::vector<std::unique_ptr<kernel::Kernel>>            mOwnedKernels;
    std::vector<std::unique_ptr<parabix::StreamSetBuffer>>  mOwnedBuffers;
    std::vector<kernel::Kernel *>                           mPipeline;
};

template <typename ExternalFunctionType>
llvm::Function * Driver::LinkFunction(kernel::Kernel & kb, llvm::StringRef name, ExternalFunctionType * functionPtr) const {
    llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(*mContext.get());
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    return addLinkFunction(kb.getModule(), name, type, reinterpret_cast<void *>(functionPtr));
}

#endif // DRIVER_H
