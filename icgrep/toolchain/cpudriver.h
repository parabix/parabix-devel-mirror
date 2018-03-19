#ifndef CPUDRIVER_H
#define CPUDRIVER_H
#include "driver.h"
#include <toolchain/toolchain.h>
namespace llvm { class ExecutionEngine; }
namespace llvm { class TargetMachine; }
namespace llvm { class raw_fd_ostream; }

#include <llvm/IR/LegacyPassManager.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#define ORCJIT
#endif
#ifdef ORCJIT
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/IRTransformLayer.h>
typedef llvm::orc::IRCompileLayer<llvm::orc::ObjectLinkingLayer<>> CompileLayerT;
using OptimizeFnT = std::function<std::unique_ptr<llvm::Module>(std::unique_ptr<llvm::Module>)>;
typedef llvm::orc::IRTransformLayer<CompileLayerT, OptimizeFnT> OptimizeLayerT;

#endif

class ParabixObjectCache;

class ParabixDriver final : public Driver {
    friend class CBuilder;
public:
    ParabixDriver(std::string && moduleName);

    ~ParabixDriver();

    void generatePipelineIR() override;

    void makeKernelCall(kernel::Kernel * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs) override;

    void finalizeObject() override;

    bool hasExternalFunction(const llvm::StringRef functionName) const override;

    void * getMain() override; // "main" exists until the driver is deleted
    
    void performIncrementalCacheCleanupStep() override;

private:
    void preparePassManager();

    llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const override;

    llvm::TargetMachine *                                   mTarget;
    llvm::legacy::PassManager                               mPassManager;

#ifdef ORCJIT
    llvm::orc::ObjectLinkingLayer<>  mObjectLayer;
    std::unique_ptr<CompileLayerT> mCompileLayer;
    
    //std::unique_ptr<OptimizeLayerT> mOptimizeLayer;
    
    //OptimizeLayerT::ModuleSetHandleT addModule(std::unique_ptr<llvm::Module> M);

#else
    llvm::ExecutionEngine *                                 mEngine;
#endif
    ParabixObjectCache *                                    mCache;
    std::vector<kernel::Kernel *>                           mUncachedKernel;
    // NOTE: when printing the IR/ASM, we cannot assume they're completely finished after finalizeObject is executed. Instead we store a
    // pointer and delete them once the driver (and any processing) is complete. This prevents us from reclaiming the memory early but
    // also avoids a potential segmentation fault when writing large files.
    llvm::raw_fd_ostream *                                  mIROutputStream;
    llvm::raw_fd_ostream *                                  mASMOutputStream;
};

#endif // CPUDRIVER_H
