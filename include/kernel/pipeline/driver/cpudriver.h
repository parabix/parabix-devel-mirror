#ifndef CPUDRIVER_H
#define CPUDRIVER_H
#include <kernel/pipeline/driver/driver.h>
#include <toolchain/toolchain.h>
namespace llvm { class ExecutionEngine; }
namespace llvm { class TargetMachine; }
namespace llvm { class raw_fd_ostream; }
namespace llvm { class ModulePass; }
namespace kernel { class KernelBuilder; }

#include <llvm/IR/LegacyPassManager.h>

class CPUDriver final : public BaseDriver {
public:

    CPUDriver(std::string && moduleName);

    ~CPUDriver();

    void generateUncachedKernels() override;

    void * finalizeObject(kernel::Kernel * const pipeline) override;

    bool hasExternalFunction(const llvm::StringRef functionName) const override;

    llvm::ModulePass * createTracePass(kernel::KernelBuilder * kb, llvm::StringRef to_trace);

private:

    std::string getMangledName(std::string s);

    void preparePassManager();

    llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const override;

private:
    llvm::TargetMachine *                                   mTarget;
    llvm::ExecutionEngine *                                 mEngine;
    std::unique_ptr<llvm::raw_fd_ostream>                   mUnoptimizedIROutputStream;
    std::unique_ptr<llvm::raw_fd_ostream>                   mIROutputStream;
    std::unique_ptr<llvm::raw_fd_ostream>                   mASMOutputStream;
    std::unique_ptr<llvm::legacy::PassManager>              mPassManager;
};

#endif // CPUDRIVER_H
