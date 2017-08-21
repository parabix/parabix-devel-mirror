#ifndef CPUDRIVER_H
#define CPUDRIVER_H
#include "driver.h"

namespace llvm { class ExecutionEngine; }
namespace llvm { class TargetMachine; }
namespace llvm { class raw_fd_ostream; }

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

private:

    llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const override;

private:
    llvm::TargetMachine *                                   mTarget;
    llvm::ExecutionEngine *                                 mEngine;
    ParabixObjectCache *                                    mCache;
    // NOTE: when printing the IR/ASM, we cannot assume they're completely finished after finalizeObject is executed. Instead we store a
    // pointer and delete them once the driver (and any processing) is complete. This prevents us from reclaiming the memory early but
    // also avoids a potential segmentation fault when writing large files.
    llvm::raw_fd_ostream *                                  mIROutputStream;
    llvm::raw_fd_ostream *                                  mASMOutputStream;
};

#endif // CPUDRIVER_H
