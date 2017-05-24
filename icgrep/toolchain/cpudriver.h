#ifndef CPUDRIVER_H
#define CPUDRIVER_H
#include "driver.h"

namespace llvm { class ExecutionEngine; }
namespace llvm { class TargetMachine; }

class ParabixObjectCache;

class ParabixDriver final : public Driver {
    friend class CBuilder;
public:
    ParabixDriver(std::string && moduleName);

    ~ParabixDriver();

    void generatePipelineIR() override;

    void makeKernelCall(kernel::Kernel * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs) override;

    void finalizeObject() override;

    void * getMain() override; // "main" exists until the driver is deleted

private:

    llvm::Function * addLinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const override;

private:
    llvm::TargetMachine *                                   mTarget;
    llvm::ExecutionEngine *                                 mEngine;
    ParabixObjectCache *                                    mCache;
};

#endif // CPUDRIVER_H
