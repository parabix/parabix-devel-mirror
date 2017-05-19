/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef NVPTXDRIVER_H
#define NVPTXDRIVER_H
#include <string>
#include <IR_Gen/FunctionTypeBuilder.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>

namespace llvm { class ExecutionEngine; }
namespace llvm { class Function; }
namespace llvm { class Module; }
namespace llvm { class TargetMachine; }
namespace llvm { class formatted_raw_ostream; }
namespace llvm { namespace cl { class OptionCategory; } }
namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }
namespace IDISA { class IDISA_Builder; }

class NVPTXDriver {
    friend class CBuilder;
public:
    NVPTXDriver(std::string && moduleName);

    ~NVPTXDriver();
    
    const std::unique_ptr<kernel::KernelBuilder> & getBuilder();
    
    parabix::ExternalBuffer * addExternalBuffer(std::unique_ptr<parabix::ExternalBuffer> b);
    
    parabix::StreamSetBuffer * addBuffer(std::unique_ptr<parabix::StreamSetBuffer> b);
    
    kernel::Kernel * addKernelInstance(std::unique_ptr<kernel::Kernel> kb);
    
    void addKernelCall(kernel::Kernel & kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);

    void makeKernelCall(kernel::Kernel * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);
    
    void generatePipelineIR();
    
    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(kernel::Kernel & kb, llvm::StringRef name, ExternalFunctionType * functionPtr) const;

    void finalizeAndCompile(llvm::Function * mainFunc, std::string IRFilename, std::string PTXFilename);
    
    void * getPointerToMain();

protected:

    llvm::Function * LinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const;

private:
    std::unique_ptr<llvm::LLVMContext>                      mContext;
    llvm::Module * const                                    mMainModule;
    std::unique_ptr<kernel::KernelBuilder>                  iBuilder;
    llvm::TargetMachine *                                   mTarget;
    llvm::ExecutionEngine *                                 mEngine;

    std::vector<kernel::Kernel *>                           mPipeline;
    // Owned kernels and buffers that will persist with this ParabixDriver instance.
    std::vector<std::unique_ptr<kernel::Kernel>>            mOwnedKernels;
    std::vector<std::unique_ptr<parabix::StreamSetBuffer>>  mOwnedBuffers;
};

#endif
