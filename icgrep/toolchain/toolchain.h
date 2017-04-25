/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H
#include <string>
#include <IR_Gen/idisa_builder.h>
#include <IR_Gen/FunctionTypeBuilder.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>

namespace llvm { class ExecutionEngine; }
namespace llvm { class Module; }
namespace llvm { class TargetMachine; }
namespace llvm { class formatted_raw_ostream; }
namespace llvm { namespace cl { class OptionCategory; } }
namespace IDISA { class IDISA_Builder; }
namespace kernel { class KernelBuilder; }

class ParabixObjectCache;

namespace codegen {
const llvm::cl::OptionCategory * codegen_flags();

// Command Parameters
enum DebugFlags {
    ShowIR,
#ifndef USE_LLVM_3_6
    ShowASM,
#endif
    SerializeThreads
};

bool DebugOptionIsSet(DebugFlags flag);


extern char OptLevel;  // set from command line
extern int BlockSize;  // set from command line
extern int SegmentSize;  // set from command line
extern int BufferSegments;
extern int ThreadNum;
extern bool EnableAsserts;
extern bool EnableCycleCounter;
#ifdef CUDA_ENABLED
extern bool NVPTX;
extern int GroupNum;
#endif
}

#ifdef CUDA_ENABLED
void setNVPTXOption();
void Compile2PTX (llvm::Module * m, std::string IRFilename, std::string PTXFilename);
#endif

void AddParabixVersionPrinter();

bool AVX2_available();

class ParabixDriver {
    friend class CBuilder;
public:
    ParabixDriver(std::string && moduleName);

    ~ParabixDriver();
    
    IDISA::IDISA_Builder * getIDISA_Builder() { return iBuilder.get(); }
    
    parabix::ExternalFileBuffer * addExternalBuffer(std::unique_ptr<parabix::ExternalFileBuffer> b, llvm::Value * externalBuf);
    
    parabix::StreamSetBuffer * addBuffer(std::unique_ptr<parabix::StreamSetBuffer> b);
    
    kernel::KernelBuilder * addKernelInstance(std::unique_ptr<kernel::KernelBuilder> kb);
    
    void addKernelCall(kernel::KernelBuilder & kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);

    void makeKernelCall(kernel::KernelBuilder * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);
    
    void generatePipelineIR();
    
    template <typename ExternalFunctionType>
    llvm::Function * LinkFunction(kernel::KernelBuilder & kb, llvm::StringRef name, ExternalFunctionType * functionPtr) const;

    void linkAndFinalize();
    
    void * getPointerToMain();

protected:

    llvm::Function * LinkFunction(llvm::Module * mod, llvm::StringRef name, llvm::FunctionType * type, void * functionPtr) const;

private:
    std::unique_ptr<llvm::LLVMContext>      mContext;
    llvm::Module * const                    mMainModule;
    std::unique_ptr<IDISA::IDISA_Builder>   iBuilder;
    llvm::TargetMachine *                   mTarget;
    llvm::ExecutionEngine *                 mEngine;
    ParabixObjectCache *                    mCache;

    std::vector<kernel::KernelBuilder *>    mPipeline;
    // Owned kernels and buffers that will persist with this ParabixDriver instance.
    std::vector<std::unique_ptr<kernel::KernelBuilder>> mOwnedKernels;
    std::vector<std::unique_ptr<parabix::StreamSetBuffer>> mOwnedBuffers;
};

template <typename ExternalFunctionType>
llvm::Function * ParabixDriver::LinkFunction(kernel::KernelBuilder & kb, llvm::StringRef name, ExternalFunctionType * functionPtr) const {
    llvm::FunctionType * const type = FunctionTypeBuilder<ExternalFunctionType>::get(iBuilder->getContext());
    assert ("FunctionTypeBuilder did not resolve a function type." && type);
    return LinkFunction(kb.getModule(), name, type, reinterpret_cast<void *>(functionPtr));
}

#endif
