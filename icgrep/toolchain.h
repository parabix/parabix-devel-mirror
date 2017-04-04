/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H
#include <string>
#include <IR_Gen/idisa_builder.h>
#include <object_cache.h>

namespace llvm { class ExecutionEngine; }
namespace llvm { class Module; }
namespace llvm { namespace cl { class OptionCategory; } }
namespace IDISA { class IDISA_Builder; }
namespace kernel { class KernelBuilder; }
namespace parabix { class StreamSetBuffer; }

namespace codegen {
const llvm::cl::OptionCategory * codegen_flags();

// Command Parameters
enum DebugFlags {
    ShowIR,
#if LLVM_VERSION_MINOR > 6
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

llvm::ExecutionEngine * JIT_to_ExecutionEngine (llvm::Module * m);

void ApplyObjectCache(llvm::ExecutionEngine * e);

void generatePipeline(IDISA::IDISA_Builder * iBuilder, const std::vector<kernel::KernelBuilder *> & kernels);


class ParabixDriver {
public:
    ParabixDriver(IDISA::IDISA_Builder * iBuilder);
    
    IDISA::IDISA_Builder * getIDISA_Builder() {return iBuilder;}
    
    void JITcompileMain ();
    
    void addKernelCall(kernel::KernelBuilder & kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);
    
    void generatePipelineIR();
    
    void linkAndFinalize();
    
    void * getPointerToMain();

    
private:
    llvm::Module * mMainModule;
    IDISA::IDISA_Builder * iBuilder;
    std::unique_ptr<ParabixObjectCache> mCache;
    //std::unique_ptr<llvm::ExecutionEngine> mEngine;
    llvm::ExecutionEngine * mEngine;
    std::vector<kernel::KernelBuilder *> mKernelList;
};
#endif
