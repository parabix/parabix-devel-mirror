/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H
#include <string>
namespace llvm { class ExecutionEngine; }
namespace llvm { class Module; }
namespace llvm { namespace cl { class OptionCategory; } }

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
#ifdef CUDA_ENABLED
extern bool NVPTX;
#endif
}

#ifdef CUDA_ENABLED
void setNVPTXOption();
void Compile2PTX (llvm::Module * m, std::string IRFilename, std::string PTXFilename);
#endif

bool AVX2_available();

llvm::ExecutionEngine * JIT_to_ExecutionEngine (llvm::Module * m);

void ApplyObjectCache(llvm::ExecutionEngine * e);

#endif
