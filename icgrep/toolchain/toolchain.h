/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TOOLCHAIN_H
#define TOOLCHAIN_H

#include <llvm/ADT/StringRef.h>
#include <llvm/Support/CodeGen.h>
#include <llvm/Target/TargetOptions.h>
#include <llvm/Target/TargetMachine.h>

// FIXME: llvm/CodeGen/CommandFlags.h can only be included once or the various cl::opt causes multiple definition
// errors. To bypass for now, the relevant options and functions are accessible from here. Re-evaluate with later
// versions of LLVM.

namespace llvm { namespace cl { class OptionCategory; } }

namespace codegen {

const llvm::cl::OptionCategory * codegen_flags();

// Command Parameters
enum DebugFlags {
    ShowUnoptimizedIR,
    ShowIR,
    VerifyIR,
#ifndef USE_LLVM_3_6
    ShowASM,
#endif
    SerializeThreads
};

bool DebugOptionIsSet(const DebugFlags flag);

extern bool PipelineParallel;
extern bool SegmentPipelineParallel;
#ifndef USE_LLVM_3_6
extern const char * ASMOutputFilename;
#endif
extern const char * IROutputFilename;
extern const char * ObjectCacheDir;
extern llvm::CodeGenOpt::Level OptLevel;  // set from command line
extern int BlockSize;  // set from command line
extern int SegmentSize;  // set from command line
extern int BufferSegments;
extern int ThreadNum;
extern bool EnableObjectCache;
extern bool EnableAsserts;
extern bool EnableCycleCounter;
extern bool NVPTX;
extern int GroupNum;
extern std::string ProgramName;
//extern llvm::TargetOptions Options;
//extern const llvm::Reloc::Model RelocModel;
//extern const llvm::CodeModel::Model CMModel;
//extern const std::string MArch;
//extern const std::string RunPass;
//extern const llvm::TargetMachine::CodeGenFileType FileType;
//extern const std::string StopAfter;
//extern const std::string StartAfter;

std::string getCPUStr();
std::string getFeaturesStr();
void setFunctionAttributes(llvm::StringRef CPU, llvm::StringRef Features, llvm::Module &M);

void ParseCommandLineOptions(int argc, const char *const *argv, std::initializer_list<const llvm::cl::OptionCategory *> hiding = {});

}

void AddParabixVersionPrinter();

bool AVX2_available();

#endif
