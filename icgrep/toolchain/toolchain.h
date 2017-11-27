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

// #defines for comparison with LLVM_VERSION_INTEGER
#define LLVM_3_6_0 30600
#define LLVM_3_7_0 30700
#define LLVM_3_8_0 30800
#define LLVM_3_9_0 30900
#define LLVM_4_0_0 40000
#define LLVM_5_0_0 50000
#define LLVM_6_0_0 60000

// From LLVM 4.0.0 the clEnumValEnd sentinel is no longer needed.
// We define a macro to adapt to the CommandLine syntax based on LLVM version.
#if LLVM_VERSION_INTEGER < LLVM_4_0_0
#define CL_ENUM_VAL_SENTINEL , clEnumValEnd
#else
#define CL_ENUM_VAL_SENTINEL
#endif

// FIXME: llvm/CodeGen/CommandFlags.h can only be included once or the various cl::opt causes multiple definition
// errors. To bypass for now, the relevant options and functions are accessible from here. Re-evaluate with later
// versions of LLVM.

namespace llvm { namespace cl { class OptionCategory; } }

namespace codegen {

const llvm::cl::OptionCategory * codegen_flags();

// Command Parameters
enum DebugFlags {
    VerifyIR,
    SerializeThreads,
    TraceCounts,
    TraceDynamicBuffers,
    EnableAsserts,
    EnableCycleCounter,
    DisableIndirectBranch,
    DebugFlagSentinel
};

bool DebugOptionIsSet(const DebugFlags flag);

extern bool PipelineParallel;
extern bool SegmentPipelineParallel;
    
// Options for generating IR or ASM to files
const std::string OmittedOption = ".";
extern std::string ShowUnoptimizedIROption;
extern std::string ShowIROption;
#if LLVM_VERSION_INTEGER >= LLVM_3_7_0
extern std::string ShowASMOption;
#endif
extern const char * ObjectCacheDir;
extern llvm::CodeGenOpt::Level OptLevel;  // set from command line
extern int BlockSize;  // set from command line
extern int SegmentSize;  // set from command line
extern int BufferSegments;
extern int ThreadNum;
extern bool EnableObjectCache;
extern bool NVPTX;
extern int GroupNum;
extern std::string ProgramName;
extern llvm::TargetOptions Options;
extern const llvm::Reloc::Model RelocModel;
extern const llvm::CodeModel::Model CMModel;
extern const std::string MArch;
extern const std::string RunPass;
extern const llvm::TargetMachine::CodeGenFileType FileType;
extern const std::string StopAfter;
extern const std::string StartAfter;

std::string getCPUStr();
std::string getFeaturesStr();
void setFunctionAttributes(llvm::StringRef CPU, llvm::StringRef Features, llvm::Module &M);

void ParseCommandLineOptions(int argc, const char *const *argv, std::initializer_list<const llvm::cl::OptionCategory *> hiding = {});

}

void AddParabixVersionPrinter();

bool AVX2_available();

#endif
