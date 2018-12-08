/*
 *  Copyright (c) 2018 International Characters.
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
#define LLVM_VERSION_CODE(major, minor, point) ((10000 * major) + (100 * minor) + point)

// From LLVM 4.0.0 the clEnumValEnd sentinel is no longer needed.
// We define a macro to adapt to the CommandLine syntax based on LLVM version.
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#define CL_ENUM_VAL_SENTINEL , clEnumValEnd
#else
#define CL_ENUM_VAL_SENTINEL
#endif

namespace llvm { namespace cl { class OptionCategory; } }

namespace codegen {

const llvm::cl::OptionCategory * LLVM_READONLY codegen_flags();

// Command Parameters
enum DebugFlags {
    VerifyIR,
    SerializeThreads,
    TraceCounts,
    TraceDynamicBuffers,
    EnableAsserts,
    EnableMProtect,
    EnableCycleCounter,
    DisableIndirectBranch,
    DebugFlagSentinel
};

bool LLVM_READONLY DebugOptionIsSet(const DebugFlags flag);

extern bool SegmentPipelineParallel;
    
// Options for generating IR or ASM to files
const std::string OmittedOption = ".";
extern std::string ShowUnoptimizedIROption;
extern std::string ShowIROption;
extern std::string TraceOption;
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
extern std::string ShowASMOption;
#endif
extern const char * ObjectCacheDir;
extern unsigned CacheDaysLimit;  // set from command line
extern llvm::CodeGenOpt::Level OptLevel;  // set from command line
const unsigned LaneWidth = 64;
extern unsigned BlockSize;  // set from command line
extern unsigned SegmentSize; // set from command line
extern unsigned BufferSegments;
extern unsigned ThreadNum;
extern bool EnableObjectCache;
extern bool NVPTX;
extern unsigned GroupNum;
extern std::string ProgramName;
extern llvm::TargetOptions target_Options;

void ParseCommandLineOptions(int argc, const char *const *argv, std::initializer_list<const llvm::cl::OptionCategory *> hiding = {});

void AddParabixVersionPrinter();

}

#endif
