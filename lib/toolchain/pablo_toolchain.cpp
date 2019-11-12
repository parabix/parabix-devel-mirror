/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/pablo_toolchain.h>

#include <toolchain/toolchain.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace pablo {

static cl::OptionCategory PabloOptions("Pablo Options", "These options control printing, generation and instrumentation of Pablo intermediate code.");

const cl::OptionCategory * pablo_toolchain_flags() {
    return &PabloOptions;
}
    
    
static cl::bits<PabloDebugFlags> 
DebugOptions(cl::values(clEnumVal(VerifyPablo, "Run the Pablo verifier"),
                        clEnumVal(DumpTrace, "Generate dynamic traces of executed Pablo assignments.")
                        CL_ENUM_VAL_SENTINEL), cl::cat(PabloOptions));
    
std::string ShowPabloOption = codegen::OmittedOption;
static cl::opt<std::string, true> PabloOutputOption("ShowPablo", cl::location(ShowPabloOption), cl::ValueOptional,
                                                    cl::desc("Print generated Pablo code to stderr (by omitting =<filename>) or a file"), cl::value_desc("filename"), cl::cat(PabloOptions));
sys::fs::OpenFlags PabloOutputFileFlag = sys::fs::OpenFlags::F_None;
    
std::string ShowOptimizedPabloOption = codegen::OmittedOption;
static cl::opt<std::string, true> OptimizedPabloOutputOption("ShowOptimizedPablo", cl::location(ShowOptimizedPabloOption), cl::ValueOptional,
                                                    cl::desc("Print optimized Pablo code to stderr (by omitting =<filename>) or a file"), cl::value_desc("filename"), cl::cat(PabloOptions));
sys::fs::OpenFlags PabloOptimizedOutputFileFlag = sys::fs::OpenFlags::F_None;

static cl::bits<PabloCompilationFlags> 
    PabloOptimizationsOptions(cl::values(clEnumVal(Flatten, "Flatten all the Ifs in the Pablo AST"),
                                         clEnumVal(DisableSimplification, "Disable Pablo Simplification pass (not recommended)"),
                                         clEnumVal(DisableCodeMotion, "Moves statements into the innermost legal If-scope and moves invariants out of While-loops."),
                                         clEnumVal(EnableDistribution, "Apply distribution law optimization."),                                         
                                         clEnumVal(EnableSchedulingPrePass, "Pablo Statement Scheduling Pre-Pass"),
                                         clEnumVal(EnableProfiling, "Profile branch statistics."),
                                         clEnumVal(EnableTernaryOpt, "Enable ternary optimization.")
                                         CL_ENUM_VAL_SENTINEL), cl::cat(PabloOptions));

PabloCarryMode CarryMode;
static cl::opt<PabloCarryMode, true> PabloCarryModeOptions("CarryMode", cl::desc("Carry mode for pablo compiler (default BitBlock)"), 
    cl::location(CarryMode), cl::ValueOptional,
    cl::values(
        clEnumValN(PabloCarryMode::BitBlock, "BitBlock", "All carries are stored as bit blocks."),
        clEnumValN(PabloCarryMode::Compressed, "Compressed", "When possible, carries are stored as 64-bit integers.")
        CL_ENUM_VAL_SENTINEL), cl::cat(PabloOptions), cl::init(PabloCarryMode::Compressed));

bool DebugOptionIsSet(const PabloDebugFlags flag) {return DebugOptions.isSet(flag);}
    
bool CompileOptionIsSet(const PabloCompilationFlags flag) {return PabloOptimizationsOptions.isSet(flag);}

}
