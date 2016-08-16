#include <string>
#include <iostream>
#include <fstream>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <cassert>

#include "llvm/ADT/Triple.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/CodeGen/CommandFlags.h"
#include "llvm/CodeGen/LinkAllCodegenComponents.h"
#include "llvm/CodeGen/MIRParser/MIRParser.h"
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/LLVMContext.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/Module.h"
#include "llvm/IR/Verifier.h"
#include "llvm/IRReader/IRReader.h"
#include "llvm/Pass.h"
#include "llvm/Support/CommandLine.h"
#include "llvm/Support/FileSystem.h"
#include "llvm/Support/FormattedStream.h"
#include "llvm/Support/SourceMgr.h"
#include "llvm/Support/TargetRegistry.h"
#include "llvm/Support/TargetSelect.h"
#include "llvm/Support/ToolOutputFile.h"
#include "llvm/Target/TargetMachine.h"
#include "llvm/Target/TargetSubtargetInfo.h"
#include <memory>
using namespace llvm;

static int llvm2ptx(std::string IRFilename, std::string PTXFilename) {

  LLVMContext &Context = getGlobalContext();

  // Load the module to be compiled...
  SMDiagnostic Err;
  std::unique_ptr<Module> M;
  std::unique_ptr<MIRParser> MIR;
  Triple TheTriple;

  M = parseIRFile(IRFilename, Err, Context);
  if (!M) {
    errs() << IRFilename << ": Cannot parse input file!\n";
    return 1;
  }

#ifndef NDEBUG
  if (verifyModule(*M, &errs())) {
    errs() << IRFilename << ": error: input module is broken!\n";
    return 1;
  }
#endif

  TheTriple = Triple(M->getTargetTriple());

  if (TheTriple.getTriple().empty())
    TheTriple.setTriple(sys::getDefaultTargetTriple());

  // Get the target specific parser.
  std::string Error;
  const Target *TheTarget = TargetRegistry::lookupTarget(MArch, TheTriple,
                                                         Error);
  if (!TheTarget) {
    errs() << Error;
    return 1;
  }

  std::string CPUStr = getCPUStr(), FeaturesStr = getFeaturesStr();

  CodeGenOpt::Level OLvl = CodeGenOpt::Default;

  TargetOptions Options = InitTargetOptionsFromCodeGenFlags();

  std::unique_ptr<TargetMachine> Target(
      TheTarget->createTargetMachine(TheTriple.getTriple(), CPUStr, FeaturesStr,
                                     Options, RelocModel, CMModel, OLvl));

  assert(Target && "Could not allocate target machine!");

  // Figure out where we are going to send the output.
  std::error_code EC;
  sys::fs::OpenFlags OpenFlags = sys::fs::F_None | sys::fs::F_Text;
  std::unique_ptr<tool_output_file> Out = llvm::make_unique<tool_output_file>(PTXFilename, EC, OpenFlags);
  if (EC) {
    errs() << EC.message() << '\n';
    return 1;
  }

  // Build up all of the passes that we want to do to the module.
  legacy::PassManager PM;

  // Add an appropriate TargetLibraryInfo pass for the module's triple.
  TargetLibraryInfoImpl TLII(Triple(M->getTargetTriple()));

  PM.add(new TargetLibraryInfoWrapperPass(TLII));

  // Add the target data from the target machine, if it exists, or the module.
  M->setDataLayout(Target->createDataLayout());

  // Override function attributes based on CPUStr, FeaturesStr, and command line
  // flags.
  setFunctionAttributes(CPUStr, FeaturesStr, *M);

  {
    raw_pwrite_stream *OS = &Out->os();

    AnalysisID StartBeforeID = nullptr;
    AnalysisID StartAfterID = nullptr;
    AnalysisID StopAfterID = nullptr;
    const PassRegistry *PR = PassRegistry::getPassRegistry();
    if (!RunPass.empty()) {
      if (!StartAfter.empty() || !StopAfter.empty()) {
        errs() << "start-after and/or stop-after passes are redundant when run-pass is specified.\n";
        return 1;
      }
      const PassInfo *PI = PR->getPassInfo(RunPass);
      if (!PI) {
        errs() << "run-pass pass is not registered.\n";
        return 1;
      }
      StopAfterID = StartBeforeID = PI->getTypeInfo();
    } else {
      if (!StartAfter.empty()) {
        const PassInfo *PI = PR->getPassInfo(StartAfter);
        if (!PI) {
          errs() << "start-after pass is not registered.\n";
          return 1;
        }
        StartAfterID = PI->getTypeInfo();
      }
      if (!StopAfter.empty()) {
        const PassInfo *PI = PR->getPassInfo(StopAfter);
        if (!PI) {
          errs() << "stop-after pass is not registered.\n";
          return 1;
        }
        StopAfterID = PI->getTypeInfo();
      }
    }

    // Ask the target to add backend passes as necessary.
    if (Target->addPassesToEmitFile(PM, *OS, FileType, false, StartBeforeID,
                                    StartAfterID, StopAfterID, MIR.get())) {
      errs() << " target does not support generation of this file type!\n";
      return 1;
    }

    PM.run(*M);
  }
  // Declare success.
  Out->keep();

  return 0;
}
