/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <kernel/pipeline/driver/NVPTXDriver.h>

#include <kernel/core/idisa_target.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/kernel.h>
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#include <toolchain/toolchain.h>
#include <llvm/Analysis/TargetLibraryInfo.h>
#include <llvm/CodeGen/MIRParser/MIRParser.h>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/TargetRegistry.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/ToolOutputFile.h>
#include <llvm/Target/TargetMachine.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#include <llvm/Transforms/Scalar/GVN.h>
#endif

using namespace llvm;

using StreamSetBuffer = kernel::StreamSetBuffer;

NVPTXDriver::NVPTXDriver(std::string && moduleName)
: BaseDriver(std::move(moduleName)) {

    InitializeAllTargets();
    InitializeAllTargetMCs();
    InitializeAllAsmPrinters();
    InitializeAllAsmParsers();

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLoopStrengthReducePass(*Registry);
    initializeLowerIntrinsicsPass(*Registry);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(3, 9, 0)
    initializeUnreachableBlockElimPass(*Registry);
#else
    initializeUnreachableBlockElimLegacyPassPass(*Registry);
#endif

    mMainModule->setDataLayout("e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v16:16:16-v32:32:32-v64:64:64-v128:128:128-n16:32:64");
    mMainModule->setTargetTriple("nvptx64-nvidia-cuda");

    mBuilder.reset(IDISA::GetIDISA_GPU_Builder(*mContext));
    mBuilder->setModule(mMainModule);
    mBuilder->CreateBaseFunctions();
}

Function * NVPTXDriver::addLinkFunction(Module *, llvm::StringRef, FunctionType *, void *) const {
    report_fatal_error("NVPTX does not support linked functions");
}


static int llvm2ptx(Module * M, std::string PTXFilename) {

    std::unique_ptr<MIRParser> MIR;
    Triple TheTriple(M->getTargetTriple());

    if (TheTriple.getTriple().empty())
        TheTriple.setTriple(sys::getDefaultTargetTriple());

    // Get the target specific parser.
    std::string Error;
    const auto TheTarget = TargetRegistry::lookupTarget(codegen::MArch, TheTriple, Error);
    if (!TheTarget) {
        report_fatal_error(Error);
    }

    const auto CPUStr = codegen::getCPUStr();
    const auto FeaturesStr = codegen::getFeaturesStr();

    std::unique_ptr<TargetMachine> Target(
                TheTarget->createTargetMachine(TheTriple.getTriple(), CPUStr, FeaturesStr,
                                               codegen::Options, codegen::RelocModel, codegen::CMModel, codegen::OptLevel));

    assert(Target && "Could not allocate target machine!");

    // Figure out where we are going to send the output.
    std::error_code EC;
    sys::fs::OpenFlags OpenFlags = sys::fs::F_None | sys::fs::F_Text;
    std::unique_ptr<tool_output_file> Out = std::make_unique<tool_output_file>(PTXFilename, EC, OpenFlags);
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
    codegen::setFunctionAttributes(CPUStr, FeaturesStr, *M);

    {
        raw_pwrite_stream *OS = &Out->os();

        AnalysisID StartBeforeID = nullptr;
        AnalysisID StartAfterID = nullptr;
        AnalysisID StopAfterID = nullptr;
        const PassRegistry *PR = PassRegistry::getPassRegistry();
        // Ask the target to add backend passes as necessary.
        if (Target->addPassesToEmitFile(PM, *OS, codegen::FileType, false, StartBeforeID,
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

void * NVPTXDriver::finalizeObject(Function * mainMethod) {

    legacy::PassManager PM;
    PM.add(createPromoteMemoryToRegisterPass()); //Force the use of mem2reg to promote stack variables.
    PM.add(createReassociatePass());             //Reassociate expressions.
    PM.add(createGVNPass());                     //Eliminate common subexpressions.
    PM.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    PM.add(createCFGSimplificationPass());

    for (kernel::Kernel * const kb : mPipeline) {
        mBuilder->setKernel(kb);
        kb->generateKernel(mBuilder);
    }

    Function * mainFunc = mMainModule->getFunction("Main");

    MDNode * Node = MDNode::get(mMainModule->getContext(),
                                {llvm::ValueAsMetadata::get(mainFunc),
                                 MDString::get(mMainModule->getContext(), "kernel"),
                                 ConstantAsMetadata::get(ConstantInt::get(mBuilder->getInt32Ty(), 1))});
    NamedMDNode *NMD = mMainModule->getOrInsertNamedMetadata("nvvm.annotations");
    NMD->addOperand(Node);

    PM.run(*mMainModule);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR))) {
        mMainModule->dump();
    }

    const auto PTXFilename = mMainModule->getModuleIdentifier() + ".ptx";

    llvm2ptx(mMainModule, PTXFilename);

    return nullptr;
}

NVPTXDriver::~NVPTXDriver() {

}
