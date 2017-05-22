#include "cpudriver.h"

#include <IR_Gen/idisa_target.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/IR/LegacyPassManager.h>             // for PassManager
#include <llvm/IR/IRPrintingPasses.h>
#include <llvm/InitializePasses.h>                 // for initializeCodeGen
#include <llvm/PassRegistry.h>                     // for PassRegistry
#include <llvm/Support/CodeGen.h>                  // for Level, Level::None
#include <llvm/Support/Compiler.h>                 // for LLVM_UNLIKELY
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#include <toolchain/object_cache.h>
#include <toolchain/toolchain.h>
#include <toolchain/pipeline.h>
#include <kernels/kernel_builder.h>
#include <kernels/kernel.h>
#include <llvm/IR/Verifier.h>

#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

using namespace llvm;
using Kernel = kernel::Kernel;
using StreamSetBuffer = parabix::StreamSetBuffer;
using KernelBuilder = kernel::KernelBuilder;

ParabixDriver::ParabixDriver(std::string && moduleName)
: Driver(std::move(moduleName))
, mTarget(nullptr)
, mEngine(nullptr)
, mCache(nullptr) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);

    std::string errMessage;
    EngineBuilder builder{std::unique_ptr<Module>(mMainModule)};
    builder.setErrorStr(&errMessage);
    builder.setUseOrcMCJITReplacement(true);
    builder.setTargetOptions(codegen::Options);
    builder.setVerifyModules(false);
    builder.setOptLevel(codegen::OptLevel);

    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        std::vector<std::string> attrs;
        for (auto &flag : HostCPUFeatures) {
            auto enabled = flag.second ? "+" : "-";
            attrs.push_back(enabled + flag.first().str());
        }
        builder.setMAttrs(attrs);
    }

    mEngine = builder.create();
    if (mEngine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }
    mTarget = builder.selectTarget();
    if (LLVM_LIKELY(codegen::EnableObjectCache)) {
        if (codegen::ObjectCacheDir.empty()) {
            mCache = new ParabixObjectCache();
        } else {
            mCache = new ParabixObjectCache(codegen::ObjectCacheDir);
        }
        mEngine->setObjectCache(mCache);
    }

    mMainModule->setTargetTriple(mTarget->getTargetTriple().getTriple());

    iBuilder.reset(IDISA::GetIDISA_Builder(*mContext, mMainModule->getTargetTriple()));
    iBuilder->setDriver(this);
    iBuilder->setModule(mMainModule);
}

void ParabixDriver::makeKernelCall(Kernel * kb, const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    assert ("addKernelCall or makeKernelCall was already run on this kernel." && (kb->getModule() == nullptr));
    mPipeline.emplace_back(kb);
    kb->bindPorts(inputs, outputs);
    kb->makeModule(iBuilder);
}

void ParabixDriver::generatePipelineIR() {
    #ifndef NDEBUG
    if (LLVM_UNLIKELY(mPipeline.empty())) {
        report_fatal_error("Pipeline cannot be empty");
    } else {
        for (auto i = mPipeline.begin(); i != mPipeline.end(); ++i) {
            for (auto j = i; ++j != mPipeline.end(); ) {
                if (LLVM_UNLIKELY(*i == *j)) {
                    report_fatal_error("Kernel instances cannot occur twice in the pipeline");
                }
            }
        }
    }
    #endif
    // note: instantiation of all kernels must occur prior to initialization
    for (const auto & k : mPipeline) {
        k->addKernelDeclarations(iBuilder);
    }
    for (const auto & k : mPipeline) {
        k->createInstance(iBuilder);
    }
    for (const auto & k : mPipeline) {
        k->initializeInstance(iBuilder);
    }
    if (codegen::pipelineParallel) {
        generateParallelPipeline(iBuilder, mPipeline);
    } else if (codegen::segmentPipelineParallel) {
        generateSegmentParallelPipeline(iBuilder, mPipeline);
    } else {
        codegen::ThreadNum = 1;
        generatePipelineLoop(iBuilder, mPipeline);
    }
    for (const auto & k : mPipeline) {
        k->finalizeInstance(iBuilder);
    }
}

Function * ParabixDriver::addLinkFunction(Module * mod, llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    assert ("addKernelCall or makeKernelCall must be called before LinkFunction" && (mod != nullptr));
    Function * const f = cast<Function>(mod->getOrInsertFunction(name, type));
    mEngine->addGlobalMapping(f, functionPtr);
    return f;
}

void ParabixDriver::linkAndFinalize() {

    legacy::PassManager PM;
    std::unique_ptr<raw_fd_ostream> IROutputStream(nullptr);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowUnoptimizedIR))) {
        if (codegen::IROutputFilename.empty()) {
            IROutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
        } else {
            std::error_code error;
            IROutputStream.reset(new raw_fd_ostream(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None));
        }
        PM.add(createPrintModulePass(*IROutputStream));
    }

    if (IN_DEBUG_MODE || LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::VerifyIR))) {
        PM.add(createVerifierPass());
    }
    PM.add(createPromoteMemoryToRegisterPass()); //Force the use of mem2reg to promote stack variables.
    PM.add(createReassociatePass());             //Reassociate expressions.
    PM.add(createGVNPass());                     //Eliminate common subexpressions.
    PM.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    PM.add(createCFGSimplificationPass());
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR))) {
        if (LLVM_LIKELY(IROutputStream == nullptr)) {
            if (codegen::IROutputFilename.empty()) {
                IROutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
            } else {
                std::error_code error;
                IROutputStream.reset(new raw_fd_ostream(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None));
            }
        }
        PM.add(createPrintModulePass(*IROutputStream));
    }

    #ifndef USE_LLVM_3_6
    std::unique_ptr<raw_fd_ostream> ASMOutputStream(nullptr);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowASM))) {
        if (codegen::ASMOutputFilename.empty()) {
            ASMOutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
        } else {
            std::error_code error;
            ASMOutputStream.reset(new raw_fd_ostream(codegen::ASMOutputFilename, error, sys::fs::OpenFlags::F_None));
        }
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(PM, *ASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
            report_fatal_error("LLVM error: could not add emit assembly pass");
        }
    }
    #endif

    Module * module = nullptr;

    try {

        for (Kernel * const kernel : mPipeline) {
            iBuilder->setKernel(kernel);
            module = kernel->getModule();
            assert (module != mMainModule);
            bool uncachedObject = true;
            if (mCache && mCache->loadCachedObjectFile(iBuilder, kernel)) {
                uncachedObject = false;
            }
            if (uncachedObject) {
                module->setTargetTriple(mMainModule->getTargetTriple());
                kernel->generateKernel(iBuilder);
                PM.run(*module);
            }
            mEngine->addModule(std::unique_ptr<Module>(module));
        }

        iBuilder->setKernel(nullptr);
        module = mMainModule;
        PM.run(*mMainModule);
        mEngine->finalizeObject();

        if (mCache) mCache->cleanUpObjectCacheFiles();

    } catch (const std::exception & e) {
        report_fatal_error(e.what());
    }

}

void * ParabixDriver::getPointerToMain() {
    return mEngine->getPointerToNamedFunction("Main");
}

ParabixDriver::~ParabixDriver() {
//    delete mEngine;
    delete mCache;
    delete mTarget;
}
