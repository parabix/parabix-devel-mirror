#include "cpudriver.h"

#include <IR_Gen/idisa_target.h>
#include <llvm/Support/DynamicLibrary.h>           // for LoadLibraryPermanently
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/ExecutionEngine/MCJIT.h>
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
#include <llvm/Transforms/Scalar/GVN.h>
#include <llvm/Transforms/Scalar/SROA.h>
#include <llvm/Transforms/InstCombine/InstCombine.h>
#include <llvm/Transforms/Utils.h>

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
    llvm::sys::DynamicLibrary::LoadLibraryPermanently(nullptr);

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);

    std::string errMessage;
    EngineBuilder builder{std::unique_ptr<Module>(mMainModule)};
    builder.setErrorStr(&errMessage);
    builder.setEngineKind(EngineKind::JIT);
    llvm::TargetOptions target_Options;
    target_Options.MCOptions.AsmVerbose = true;
    builder.setTargetOptions(target_Options);
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

    mTarget = builder.selectTarget();
    if (mTarget == nullptr) {
        throw std::runtime_error("Could not selectTarget");
    }

    mEngine = builder.create();
    if (mEngine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }


    if (LLVM_LIKELY(codegen::EnableObjectCache)) {
        if (codegen::ObjectCacheDir) {
            mCache = new ParabixObjectCache(codegen::ObjectCacheDir);
        } else {
            mCache = new ParabixObjectCache();
        }
        mEngine->setObjectCache(mCache);
    }
    mEngine->DisableSymbolSearching(false);
    mEngine->DisableLazyCompilation(true);
    mEngine->DisableGVCompilation(true);


    auto triple = mTarget->getTargetTriple().getTriple();
    const DataLayout DL(mTarget->createDataLayout());
    mMainModule->setTargetTriple(triple);
    mMainModule->setDataLayout(DL);

    iBuilder.reset(IDISA::GetIDISA_Builder(*mContext));
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

    if (codegen::ThreadNum == 1) {
        generatePipelineLoop(iBuilder, mPipeline);
    } else {
        generateSegmentParallelPipeline(iBuilder, mPipeline);
    }

    for (const auto & k : mPipeline) {
        k->finalizeInstance(iBuilder);
    }
}

Function * ParabixDriver::addLinkFunction(Module * mod, llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    assert ("addKernelCall or makeKernelCall must be called before LinkFunction" && (mod != nullptr));
    Function * f = mod->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        f = Function::Create(type, Function::ExternalLinkage, name, mod);
        mEngine->addGlobalMapping(f, functionPtr);
    } else if (LLVM_UNLIKELY(f->getType() != type->getPointerTo())) {
        report_fatal_error("Cannot link " + name + ": a function with a different signature already exists with that name in " + mod->getName());
    }
    return f;
}

void ParabixDriver::finalizeObject() {

    legacy::PassManager PM;
    std::unique_ptr<raw_fd_ostream> IROutputStream(nullptr);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowUnoptimizedIR))) {
        if (codegen::IROutputFilename) {
            std::error_code error;
            IROutputStream.reset(new raw_fd_ostream(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None));
        } else {
            IROutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
        }
        PM.add(createPrintModulePass(*IROutputStream));
    }
    if (IN_DEBUG_MODE || LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::VerifyIR))) {
        PM.add(createVerifierPass());
    }
    PM.add(createPromoteMemoryToRegisterPass());    // Promote stack variables to constants or PHI nodes
    PM.add(createCFGSimplificationPass());          // Remove dead basic blocks and unnecessary branch statements / phi nodes
    PM.add(createEarlyCSEPass());                   // Simple common subexpression elimination pass
    PM.add(createInstructionCombiningPass());       // Simple peephole optimizations and bit-twiddling.
    PM.add(createReassociatePass());                // Canonicalizes commutative expressions
    PM.add(createGVNPass());                        // Global value numbering redundant expression elimination pass
    PM.add(createCFGSimplificationPass());          // Repeat CFG Simplification to "clean up" any newly found redundant phi nodes

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR))) {
        if (LLVM_LIKELY(IROutputStream == nullptr)) {
            if (codegen::IROutputFilename) {
                std::error_code error;
                IROutputStream.reset(new raw_fd_ostream(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None));
            } else {
                IROutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
            }
        }
        PM.add(createPrintModulePass(*IROutputStream));
    }

    #ifndef USE_LLVM_3_6
    std::unique_ptr<raw_fd_ostream> ASMOutputStream(nullptr);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowASM))) {
        if (codegen::ASMOutputFilename) {
            std::error_code error;
            ASMOutputStream.reset(new raw_fd_ostream(codegen::ASMOutputFilename, error, sys::fs::OpenFlags::F_None));
        } else {
            ASMOutputStream.reset(new raw_fd_ostream(STDERR_FILENO, false, false));
        }

        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(PM, *ASMOutputStream, nullptr, TargetMachine::CGFT_AssemblyFile))) {
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

bool ParabixDriver::hasExternalFunction(llvm::StringRef functionName) const {
    return mEngine->getPointerToNamedFunction(functionName, false) != nullptr;
}

void * ParabixDriver::getMain() {
    void * mainFunc = mEngine->getPointerToNamedFunction("Main");
    assert (mainFunc);
    return mainFunc;
}

ParabixDriver::~ParabixDriver() {
    delete mEngine;
    delete mCache;
    delete mTarget;
}
