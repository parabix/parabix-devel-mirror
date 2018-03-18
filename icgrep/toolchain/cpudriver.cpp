#include "cpudriver.h"

#include <IR_Gen/idisa_target.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/DynamicLibrary.h>           // for LoadLibraryPermanently
#include <llvm/ExecutionEngine/RuntimeDyld.h>
#include <llvm/ExecutionEngine/RTDyldMemoryManager.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/IR/LegacyPassManager.h>             // for PassManager
#include <llvm/IR/IRPrintingPasses.h>
#include <llvm/InitializePasses.h>                 // for initializeCodeGen
#include <llvm/PassRegistry.h>                     // for PassRegistry
#include <llvm/Support/CodeGen.h>                  // for Level, Level::None
#include <llvm/Support/Compiler.h>                 // for LLVM_UNLIKELY
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#include <llvm/Transforms/Scalar/GVN.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <toolchain/object_cache.h>
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
, mCache(nullptr)
, mIROutputStream(nullptr)
, mASMOutputStream(nullptr) {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();
    llvm::sys::DynamicLibrary::LoadLibraryPermanently(nullptr);

    preparePassManager();

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
        if (codegen::ObjectCacheDir) {
            mCache = new ParabixObjectCache(codegen::ObjectCacheDir);
        } else {
            mCache = new ParabixObjectCache();
        }
        mEngine->setObjectCache(mCache);
    }
    mMainModule->setTargetTriple(mTarget->getTargetTriple().getTriple());
    iBuilder.reset(IDISA::GetIDISA_Builder(*mContext));
    iBuilder->setDriver(this);
    iBuilder->setModule(mMainModule);
}

void ParabixDriver::makeKernelCall(Kernel * kernel, const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    assert ("makeKernelCall was already run on this kernel." && (kernel->getModule() == nullptr));
    mPipeline.emplace_back(kernel);
    kernel->bindPorts(inputs, outputs);
    if (!mCache || !mCache->loadCachedObjectFile(iBuilder, kernel)) {
        mUncachedKernel.push_back(kernel);
    }
    if (kernel->getModule() == nullptr) {
        kernel->makeModule(iBuilder);
    }
    assert (kernel->getModule());
}

void ParabixDriver::generatePipelineIR() {

    for (Kernel * const kernel : mUncachedKernel) {
        kernel->prepareKernel(iBuilder);
    }

    // note: instantiation of all kernels must occur prior to initialization
    for (Kernel * const k : mPipeline) {
        k->addKernelDeclarations(iBuilder);
    }
    for (Kernel * const k : mPipeline) {
        k->createInstance(iBuilder);
    }
    for (Kernel * const k : mPipeline) {
        k->initializeInstance(iBuilder);
    }
    if (codegen::SegmentPipelineParallel) {
        generateSegmentParallelPipeline(iBuilder, mPipeline);
    } else {
        generatePipelineLoop(iBuilder, mPipeline);
    }
    for (const auto & k : mPipeline) {
        k->finalizeInstance(iBuilder);
    }
}

Function * ParabixDriver::addLinkFunction(Module * mod, llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    if (LLVM_UNLIKELY(mod == nullptr)) {
        report_fatal_error("addLinkFunction(" + name + ") cannot be called until after addKernelCall or makeKernelCall");
    }
    Function * f = mod->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        f = Function::Create(type, Function::ExternalLinkage, name, mod);
        mEngine->updateGlobalMapping(f, functionPtr);
    } else if (LLVM_UNLIKELY(f->getType() != type->getPointerTo())) {
        report_fatal_error("Cannot link " + name + ": a function with a different signature already exists with that name in " + mod->getName());
    }
    return f;
}

void ParabixDriver::preparePassManager() {
    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);
    
    if (LLVM_UNLIKELY(codegen::ShowUnoptimizedIROption != codegen::OmittedOption)) {
        if (LLVM_LIKELY(mIROutputStream == nullptr)) {
            if (codegen::ShowUnoptimizedIROption != "") {
                std::error_code error;
                mIROutputStream = new raw_fd_ostream(codegen::ShowUnoptimizedIROption, error, sys::fs::OpenFlags::F_None);
            } else {
                mIROutputStream = new raw_fd_ostream(STDERR_FILENO, false, true);
            }
        }
        mPassManager.add(createPrintModulePass(*mIROutputStream));
    }
    if (IN_DEBUG_MODE || LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::VerifyIR))) {
        mPassManager.add(createVerifierPass());
    }
    mPassManager.add(createPromoteMemoryToRegisterPass());    // Promote stack variables to constants or PHI nodes
    mPassManager.add(createCFGSimplificationPass());          // Remove dead basic blocks and unnecessary branch statements / phi nodes
    mPassManager.add(createEarlyCSEPass());                   // Simple common subexpression elimination pass
    mPassManager.add(createInstructionCombiningPass());       // Simple peephole optimizations and bit-twiddling.
    mPassManager.add(createReassociatePass());                // Canonicalizes commutative expressions
    mPassManager.add(createGVNPass());                        // Global value numbering redundant expression elimination pass
    mPassManager.add(createCFGSimplificationPass());          // Repeat CFG Simplification to "clean up" any newly found redundant phi nodes
    
    if (LLVM_UNLIKELY(codegen::ShowIROption != codegen::OmittedOption)) {
        if (LLVM_LIKELY(mIROutputStream == nullptr)) {
            if (codegen::ShowIROption != "") {
                std::error_code error;
                mIROutputStream = new raw_fd_ostream(codegen::ShowIROption, error, sys::fs::OpenFlags::F_None);
            } else {
                mIROutputStream = new raw_fd_ostream(STDERR_FILENO, false, true);
            }
        }
        mPassManager.add(createPrintModulePass(*mIROutputStream));
    }
    
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (LLVM_UNLIKELY(codegen::ShowASMOption != codegen::OmittedOption)) {
        if (codegen::ShowASMOption != "") {
            std::error_code error;
            mASMOutputStream = new raw_fd_ostream(codegen::ShowASMOption, error, sys::fs::OpenFlags::F_None);
        } else {
            mASMOutputStream = new raw_fd_ostream(STDERR_FILENO, false, true);
        }
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(mPassManager, *mASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
            report_fatal_error("LLVM error: could not add emit assembly pass");
        }
    }
#endif
}


void ParabixDriver::finalizeObject() {

    Module * module = nullptr;
    try {
        for (Kernel * const kernel : mUncachedKernel) {
            iBuilder->setKernel(kernel);
            kernel->generateKernel(iBuilder);
            module = kernel->getModule(); assert (module);
            module->setTargetTriple(mMainModule->getTargetTriple());
            mPassManager.run(*module);
        }
        module = mMainModule;
        iBuilder->setKernel(nullptr);
        mPassManager.run(*mMainModule);
        for (Kernel * const kernel : mPipeline) {
            if (LLVM_UNLIKELY(kernel->getModule() == nullptr)) {
                report_fatal_error(kernel->getName() + " was neither loaded from cache nor generated prior to finalizeObject");
            }
            mEngine->addModule(std::unique_ptr<Module>(kernel->getModule()));
        }
        mEngine->finalizeObject();
    } catch (const std::exception & e) {
        report_fatal_error(module->getName() + ": " + e.what());
    }

}

bool ParabixDriver::hasExternalFunction(llvm::StringRef functionName) const {
    return RTDyldMemoryManager::getSymbolAddressInProcess(functionName);
}

void * ParabixDriver::getMain() {
    return mEngine->getPointerToNamedFunction("Main");
}

void ParabixDriver::performIncrementalCacheCleanupStep() {
    if (mCache) mCache->performIncrementalCacheCleanupStep();
}

ParabixDriver::~ParabixDriver() {
    delete mEngine;
    delete mCache;
    delete mTarget;
    delete mIROutputStream;
    delete mASMOutputStream;
}
