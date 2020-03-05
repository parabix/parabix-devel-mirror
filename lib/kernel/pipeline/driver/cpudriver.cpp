#include <kernel/pipeline/driver/cpudriver.h>
#include <kernel/core/kernel_compiler.h>
#include <kernel/core/idisa_target.h>
#include <toolchain/toolchain.h>
#include <llvm/Support/DynamicLibrary.h>           // for LoadLibraryPermanently
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/ExecutionEngine/RTDyldMemoryManager.h>
#include <llvm/IR/LegacyPassManager.h>             // for PassManager
#include <llvm/IR/IRPrintingPasses.h>
#include <llvm/InitializePasses.h>                 // for initializeCodeGencd .
#include <llvm/PassRegistry.h>                     // for PassRegistry
#include <llvm/Support/CodeGen.h>                  // for Level, Level::None
#include <llvm/Support/Compiler.h>                 // for LLVM_UNLIKELY
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/Timer.h>
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#include <llvm/Transforms/Utils/Cloning.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#include <llvm/Transforms/Scalar/GVN.h>
#endif
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(6, 0, 0)
#include <llvm/Transforms/Scalar/SROA.h>
#endif
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
#include <llvm/Transforms/InstCombine/InstCombine.h>
#include <llvm/Transforms/Utils.h>
#endif
#include <objcache/object_cache.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/pipeline/pipeline_builder.h>
#include <llvm/IR/Verifier.h>
#include "llvm/IR/Mangler.h"
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/ExecutionEngine/MCJIT.h>
#endif
#include <llvm/ADT/Statistic.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(8, 0, 0)
#include <llvm/IR/LegacyPassManager.h>
#else
#include <llvm/IR/PassTimingInfo.h>
#endif
#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

using namespace llvm;
using namespace kernel;

using AttrId = kernel::Attribute::KindId;

CPUDriver::CPUDriver(std::string && moduleName)
: BaseDriver(std::move(moduleName))
, mTarget(nullptr)
, mEngine(nullptr)
, mUnoptimizedIROutputStream{}
, mIROutputStream{}
, mASMOutputStream{}
, mPassManager{} {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    llvm::sys::DynamicLibrary::LoadLibraryPermanently(nullptr);

    std::string errMessage;
    EngineBuilder builder{std::unique_ptr<Module>(mMainModule)};
    builder.setErrorStr(&errMessage);
    builder.setVerifyModules(false);
    builder.setEngineKind(EngineKind::JIT);
    builder.setTargetOptions(codegen::target_Options);
    builder.setOptLevel(codegen::BackEndOptLevel);

    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        std::vector<std::string> attrs;
        for (auto &flag : HostCPUFeatures) {
            if (flag.second) {
                attrs.push_back("+" + flag.first().str());
                //llvm::errs() << flag.first().str() << "\n";
            }
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
    if (mObjectCache) {
        mEngine->setObjectCache(mObjectCache.get());
    }
    mEngine->DisableSymbolSearching(false);
    mEngine->DisableLazyCompilation(true);
    mEngine->DisableGVCompilation(true);

    auto triple = mTarget->getTargetTriple().getTriple();
    const DataLayout DL(mTarget->createDataLayout());
    mMainModule->setTargetTriple(triple);
    mMainModule->setDataLayout(DL);
    mBuilder.reset(IDISA::GetIDISA_Builder(*mContext));
    mBuilder->setDriver(*this);
    mBuilder->setModule(mMainModule);


}

Function * CPUDriver::addLinkFunction(Module * mod, llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    if (LLVM_UNLIKELY(mod == nullptr)) {
        report_fatal_error("addLinkFunction(" + name + ") cannot be called until after addKernel");
    }
    Function * f = mod->getFunction(name);
    if (LLVM_UNLIKELY(f == nullptr)) {
        f = Function::Create(type, Function::ExternalLinkage, name, mod);
        #ifndef ORCJIT
        mEngine->updateGlobalMapping(f, functionPtr);
        #endif
    } else if (LLVM_UNLIKELY(f->getType() != type->getPointerTo())) {
        report_fatal_error("Cannot link " + name + ": a function with a different signature already exists with that name in " + mod->getName());
    }
    return f;
}

inline void CPUDriver::preparePassManager() {

    if (mPassManager) return;

    mPassManager = make_unique<legacy::PassManager>();

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);
    if (LLVM_UNLIKELY(codegen::ShowUnoptimizedIROption != codegen::OmittedOption)) {
        if (LLVM_LIKELY(mIROutputStream == nullptr)) {
            if (!codegen::ShowUnoptimizedIROption.empty()) {
                std::error_code error;
                mUnoptimizedIROutputStream = make_unique<raw_fd_ostream>(codegen::ShowUnoptimizedIROption, error, sys::fs::OpenFlags::F_None);
            } else {
                mUnoptimizedIROutputStream = make_unique<raw_fd_ostream>(STDERR_FILENO, false, true);
            }
        }
        mPassManager->add(createPrintModulePass(*mUnoptimizedIROutputStream));
    }
    if (IN_DEBUG_MODE || LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::VerifyIR))) {
        mPassManager->add(createVerifierPass());
    }
    if (LLVM_UNLIKELY(!codegen::TraceOption.empty())) {
        mPassManager->add(createTracePass(mBuilder.get(), codegen::TraceOption));
    }
    if (LLVM_UNLIKELY(codegen::ShowIROption != codegen::OmittedOption)) {
        if (LLVM_LIKELY(mIROutputStream == nullptr)) {
            if (!codegen::ShowIROption.empty()) {
                std::error_code error;
                mIROutputStream = make_unique<raw_fd_ostream>(codegen::ShowIROption, error, sys::fs::OpenFlags::F_None);
            } else {
                mIROutputStream = make_unique<raw_fd_ostream>(STDERR_FILENO, false, true);
            }
        }
        mPassManager->add(createPrintModulePass(*mIROutputStream));
    }
    mPassManager->add(createDeadCodeEliminationPass());        // Eliminate any trivially dead code
    mPassManager->add(createPromoteMemoryToRegisterPass());    // Promote stack variables to constants or PHI nodes
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(6, 0, 0)
    mPassManager->add(createSROAPass());                       // Promote elements of aggregate allocas whose addresses are not taken to registers.
    #endif
    mPassManager->add(createCFGSimplificationPass());          // Remove dead basic blocks and unnecessary branch statements / phi nodes
    mPassManager->add(createEarlyCSEPass());                   // Simple common subexpression elimination pass
    mPassManager->add(createInstructionCombiningPass());       // Simple peephole optimizations and bit-twiddling.
    mPassManager->add(createReassociatePass());                // Canonicalizes commutative expressions
    mPassManager->add(createGVNPass());                        // Global value numbering redundant expression elimination pass
    mPassManager->add(createCFGSimplificationPass());          // Repeat CFG Simplification to "clean up" any newly found redundant phi nodes
    #ifdef NDEBUG
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
    #endif
        mPassManager->add(createRemoveRedundantAssertionsPass());
        mPassManager->add(createDeadCodeEliminationPass());
        mPassManager->add(createCFGSimplificationPass());
    #ifdef NDEBUG
    }
    #endif
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (LLVM_UNLIKELY(codegen::ShowASMOption != codegen::OmittedOption)) {
        if (!codegen::ShowASMOption.empty()) {
            std::error_code error;
            mASMOutputStream = make_unique<raw_fd_ostream>(codegen::ShowASMOption, error, sys::fs::OpenFlags::F_None);
        } else {
            mASMOutputStream = make_unique<raw_fd_ostream>(STDERR_FILENO, false, true);
        }
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, nullptr, TargetMachine::CGFT_AssemblyFile))) {
#else
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
#endif
            report_fatal_error("LLVM error: could not add emit assembly pass");
        }
    }
    #endif
}

void CPUDriver::generateUncachedKernels() {
    if (mUncachedKernel.empty()) return;
    preparePassManager();
    mCachedKernel.reserve(mUncachedKernel.size());
    for (auto & kernel : mUncachedKernel) {
        {
            NamedRegionTimer T(kernel->getName(),
                               kernel->getSignature(),
                               "kernel",
                               "Kernel Generation",
                               codegen::KernelTimePassesIsEnabled);
            kernel->generateKernel(mBuilder);
            Module * const module = kernel->getModule(); assert (module);
            module->setTargetTriple(mMainModule->getTargetTriple());
            mPassManager->run(*module);
            mCachedKernel.emplace_back(kernel.release());
        }
    }
    mUncachedKernel.clear();
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(5, 0, 0)
    llvm::reportAndResetTimings();
    #endif
    llvm::PrintStatistics();
}

void * CPUDriver::finalizeObject(kernel::Kernel * const pipeline) {

    using ModuleSet = llvm::SmallVector<Module *, 32>;

    ModuleSet Infrequent;
    ModuleSet Normal;

    for (const auto & kernel : mCompiledKernel) {
        kernel->ensureLoaded();
    }

    for (const auto & kernel : mCachedKernel) {
        if (LLVM_UNLIKELY(kernel->getModule() == nullptr)) {
            report_fatal_error(kernel->getName() + " was neither loaded from cache nor generated prior to finalizeObject");
        }
        Module * const m = kernel->getModule();
        assert ("cached kernel has no module?" && m);
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::InfrequentlyUsed))) {
            Infrequent.emplace_back(m);
        } else {
            Normal.emplace_back(m);
        }
    }

    auto addModules = [&](const ModuleSet & S, const CodeGenOpt::Level level) {
        if (S.empty()) return;
        mEngine->getTargetMachine()->setOptLevel(level);
        for (Module * M : S) {
            mEngine->addModule(std::unique_ptr<Module>(M));
        }
        mEngine->finalizeObject();
    };

    auto removeModules = [&](const ModuleSet & S) {
        for (Module * M : S) {
            mEngine->removeModule(M);
        }
    };

    // compile any uncompiled kernels
    addModules(Normal, codegen::BackEndOptLevel);
    addModules(Infrequent, CodeGenOpt::None);

    // write/declare the "main" method
    auto mainModule = make_unique<Module>("main", *mContext);
    mainModule->setTargetTriple(mMainModule->getTargetTriple());
    mainModule->setDataLayout(mMainModule->getDataLayout());
    mBuilder->setModule(mainModule.get());
    pipeline->addKernelDeclarations(mBuilder);
    const auto method = pipeline->externallyInitialized() ? Kernel::AddInternal : Kernel::DeclareExternal;
    Function * const main = pipeline->addOrDeclareMainFunction(mBuilder, method);
    mBuilder->setModule(mMainModule);

    // NOTE: the pipeline kernel is destructed after calling clear unless this driver preserves kernels!
    if (getPreservesKernels()) {
        for (auto & kernel : mCachedKernel) {
            mPreservedKernel.emplace_back(kernel.release());
        }
        for (auto & kernel : mCompiledKernel) {
            mPreservedKernel.emplace_back(kernel.release());
        }
    } else {
        mPreservedKernel.clear();
    }
    mCachedKernel.clear();
    mCompiledKernel.clear();

    // return the compiled main method
    mEngine->getTargetMachine()->setOptLevel(CodeGenOpt::Less);
    mEngine->addModule(std::move(mainModule));
    mEngine->finalizeObject();
    auto mainFnPtr = mEngine->getFunctionAddress(main->getName());
    removeModules(Normal);
    removeModules(Infrequent);
    return reinterpret_cast<void *>(mainFnPtr);
}

bool CPUDriver::hasExternalFunction(llvm::StringRef functionName) const {
    return RTDyldMemoryManager::getSymbolAddressInProcess(functionName);
}

CPUDriver::~CPUDriver() {
    #ifndef ORCJIT
    delete mEngine;
    #endif
    delete mTarget;
}


class TracePass : public ModulePass {
public:
    static char ID;
    TracePass(kernel::KernelBuilder * kb, StringRef to_trace) : ModulePass(ID), iBuilder(kb), mToTrace(to_trace) { }

    bool addTraceStmt(BasicBlock * BB, BasicBlock::iterator to_trace, BasicBlock::iterator insert_pt) {
        bool modified = false;
        Type * t = (*to_trace).getType();
        //t->dump();
        if (t == iBuilder->getBitBlockType()) {
            iBuilder->SetInsertPoint(BB, insert_pt);
            iBuilder->CallPrintRegister((*to_trace).getName(), &*to_trace);
            modified = true;
        }
        else if (t == iBuilder->getInt64Ty()) {
            iBuilder->SetInsertPoint(BB, insert_pt);
            iBuilder->CallPrintInt((*to_trace).getName(), &*to_trace);
            modified = true;
        }
        return modified;
    }

    virtual bool runOnModule(Module &M) override;
private:
    kernel::KernelBuilder * iBuilder;
    StringRef mToTrace;
};

char TracePass::ID = 0;

bool TracePass::runOnModule(Module & M) {
    Module * saveModule = iBuilder->getModule();
    iBuilder->setModule(&M);
    bool modified = false;
    for (auto & F : M) {
        for (auto & B : F) {
            std::vector<BasicBlock::iterator> tracedPhis;
            BasicBlock::iterator i = B.begin();
            while (isa<PHINode>(*i)) {
                if ((*i).getName().startswith(mToTrace)) {
                    tracedPhis.push_back(i);
                }
                ++i;
            }
            for (auto t : tracedPhis) {
                modified = addTraceStmt(&B, t, i) || modified;
            }
            while (i != B.end()) {
                auto i0 = i;
                ++i;
                if ((*i0).getName().startswith(mToTrace)) {
                    modified = addTraceStmt(&B, i0, i) || modified;
                }
            }
        }
    }
    //if (modified) M.dump();
    iBuilder->setModule(saveModule);
    return modified;
}

ModulePass * CPUDriver::createTracePass(kernel::KernelBuilder * kb, StringRef to_trace) {
    return new TracePass(mBuilder.get(), to_trace);
}
