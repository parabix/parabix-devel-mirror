#include "cpudriver.h"

#include <IR_Gen/idisa_target.h>
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
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#include <llvm/Transforms/Scalar/GVN.h>
#endif
#include <llvm/Transforms/Utils/Local.h>
#include <toolchain/object_cache.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline_builder.h>
#include <llvm/IR/Verifier.h>
#include "llvm/IR/Mangler.h"
#ifndef ORCJIT
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/ExecutionEngine/MCJIT.h>
#endif
#endif
#ifdef ORCJIT
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/ExecutionEngine/Orc/JITSymbol.h>
#else
#include <llvm/ExecutionEngine/JITSymbol.h>
#endif
#include <llvm/ExecutionEngine/RuntimeDyld.h>
#include <llvm/ExecutionEngine/SectionMemoryManager.h>
#include <llvm/ExecutionEngine/Orc/CompileUtils.h>
#include <llvm/ExecutionEngine/Orc/IRCompileLayer.h>
#include <llvm/ExecutionEngine/Orc/IRTransformLayer.h>
#include <llvm/ExecutionEngine/Orc/LambdaResolver.h>
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
#include <llvm/ExecutionEngine/Orc/ObjectLinkingLayer.h>
#else
#include <llvm/ExecutionEngine/Orc/RTDyldObjectLinkingLayer.h>
#endif
#include <llvm/ExecutionEngine/Orc/GlobalMappingLayer.h>
#endif

#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

using namespace llvm;
using kernel::Kernel;
using kernel::PipelineKernel;
using kernel::StreamSetBuffer;
using kernel::StreamSetBuffers;
using kernel::KernelBuilder;

CPUDriver::CPUDriver(std::string && moduleName)
: BaseDriver(std::move(moduleName))
, mTarget(nullptr)
#ifndef ORCJIT
, mEngine(nullptr)
#endif
, mPassManager{}
, mUnoptimizedIROutputStream{}
, mIROutputStream{}
, mASMOutputStream{} {

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    llvm::sys::DynamicLibrary::LoadLibraryPermanently(nullptr);

    #ifdef ORCJIT
    EngineBuilder builder;
    #else
    std::string errMessage;
    EngineBuilder builder{std::unique_ptr<Module>(mMainModule)};
    builder.setErrorStr(&errMessage);
    builder.setUseOrcMCJITReplacement(true);
    builder.setVerifyModules(false);
    builder.setEngineKind(EngineKind::JIT);
    #endif
    builder.setTargetOptions(codegen::target_Options);
    builder.setOptLevel(codegen::OptLevel);

    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        std::vector<std::string> attrs;
        for (auto &flag : HostCPUFeatures) {
            if (flag.second) {
                attrs.push_back("+" + flag.first().str());
            }
        }
        builder.setMAttrs(attrs);
    }

    mTarget = builder.selectTarget();

    if (mTarget == nullptr) {
        throw std::runtime_error("Could not selectTarget");
    }
    #ifdef ORCJIT
    mCompileLayer = make_unique<CompileLayerT>(mObjectLayer, orc::SimpleCompiler(*mTarget));
    #else
    mEngine = builder.create();
    if (mEngine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }
    #endif
    auto cache = ParabixObjectCache::getInstance();
    if (cache) {
        #ifdef ORCJIT
        #if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
        mCompileLayer->setObjectCache(cache);
        #else
        mCompileLayer->getCompiler().setObjectCache(cache);
        #endif
        #else
        mEngine->setObjectCache(cache);
        #endif
    }
    auto triple = mTarget->getTargetTriple().getTriple();
    const DataLayout DL(mTarget->createDataLayout());
    mMainModule->setTargetTriple(triple);
    mMainModule->setDataLayout(DL);
    iBuilder.reset(IDISA::GetIDISA_Builder(*mContext));
    iBuilder->setDriver(this);
    iBuilder->setModule(mMainModule);
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

std::string CPUDriver::getMangledName(std::string s) {
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
    DataLayout DL(mTarget->createDataLayout());
    std::string MangledName;
    raw_string_ostream MangledNameStream(MangledName);
    Mangler::getNameWithPrefix(MangledNameStream, s, DL);
    return MangledName;
    #else
    return s;
    #endif
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
    mPassManager->add(createDeadCodeEliminationPass());        // Eliminate any trivially dead code
    mPassManager->add(createPromoteMemoryToRegisterPass());    // Promote stack variables to constants or PHI nodes
    mPassManager->add(createCFGSimplificationPass());          // Remove dead basic blocks and unnecessary branch statements / phi nodes
    mPassManager->add(createEarlyCSEPass());                   // Simple common subexpression elimination pass
    mPassManager->add(createInstructionCombiningPass());       // Simple peephole optimizations and bit-twiddling.
    mPassManager->add(createReassociatePass());                // Canonicalizes commutative expressions
    mPassManager->add(createGVNPass());                        // Global value numbering redundant expression elimination pass
    mPassManager->add(createCFGSimplificationPass());          // Repeat CFG Simplification to "clean up" any newly found redundant phi nodes
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        mPassManager->add(createRemoveRedundantAssertionsPass());
        mPassManager->add(createDeadCodeEliminationPass());
        mPassManager->add(createCFGSimplificationPass());
    }
    if (LLVM_UNLIKELY(!codegen::TraceOption.empty())) {
        mPassManager->add(createTracePass(iBuilder.get(), codegen::TraceOption));
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
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (LLVM_UNLIKELY(codegen::ShowASMOption != codegen::OmittedOption)) {
        if (!codegen::ShowASMOption.empty()) {
            std::error_code error;
            mASMOutputStream = make_unique<raw_fd_ostream>(codegen::ShowASMOption, error, sys::fs::OpenFlags::F_None);
        } else {
            mASMOutputStream = make_unique<raw_fd_ostream>(STDERR_FILENO, false, true);
        }
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
            report_fatal_error("LLVM error: could not add emit assembly pass");
        }
    }
    #endif
}

void CPUDriver::generateUncachedKernels() {
    if (mUncachedKernel.empty()) return;
    preparePassManager();
    for (auto & kernel : mUncachedKernel) {
        kernel->prepareKernel(iBuilder);
    }
    mCachedKernel.reserve(mUncachedKernel.size());
    for (auto & kernel : mUncachedKernel) {
        kernel->generateKernel(iBuilder);
        Module * const module = kernel->getModule(); assert (module);
        module->setTargetTriple(mMainModule->getTargetTriple());
        mPassManager->run(*module);
        mCachedKernel.emplace_back(kernel.release());
    }
    mUncachedKernel.clear();
}

void * CPUDriver::finalizeObject(PipelineKernel * const pipeline) {

    #ifdef ORCJIT
    auto Resolver = llvm::orc::createLambdaResolver(
        [&](const std::string &Name) {
            auto Sym = mCompileLayer->findSymbol(Name, false);
            if (!Sym) Sym = mCompileLayer->findSymbol(getMangledName(Name), false);
            #if LLVM_VERSION_INTEGER <= LLVM_VERSION_CODE(3, 9, 1)
            if (Sym) return Sym.toRuntimeDyldSymbol();
            return RuntimeDyld::SymbolInfo(nullptr);
            #else
            if (Sym) return Sym;
            return JITSymbol(nullptr);
            #endif
        },
        [&](const std::string &Name) {
            auto SymAddr = RTDyldMemoryManager::getSymbolAddressInProcess(Name);
            if (!SymAddr) SymAddr = RTDyldMemoryManager::getSymbolAddressInProcess(getMangledName(Name));
            #if LLVM_VERSION_INTEGER <= LLVM_VERSION_CODE(3, 9, 1)
            if (SymAddr) return RuntimeDyld::SymbolInfo(SymAddr, JITSymbolFlags::Exported);
            return RuntimeDyld::SymbolInfo(nullptr);
            #else
            if (SymAddr) return JITSymbol(SymAddr, JITSymbolFlags::Exported);
            return JITSymbol(nullptr);
            #endif
        });
    #endif

    iBuilder->setModule(mMainModule);
    // write/declare the "main" method
    const auto method = pipeline->hasStaticMain() ? PipelineKernel::DeclareExternal : PipelineKernel::AddInternal;
    Function * const main = pipeline->addOrDeclareMainFunction(iBuilder, method);

    #ifdef ORCJIT
    std::vector<std::unique_ptr<Module>> moduleSet;
    moduleSet.reserve(mCachedKernel.size());
    #endif
    for (const auto & kernel : mCachedKernel) {
        if (LLVM_UNLIKELY(kernel->getModule() == nullptr)) {
            report_fatal_error(kernel->getName() + " was neither loaded from cache nor generated prior to finalizeObject");
        }
        kernel->addKernelDeclarations(iBuilder);
        #ifndef ORCJIT
        mEngine->addModule(std::unique_ptr<Module>(kernel->getModule()));
        #else
        moduleSet.push_back(std::unique_ptr<Module>(kernel->getModule()));
        #endif
    }
    mCachedKernel.clear();
    // compile any uncompiled kernel/method
    #ifndef ORCJIT
    mEngine->finalizeObject();
    #else
    moduleSet.push_back(std::unique_ptr<Module>(mMainModule));
    mCompileLayer->addModuleSet(std::move(moduleSet), make_unique<SectionMemoryManager>(), std::move(Resolver));
    #endif
    // return the compiled main method
    #ifndef ORCJIT
    return mEngine->getPointerToFunction(main);
    #else
    auto MainSym = mCompileLayer->findSymbol(getMangledName(main->getName()), false);
    assert (MainSym && "Main not found");
    return (void *)MainSym.getAddress();
    #endif
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
    return new TracePass(iBuilder.get(), to_trace);
}

