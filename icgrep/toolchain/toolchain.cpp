/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "toolchain.h"
#include <IR_Gen/idisa_target.h>
#include <llvm/CodeGen/CommandFlags.h>             // for InitTargetOptionsF...
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/Support/CommandLine.h>              // for OptionCategory
#include <llvm/Support/TargetSelect.h>             // for InitializeNativeTa...
#include <llvm/Support/raw_ostream.h>              // for errs, raw_ostream
#include <llvm/IR/LegacyPassManager.h>             // for PassManager
#include <llvm/IR/IRPrintingPasses.h>
#include <llvm/InitializePasses.h>                 // for initializeCodeGen
#ifndef NDEBUG
#include <llvm/IR/Verifier.h>
#include <boost/container/flat_set.hpp>
#endif
#include <llvm/PassRegistry.h>                     // for PassRegistry
#include <llvm/Support/CodeGen.h>                  // for Level, Level::None
#include <llvm/Support/Compiler.h>                 // for LLVM_UNLIKELY
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#include <llvm/IR/Module.h>
#include <toolchain/object_cache.h>
#include <toolchain/pipeline.h>
#include <kernels/kernel.h>
#include <sys/stat.h>

using namespace llvm;
using namespace parabix;

namespace codegen {

static cl::OptionCategory CodeGenOptions("Code Generation Options", "These options control code generation.");

static cl::bits<DebugFlags>
DebugOptions(cl::values(clEnumVal(ShowIR, "Print generated LLVM IR."),
#ifndef USE_LLVM_3_6
                        clEnumVal(ShowASM, "Print assembly code."),
#endif
                        clEnumVal(SerializeThreads, "Force segment threads to run sequentially."),
                        clEnumValEnd), cl::cat(CodeGenOptions));

static cl::opt<std::string> IROutputFilename("dump-generated-IR-output", cl::init(""), cl::desc("output IR filename"), cl::cat(CodeGenOptions));
#ifndef USE_LLVM_3_6
static cl::opt<std::string> ASMOutputFilename("asm-output", cl::init(""), cl::desc("output ASM filename"), cl::cat(CodeGenOptions));
static cl::opt<bool> AsmVerbose("asm-verbose",
                                cl::desc("Add comments to directives."),
                                cl::init(true), cl::cat(CodeGenOptions));
#endif

char OptLevel;
static cl::opt<char, true> OptLevelOption("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O1')"), cl::location(OptLevel),
                              cl::cat(CodeGenOptions), cl::Prefix, cl::ZeroOrMore, cl::init('1'));


static cl::opt<bool> EnableObjectCache("enable-object-cache", cl::init(true), cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDir("object-cache-dir", cl::init(""), cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


int BlockSize;
int SegmentSize;
int BufferSegments;
int ThreadNum;
bool EnableAsserts;
bool EnableCycleCounter;
#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

static cl::opt<int, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0), cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));
static cl::opt<int, true> SegmentSizeOption("segment-size", cl::location(SegmentSize), cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));
static cl::opt<int, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::desc("Buffer Segments"), cl::value_desc("positive integer"), cl::init(1));
static cl::opt<int, true> ThreadNumOption("thread-num", cl::location(ThreadNum), cl::desc("Number of threads used for segment pipeline parallel"), cl::value_desc("positive integer"), cl::init(2));
static cl::opt<bool, true> EnableAssertsOption("ea", cl::location(EnableAsserts), cl::desc("Enable Asserts"), cl::init(IN_DEBUG_MODE));
static cl::opt<bool, true> EnableCycleCountOption("ShowKernelCycles", cl::location(EnableCycleCounter), cl::desc("Count and report CPU cycles per kernel"), cl::init(false), cl::cat(CodeGenOptions));

const cl::OptionCategory * codegen_flags() {return &CodeGenOptions;}

bool DebugOptionIsSet(DebugFlags flag) {return DebugOptions.isSet(flag);}

static cl::opt<bool> pipelineParallel("enable-pipeline-parallel", cl::desc("Enable multithreading with pipeline parallelism."), cl::cat(CodeGenOptions));
    
static cl::opt<bool> segmentPipelineParallel("enable-segment-pipeline-parallel", cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(CodeGenOptions));

}

void printParabixVersion () {
    raw_ostream &OS = outs();
    OS << "Parabix (http://parabix.costar.sfu.ca/):\n  " << "Parabix revision " << PARABIX_VERSION << "\n";
}

void AddParabixVersionPrinter() {
    cl::AddExtraVersionPrinter(&printParabixVersion);
}

void setAllFeatures(EngineBuilder &builder) {
    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        std::vector<std::string> attrs;
        for (auto &flag : HostCPUFeatures) {
            auto enabled = flag.second ? "+" : "-";
            attrs.push_back(enabled + flag.first().str());
        }
        builder.setMAttrs(attrs);
    }
}

bool AVX2_available() {
    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        auto f = HostCPUFeatures.find("avx2");
        return ((f != HostCPUFeatures.end()) && f->second);
    }
    return false;
}

ParabixDriver::ParabixDriver(std::string && moduleName)
: mContext(new llvm::LLVMContext())
, mMainModule(new Module(moduleName, *mContext))
, iBuilder(nullptr)
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
    TargetOptions opts = InitTargetOptionsFromCodeGenFlags();
    opts.MCOptions.AsmVerbose = codegen::AsmVerbose;

    builder.setTargetOptions(opts);
    builder.setVerifyModules(IN_DEBUG_MODE);
    CodeGenOpt::Level optLevel = CodeGenOpt::Level::None;
    switch (codegen::OptLevel) {
        case '0': optLevel = CodeGenOpt::None; break;
        case '1': optLevel = CodeGenOpt::Less; break;
        case '2': optLevel = CodeGenOpt::Default; break;
        case '3': optLevel = CodeGenOpt::Aggressive; break;
        default: errs() << codegen::OptLevel << " is an invalid optimization level.\n";
    }
    builder.setOptLevel(optLevel);

    setAllFeatures(builder);
    mEngine = builder.create();
    if (mEngine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }
    mTarget = builder.selectTarget();
    if (LLVM_LIKELY(codegen::EnableObjectCache && codegen::DebugOptions.getBits() == 0)) {
        if (codegen::ObjectCacheDir.empty()) {
            mCache = new ParabixObjectCache();
        } else {
            mCache = new ParabixObjectCache(codegen::ObjectCacheDir);
        }
        assert (mCache);
        mEngine->setObjectCache(mCache);
    }

    mMainModule->setTargetTriple(mTarget->getTargetTriple().getTriple());

    iBuilder.reset(IDISA::GetIDISA_Builder(mMainModule));
    iBuilder->setDriver(this);
}

ExternalBuffer * ParabixDriver::addExternalBuffer(std::unique_ptr<ExternalBuffer> b) {
    mOwnedBuffers.emplace_back(std::move(b));
    return cast<ExternalBuffer>(mOwnedBuffers.back().get());
}

StreamSetBuffer * ParabixDriver::addBuffer(std::unique_ptr<StreamSetBuffer> b) {
    b->allocateBuffer(iBuilder.get());
    mOwnedBuffers.emplace_back(std::move(b));
    return mOwnedBuffers.back().get();
}

kernel::KernelBuilder * ParabixDriver::addKernelInstance(std::unique_ptr<kernel::KernelBuilder> kb) {
    kb->setBuilder(iBuilder.get());
    mOwnedKernels.emplace_back(std::move(kb));
    return mOwnedKernels.back().get();
}

void ParabixDriver::addKernelCall(kernel::KernelBuilder & kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs) {
    assert ("addKernelCall or makeKernelCall was already run on this kernel." && (kb.getModule() == nullptr));
    mPipeline.emplace_back(&kb);
    kb.createKernelStub(inputs, outputs);
}

void ParabixDriver::makeKernelCall(kernel::KernelBuilder * kb, const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs) {
    assert ("addKernelCall or makeKernelCall was already run on this kernel." && (kb->getModule() == nullptr));
    mPipeline.emplace_back(kb);
    kb->createKernelStub(inputs, outputs);
}

void ParabixDriver::generatePipelineIR() {
    #ifndef NDEBUG
    if (LLVM_UNLIKELY(mPipeline.empty())) {
        report_fatal_error("Pipeline must contain at least one kernel");
    } else {
        boost::container::flat_set<kernel::KernelBuilder *> K(mPipeline.begin(), mPipeline.end());
        if (LLVM_UNLIKELY(K.size() != mPipeline.size())) {
            report_fatal_error("Kernel definitions can only occur once in the pipeline");
        }
    }
    #endif

    // note: instantiation of all kernels must occur prior to initialization
    for (const auto & k : mPipeline) {
        k->addKernelDeclarations();
    }
    for (const auto & k : mPipeline) {
        k->createInstance();
    }
    for (const auto & k : mPipeline) {
        k->initializeInstance();
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
        k->finalizeInstance();
    }
}

Function * ParabixDriver::LinkFunction(Module * mod, llvm::StringRef name, FunctionType * type, void * functionPtr) const {
    assert ("addKernelCall or makeKernelCall must be called before LinkFunction" && (mod != nullptr));
    Function * f = cast<Function>(mod->getOrInsertFunction(name, type));
    mEngine->addGlobalMapping(f, functionPtr);
    return f;
}

void ParabixDriver::linkAndFinalize() {
    Module * m = mMainModule;
    #ifndef NDEBUG
    try {
    #endif
    legacy::PassManager PM;
    #ifndef NDEBUG
    PM.add(createVerifierPass());
    #endif
    PM.add(createPromoteMemoryToRegisterPass()); //Force the use of mem2reg to promote stack variables.
    PM.add(createReassociatePass());             //Reassociate expressions.
    PM.add(createGVNPass());                     //Eliminate common subexpressions.
    PM.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    PM.add(createCFGSimplificationPass());

    raw_fd_ostream * IROutputStream = nullptr;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR))) {
        if (codegen::IROutputFilename.empty()) {
            IROutputStream = new raw_fd_ostream(STDERR_FILENO, false, false);
        } else {
            std::error_code error;
            IROutputStream = new raw_fd_ostream(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None);
        }
        PM.add(createPrintModulePass(*IROutputStream));
    }

    #ifndef USE_LLVM_3_6
    raw_fd_ostream * ASMOutputStream = nullptr;
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowASM))) {
        if (codegen::ASMOutputFilename.empty()) {
            ASMOutputStream = new raw_fd_ostream(STDERR_FILENO, false, false);
        } else {
            std::error_code error;
            ASMOutputStream = new raw_fd_ostream(codegen::ASMOutputFilename, error, sys::fs::OpenFlags::F_None);
        }
        if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(PM, *ASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
            report_fatal_error("LLVM error: could not add emit assembly pass");
        }
    }
    #endif

    PM.run(*m);

    for (kernel::KernelBuilder * const kb : mPipeline) {
        m = kb->getModule();
        bool uncachedObject = true;
        if (mCache && mCache->loadCachedObjectFile(kb)) {
            uncachedObject = false;
        }
        if (uncachedObject) {
            iBuilder->setModule(kb->getModule());
            kb->generateKernel();
            PM.run(*m);
        }
        mEngine->addModule(std::unique_ptr<Module>(m));
    }    
    mEngine->finalizeObject();

    iBuilder->setModule(mMainModule);

    delete IROutputStream;
    #ifndef USE_LLVM_3_6
    delete ASMOutputStream;
    #endif
    #ifndef NDEBUG
    } catch (...) { m->dump(); throw; }
    #endif
}

void * ParabixDriver::getPointerToMain() {
    return mEngine->getPointerToNamedFunction("Main");
}

ParabixDriver::~ParabixDriver() {
    delete mCache;
}
