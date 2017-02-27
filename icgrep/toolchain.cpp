/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "toolchain.h"
#include <llvm/CodeGen/CommandFlags.h>             // for InitTargetOptionsF...
#include <llvm/ExecutionEngine/ExecutionEngine.h>  // for EngineBuilder
#include <llvm/Support/CommandLine.h>              // for OptionCategory
#include <llvm/Support/TargetSelect.h>             // for InitializeNativeTa...
#include <llvm/Support/raw_ostream.h>              // for errs, raw_ostream
#include <llvm/ADT/SmallString.h>                  // for SmallString
#include <llvm/IR/LegacyPassManager.h>             // for PassManager
#include <llvm/InitializePasses.h>                 // for initializeCodeGen
#include <llvm/PassRegistry.h>                     // for PassRegistry
#include <llvm/Support/CodeGen.h>                  // for Level, Level::None
#include <llvm/Support/Compiler.h>                 // for LLVM_UNLIKELY
#include <llvm/Target/TargetMachine.h>             // for TargetMachine, Tar...
#include <llvm/Target/TargetOptions.h>             // for TargetOptions
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#ifndef NDEBUG
#include <llvm/IR/Verifier.h>
#endif
#include <object_cache.h>
namespace llvm { class Module; }
#ifdef CUDA_ENABLED
#include <IR_Gen/llvm2ptx.h>
#endif
 
using namespace llvm;

namespace codegen {

static cl::OptionCategory CodeGenOptions("Code Generation Options", "These options control code generation.");

static cl::bits<DebugFlags>
DebugOptions(cl::values(clEnumVal(ShowIR, "Print generated LLVM IR."),
#if LLVM_VERSION_MINOR > 6
                        clEnumVal(ShowASM, "Print assembly code."),
#endif
                        clEnumVal(SerializeThreads, "Force segment threads to run sequentially."),
                        clEnumValEnd), cl::cat(CodeGenOptions));

static cl::opt<std::string> IROutputFilename("dump-generated-IR-output", cl::init(""), cl::desc("output IR filename"), cl::cat(CodeGenOptions));
#if LLVM_VERSION_MINOR > 6
static cl::opt<std::string> ASMOutputFilename("asm-output", cl::init(""), cl::desc("output ASM filename"), cl::cat(CodeGenOptions));
static cl::opt<bool> AsmVerbose("asm-verbose",
                                cl::desc("Add comments to directives."),
                                cl::init(true), cl::cat(CodeGenOptions));
#endif

char OptLevel;
static cl::opt<char, true> OptLevelOption("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O1')"), cl::location(OptLevel),
                              cl::cat(CodeGenOptions), cl::Prefix, cl::ZeroOrMore, cl::init('1'));


static cl::opt<bool> EnableObjectCache("enable-object-cache", cl::init(false), cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDir("object-cache-dir", cl::init(""), cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


int BlockSize;
int SegmentSize;
int BufferSegments;
int ThreadNum;
bool EnableAsserts;

static cl::opt<int, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0), cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));
static cl::opt<int, true> SegmentSizeOption("segment-size", cl::location(SegmentSize), cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));
static cl::opt<int, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::desc("Buffer Segments"), cl::value_desc("positive integer"), cl::init(1));
static cl::opt<int, true> ThreadNumOption("thread-num", cl::location(ThreadNum), cl::desc("Number of threads used for segment pipeline parallel"), cl::value_desc("positive integer"), cl::init(2));

static cl::opt<bool, true> EnableAssertsOption("ea", cl::location(EnableAsserts), cl::desc("Enable Asserts"), cl::init(
#ifndef NDEBUG
true
#else
false
#endif
));

const cl::OptionCategory * codegen_flags() {return &CodeGenOptions;}

bool DebugOptionIsSet(DebugFlags flag) {return DebugOptions.isSet(flag);}

    
#ifdef CUDA_ENABLED
bool NVPTX;
int GroupNum;
static cl::opt<bool> USENVPTX("NVPTX", cl::desc("Run on GPU only."), cl::init(false));
static cl::opt<int, true> GroupNumOption("group-num", cl::location(GroupNum), cl::desc("NUmber of groups declared on GPU"), cl::value_desc("positive integer"), cl::init(256));
#endif

}


#ifdef CUDA_ENABLED
void setNVPTXOption(){
    codegen::NVPTX = codegen::USENVPTX;
}

void Compile2PTX (Module * m, std::string IRFilename, std::string PTXFilename) {
    InitializeAllTargets();
    InitializeAllTargetMCs();
    InitializeAllAsmPrinters();
    InitializeAllAsmParsers();

    PassRegistry *Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLoopStrengthReducePass(*Registry);
    initializeLowerIntrinsicsPass(*Registry);
    initializeUnreachableBlockElimPass(*Registry);

    std::error_code error;
    raw_fd_ostream out(IRFilename, error, sys::fs::OpenFlags::F_None);
    m->print(out, nullptr);

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR)))
            m->dump();

    llvm2ptx(IRFilename, PTXFilename);
}
#endif


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

#ifndef USE_LLVM_3_6
void WriteAssembly (TargetMachine *TM, Module * m) {
    legacy::PassManager PM;

    SmallString<128> Str;
    raw_svector_ostream dest(Str);

    if (TM->addPassesToEmitFile(PM, dest, TargetMachine::CGFT_AssemblyFile ) ) {
        throw std::runtime_error("LLVM error: addPassesToEmitFile failed.");
    }
    PM.run(*m);

    if (codegen::ASMOutputFilename.empty()) {
        errs() << Str;
    } else {
        std::error_code error;
        raw_fd_ostream out(codegen::ASMOutputFilename, error, sys::fs::OpenFlags::F_None);
        out << Str;
    }
}
#endif

ExecutionEngine * JIT_to_ExecutionEngine (Module * m) {

    // Use the pass manager to optimize the function.
    legacy::PassManager PM;
    #ifndef NDEBUG
    PM.add(createVerifierPass());
    #endif
    PM.add(createReassociatePass());             //Reassociate expressions.
    PM.add(createGVNPass());                     //Eliminate common subexpressions.
    PM.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    PM.add(createCFGSimplificationPass());
    PM.run(*m);

    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();

    PassRegistry * Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLowerIntrinsicsPass(*Registry);

    std::string errMessage;
    EngineBuilder builder{std::unique_ptr<Module>(m)};
    builder.setErrorStr(&errMessage);
    TargetOptions opts = InitTargetOptionsFromCodeGenFlags();
    opts.MCOptions.AsmVerbose = codegen::AsmVerbose;

    builder.setTargetOptions(opts);
    builder.setVerifyModules(true);
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

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR))) {
        if (codegen::IROutputFilename.empty()) {
            m->dump();
        } else {
            std::error_code error;
            raw_fd_ostream out(codegen::IROutputFilename, error, sys::fs::OpenFlags::F_None);
            m->print(out, nullptr);
        }
    }
#if LLVM_VERSION_MINOR > 6
    if (codegen::DebugOptionIsSet(codegen::ShowASM)) {
      WriteAssembly(builder.selectTarget(), m);
    }
#endif
    ExecutionEngine * engine = builder.create();
    if (engine == nullptr) {
        throw std::runtime_error("Could not create ExecutionEngine: " + errMessage);
    }    
    return engine;
}

void ApplyObjectCache(ExecutionEngine * e) {
    ICGrepObjectCache * cache = nullptr;
    if (codegen::EnableObjectCache) {
        if (codegen::ObjectCacheDir.empty())
            // Default is $HOME/.cache/icgrep
            cache = new ICGrepObjectCache();
        else
            cache = new ICGrepObjectCache(codegen::ObjectCacheDir);
        e->setObjectCache(cache);
    }    
}


