/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/toolchain.h>
#include <llvm/CodeGen/CommandFlags.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

namespace codegen {

static cl::OptionCategory CodeGenOptions("Code Generation Options", "These options control code generation.");

static cl::bits<DebugFlags>
DebugOptions(cl::values(clEnumVal(ShowUnoptimizedIR, "Print generated LLVM IR."),
                        clEnumVal(ShowIR, "Print optimized LLVM IR."),
                        clEnumVal(VerifyIR, "Run the IR verification pass."),
#ifndef USE_LLVM_3_6
                        clEnumVal(ShowASM, "Print assembly code."),
#endif
                        clEnumVal(SerializeThreads, "Force segment threads to run sequentially."),
                        clEnumValEnd), cl::cat(CodeGenOptions));

static cl::opt<std::string> IROutputFilenameOption("dump-generated-IR-output", cl::init(""),
                                                       cl::desc("output IR filename"), cl::cat(CodeGenOptions));

#ifndef USE_LLVM_3_6
static cl::opt<std::string> ASMOutputFilenameOption("asm-output", cl::init(""),
                                                    cl::desc("output ASM filename"), cl::cat(CodeGenOptions));

static cl::opt<bool> AsmVerbose("asm-verbose", cl::init(true),
                                cl::desc("Add comments to directives."), cl::cat(CodeGenOptions));
#endif

static cl::opt<char> OptLevelOption("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O1')"),
                                    cl::cat(CodeGenOptions), cl::Prefix, cl::ZeroOrMore, cl::init('1'));


static cl::opt<bool> EnableObjectCacheOption("enable-object-cache",
                                             cl::init(true), cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDirOption("object-cache-dir",
                                                 cl::init(""), cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


static cl::opt<int, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0),
                                          cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));


static cl::opt<int, true> SegmentSizeOption("segment-size", cl::location(SegmentSize),
                                            cl::desc("Segment Size"), cl::value_desc("positive integer"), cl::init(1));

static cl::opt<int, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::init(1),
                                               cl::desc("Buffer Segments"), cl::value_desc("positive integer"));


static cl::opt<int, true> ThreadNumOption("thread-num", cl::location(ThreadNum), cl::init(2),
                                          cl::desc("Number of threads used for segment pipeline parallel"), cl::value_desc("positive integer"));


static cl::opt<bool, true> EnableAssertsOption("ea", cl::location(EnableAsserts), cl::init(IN_DEBUG_MODE),
                                               cl::desc("Enable Asserts"));

static cl::opt<bool, true> EnableCycleCountOption("ShowKernelCycles", cl::location(EnableCycleCounter), cl::init(false),
                                                  cl::desc("Count and report CPU cycles per kernel"), cl::cat(CodeGenOptions));

static cl::opt<bool, true> pipelineParallelOption("enable-pipeline-parallel", cl::location(pipelineParallel),
                                                  cl::desc("Enable multithreading with pipeline parallelism."), cl::cat(CodeGenOptions));
    
static cl::opt<bool, true> segmentPipelineParallelOption("enable-segment-pipeline-parallel", cl::location(segmentPipelineParallel),
                                                         cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(CodeGenOptions));

static cl::opt<bool> USENVPTX("NVPTX", cl::init(false),
                              cl::desc("Run on GPU only."));

static cl::opt<int, true> GroupNumOption("group-num", cl::location(GroupNum), cl::init(256),
                                         cl::desc("NUmber of groups declared on GPU"), cl::value_desc("positive integer"));


const CodeGenOpt::Level OptLevel = [](const char optLevel) {
    switch (optLevel) {
        case '0': return CodeGenOpt::None;
        case '1': return CodeGenOpt::Less;
        case '2': return CodeGenOpt::Default;
        case '3': return CodeGenOpt::Aggressive;
        default: report_fatal_error(optLevel + " is an invalid optimization level.");
    }
}(OptLevelOption);

bool pipelineParallel;
bool segmentPipelineParallel;
const std::string ASMOutputFilename = ASMOutputFilenameOption;
const std::string IROutputFilename = IROutputFilenameOption;
const std::string ObjectCacheDir = ObjectCacheDirOption;
int BlockSize;
int SegmentSize;
int BufferSegments;
int ThreadNum;
bool EnableAsserts;
bool EnableCycleCounter;
const bool EnableObjectCache = EnableObjectCacheOption && (DebugOptions.getBits() == 0);
bool NVPTX;
int GroupNum;

const llvm::Reloc::Model RelocModel = ::RelocModel;
const llvm::CodeModel::Model CMModel = ::CMModel;
const std::string MArch = ::MArch;
const std::string RunPass = ::RunPass;
const llvm::TargetMachine::CodeGenFileType FileType = ::FileType;
const std::string StopAfter = ::StopAfter;
const std::string StartAfter = ::StartAfter;
#ifndef USE_LLVM_3_6
const TargetOptions Options = [](const bool asmVerbose) {
    TargetOptions opt = InitTargetOptionsFromCodeGenFlags();
    opt.MCOptions.AsmVerbose = AsmVerbose;
    return opt;
}(AsmVerbose);
#else
const TargetOptions Options = InitTargetOptionsFromCodeGenFlags();
#endif

const cl::OptionCategory * codegen_flags() {
    return &CodeGenOptions;
}

bool DebugOptionIsSet(const DebugFlags flag) {
    return DebugOptions.isSet(flag);
}

std::string getCPUStr() {
    return ::getCPUStr();
}

std::string getFeaturesStr() {
    return ::getFeaturesStr();
}

void setFunctionAttributes(llvm::StringRef CPU, llvm::StringRef Features, llvm::Module &M) {
    return ::setFunctionAttributes(CPU, Features, M);
}


}

void setNVPTXOption() {
    codegen::NVPTX = codegen::USENVPTX; 
    if (codegen::NVPTX) {
        #ifndef CUDA_ENABLED
        report_fatal_error("CUDA compiler is not supported.");
        #endif
    }
}

void printParabixVersion () {
    raw_ostream &OS = outs();
    OS << "Parabix (http://parabix.costar.sfu.ca/):\n  " << "Parabix revision " << PARABIX_VERSION << "\n";
}

void AddParabixVersionPrinter() {
    cl::AddExtraVersionPrinter(&printParabixVersion);
}

bool AVX2_available() {
    StringMap<bool> HostCPUFeatures;
    if (sys::getHostCPUFeatures(HostCPUFeatures)) {
        auto f = HostCPUFeatures.find("avx2");
        return ((f != HostCPUFeatures.end()) && f->second);
    }
    return false;
}
