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


static cl::opt<bool, true> EnableObjectCacheOption("enable-object-cache", cl::location(EnableObjectCache), cl::init(true),
                                                   cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDirOption("object-cache-dir", cl::init(""),
                                                 cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


static cl::opt<int, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0),
                                          cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));


static cl::opt<int, true> SegmentSizeOption("segment-size", cl::location(SegmentSize), cl::init(1),
                                            cl::desc("Segment Size"), cl::value_desc("positive integer"));

static cl::opt<int, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::init(1),
                                               cl::desc("Buffer Segments"), cl::value_desc("positive integer"));


static cl::opt<int, true> ThreadNumOption("thread-num", cl::location(ThreadNum), cl::init(2),
                                          cl::desc("Number of threads used for segment pipeline parallel"), cl::value_desc("positive integer"));


static cl::opt<bool, true> EnableAssertsOption("ea", cl::location(EnableAsserts), cl::init(IN_DEBUG_MODE),
                                               cl::desc("Enable Asserts"), cl::cat(CodeGenOptions));

static cl::opt<bool, true> EnableCycleCountOption("ShowKernelCycles", cl::location(EnableCycleCounter), cl::init(false),
                                             cl::desc("Count and report CPU cycles per kernel"), cl::cat(CodeGenOptions));

static cl::opt<bool, true> pipelineParallelOption("enable-pipeline-parallel", cl::location(PipelineParallel), cl::init(false),
                                                  cl::desc("Enable multithreading with pipeline parallelism."), cl::cat(CodeGenOptions));
    
static cl::opt<bool, true> segmentPipelineParallelOption("enable-segment-pipeline-parallel", cl::location(SegmentPipelineParallel),
                                                         cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(CodeGenOptions));

static cl::opt<bool, true> NVPTXOption("NVPTX", cl::location(NVPTX), cl::init(false),
                                 cl::desc("Run on GPU only."), cl::cat(CodeGenOptions));

static cl::opt<int, true> GroupNumOption("group-num", cl::location(GroupNum), cl::init(256),
                                         cl::desc("NUmber of groups declared on GPU"), cl::value_desc("positive integer"), cl::cat(CodeGenOptions));

CodeGenOpt::Level OptLevel;

bool PipelineParallel;

bool SegmentPipelineParallel;

const char * ASMOutputFilename;

const char * IROutputFilename;

const char * ObjectCacheDir;

int BlockSize;

int SegmentSize;

int BufferSegments;

int ThreadNum;

bool EnableAsserts;

bool EnableCycleCounter;

bool EnableObjectCache;

bool NVPTX = [](const bool nvptx) {
    #ifndef CUDA_ENABLED
    if (nvptx) {
        report_fatal_error("CUDA compiler is not supported.");
    }
    #endif
    return nvptx;
}(NVPTXOption);

int GroupNum;

const llvm::Reloc::Model RelocModel = ::RelocModel;

const llvm::CodeModel::Model CMModel = ::CMModel;

const std::string MArch = ::MArch;

const std::string RunPass = ::RunPass;

const llvm::TargetMachine::CodeGenFileType FileType = ::FileType;

const std::string StopAfter = ::StopAfter;

const std::string StartAfter = ::StartAfter;

TargetOptions Options;

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

std::string ProgramName;

void ParseCommandLineOptions(int argc, const char * const *argv, std::initializer_list<const cl::OptionCategory *> hiding) {
    AddParabixVersionPrinter();
    codegen::ProgramName = argv[0];
    #ifndef USE_LLVM_3_6
    if (hiding.size() != 0) {
        cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>(hiding));
    }
    #endif
    cl::ParseCommandLineOptions(argc, argv);
    if (DebugOptions.getBits()) {
        EnableObjectCache = false;
    }
    ObjectCacheDir = ObjectCacheDirOption.empty() ? nullptr : ObjectCacheDirOption.data();
    IROutputFilename = IROutputFilenameOption.empty() ? nullptr : IROutputFilenameOption.data();
    ASMOutputFilename = ASMOutputFilenameOption.empty() ? nullptr : ASMOutputFilenameOption.data();
    Options = InitTargetOptionsFromCodeGenFlags();
    #ifndef USE_LLVM_3_6
    Options.MCOptions.AsmVerbose = AsmVerbose;
    #endif
    switch (OptLevelOption) {
        case '0': OptLevel = CodeGenOpt::None; break;
        case '1': OptLevel = CodeGenOpt::Less; break;
        case '2': OptLevel = CodeGenOpt::Default; break;
        case '3': OptLevel = CodeGenOpt::Aggressive; break;
        default: report_fatal_error(std::string(1, OptLevelOption) + " is an invalid optimization level.");
    }
    #ifndef CUDA_ENABLED
    if (NVPTX) {
        report_fatal_error("CUDA compiler is not supported.");
    }
    #endif
}

}

void printParabixVersion () {
    outs() << "Parabix (http://parabix.costar.sfu.ca/):\n  " << "Parabix revision " << PARABIX_VERSION << "\n";
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
