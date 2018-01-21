/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <UCD/UCD_Config.h>
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
DebugOptions(cl::values(clEnumVal(VerifyIR, "Run the IR verification pass."),
                        clEnumVal(SerializeThreads, "Force segment threads to run sequentially."),
                        clEnumVal(TraceCounts, "Show kernel processed and produced item counts."),
                        clEnumVal(TraceDynamicBuffers, "Show dynamic buffer allocations and deallocations."),
                        clEnumVal(EnableAsserts, "Enable built-in Parabix framework asserts in generated IR."),
                        clEnumVal(EnableCycleCounter, "Count and report CPU cycles per kernel."),
                        clEnumVal(DisableIndirectBranch, "Disable use of indirect branches in kernel code.")
                        CL_ENUM_VAL_SENTINEL), cl::cat(CodeGenOptions));

std::string ShowIROption = OmittedOption;
static cl::opt<std::string, true> IROutputOption("ShowIR", cl::location(ShowIROption), cl::ValueOptional,
                                                         cl::desc("Print optimized LLVM IR to stderr (by omitting =<filename>) or a file"), cl::value_desc("filename"), cl::cat(CodeGenOptions));

std::string ShowUnoptimizedIROption = OmittedOption;
static cl::opt<std::string, true> UnoptimizedIROutputOption("ShowUnoptimizedIR", cl::location(ShowUnoptimizedIROption), cl::ValueOptional,
                                                         cl::desc("Print generated LLVM IR to stderr (by omitting =<filename> or a file"), cl::value_desc("filename"), cl::cat(CodeGenOptions));

#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
std::string ShowASMOption = OmittedOption;
static cl::opt<std::string, true> ASMOutputFilenameOption("ShowASM", cl::location(ShowASMOption), cl::ValueOptional,
                                                         cl::desc("Print generated assembly code to stderr (by omitting =<filename> or a file"), cl::value_desc("filename"), cl::cat(CodeGenOptions));
#endif

static cl::opt<char> OptLevelOption("O", cl::desc("Optimization level. [-O0, -O1, -O2, or -O3] (default = '-O1')"),
                                    cl::cat(CodeGenOptions), cl::Prefix, cl::ZeroOrMore, cl::init('1'));


static cl::opt<bool, true> EnableObjectCacheOption("enable-object-cache", cl::location(EnableObjectCache), cl::init(true),
                                                   cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDirOption("object-cache-dir", cl::init(""),
                                                 cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


static cl::opt<unsigned, true> CacheLimitOption("cache-days-limit", cl::location(CacheDaysLimit), cl::init(15),
                                          cl::desc("number of days a cache entry may be unused before auto deletion may be applied"), cl::cat(CodeGenOptions));


static cl::opt<unsigned, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0),
                                          cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));


static cl::opt<unsigned, true> SegmentSizeOption("segment-size", cl::location(SegmentSize), cl::init(8),
                                            cl::desc("Segment Size"), cl::value_desc("positive integer"));

static cl::opt<unsigned, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::init(1),
                                               cl::desc("Buffer Segments"), cl::value_desc("positive integer"));


static cl::opt<unsigned, true> ThreadNumOption("thread-num", cl::location(ThreadNum), cl::init(2),
                                          cl::desc("Number of threads used for segment pipeline parallel"), cl::value_desc("positive integer"));


static cl::opt<bool, true> segmentPipelineParallelOption("enable-segment-pipeline-parallel", cl::location(SegmentPipelineParallel),
                                                         cl::desc("Enable multithreading with segment pipeline parallelism."), cl::cat(CodeGenOptions));

static cl::opt<bool, true> NVPTXOption("NVPTX", cl::location(NVPTX), cl::init(false),
                                 cl::desc("Run on GPU only."), cl::cat(CodeGenOptions));

static cl::opt<unsigned, true> GroupNumOption("group-num", cl::location(GroupNum), cl::init(256),
                                         cl::desc("NUmber of groups declared on GPU"), cl::value_desc("positive integer"), cl::cat(CodeGenOptions));

CodeGenOpt::Level OptLevel;

bool SegmentPipelineParallel;

const char * ObjectCacheDir;

unsigned BlockSize;

unsigned SegmentSize;

unsigned BufferSegments;

unsigned ThreadNum;

bool EnableObjectCache;

unsigned CacheDaysLimit;

bool NVPTX = [](const bool nvptx) {
    #ifndef CUDA_ENABLED
    if (nvptx) {
        report_fatal_error("CUDA compiler is not supported.");
    }
    #endif
    return nvptx;
}(NVPTXOption);

unsigned GroupNum;

const llvm::Reloc::Model RelocModel = ::RelocModel;

const llvm::CodeModel::Model CMModel = ::CMModel;

const std::string MArch = ::MArch;

const llvm::TargetMachine::CodeGenFileType FileType = ::FileType;

TargetOptions Options;

const cl::OptionCategory * LLVM_READONLY codegen_flags() {
    return &CodeGenOptions;
}

bool LLVM_READONLY DebugOptionIsSet(const DebugFlags flag) {
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
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (hiding.size() != 0) {
        cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>(hiding));
    }
#endif
    cl::ParseCommandLineOptions(argc, argv);
    if (DebugOptions.getBits() || (ShowIROption != OmittedOption) || (ShowUnoptimizedIROption != OmittedOption) || (ShowASMOption != OmittedOption)
        || (pablo::ShowPabloOption != OmittedOption) || (pablo::ShowOptimizedPabloOption != OmittedOption)) {
        EnableObjectCache = false;
    }
    ObjectCacheDir = ObjectCacheDirOption.empty() ? nullptr : ObjectCacheDirOption.data();
    Options = InitTargetOptionsFromCodeGenFlags();
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    Options.MCOptions.AsmVerbose = true;
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
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(6, 0, 0)
void printParabixVersion () {
    outs() << "Unicode version " << UCD::UnicodeVersion << "\n";
    outs() << "Parabix (http://parabix.costar.sfu.ca/):\n  " << "Parabix revision " << PARABIX_VERSION << "\n";
}
#else
void printParabixVersion (raw_ostream & outs) {
    outs << "Unicode version " << UCD::UnicodeVersion << "\n";
    outs << "Parabix (http://parabix.costar.sfu.ca/):\n  " << "Parabix revision " << PARABIX_VERSION << "\n";
}
#endif

void AddParabixVersionPrinter() {
    cl::AddExtraVersionPrinter(&printParabixVersion);
}
