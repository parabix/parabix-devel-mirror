/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <toolchain/toolchain.h>
#include <toolchain/pablo_toolchain.h>
#include <unicode/core/UCD_Config.h>
#include <llvm/Support/CommandLine.h>
#include <llvm/Support/Host.h>
#include <llvm/Support/raw_ostream.h>
#include <boost/interprocess/mapped_region.hpp>

using namespace llvm;

#ifndef NDEBUG
#define IN_DEBUG_MODE true
#else
#define IN_DEBUG_MODE false
#endif

// #define FORCE_ASSERTIONS

// #define DISABLE_OBJECT_CACHE

namespace codegen {

inline unsigned getPageSize() {
    return boost::interprocess::mapped_region::get_page_size();
}

static cl::OptionCategory CodeGenOptions("Code Generation Options", "These options control code generation.");

static cl::bits<DebugFlags>
DebugOptions(cl::desc("Debugging Options"), cl::values(clEnumVal(VerifyIR, "Run the IR verification pass."),
                        clEnumVal(SerializeThreads, "Force segment threads to run sequentially."),
                        clEnumVal(TraceCounts, "Trace kernel processed, consumed and produced item counts."),
                        clEnumVal(TraceDynamicBuffers, "Trace dynamic buffer allocations and deallocations."),
                        clEnumVal(TraceBlockedIO, "Trace kernels prevented from processing any strides "
                                                  "due to insufficient input items / output space."),
                        clEnumVal(TraceStridesPerSegment, "Trace number of strides executed over segments."),
                        clEnumVal(TraceProducedItemCounts, "Trace produced item count deltas over segments."),
                        clEnumVal(TraceUnconsumedItemCounts, "Trace unconsumed item counts over segments."),
                        clEnumVal(EnableAsserts, "Enable built-in Parabix framework asserts in generated IR."),
                        clEnumVal(EnableMProtect, "Use mprotect to cause a write fault when erroneously "
                                                  "overwriting kernel state / stream space."),
                        clEnumVal(EnableCycleCounter, "Count and report CPU cycles per kernel."),
                        clEnumVal(EnableBlockingIOCounter, "Count and report the number of blocked kernel "
                                                           "executions due to insufficient data/space of a "
                                                           "particular stream."),
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


// Enable Debug Options to be specified on the command line
static cl::opt<CodeGenOpt::Level, true>
OptimizationLevel("optimization-level", cl::location(OptLevel), cl::init(CodeGenOpt::None), cl::desc("Set the front-end optimization level:"),
                  cl::values(clEnumValN(CodeGenOpt::None, "none", "no optimizations (default)"),
                             clEnumValN(CodeGenOpt::Less, "less", "trivial optimizations"),
                             clEnumValN(CodeGenOpt::Default, "standard", "standard optimizations"),
                             clEnumValN(CodeGenOpt::Aggressive, "aggressive", "aggressive optimizations")
                  CL_ENUM_VAL_SENTINEL), cl::cat(CodeGenOptions));
static cl::opt<CodeGenOpt::Level, true>
BackEndOptOption("backend-optimization-level", cl::location(BackEndOptLevel), cl::init(CodeGenOpt::None), cl::desc("Set the back-end optimization level:"),
                  cl::values(clEnumValN(CodeGenOpt::None, "none", "no optimizations (default)"),
                             clEnumValN(CodeGenOpt::Less, "less", "trivial optimizations"),
                             clEnumValN(CodeGenOpt::Default, "standard", "standard optimizations"),
                             clEnumValN(CodeGenOpt::Aggressive, "aggressive", "aggressive optimizations")
                CL_ENUM_VAL_SENTINEL), cl::cat(CodeGenOptions));

static cl::opt<bool, true> EnableObjectCacheOption("enable-object-cache", cl::location(EnableObjectCache), cl::init(true),
                                                   cl::desc("Enable object caching"), cl::cat(CodeGenOptions));

static cl::opt<bool, true> TraceObjectCacheOption("trace-object-cache", cl::location(TraceObjectCache), cl::init(false),
                                                   cl::desc("Trace object cache retrieval."), cl::cat(CodeGenOptions));

static cl::opt<std::string> ObjectCacheDirOption("object-cache-dir", cl::init(""),
                                                 cl::desc("Path to the object cache diretory"), cl::cat(CodeGenOptions));


static cl::opt<int, true> FreeCallBisectOption("free-bisect-value", cl::location(FreeCallBisectLimit), cl::init(-1),
                                                    cl::desc("The number of free calls to allow in bisecting"), cl::cat(CodeGenOptions));

static cl::opt<unsigned, true> BlockSizeOption("BlockSize", cl::location(BlockSize), cl::init(0),
                                          cl::desc("specify a block size (defaults to widest SIMD register width in bits)."), cl::cat(CodeGenOptions));


const unsigned DefaultSegmentSize = 16384;
static cl::opt<unsigned, true> SegmentSizeOption("segment-size", cl::location(SegmentSize),
                                               cl::init(DefaultSegmentSize),
                                               cl::desc("Expected amount of input data to process per segment"), cl::value_desc("positive integer"), cl::cat(CodeGenOptions));

static cl::opt<unsigned, true> BufferSegmentsOption("buffer-segments", cl::location(BufferSegments), cl::init(1),
                                               cl::desc("Buffer Segments"), cl::value_desc("positive integer"));

static cl::opt<unsigned, true>
MaxTaskThreadsOption("max-task-threads", cl::location(TaskThreads),
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
                     cl::init(llvm::sys::getHostNumPhysicalCores()),
#else
                     cl::init(2),
#endif
                     cl::desc("Maximum number of threads to assign for separate pipeline tasks."),
                     cl::value_desc("positive integer"));

static cl::opt<unsigned, true>
ThreadNumOption("thread-num", cl::location(SegmentThreads),
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
                cl::init(llvm::sys::getHostNumPhysicalCores()),
#else
                cl::init(2),
#endif
                cl::desc("Number of threads used for segment pipeline parallel"),
                cl::value_desc("positive integer"));

static cl::opt<unsigned, true> ScanBlocksOption("scan-blocks", cl::location(ScanBlocks), cl::init(4),
                                          cl::desc("Number of blocks per stride for scanning kernels"), cl::value_desc("positive initeger"));

static cl::opt<unsigned, true> GroupNumOption("group-num", cl::location(GroupNum), cl::init(256),
                                         cl::desc("NUmber of groups declared on GPU"), cl::value_desc("positive integer"), cl::cat(CodeGenOptions));

std::string TraceOption = "";
static cl::opt<std::string, true> TraceValueOption("trace", cl::location(TraceOption),
                                            cl::desc("Trace the values of variables beginning with the given prefix."), cl::value_desc("prefix"), cl::cat(CodeGenOptions));

std::string CCCOption = "";
static cl::opt<std::string, true> CCTypeOption("ccc-type", cl::location(CCCOption), cl::init("binary"),
                                            cl::desc("The character class compiler"), cl::value_desc("[binary, ternary]"));

bool TimeKernelsIsEnabled;
static cl::opt<bool, true> OptCompileTime("time-kernels", cl::location(TimeKernelsIsEnabled),
                                        cl::desc("Times each kernel, printing elapsed time for each on exit"), cl::init(false));

CodeGenOpt::Level OptLevel;
CodeGenOpt::Level BackEndOptLevel;

const char * ObjectCacheDir;

unsigned BlockSize;

unsigned SegmentSize;

unsigned BufferSegments;
unsigned TaskThreads;
unsigned SegmentThreads;

unsigned ScanBlocks;

bool EnableObjectCache;
bool TraceObjectCache;

unsigned CacheDaysLimit;

int FreeCallBisectLimit;

unsigned GroupNum;

TargetOptions target_Options;

const cl::OptionCategory * LLVM_READONLY codegen_flags() {
    return &CodeGenOptions;
}

bool LLVM_READONLY DebugOptionIsSet(const DebugFlags flag) {
    #ifdef FORCE_ASSERTIONS
    if (flag == DebugFlags::EnableAsserts) return true;
    #endif
    return DebugOptions.isSet(flag);
}


std::string ProgramName;

inline bool disableObjectCacheDueToCommandLineOptions() {
    #ifdef DISABLE_OBJECT_CACHE
    return true;
    #else
    if (!TraceOption.empty()) return true;
    // if (!DebugOptions.empty()) return true;
    if (ShowIROption != OmittedOption) return true;
    if (ShowUnoptimizedIROption != OmittedOption) return true;
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (ShowASMOption != OmittedOption) return true;
    #endif
    if (pablo::ShowPabloOption != OmittedOption) return true;
    if (pablo::ShowOptimizedPabloOption != OmittedOption) return true;
    return false;
    #endif
}

void ParseCommandLineOptions(int argc, const char * const *argv, std::initializer_list<const cl::OptionCategory *> hiding) {
    AddParabixVersionPrinter();

    codegen::ProgramName = argv[0];
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    if (hiding.size() != 0) {
        cl::HideUnrelatedOptions(ArrayRef<const cl::OptionCategory *>(hiding));
    }
#endif
    cl::ParseCommandLineOptions(argc, argv);
    if (disableObjectCacheDueToCommandLineOptions()) {
        EnableObjectCache = false;
    }
    ObjectCacheDir = ObjectCacheDirOption.empty() ? nullptr : ObjectCacheDirOption.data();
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
    target_Options.MCOptions.AsmVerbose = true;
#endif
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

void setTaskThreads(unsigned taskThreads) {
    TaskThreads = std::max(taskThreads, 1u);
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
    unsigned coresPerTask = llvm::sys::getHostNumPhysicalCores()/TaskThreads;
#else
    unsigned coresPerTask = 2;  // assumption
#endif
    SegmentThreads = std::min(coresPerTask, SegmentThreads);
}

}
