#include <grep/grep_toolchain.h>
#include <llvm/Support/CommandLine.h>

using namespace llvm;

constexpr auto DefaultByteCClimit = 6;

namespace grep {

int Threads;
static cl::opt<int, true> OptThreads("t", cl::location(Threads),
                                     cl::desc("Total number of threads."), cl::init(2));

bool PabloTransposition;
static cl::opt<bool, true> OptPabloTransposition("enable-pablo-s2p", cl::location(PabloTransposition),
                                                 cl::desc("Enable experimental pablo transposition."), cl::init(false));

bool SplitTransposition;
static cl::opt<bool, true> OptSplitTransposition("enable-split-s2p", cl::location(SplitTransposition),
                                                 cl::desc("Enable experimental split transposition."), cl::init(false));

bool UnicodeIndexing;
static cl::opt<bool, true> OptUnicodeIndexing("UnicodeIndexing", cl::location(UnicodeIndexing),
                                              cl::desc("Enable CC multiplexing and Unicode indexing."), cl::init(false));

bool PropertyKernels;
static cl::opt<bool, true> OptPropertyKernels("enable-property-kernels", cl::location(PropertyKernels),
                                              cl::desc("Enable Unicode property kernels."), cl::init(true));

bool MultithreadedSimpleRE;
static cl::opt<bool, true> OptMultithreadedSimpleRE("enable-simple-RE-kernels", cl::location(MultithreadedSimpleRE),
                                                    cl::desc("Enable individual CC kernels for simple REs."), cl::init(false));

int ScanMatchBlocks;
static cl::opt<int, true> OptScanMatchBlocks("scanmatch-blocks", cl::location(ScanMatchBlocks),
                                             cl::desc("Scanmatch blocks per stride"), cl::init(4));

int MatchCoordinateBlocks;
static cl::opt<int, true> OptMatchCoordinateBlocks("match-coordinates", cl::location(MatchCoordinateBlocks),
                                                   cl::desc("Enable experimental MatchCoordinates kernels with a given number of blocks per stride"), cl::init(0));

unsigned ByteCClimit;
static cl::opt<unsigned, true> OptByteCClimit("byte-CC-limit", cl::location(ByteCClimit),
                                              cl::desc("Max number of CCs for byte CC pipeline."), cl::init(DefaultByteCClimit));
bool TraceFiles;
static cl::opt<bool, true> OptTraceFiles("TraceFiles", cl::location(TraceFiles),
                                         cl::desc("Report files as they are opened."), cl::init(false));

}
