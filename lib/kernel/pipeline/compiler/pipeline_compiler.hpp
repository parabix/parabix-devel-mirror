#ifndef PIPELINE_COMPILER_HPP
#define PIPELINE_COMPILER_HPP

#include <kernel/pipeline/pipeline_kernel.h>
#include <kernel/core/kernel_compiler.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Transforms/Utils/Local.h>
#include <llvm/ADT/STLExtras.h>
#include "analysis/pipeline_analysis.hpp"
#include <boost/multi_array.hpp>
#include <boost/intrusive/detail/math.hpp>
#include <boost/utility/value_init.hpp>
#include "config.h"

using namespace boost;
using namespace boost::math;
using namespace boost::adaptors;
using boost::container::flat_set;
using boost::container::flat_map;
using boost::intrusive::detail::floor_log2;
using boost::intrusive::detail::ceil_log2;
using boost::intrusive::detail::ceil_pow2;
using boost::intrusive::detail::is_pow2;
using namespace llvm;


// TODO: merge Cycle counter and PAPI?

namespace kernel {

enum CycleCounter {
  KERNEL_SYNCHRONIZATION
  , PARTITION_JUMP_SYNCHRONIZATION
  , BUFFER_EXPANSION  
  , BUFFER_COPY
  , KERNEL_EXECUTION
  , TOTAL_TIME
  // ------------------
  , NUM_OF_CYCLE_COUNTERS
};

#ifdef ENABLE_PAPI
enum PAPIKernelCounter {
  PAPI_KERNEL_SYNCHRONIZATION
  , PAPI_PARTITION_JUMP_SYNCHRONIZATION
  , PAPI_BUFFER_EXPANSION
  , PAPI_BUFFER_COPY
  , PAPI_KERNEL_EXECUTION
  , PAPI_KERNEL_TOTAL
  // ------------------
  , NUM_OF_PAPI_COUNTERS
};
#endif

const static std::string BASE_THREAD_LOCAL_STREAMSET_MEMORY = "BLSM";

const static std::string EXPECTED_NUM_OF_STRIDES_MULTIPLIER = "EnSM";

const static std::string ZERO_EXTENDED_BUFFER = "ZeB";
const static std::string ZERO_EXTENDED_SPACE = "ZeS";

const static std::string KERNEL_THREAD_LOCAL_SUFFIX = ".KTL";
#ifndef USE_FIXED_SEGMENT_NUMBER_INCREMENTS
const static std::string NEXT_LOGICAL_SEGMENT_NUMBER = "@NLSN";
#endif
const static std::string LOGICAL_SEGMENT_SUFFIX = ".LSN";

const static std::string DEBUG_FD = ".DFd";

const static std::string ITERATION_COUNT_SUFFIX = ".ITC";
const static std::string TERMINATION_PREFIX = "@TERM";
const static std::string CONSUMER_TERMINATION_COUNT_PREFIX = "@PTC";
const static std::string ITEM_COUNT_SUFFIX = ".IN";
const static std::string DEFERRED_ITEM_COUNT_SUFFIX = ".DC";
const static std::string EXTERNAL_IO_PRIOR_ITEM_COUNT_SUFFIX = ".EP";
const static std::string CONSUMED_ITEM_COUNT_SUFFIX = ".CON";
const static std::string DEBUG_CONSUMED_ITEM_COUNT_SUFFIX = ".DCON";

const static std::string STATISTICS_CYCLE_COUNT_SUFFIX = ".SCY";
#ifdef ENABLE_PAPI
const static std::string STATISTICS_PAPI_COUNT_ARRAY_SUFFIX = ".PCS";
const static std::string STATISTICS_GLOBAL_PAPI_COUNT_ARRAY = "!PCS";
const static std::string STATISTICS_THREAD_LOCAL_PAPI_COUNT_ARRAY = "tPCS";
#endif
const static std::string STATISTICS_SEGMENT_COUNT_SUFFIX = ".SSC";
const static std::string STATISTICS_BLOCKING_IO_SUFFIX = ".SBY";
const static std::string STATISTICS_BLOCKING_IO_HISTORY_SUFFIX = ".SHY";
const static std::string STATISTICS_BUFFER_EXPANSION_SUFFIX = ".SBX";
const static std::string STATISTICS_STRIDES_PER_SEGMENT_SUFFIX = ".SSPS";
const static std::string STATISTICS_PRODUCED_ITEM_COUNT_SUFFIX = ".SPIC";
const static std::string STATISTICS_UNCONSUMED_ITEM_COUNT_SUFFIX = ".SUIC";

const static std::string LAST_GOOD_VIRTUAL_BASE_ADDRESS = ".LGA";

using ArgVec = Vec<Value *, 64>;

using BufferPortMap = flat_set<std::pair<unsigned, unsigned>>;

using PartitionJumpPhiOutMap = flat_map<std::pair<unsigned, unsigned>, Value *>;

using PartitionPhiNodeTable = multi_array<PHINode *, 2>;

class PipelineCompiler final : public KernelCompiler, public PipelineCommonGraphFunctions {

    template<typename T> friend struct OutputPortVec;

    enum { WITH_OVERFLOW = 0, WITHOUT_OVERFLOW = 1};
    using OverflowItemCounts = Vec<std::array<Value *, 2>, 8>;

public:

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel);

    void generateImplicitKernels(BuilderRef b);
    void addPipelineKernelProperties(BuilderRef b);
    void constructStreamSetBuffers(BuilderRef b) override;
    void generateInitializeMethod(BuilderRef b);
    void generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides);
    void generateInitializeThreadLocalMethod(BuilderRef b);
    void generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * expectedNumOfStrides);
    void generateKernelMethod(BuilderRef b);
    void generateFinalizeMethod(BuilderRef b);
    void generateFinalizeThreadLocalMethod(BuilderRef b);
    std::vector<Value *> getFinalOutputScalars(BuilderRef b) override;
    void runOptimizationPasses(BuilderRef b);

    static void linkPThreadLibrary(BuilderRef b);
    #ifdef ENABLE_PAPI
    static void linkPAPILibrary(BuilderRef b);
    #endif

private:

    PipelineCompiler(PipelineKernel * const pipelineKernel, PipelineAnalysis && P);

// internal pipeline state construction functions

public:

    void addPipelinePriorItemCountProperties(BuilderRef b);
    void addInternalKernelProperties(BuilderRef b, const unsigned kernelId);
    void generateSingleThreadKernelMethod(BuilderRef b);
    void generateMultiThreadKernelMethod(BuilderRef b);

// main doSegment functions

    void start(BuilderRef b);
    void setActiveKernel(BuilderRef b, const unsigned index, const bool allowThreadLocal);
    void executeKernel(BuilderRef b);
    void end(BuilderRef b);

    void readPipelineIOItemCounts(BuilderRef b);
    void writeExternalProducedItemCounts(BuilderRef b);

// internal pipeline functions

    LLVM_READNONE StructType * getThreadStuctType(BuilderRef b) const;
    Value * constructThreadStructObject(BuilderRef b, Value * const threadId, Value * const threadLocal, const unsigned threadNum);
    void readThreadStuctObject(BuilderRef b, Value * threadState);
    void deallocateThreadState(BuilderRef b, Value * const threadState);

    void allocateThreadLocalState(BuilderRef b, Value * const localState, Value * const threadId = nullptr);
    void createThreadStateForSingleThread(BuilderRef b);
    void deallocateThreadLocalState(BuilderRef b, Value * const localState);
    Value * readTerminationSignalFromLocalState(BuilderRef b, Value * const threadState) const;
    inline Value * isProcessThread(BuilderRef b, Value * const threadState) const;

// partitioning codegen functions

    void makePartitionEntryPoints(BuilderRef b);
    void branchToInitialPartition(BuilderRef b);
    BasicBlock * getPartitionExitPoint(BuilderRef b);
    void checkForPartitionEntry(BuilderRef b);

    void identifyPartitionKernelRange();
    void determinePartitionStrideRateScalingFactor();

    void writePartitionEntryIOGuard(BuilderRef b);
    Value * calculatePartitionSegmentLength(BuilderRef b);

    void loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const;

    void phiOutPartitionItemCounts(BuilderRef b, const unsigned kernel, const unsigned targetPartitionId, const bool fromKernelEntryBlock, BasicBlock * const entryPoint);
    void phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId, const bool fromKernelEntryBlock, BasicBlock * const entryPoint);

    Value * acquireAndReleaseAllSynchronizationLocksUntil(BuilderRef b, const unsigned partitionId);

    void writeInitiallyTerminatedPartitionExit(BuilderRef b);
    void checkForPartitionExit(BuilderRef b);

// inter-kernel codegen functions

    void readProcessedItemCounts(BuilderRef b);
    void readProducedItemCounts(BuilderRef b);

    void initializeKernelLoopEntryPhis(BuilderRef b);
    void initializeKernelCheckOutputSpacePhis(BuilderRef b);
    void initializeKernelTerminatedPhis(BuilderRef b);
    void initializeJumpToNextUsefulPartitionPhis(BuilderRef b);
    void initializeKernelInsufficientIOExitPhis(BuilderRef b);
    void initializeKernelLoopExitPhis(BuilderRef b);
    void initializeKernelExitPhis(BuilderRef b);

    void detemineMaximumNumberOfStrides(BuilderRef b);
    void determineNumOfLinearStrides(BuilderRef b);
    void checkForSufficientInputData(BuilderRef b, const BufferPort & inputPort, const unsigned streamSet);
    void ensureSufficientOutputSpace(BuilderRef b, const BufferPort & port, const unsigned streamSet);
    void updatePHINodesForLoopExit(BuilderRef b);

    Value * calculateTransferableItemCounts(BuilderRef b, Value * const numOfLinearStrides);

    enum class InputExhaustionReturnType {
        Conjunction, Disjunction
    };

    Value * checkIfInputIsExhausted(BuilderRef b, InputExhaustionReturnType returnValType);
    void determineIsFinal(BuilderRef b, Value * const numOfLinearStrides);
    Value * hasMoreInput(BuilderRef b);

    struct FinalItemCount {
        Value * minFixedRateFactor;
        Value * partialPartitionStrides;
    };

    void calculateFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems, Value *& minFixedRateFactor, Value *& finalStrideRemainder);

    void zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddresses);

    Value * allocateLocalZeroExtensionSpace(BuilderRef b, BasicBlock * const insertBefore) const;

    void writeKernelCall(BuilderRef b);
    ArgVec buildKernelCallArgumentList(BuilderRef b);
    void updateProcessedAndProducedItemCounts(BuilderRef b);
    void readReturnedOutputVirtualBaseAddresses(BuilderRef b) const;
    Value * addVirtualBaseAddressArg(BuilderRef b, const StreamSetBuffer * buffer, ArgVec & args);

    void normalCompletionCheck(BuilderRef b);

    void writeInsufficientIOExit(BuilderRef b);
    void writeJumpToNextPartition(BuilderRef b);

    void writeCopyBackLogic(BuilderRef b);
    void writeLookAheadLogic(BuilderRef b);
    void writeLookBehindLogic(BuilderRef b);
    void writeDelayReflectionLogic(BuilderRef b);
    enum class CopyMode { CopyBack, LookAhead, LookBehind, Delay };
    void copy(BuilderRef b, const CopyMode mode, Value * cond, const StreamSetPort outputPort, const StreamSetBuffer * const buffer, const unsigned itemsToCopy);


    void computeFullyProcessedItemCounts(BuilderRef b, Value * const terminated);
    void computeFullyProducedItemCounts(BuilderRef b, Value * const terminated);
    Value * computeFullyProducedItemCount(BuilderRef b, const size_t kernel, const StreamSetPort port, Value * produced, Value * const terminationSignal);

    void updateKernelExitPhisAfterInitiallyTerminated(BuilderRef b);
    void updatePhisAfterTermination(BuilderRef b);

    void clearUnwrittenOutputData(BuilderRef b);

    void computeMinimumConsumedItemCounts(BuilderRef b);
    void writeConsumedItemCounts(BuilderRef b);
    void recordFinalProducedItemCounts(BuilderRef b);
    void writeUpdatedItemCounts(BuilderRef b);

    void writeOutputScalars(BuilderRef b, const size_t index, std::vector<Value *> & args);
    Value * getScalar(BuilderRef b, const size_t index);

// intra-kernel codegen functions

    Value * getInputStrideLength(BuilderRef b, const BufferPort &inputPort);
    Value * getOutputStrideLength(BuilderRef b, const BufferPort &outputPort);
    Value * calculateStrideLength(BuilderRef b, const BufferPort & port, Value * const previouslyTransferred, Value * const strideIndex);
    Value * calculateNumOfLinearItems(BuilderRef b, const BufferPort &port, Value * const adjustment);
    Value * getAccessibleInputItems(BuilderRef b, const BufferPort & inputPort, const bool useOverflow = true);
    Value * getNumOfAccessibleStrides(BuilderRef b, const BufferPort & inputPort, Value * const numOfLinearStrides);
    Value * getWritableOutputItems(BuilderRef b, const BufferPort & outputPort, const bool useOverflow = true);
    Value * getNumOfWritableStrides(BuilderRef b, const BufferPort & port, Value * const numOfLinearStrides);
    Value * addLookahead(BuilderRef b, const BufferPort & inputPort, Value * const itemCount) const;
    Value * subtractLookahead(BuilderRef b, const BufferPort & inputPort, Value * const itemCount);
    Value * truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * const terminationSignal) const;

    void initializeLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    void updateLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    Value * getLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort) const;
    void setLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort, Value * const available);

    Value * getPartialSumItemCount(BuilderRef b, const BufferPort &port, Value * const previouslyTransferred, Value * const offset) const;
    Value * getMaximumNumOfPartialSumStrides(BuilderRef b, const BufferPort &port, Value * const numOfLinearStrides);

// termination codegen functions

    void addTerminationProperties(BuilderRef b, const size_t kernel);
    void initializePipelineInputTerminationSignal(BuilderRef b);
    void setCurrentTerminationSignal(BuilderRef b, Value * const signal);
    Value * getCurrentTerminationSignal() const;
    Value * hasKernelTerminated(BuilderRef b, const size_t kernel, const bool normally = false) const;
    Value * isClosed(BuilderRef b, const StreamSetPort inputPort) const;
    Value * isClosed(BuilderRef b, const unsigned streamSet) const;

    Value * isClosedNormally(BuilderRef b, const StreamSetPort inputPort) const;
    bool kernelCanTerminateAbnormally(const unsigned kernel) const;
    void checkIfKernelIsAlreadyTerminated(BuilderRef b);
    Value * readTerminationSignal(BuilderRef b, const unsigned kernelId);
    void writeTerminationSignal(BuilderRef b, Value * const signal);
    Value * hasPipelineTerminated(BuilderRef b) const;
    void signalAbnormalTermination(BuilderRef b);
    LLVM_READNONE static Constant * getTerminationSignal(BuilderRef b, const TerminationSignal type);

    void readCountableItemCountsAfterAbnormalTermination(BuilderRef b);
    void informInputKernelsOfTermination(BuilderRef b);
    void verifyPostInvocationTerminationSignal(BuilderRef b);

// consumer codegen functions

    void addConsumerKernelProperties(BuilderRef b, const unsigned producer);
    void initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced);
    void initializePipelineInputConsumedPhiNodes(BuilderRef b);
    void readExternalConsumerItemCounts(BuilderRef b);
    void createConsumedPhiNodes(BuilderRef b);
    void phiOutConsumedItemCountsAfterInitiallyTerminated(BuilderRef b);
    void readConsumedItemCounts(BuilderRef b);
    Value * readConsumedItemCount(BuilderRef b, const size_t streamSet, const bool useFinalCount = false);
    void setConsumedItemCount(BuilderRef b, const size_t bufferVertex, not_null<Value *> consumed, const unsigned slot) const;
    void writeExternalConsumedItemCounts(BuilderRef b);

// buffer management codegen functions

    void addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index);
    void allocateOwnedBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal);
    void loadExternalStreamSetHandles(BuilderRef b);
    void loadInternalStreamSetHandles(BuilderRef b, const bool nonLocal);
    void releaseOwnedBuffers(BuilderRef b, const bool nonLocal);
    void resetInternalBufferHandles();
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffers(BuilderRef b, const size_t kernelId) const;

    void prepareLinearThreadLocalOutputBuffers(BuilderRef b);
    Value * getVirtualBaseAddress(BuilderRef b, const BufferPort & rateData, const BufferNode & bn, Value * position, Value * isFinal) const;
    void getInputVirtualBaseAddresses(BuilderRef b, Vec<Value *> & baseAddresses) const;
    void getZeroExtendedInputVirtualBaseAddresses(BuilderRef b, const Vec<Value *> & baseAddresses, Value * const zeroExtensionSpace, Vec<Value *> & zeroExtendedVirtualBaseAddress) const;

// cycle counter functions

    void addCycleCounterProperties(BuilderRef b, const unsigned kernel, const bool isRoot);
    Value * startCycleCounter(BuilderRef b);
    void updateCycleCounter(BuilderRef b, const unsigned kernelId, Value * const start, const CycleCounter type) const;


    void incrementNumberOfSegmentsCounter(BuilderRef b) const;
    void recordBlockingIO(BuilderRef b, const StreamSetPort port) const;

    void printOptionalCycleCounter(BuilderRef b);
    StreamSetPort selectPrincipleCycleCountBinding(const unsigned kernel) const;
    void printOptionalBlockingIOStatistics(BuilderRef b);


    void initializeBufferExpansionHistory(BuilderRef b) const;
    void recordBufferExpansionHistory(BuilderRef b, const StreamSetPort outputPort, const StreamSetBuffer * const buffer) const;
    void printOptionalBufferExpansionHistory(BuilderRef b);

    void initializeStridesPerSegment(BuilderRef b) const;
    void recordStridesPerSegment(BuilderRef b) const;
    void printOptionalStridesPerSegment(BuilderRef b) const;
    void printOptionalBlockedIOPerSegment(BuilderRef b) const;

    void addProducedItemCountDeltaProperties(BuilderRef b, unsigned kernel) const;
    void recordProducedItemCountDeltas(BuilderRef b) const;
    void printProducedItemCountDeltas(BuilderRef b) const;

    void addUnconsumedItemCountProperties(BuilderRef b, unsigned kernel) const;
    void recordUnconsumedItemCounts(BuilderRef b) const;
    void printUnconsumedItemCounts(BuilderRef b) const;

    void addItemCountDeltaProperties(BuilderRef b, const unsigned kernel, const StringRef suffix) const;

    void recordItemCountDeltas(BuilderRef b, const Vec<Value *> & current, const Vec<Value *> & prior, const StringRef suffix) const;

    void printItemCountDeltas(BuilderRef b, const StringRef title, const StringRef suffix) const;

// internal optimization passes

    void simplifyPhiNodes(Module * const m) const;
    void replacePhiCatchWithCurrentBlock(BuilderRef b, BasicBlock *& toReplace, BasicBlock * const phiContainer);

// kernel config functions

    bool isBounded() const;
    bool requiresExplicitFinalStride() const ;
    bool mayLoopBackToEntry() const;
    void identifyPipelineInputs(const unsigned kernelId);
    void identifyLocalPortIds(const unsigned kernelId);
    bool hasExternalIO(const size_t kernel) const;

// synchronization functions

    void identifyAllInternallySynchronizedKernels();
    void readFirstSegmentNumber(BuilderRef b);
    void obtainCurrentSegmentNumber(BuilderRef b, BasicBlock * const entryBlock);
    void incrementCurrentSegNo(BuilderRef b, BasicBlock * const exitBlock);
    void acquireSynchronizationLock(BuilderRef b, const unsigned kernelId);
    void releaseSynchronizationLock(BuilderRef b, const unsigned kernelId);
    void verifyCurrentSynchronizationLock(BuilderRef b) const;

// family functions

    void addFamilyKernelProperties(BuilderRef b, const unsigned index) const;

    void bindFamilyInitializationArguments(BuilderRef b, ArgIterator & arg, const ArgIterator & arg_end) const override;

// thread local functions

    Value * getThreadLocalHandlePtr(BuilderRef b, const unsigned kernelIndex) const;

// papi instrumentation functions
#ifdef ENABLE_PAPI
    void convertPAPIEventNamesToCodes();
    void addPAPIEventCounterPipelineProperties(BuilderRef b);
    void addPAPIEventCounterKernelProperties(BuilderRef b, const unsigned kernel, const bool isRoot);
    void initializePAPI(BuilderRef b) const;
    void registerPAPIThread(BuilderRef b) const;
    void createEventSetAndStartPAPI(BuilderRef b);
    void readPAPIMeasurement(BuilderRef b, const unsigned kernelId, Value * const measurementArray) const;
    void accumPAPIMeasurementWithoutReset(BuilderRef b, Value * const beforeMeasurement, const unsigned kernelId, const PAPIKernelCounter measurementType) const;
    void unregisterPAPIThread(BuilderRef b) const;
    void stopPAPIAndDestroyEventSet(BuilderRef b);
    void shutdownPAPI(BuilderRef b) const;
    void accumulateFinalPAPICounters(BuilderRef b);
    void printPAPIReportIfRequested(BuilderRef b);
    void checkPAPIRetValAndExitOnError(BuilderRef b, StringRef source, const int expected, Value * const retVal) const;

#endif
// debug message functions

    #ifdef PRINT_DEBUG_MESSAGES
    void debugInit(BuilderRef b);
    template <typename ... Args>
    void debugPrint(BuilderRef b, Twine format, Args ...args) const;
    void debugHalt(BuilderRef b) const;
    void debugResume(BuilderRef b) const;
    void debugClose(BuilderRef b);
    #endif

// misc. functions

    Value * getFamilyFunctionFromKernelState(BuilderRef b, Type * const type, const std::string &suffix) const;
    Value * callKernelInitializeFunction(BuilderRef b, const ArgVec & args) const;
    Value * getKernelAllocateSharedInternalStreamSetsFunction(BuilderRef b) const;
    Value * callKernelInitializeThreadLocalFunction(BuilderRef b, Value * handle) const;
    Value * getKernelAllocateThreadLocalInternalStreamSetsFunction(BuilderRef b) const;
    Value * getKernelDoSegmentFunction(BuilderRef b) const;
    Value * callKernelFinalizeThreadLocalFunction(BuilderRef b, const SmallVector<Value *, 2> & args) const;
    Value * callKernelFinalizeFunction(BuilderRef b, const SmallVector<Value *, 1> & args) const;

    LLVM_READNONE std::string makeKernelName(const size_t kernelIndex) const;
    LLVM_READNONE std::string makeBufferName(const size_t kernelIndex, const StreamSetPort port) const;

    using PipelineCommonGraphFunctions::getReference;

    const StreamSetPort getReference(const StreamSetPort port) const;

    using PipelineCommonGraphFunctions::getInputBufferVertex;
    using PipelineCommonGraphFunctions::getInputBuffer;
    using PipelineCommonGraphFunctions::getInputBinding;

    unsigned getInputBufferVertex(const StreamSetPort inputPort) const;
    StreamSetBuffer * getInputBuffer(const StreamSetPort inputPort) const;
    const Binding & getInputBinding(const StreamSetPort inputPort) const;

    using PipelineCommonGraphFunctions::getOutputBufferVertex;
    using PipelineCommonGraphFunctions::getOutputBuffer;
    using PipelineCommonGraphFunctions::getOutputBinding;

    unsigned getOutputBufferVertex(const StreamSetPort outputPort) const;
    StreamSetBuffer * getOutputBuffer(const StreamSetPort outputPort) const;
    const Binding & getOutputBinding(const StreamSetPort outputPort) const;

    using PipelineCommonGraphFunctions::getBinding;

    const Binding & getBinding(const StreamSetPort port) const;

    void clearInternalStateForCurrentKernel();
    void initializeKernelAssertions(BuilderRef b);
    void verifyBufferRelationships() const;

    bool hasAtLeastOneNonGreedyInput() const;

protected:

    Allocator									mAllocator;

    const bool                       			CheckAssertions;
    const bool                                  mTraceProcessedProducedItemCounts;
    const bool                                  mTraceDynamicBuffers;
    const bool                       			mTraceIndividualConsumedItemCounts;

    const unsigned								mNumOfThreads;

    const LengthAssertions &                    mLengthAssertions;

    // analysis state
    static constexpr unsigned                   PipelineInput = 0;
    static constexpr unsigned                   FirstKernel = 1;
    const unsigned                              LastKernel;
    const unsigned                              PipelineOutput;
    const unsigned                              FirstStreamSet;
    const unsigned                              LastStreamSet;
    const unsigned                              FirstBinding;
    const unsigned                              LastBinding;
    const unsigned                              FirstCall;
    const unsigned                              LastCall;
    const unsigned                              FirstScalar;
    const unsigned                              LastScalar;
    const unsigned                              PartitionCount;

    const size_t                                RequiredThreadLocalStreamSetMemory;

    const bool                                  ExternallySynchronized;
    const bool                                  PipelineHasTerminationSignal;
    const bool                                  HasZeroExtendedStream;
    const bool                                  EnableCycleCounter;
    #ifdef ENABLE_PAPI
    const bool                                  EnablePAPICounters;
    #else
    constexpr static bool                       EnablePAPICounters = false;
    #endif
    const bool                                  TraceIO;
    const bool                                  TraceUnconsumedItemCounts;
    const bool                                  TraceProducedItemCounts;

    const KernelIdVector                        KernelPartitionId;
    const std::vector<unsigned>                 StrideStepLength;
    const std::vector<unsigned>                 MaximumNumOfStrides;

    const RelationshipGraph                     mStreamGraph;
    const RelationshipGraph                     mScalarGraph;
    const BufferGraph                           mBufferGraph;
    const PartitionIOGraph                      mPartitionIOGraph;
    const std::vector<unsigned>                 PartitionJumpTargetId;
    const PartitionJumpTree                     mPartitionJumpTree;
    const ConsumerGraph                         mConsumerGraph;
    const TerminationChecks                     mTerminationCheck;
    const TerminationPropagationGraph           mTerminationPropagationGraph;

    // pipeline state
    unsigned                                    mKernelId = 0;
    const Kernel *                              mKernel = nullptr;
    Value *                                     mKernelSharedHandle = nullptr;
    Value *                                     mKernelThreadLocalHandle = nullptr;
    Value *                                     mSegNo = nullptr;
    Value *                                     mNextSegNo = nullptr;
    Value *                                     mExhaustedInput = nullptr;
    PHINode *                                   mMadeProgressInLastSegment = nullptr;
    Value *                                     mPipelineProgress = nullptr;
    Value *                                     mCurrentThreadTerminationSignalPtr = nullptr;
    BasicBlock *                                mPipelineLoop = nullptr;
    BasicBlock *                                mKernelLoopStart = nullptr;
    BasicBlock *                                mKernelLoopEntry = nullptr;
    BasicBlock *                                mKernelCheckOutputSpace = nullptr;
    BasicBlock *                                mKernelLoopCall = nullptr;
    BasicBlock *                                mKernelCompletionCheck = nullptr;
    BasicBlock *                                mKernelInitiallyTerminated = nullptr;
    BasicBlock *                                mKernelInitiallyTerminatedExit = nullptr;
    BasicBlock *                                mKernelTerminated = nullptr;
    BasicBlock *                                mKernelInsufficientInput = nullptr;
    BasicBlock *                                mKernelJumpToNextUsefulPartition = nullptr;
    PHINode *                                   mExhaustedInputAtJumpPhi = nullptr;
    BasicBlock *                                mKernelLoopExit = nullptr;
    BasicBlock *                                mKernelLoopExitPhiCatch = nullptr;
    BasicBlock *                                mKernelExit = nullptr;
    BasicBlock *                                mPipelineEnd = nullptr;
    BasicBlock *                                mRethrowException = nullptr;

    Value *                                     mThreadLocalStreamSetBaseAddress = nullptr;
    Value *                                     mExpectedNumOfStridesMultiplier = nullptr;

    Vec<AllocaInst *, 16>                       mAddressableItemCountPtr;
    Vec<AllocaInst *, 16>                       mVirtualBaseAddressPtr;
    Vec<AllocaInst *, 4>                        mTruncatedInputBuffer;
    FixedVector<PHINode *>                      mInitiallyAvailableItemsPhi;
    FixedVector<Value *>                        mLocallyAvailableItems;

    FixedVector<Value *>                        mScalarValue;
    FixedVector<bool>                           RequiresSynchronization;

    // partition state
    FixedVector<BasicBlock *>                   mPartitionEntryPoint;
    unsigned                                    mCurrentPartitionId = 0;
    unsigned                                    FirstKernelInPartition = 0;
    unsigned                                    LastKernelInPartition = 0;

    Rational                                    mPartitionStrideRateScalingFactor;

    Value *                                     mFinalPartitionSegment = nullptr;
    PHINode *                                   mFinalPartitionSegmentAtLoopExitPhi = nullptr;
    PHINode *                                   mFinalPartitionSegmentAtExitPhi = nullptr;
    PHINode *                                   mFinalPartialStrideFixedRateRemainderPhi = nullptr;

    Value *                                     mNumOfPartitionStrides = nullptr;

    BasicBlock *                                mCurrentPartitionEntryGuard = nullptr;
    BasicBlock *                                mNextPartitionEntryPoint = nullptr;
    FixedVector<Value *>                        mPartitionTerminationSignal;
    FixedVector<PHINode *>                      mExhaustedPipelineInputAtPartitionEntry;
    FixedVector<Value *>                        mInitialConsumedItemCount;

    PartitionPhiNodeTable                       mPartitionProducedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionConsumedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionTerminationSignalPhi;
    FixedVector<PHINode *>                      mPartitionPipelineProgressPhi;

    // kernel state
    Value *                                     mInitialTerminationSignal = nullptr;
    Value *                                     mInitiallyTerminated = nullptr;
    Value *                                     mMaximumNumOfStrides = nullptr;
    PHINode *                                   mCurrentNumOfStridesAtLoopEntryPhi = nullptr;
    Value *                                     mUpdatedNumOfStrides = nullptr;
    PHINode *                                   mTotalNumOfStridesAtLoopExitPhi = nullptr;
    PHINode *                                   mAnyProgressedAtLoopExitPhi = nullptr;
    PHINode *                                   mAnyProgressedAtExitPhi = nullptr;
    PHINode *                                   mAlreadyProgressedPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputAtLoopExitPhi = nullptr;
    Value *                                     mExhaustedPipelineInputAtExit = nullptr;
    PHINode *                                   mExecutedAtLeastOnceAtLoopEntryPhi = nullptr;
    PHINode *                                   mTerminatedSignalPhi = nullptr;
    PHINode *                                   mTerminatedAtLoopExitPhi = nullptr;
    PHINode *                                   mTerminatedAtExitPhi = nullptr;
    PHINode *                                   mTotalNumOfStridesAtExitPhi = nullptr;
    Value *                                     mNumOfLinearStrides = nullptr;
    Value *                                     mHasZeroExtendedInput = nullptr;
    PHINode *                                   mNumOfLinearStridesPhi = nullptr;
    PHINode *                                   mFixedRateFactorPhi = nullptr;
    PHINode *                                   mIsFinalInvocationPhi = nullptr;
    Value *                                     mIsFinalInvocation = nullptr;
    Value *                                     mAnyClosed = nullptr;

    BitVector                                   mHasPipelineInput;

    Rational                                    mFixedRateLCM;
    Value *                                     mTerminatedExplicitly = nullptr;
    Value *                                     mBranchToLoopExit = nullptr;


    bool                                        mIsBounded = false;
    bool                                        mKernelIsInternallySynchronized = false;
    bool                                        mKernelCanTerminateEarly = false;
    bool                                        mHasExplicitFinalPartialStride = false;
    bool                                        mIsPartitionRoot = false;
    bool                                        mMayLoopToEntry = false;
    bool                                        mCheckInputChannels = false;

    unsigned                                    mNumOfAddressableItemCount = 0;
    unsigned                                    mNumOfVirtualBaseAddresses = 0;
    unsigned                                    mNumOfTruncatedInputBuffers = 0;

    InputPortVector<Value *>                    mInitiallyProcessedItemCount; // *before* entering the kernel
    InputPortVector<Value *>                    mInitiallyProcessedDeferredItemCount;
    InputPortVector<PHINode *>                  mAlreadyProcessedPhi; // entering the segment loop
    InputPortVector<PHINode *>                  mAlreadyProcessedDeferredPhi;

    InputPortVector<Value *>                    mIsInputZeroExtended;
    InputPortVector<PHINode *>                  mInputVirtualBaseAddressPhi;
    InputPortVector<Value *>                    mFirstInputStrideLength;

    OverflowItemCounts                          mAccessibleInputItems;
    InputPortVector<PHINode *>                  mLinearInputItemsPhi;
    InputPortVector<Value *>                    mReturnedProcessedItemCountPtr; // written by the kernel
    InputPortVector<Value *>                    mProcessedItemCountPtr; // exiting the segment loop
    InputPortVector<Value *>                    mProcessedItemCount;
    InputPortVector<Value *>                    mProcessedDeferredItemCountPtr;
    InputPortVector<Value *>                    mProcessedDeferredItemCount;
    InputPortVector<PHINode *>                  mUpdatedProcessedPhi; // exiting the kernel
    InputPortVector<PHINode *>                  mUpdatedProcessedDeferredPhi;
    InputPortVector<Value *>                    mFullyProcessedItemCount; // *after* exiting the kernel

    FixedVector<Value *>                        mInitiallyProducedItemCount; // *before* entering the kernel
    FixedVector<Value *>                        mInitiallyProducedDeferredItemCount;
    OutputPortVector<PHINode *>                 mAlreadyProducedPhi; // entering the segment loop
    OutputPortVector<Value *>                   mAlreadyProducedDelayedPhi;
    OutputPortVector<PHINode *>                 mAlreadyProducedDeferredPhi;
    OutputPortVector<Value *>                   mFirstOutputStrideLength;

    OverflowItemCounts                          mWritableOutputItems;
    OutputPortVector<PHINode *>                 mLinearOutputItemsPhi;
    OutputPortVector<Value *>                   mReturnedOutputVirtualBaseAddressPtr; // written by the kernel
    OutputPortVector<Value *>                   mReturnedProducedItemCountPtr; // written by the kernel
    OutputPortVector<Value *>                   mProducedItemCountPtr; // exiting the segment loop
    OutputPortVector<Value *>                   mProducedItemCount;
    OutputPortVector<Value *>                   mProducedDeferredItemCountPtr;
    OutputPortVector<Value *>                   mProducedDeferredItemCount;
    OutputPortVector<PHINode *>                 mProducedAtTerminationPhi; // exiting after termination
    OutputPortVector<PHINode *>                 mUpdatedProducedPhi; // exiting the kernel
    OutputPortVector<PHINode *>                 mUpdatedProducedDeferredPhi;
    OutputPortVector<PHINode *>                 mFullyProducedItemCount; // *after* exiting the kernel


    // cycle counter state
    Value *                                     mKernelStartTime = nullptr;
    Value *                                     mAcquireAndReleaseStartTime = nullptr;
    FixedVector<PHINode *>                      mPartitionStartTimePhi;
    FixedArray<Value *, NUM_OF_CYCLE_COUNTERS>  mCycleCounters;

    // papi counter state
    #ifdef ENABLE_PAPI
    SmallVector<int, 8>                         PAPIEventList;
    Value *                                     PAPIEventSet = nullptr;
    Value *                                     PAPIEventSetVal = nullptr;
    Value *                                     PAPIReadInitialMeasurementArray = nullptr;
    Value *                                     PAPIReadBeforeMeasurementArray = nullptr;
    Value *                                     PAPIReadAfterMeasurementArray = nullptr;
    #endif

    // debug state
    Value *                                     mThreadId = nullptr;
    Value *                                     mDebugFileName = nullptr;
    Value *                                     mDebugFdPtr = nullptr;
    Value *                                     mDebugActualNumOfStrides;
    Value *                                     mCurrentKernelName = nullptr;    
    FixedVector<Value *>                        mKernelName;

    #ifndef NDEBUG
    FunctionType *                              mKernelDoSegmentFunctionType = nullptr;
    #endif

    // misc.

    OwningVector<Kernel>                        mInternalKernels;
    OwningVector<Binding>                       mInternalBindings;
    OwningVector<StreamSetBuffer>               mInternalBuffers;


};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel)
: PipelineCompiler(pipelineKernel, PipelineAnalysis::analyze(b, pipelineKernel)) {
    // Use a delegating constructor to compute the pipeline graph data once and pass it to
    // the compiler. Although a const function attribute ought to suffice, gcc 8.2 does not
    // resolve it correctly and clang requires -O2 or better.
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineCompiler::PipelineCompiler(PipelineKernel * const pipelineKernel, PipelineAnalysis && P)
: KernelCompiler(pipelineKernel)
, PipelineCommonGraphFunctions(mStreamGraph, mBufferGraph)
#ifdef FORCE_PIPELINE_ASSERTIONS
, CheckAssertions(true)
#else
, CheckAssertions(codegen::DebugOptionIsSet(codegen::EnableAsserts))
#endif
, mTraceProcessedProducedItemCounts(codegen::DebugOptionIsSet(codegen::TraceCounts))
, mTraceDynamicBuffers(codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))
, mTraceIndividualConsumedItemCounts(mTraceProcessedProducedItemCounts || mTraceDynamicBuffers)
, mNumOfThreads(pipelineKernel->getNumOfThreads())
, mLengthAssertions(pipelineKernel->getLengthAssertions())
, LastKernel(P.LastKernel)
, PipelineOutput(P.PipelineOutput)
, FirstStreamSet(P.FirstStreamSet)
, LastStreamSet(P.LastStreamSet)
, FirstBinding(P.FirstBinding)
, LastBinding(P.LastBinding)
, FirstCall(P.FirstCall)
, LastCall(P.LastCall)
, FirstScalar(P.FirstScalar)
, LastScalar(P.LastScalar)
, PartitionCount(P.PartitionCount)

, RequiredThreadLocalStreamSetMemory(P.RequiredThreadLocalStreamSetMemory)

, ExternallySynchronized(pipelineKernel->hasAttribute(AttrId::InternallySynchronized))
, PipelineHasTerminationSignal(pipelineKernel->canSetTerminateSignal())
, HasZeroExtendedStream(P.HasZeroExtendedStream)
, EnableCycleCounter(DebugOptionIsSet(codegen::EnableCycleCounter))
#ifdef ENABLE_PAPI
, EnablePAPICounters(codegen::PapiCounterOptions.compare(codegen::OmittedOption) != 0)
#endif
, TraceIO(DebugOptionIsSet(codegen::EnableBlockingIOCounter) || DebugOptionIsSet(codegen::TraceBlockedIO))
, TraceUnconsumedItemCounts(DebugOptionIsSet(codegen::TraceUnconsumedItemCounts))
, TraceProducedItemCounts(DebugOptionIsSet(codegen::TraceProducedItemCounts))

, KernelPartitionId(std::move(P.KernelPartitionId))
, StrideStepLength(std::move(P.StrideStepLength))
, MaximumNumOfStrides(std::move(P.MaximumNumOfStrides))

, mStreamGraph(std::move(P.mStreamGraph))
, mScalarGraph(std::move(P.mScalarGraph))
, mBufferGraph(std::move(P.mBufferGraph))
, mPartitionIOGraph(std::move(P.mPartitionIOGraph))
, PartitionJumpTargetId(std::move(P.mPartitionJumpIndex))
, mPartitionJumpTree(std::move(P.mPartitionJumpTree))
, mConsumerGraph(std::move(P.mConsumerGraph))
, mTerminationCheck(std::move(P.mTerminationCheck))
, mTerminationPropagationGraph(std::move(P.mTerminationPropagationGraph))

, mInitiallyAvailableItemsPhi(FirstStreamSet, LastStreamSet, mAllocator)
, mLocallyAvailableItems(FirstStreamSet, LastStreamSet, mAllocator)

, mScalarValue(FirstKernel, LastScalar, mAllocator)
, RequiresSynchronization(PipelineInput, PipelineOutput, mAllocator)

, mPartitionEntryPoint(PartitionCount + 1, mAllocator)

, mPartitionTerminationSignal(PartitionCount, mAllocator)
, mExhaustedPipelineInputAtPartitionEntry(PartitionCount, mAllocator)
, mInitialConsumedItemCount(FirstStreamSet, LastStreamSet, mAllocator)

, mPartitionProducedItemCountPhi(extents[PartitionCount][LastStreamSet - FirstStreamSet + 1])
, mPartitionConsumedItemCountPhi(extents[PartitionCount][LastStreamSet - FirstStreamSet + 1])
, mPartitionTerminationSignalPhi(extents[PartitionCount][PartitionCount])
, mPartitionPipelineProgressPhi(PartitionCount, mAllocator)

, mInitiallyProcessedItemCount(P.MaxNumOfInputPorts, mAllocator)
, mInitiallyProcessedDeferredItemCount(P.MaxNumOfInputPorts, mAllocator)
, mAlreadyProcessedPhi(P.MaxNumOfInputPorts, mAllocator)
, mAlreadyProcessedDeferredPhi(P.MaxNumOfInputPorts, mAllocator)
, mIsInputZeroExtended(P.MaxNumOfInputPorts, mAllocator)
, mInputVirtualBaseAddressPhi(P.MaxNumOfInputPorts, mAllocator)
, mFirstInputStrideLength(P.MaxNumOfInputPorts, mAllocator)
, mLinearInputItemsPhi(P.MaxNumOfInputPorts, mAllocator)
, mReturnedProcessedItemCountPtr(P.MaxNumOfInputPorts, mAllocator)
, mProcessedItemCountPtr(P.MaxNumOfInputPorts, mAllocator)
, mProcessedItemCount(P.MaxNumOfInputPorts, mAllocator)
, mProcessedDeferredItemCountPtr(P.MaxNumOfInputPorts, mAllocator)
, mProcessedDeferredItemCount(P.MaxNumOfInputPorts, mAllocator)
, mUpdatedProcessedPhi(P.MaxNumOfInputPorts, mAllocator)
, mUpdatedProcessedDeferredPhi(P.MaxNumOfInputPorts, mAllocator)
, mFullyProcessedItemCount(P.MaxNumOfInputPorts, mAllocator)

, mInitiallyProducedItemCount(FirstStreamSet, LastStreamSet, mAllocator)
, mInitiallyProducedDeferredItemCount(FirstStreamSet, LastStreamSet, mAllocator)

, mAlreadyProducedPhi(P.MaxNumOfOutputPorts, mAllocator)
, mAlreadyProducedDelayedPhi(P.MaxNumOfOutputPorts, mAllocator)
, mAlreadyProducedDeferredPhi(P.MaxNumOfOutputPorts, mAllocator)
, mFirstOutputStrideLength(P.MaxNumOfOutputPorts, mAllocator)
, mLinearOutputItemsPhi(P.MaxNumOfOutputPorts, mAllocator)
, mReturnedOutputVirtualBaseAddressPtr(P.MaxNumOfOutputPorts, mAllocator)
, mReturnedProducedItemCountPtr(P.MaxNumOfOutputPorts, mAllocator)
, mProducedItemCountPtr(P.MaxNumOfOutputPorts, mAllocator)
, mProducedItemCount(P.MaxNumOfOutputPorts, mAllocator)
, mProducedDeferredItemCountPtr(P.MaxNumOfOutputPorts, mAllocator)
, mProducedDeferredItemCount(P.MaxNumOfOutputPorts, mAllocator)
, mProducedAtTerminationPhi(P.MaxNumOfOutputPorts, mAllocator)
, mUpdatedProducedPhi(P.MaxNumOfOutputPorts, mAllocator)
, mUpdatedProducedDeferredPhi(P.MaxNumOfOutputPorts, mAllocator)
, mFullyProducedItemCount(P.MaxNumOfOutputPorts, mAllocator)

, mPartitionStartTimePhi(PartitionCount, mAllocator)

, mKernelName(PipelineInput, LastKernel, mAllocator)

, mInternalKernels(std::move(P.mInternalKernels))
, mInternalBindings(std::move(P.mInternalBindings))
, mInternalBuffers(std::move(P.mInternalBuffers))
{
    #ifdef ENABLE_PAPI
    convertPAPIEventNamesToCodes();
    #endif
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeKernelName(const size_t kernelIndex) const {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << kernelIndex;
    #ifdef PRINT_DEBUG_MESSAGES
    out << '.' << getKernel(kernelIndex)->getName();
    #endif
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBufferName
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE std::string PipelineCompiler::makeBufferName(const size_t kernelIndex, const StreamSetPort port) const {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << kernelIndex;
    #ifdef PRINT_DEBUG_MESSAGES
    out << '.' << getKernel(kernelIndex)->getName()
        << '.' << getBinding(kernelIndex, port).getName();
    #else
    if (port.Type == PortType::Input) {
        out << 'I';
    } else { // if (port.Type == PortType::Output) {
        out << 'O';
    }
    out.write_hex(port.Number);
    #endif
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemWidth
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline unsigned getItemWidth(const Type * ty ) {
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

} // end of namespace

#include "analysis/pipeline_analysis.hpp"
#include "buffer_management_logic.hpp"
#include "termination_logic.hpp"
#include "consumer_logic.hpp"
#include "partition_processing_logic.hpp"
#include "kernel_segment_processing_logic.hpp"
#include "cycle_counter_logic.hpp"
#include "pipeline_logic.hpp"
#include "scalar_logic.hpp"
#include "synchronization_logic.hpp"
#include "debug_messages.hpp"
#include "codegen/buffer_manipulation_logic.hpp"
#include "pipeline_optimization_logic.hpp"
#include "papi_instrumentation_logic.hpp"

#endif // PIPELINE_COMPILER_HPP
