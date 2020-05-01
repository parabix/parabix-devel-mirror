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
#include "config.h"
using namespace boost;
using namespace boost::math;
using namespace boost::adaptors;
using boost::container::flat_set;
using boost::container::flat_map;
using namespace llvm;

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return ((sizeof(unsigned) * CHAR_BIT) - 1U) - __builtin_clz(v);
}

namespace kernel {

enum CycleCounter {
  KERNEL_SYNCHRONIZATION
  , PARTITION_JUMP_SYNCHRONIZATION
  , BUFFER_EXPANSION  
  , BUFFER_COPY
  , KERNEL_EXECUTION
  , TOTAL_TIME
  // ------------------
  , NUM_OF_STORED_COUNTERS
};


const static std::string ZERO_EXTENDED_BUFFER = "ZeB";
const static std::string ZERO_EXTENDED_SPACE = "ZeS";

const static std::string KERNEL_THREAD_LOCAL_SUFFIX = ".KTL";

const static std::string NEXT_LOGICAL_SEGMENT_NUMBER = "@NLSN";
const static std::string LOGICAL_SEGMENT_SUFFIX = ".LSN";

const static std::string DEBUG_FD = ".DFd";

const static std::string ITERATION_COUNT_SUFFIX = ".ITC";
const static std::string TERMINATION_PREFIX = "@TERM";
const static std::string CONSUMER_TERMINATION_COUNT_PREFIX = "@PTC";
const static std::string ITEM_COUNT_SUFFIX = ".IN";
const static std::string DEFERRED_ITEM_COUNT_SUFFIX = ".DC";
const static std::string CONSUMED_ITEM_COUNT_SUFFIX = ".CON";
const static std::string DEBUG_CONSUMED_ITEM_COUNT_SUFFIX = ".DCON";

const static std::string STATISTICS_CYCLE_COUNT_SUFFIX = ".SCY";
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

private:

    PipelineCompiler(PipelineKernel * const pipelineKernel, PipelineAnalysis && P);

// internal pipeline state construction functions

public:

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
    Value * constructThreadStructObject(BuilderRef b, Value * const threadId, Value * const threadLocal);
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
    void determinePartitionStrideRates();
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const;

    void phiOutPartitionItemCounts(BuilderRef b, const unsigned kernel, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const entryPoint);
    void phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const entryPoint);

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

    void calculateItemCounts(BuilderRef b);
    Value * anyInputClosed(BuilderRef b);
    void determineIsFinal(BuilderRef b);
    Value * hasMoreInput(BuilderRef b);

    struct FinalItemCount {
        Value * minFixedRateFactor;
        Value * partialPartitionStrides;
    };

    void calculateFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems, Value *& minFixedRateFactor, Value *& partialPartitionStrides);

    void zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddresses);

    Value * allocateLocalZeroExtensionSpace(BuilderRef b, BasicBlock * const insertBefore) const;

    void writeKernelCall(BuilderRef b);
    ArgVec buildKernelCallArgumentList(BuilderRef b);
    void updateProcessedAndProducedItemCounts(BuilderRef b);
    void readReturnedOutputVirtualBaseAddresses(BuilderRef b) const;
    Value * addItemCountArg(BuilderRef b, const Binding & binding, const bool addressable, Value * const itemCount, ArgVec &args);
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


    void computeFullyProcessedItemCounts(BuilderRef b);
    void computeFullyProducedItemCounts(BuilderRef b);
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

    void verifyExpectedNumOfStrides(BuilderRef b);

// intra-kernel codegen functions

    Value * getInputStrideLength(BuilderRef b, const BufferPort &inputPort);
    Value * getOutputStrideLength(BuilderRef b, const BufferPort &outputPort);
    Value * calculateStrideLength(BuilderRef b, const BufferPort & port);
    Value * calculateNumOfLinearItems(BuilderRef b, const BufferPort &port, Value * const adjustment);
    Value * getAccessibleInputItems(BuilderRef b, const BufferPort & inputPort, const bool useOverflow = true);
    Value * getNumOfAccessibleStrides(BuilderRef b, const BufferPort & inputPort, Value * const numOfLinearStrides);
    Value * getNumOfWritableStrides(BuilderRef b, const BufferPort & outputPort, Value * const numOfLinearStrides);
    Value * getWritableOutputItems(BuilderRef b, const BufferPort & outputPort, const bool useOverflow = true);
    Value * addLookahead(BuilderRef b, const BufferPort & inputPort, Value * const itemCount) const;
    Value * subtractLookahead(BuilderRef b, const BufferPort & inputPort, Value * const itemCount);
    Value * truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * const terminationSignal) const;

    void initializeLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    void updateLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    Value * getLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort) const;
    void setLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort, Value * const available);

    Value * getPartialSumItemCount(BuilderRef b, const BufferPort &port, Value * const offset = nullptr) const;
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
    Value * initiallyTerminated(BuilderRef b);
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
    void updatePipelineInputConsumedItemCounts(BuilderRef b, BasicBlock * const exit);
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
    void loadInternalStreamSetHandles(BuilderRef b, const bool nonLocal);
    void releaseOwnedBuffers(BuilderRef b, const bool nonLocal);
    void resetInternalBufferHandles();
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffers(BuilderRef b, const size_t kernelId) const;

    void prepareLinearBuffers(BuilderRef b);
    Value * getVirtualBaseAddress(BuilderRef b, const BufferPort & rateData, const StreamSetBuffer * const buffer, Value * position) const;
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
    Value * getBufferExpansionCycleCounter(BuilderRef b) const;
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

    bool supportsInternalSynchronization() const;
    bool isBounded() const;
    bool requiresExplicitFinalStride() const ;
    bool mayLoopBackToEntry() const;
    void identifyPipelineInputs(const unsigned kernelId);
    void identifyLocalPortIds(const unsigned kernelId);

// synchronization functions

    void identifyAllInternallySynchronizedKernels();
    void obtainCurrentSegmentNumber(BuilderRef b, BasicBlock * const entryBlock);
    void incrementCurrentSegNo(BuilderRef b, BasicBlock * const exitBlock);
    void acquireSynchronizationLock(BuilderRef b, const unsigned kernelId);
    void releaseSynchronizationLock(BuilderRef b, const unsigned kernelId);

// family functions

    void addFamilyKernelProperties(BuilderRef b, const unsigned index) const;

    void bindFamilyInitializationArguments(BuilderRef b, ArgIterator & arg, const ArgIterator & arg_end) const override;

// thread local functions

    Value * getThreadLocalHandlePtr(BuilderRef b, const unsigned kernelIndex) const;

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
    Value * getKernelInitializeFunction(BuilderRef b) const;
    Value * getKernelAllocateSharedInternalStreamSetsFunction(BuilderRef b) const;
    Value * getKernelInitializeThreadLocalFunction(BuilderRef b) const;
    Value * getKernelAllocateThreadLocalInternalStreamSetsFunction(BuilderRef b) const;
    Value * getKernelDoSegmentFunction(BuilderRef b) const;
    Value * getKernelFinalizeThreadLocalFunction(BuilderRef b) const;
    Value * getKernelFinalizeFunction(BuilderRef b) const;

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

    const bool                                  ExternallySynchronized;
    const bool                                  PipelineHasTerminationSignal;
    const bool                                  HasZeroExtendedStream;
    const bool                                  EnableCycleCounter;
    const bool                                  TraceIO;
    const bool                                  TraceUnconsumedItemCounts;
    const bool                                  TraceProducedItemCounts;

    const Partition                             KernelPartitionId;
    const std::vector<Rational>                 MinimumNumOfStrides;
    const std::vector<Rational>                 MaximumNumOfStrides;

    const RelationshipGraph                     mStreamGraph;
    const RelationshipGraph                     mScalarGraph;
    const BufferGraph                           mBufferGraph;
    const std::vector<unsigned>                 mPartitionJumpIndex;
    const PartitionJumpTree                     mPartitionJumpTree;
    const ConsumerGraph                         mConsumerGraph;
    const TerminationChecks                     mTerminationCheck;
    const TerminationPropagationGraph           mTerminationPropagationGraph;

    // pipeline state
    unsigned                                    mKernelId = 0;
    const Kernel *                              mKernel = nullptr;
    Value *                                     mKernelSharedHandle = nullptr;
    Value *                                     mKernelThreadLocalHandle = nullptr;
    Value *                                     mZeroExtendBuffer = nullptr;
    Value *                                     mZeroExtendSpace = nullptr;
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

    Vec<AllocaInst *, 16>                       mAddressableItemCountPtr;
    Vec<AllocaInst *, 16>                       mVirtualBaseAddressPtr;
    Vec<AllocaInst *, 4>                        mTruncatedInputBuffer;
    FixedVector<PHINode *>                      mInitiallyAvailableItemsPhi;
    FixedVector<Value *>                        mLocallyAvailableItems;

    FixedVector<PHINode *>                      mExternalConsumedItemsPhi;

    FixedVector<Value *>                        mScalarValue;
    FixedVector<bool>                           RequiresSynchronization;

    // partition state
    FixedVector<BasicBlock *>                   mPartitionEntryPoint;
    unsigned                                    mCurrentPartitionId = 0;
    unsigned                                    FirstKernelInPartition = 0;
    unsigned                                    LastKernelInPartition = 0;
    Value *                                     mNumOfPartitionStrides = nullptr;
    Rational                                    MaxPartitionStrideRate;
    Rational                                    PartitionStrideFactor;
    Value *                                     mPartitionRootTerminationSignal = nullptr;
    BasicBlock *                                mNextPartitionEntryPoint;
    FixedVector<Value *>                        mPartitionTerminationSignal;
    FixedVector<PHINode *>                      mExhaustedPipelineInputAtPartitionEntry;
    FixedVector<Value *>                        mInitialConsumedItemCount;

    PartitionPhiNodeTable                       mPartitionProducedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionConsumedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionTerminationSignalPhi;
    FixedVector<PHINode *>                      mPartitionPipelineProgressPhi;

    // kernel state
    Value *                                     mTerminatedInitially = nullptr;
    Value *                                     mMaximumNumOfStrides = nullptr;
    PHINode *                                   mPartialPartitionStridesPhi = nullptr;
    PHINode *                                   mPartialPartitionStridesAtLoopExitPhi = nullptr;
    PHINode *                                   mCurrentNumOfStridesAtLoopEntryPhi = nullptr;
    Value *                                     mUpdatedNumOfStrides = nullptr;
    Value *                                     mKernelIsPenultimate = nullptr;
    Value *                                     mKernelIsFinal = nullptr;
    PHINode *                                   mTotalNumOfStridesAtLoopExitPhi = nullptr;
    PHINode *                                   mAnyProgressedAtLoopExitPhi = nullptr;
    PHINode *                                   mAnyProgressedAtExitPhi = nullptr;
    PHINode *                                   mAlreadyProgressedPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputAtPartitionJumpPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputAtLoopExitPhi = nullptr;
    Value *                                     mExhaustedPipelineInputAtExit = nullptr;
    PHINode *                                   mExecutedAtLeastOnceAtLoopEntryPhi = nullptr;
    PHINode *                                   mTerminatedSignalPhi = nullptr;
    PHINode *                                   mTerminatedAtLoopExitPhi = nullptr;
    PHINode *                                   mTerminatedAtExitPhi = nullptr;
    PHINode *                                   mTotalNumOfStridesAtExitPhi = nullptr;
    Value *                                     mLastPartialSegment = nullptr;
    Value *                                     mNumOfLinearStrides = nullptr;
    Value *                                     mHasZeroExtendedInput = nullptr;
    Value *                                     mAnyRemainingInput = nullptr;
    PHINode *                                   mFixedRateFactorPhi = nullptr;
    PHINode *                                   mIsFinalInvocationPhi = nullptr;
    BasicBlock *                                mNextPartitionWithPotentialInput = nullptr;
    BasicBlock *                                mGotoNextPartition = nullptr;

    BitVector                                   mHasPipelineInput;

    Rational                                    mFixedRateLCM;
    Value *                                     mTerminatedExplicitly = nullptr;
    Value *                                     mBranchToLoopExit = nullptr;


    bool                                        mMayHaveNonLinearIO = false;
    bool                                        mIsBounded = false;
    bool                                        mKernelIsInternallySynchronized = false;
    bool                                        mKernelCanTerminateEarly = false;
    bool                                        mHasExplicitFinalPartialStride = false;
    bool                                        mIsPartitionRoot = false;
    bool                                        mMayLoopToEntry = false;
    bool                                        mCheckIO = false;

    unsigned                                    mNumOfAddressableItemCount = 0;
    unsigned                                    mNumOfVirtualBaseAddresses = 0;
    unsigned                                    mNumOfTruncatedInputBuffers = 0;

    unsigned                                    mNumOfLocalInputPortIds = 0;
    unsigned                                    mNumOfLocalOutputPortIds = 0;

    PHINode *                                   mZeroExtendBufferPhi = nullptr;

    InputPortVector<Value *>                    mInitiallyProcessedItemCount; // *before* entering the kernel
    InputPortVector<Value *>                    mInitiallyProcessedDeferredItemCount;
    InputPortVector<PHINode *>                  mAlreadyProcessedPhi; // entering the segment loop
    InputPortVector<PHINode *>                  mAlreadyProcessedDeferredPhi;

    InputPortVector<Value *>                    mInputEpoch;
    InputPortVector<Value *>                    mIsInputZeroExtended;
    InputPortVector<PHINode *>                  mInputVirtualBaseAddressPhi;
    InputPortVector<Value *>                    mFirstInputStrideLength;

    OverflowItemCounts                          mAccessibleInputItems;
    InputPortVector<PHINode *>                  mLinearInputItemsPhi;
    InputPortVector<Value *>                    mReturnedProcessedItemCountPtr; // written by the kernel
    InputPortVector<Value *>                    mProcessedItemCount; // exiting the segment loop
    InputPortVector<Value *>                    mProcessedDeferredItemCount;
    InputPortVector<PHINode *>                  mInsufficientIOProcessedPhi; // exiting insufficient io
    InputPortVector<PHINode *>                  mInsufficientIOProcessedDeferredPhi;
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
    OutputPortVector<Value *>                   mProducedItemCount; // exiting the segment loop
    OutputPortVector<Value *>                   mProducedDeferredItemCount;
    OutputPortVector<PHINode *>                 mProducedAtTerminationPhi; // exiting after termination
    OutputPortVector<PHINode *>                 mInsufficientIOProducedPhi; // exiting insufficient io
    OutputPortVector<PHINode *>                 mInsufficientIOProducedDeferredPhi;
    OutputPortVector<PHINode *>                 mUpdatedProducedPhi; // exiting the kernel
    OutputPortVector<PHINode *>                 mUpdatedProducedDeferredPhi;
    OutputPortVector<PHINode *>                 mFullyProducedItemCount; // *after* exiting the kernel

    // cycle counter state
    Value *                                     mKernelStartTime = nullptr;
    FixedVector<PHINode *>                      mPartitionStartTimePhi;
    std::array<Value *, NUM_OF_STORED_COUNTERS> mCycleCounters;

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

    OwningVec<Kernel>                           mInternalKernels;
    OwningVec<Binding>                          mInternalBindings;
    OwningVec<StreamSetBuffer>                  mInternalBuffers;


};

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel)
: PipelineCompiler(pipelineKernel, std::move(PipelineAnalysis::analyze(b, pipelineKernel))) {
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

, ExternallySynchronized(pipelineKernel->hasAttribute(AttrId::InternallySynchronized))
, PipelineHasTerminationSignal(pipelineKernel->canSetTerminateSignal())
, HasZeroExtendedStream(P.HasZeroExtendedStream)
, EnableCycleCounter(DebugOptionIsSet(codegen::EnableCycleCounter))
, TraceIO(DebugOptionIsSet(codegen::EnableBlockingIOCounter) || DebugOptionIsSet(codegen::TraceBlockedIO))
, TraceUnconsumedItemCounts(DebugOptionIsSet(codegen::TraceUnconsumedItemCounts))
, TraceProducedItemCounts(DebugOptionIsSet(codegen::TraceProducedItemCounts))

, KernelPartitionId(std::move(P.KernelPartitionId))
, MinimumNumOfStrides(std::move(P.MinimumNumOfStrides))
, MaximumNumOfStrides(std::move(P.MaximumNumOfStrides))

, mStreamGraph(std::move(P.mStreamGraph))
, mScalarGraph(std::move(P.mScalarGraph))
, mBufferGraph(std::move(P.mBufferGraph))
, mPartitionJumpIndex(std::move(P.mPartitionJumpIndex))
, mPartitionJumpTree(std::move(P.mPartitionJumpTree))
, mConsumerGraph(std::move(P.mConsumerGraph))
, mTerminationCheck(std::move(P.mTerminationCheck))
, mTerminationPropagationGraph(std::move(P.mTerminationPropagationGraph))

, mInitiallyAvailableItemsPhi(FirstStreamSet, LastStreamSet, mAllocator)
, mLocallyAvailableItems(FirstStreamSet, LastStreamSet, mAllocator)

, mExternalConsumedItemsPhi(FirstStreamSet, LastStreamSet, mAllocator)

, mScalarValue(FirstKernel, LastScalar, mAllocator)
, RequiresSynchronization(PipelineInput, PipelineOutput, mAllocator)

, mPartitionEntryPoint(PartitionCount, mAllocator)

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
, mInputEpoch(P.MaxNumOfInputPorts, mAllocator)
, mIsInputZeroExtended(P.MaxNumOfInputPorts, mAllocator)
, mInputVirtualBaseAddressPhi(P.MaxNumOfInputPorts, mAllocator)
, mFirstInputStrideLength(P.MaxNumOfInputPorts, mAllocator)
, mLinearInputItemsPhi(P.MaxNumOfInputPorts, mAllocator)
, mReturnedProcessedItemCountPtr(P.MaxNumOfInputPorts, mAllocator)
, mProcessedItemCount(P.MaxNumOfInputPorts, mAllocator)
, mProcessedDeferredItemCount(P.MaxNumOfInputPorts, mAllocator)
, mInsufficientIOProcessedPhi(P.MaxNumOfInputPorts, mAllocator)
, mInsufficientIOProcessedDeferredPhi(P.MaxNumOfInputPorts, mAllocator)
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
, mProducedItemCount(P.MaxNumOfOutputPorts, mAllocator)
, mProducedDeferredItemCount(P.MaxNumOfOutputPorts, mAllocator)
, mProducedAtTerminationPhi(P.MaxNumOfOutputPorts, mAllocator)
, mInsufficientIOProducedPhi(P.MaxNumOfOutputPorts, mAllocator)
, mInsufficientIOProducedDeferredPhi(P.MaxNumOfOutputPorts, mAllocator)
, mUpdatedProducedPhi(P.MaxNumOfOutputPorts, mAllocator)
, mUpdatedProducedDeferredPhi(P.MaxNumOfOutputPorts, mAllocator)
, mFullyProducedItemCount(P.MaxNumOfOutputPorts, mAllocator)

, mPartitionStartTimePhi(PartitionCount, mAllocator)

, mKernelName(FirstKernel, LastKernel, mAllocator)

, mInternalKernels(std::move(P.mInternalKernels))
, mInternalBindings(std::move(P.mInternalBindings))
, mInternalBuffers(std::move(P.mInternalBuffers))
{

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

#endif // PIPELINE_COMPILER_HPP
