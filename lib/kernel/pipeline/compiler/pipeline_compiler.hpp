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

// #define PRINT_DEBUG_MESSAGES

// #define PERMIT_THREAD_LOCAL_BUFFERS

// #define DISABLE_ZERO_EXTEND

// #define DISABLE_INPUT_ZEROING

// #define DISABLE_OUTPUT_ZEROING

// #define FORCE_PIPELINE_ASSERTIONS

// TODO: create a preallocation phase for source kernels to add capacity suggestions

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
  INITIAL
  , BEFORE_KERNEL_CALL
  , BEFORE_SYNCHRONIZATION
  , BEFORE_COPY
  // ------------------
  , NUM_OF_STORED_COUNTERS
  // ------------------
  , AFTER_SYNCHRONIZATION
  , BUFFER_EXPANSION
  , AFTER_COPY
  , AFTER_KERNEL_CALL
  , FINAL
};


const static std::string ZERO_EXTENDED_BUFFER = "ZeB";
const static std::string ZERO_EXTENDED_SPACE = "ZeS";

const static std::string KERNEL_THREAD_LOCAL_SUFFIX = ".KTL";

const static std::string NEXT_LOGICAL_SEGMENT_NUMBER = "@NLSN";
const static std::string LOGICAL_SEGMENT_SUFFIX = ".LSN";

const static std::string DEBUG_FD = ".DFd";

const static std::string PARTITION_LOGICAL_SEGMENT_NUMBER = ".PLS";
const static std::string PARTITION_ITEM_COUNT_SUFFIX = ".PIC";
const static std::string PARTITION_FIXED_RATE_SUFFIX = ".PFR";
const static std::string PARTITION_TERMINATION_SIGNAL_SUFFIX = ".PTS";

const static std::string ITERATION_COUNT_SUFFIX = ".ITC";
const static std::string TERMINATION_PREFIX = "@TERM";
const static std::string ITEM_COUNT_SUFFIX = ".IN";
const static std::string DEFERRED_ITEM_COUNT_SUFFIX = ".DC";
const static std::string CONSUMED_ITEM_COUNT_SUFFIX = ".CON";

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

    template<typename T>
    struct InputPortVec {
    public:
        InputPortVec(PipelineCompiler * const pc)
        : mPC(*pc)
        , mVec(pc->mAllocator.allocate<T>(pc->mInputPortSet.size())) {
            #ifndef NDEBUG
            std::memset(mVec, 0, sizeof(T) * pc->mInputPortSet.size());
            #endif
        }
        BOOST_FORCEINLINE
        LLVM_READNONE T & operator()(const unsigned kernel, const StreamSetPort port) const {
            return mVec[mPC.getInputPortIndex(kernel, port)];
        }
        BOOST_FORCEINLINE
        T & operator()(const StreamSetPort port) const {
            return operator()(mPC.mKernelId, port);
        }
    private:
        const PipelineCompiler & mPC;
        T * const mVec;
    };

    template<typename T> friend struct InputPortVec;

    template<typename T>
    struct OutputPortVec {
    public:
        OutputPortVec(PipelineCompiler * const pc)
        : mPC(*pc)
        , mVec(pc->mAllocator.allocate<T>(pc->mOutputPortSet.size())) {
            #ifndef NDEBUG
            std::memset(mVec, 0, sizeof(T) * pc->mOutputPortSet.size());
            #endif
        }
        BOOST_FORCEINLINE
        LLVM_READNONE T & operator()(const unsigned kernel, const StreamSetPort port) const {
            return mVec[mPC.getOutputPortIndex(kernel, port)];
        }
        BOOST_FORCEINLINE
        T & operator()(const StreamSetPort port) const {
            return operator()(mPC.mKernelId, port);
        }
    private:
        const PipelineCompiler & mPC;
        T * const mVec;
    };

    template<typename T> friend struct OutputPortVec;

public:

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel);

    void generateImplicitKernels(BuilderRef b);
    void addPipelineKernelProperties(BuilderRef b);
    void constructStreamSetBuffers(BuilderRef b) override;
    void generateInitializeMethod(BuilderRef b);
    void generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * expectedNumOfStrides);
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
    void executeExternallySynchronizedKernel(BuilderRef b);
    void executeInternallySynchronizedKernel(BuilderRef b);
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

    void addTerminationProperties(BuilderRef b, const size_t kernel);

// partitioning codegen functions

    void addPartitionInputItemCounts(BuilderRef b, const size_t partitionId) const;
    void makePartitionEntryPoints(BuilderRef b);
    void branchToInitialPartition(BuilderRef b);
    BasicBlock * getPartitionExitPoint(BuilderRef b);
    void checkForPartitionEntry(BuilderRef b);
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const;

    void phiOutPartitionItemCounts(BuilderRef b, const unsigned kernel, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const exitBlock);
    void phiOutPartitionStatusFlags(BuilderRef b, const unsigned targetPartitionId, const bool fromKernelEntry, BasicBlock * const exitBlock);

    void writeInitiallyTerminatedPartitionExit(BuilderRef b);
    void checkForPartitionExit(BuilderRef b);
    void setCurrentPartitionTerminationSignal(Value * const signal);
    Value * getCurrentPartitionTerminationSignal() const;

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

    void determineNumOfLinearStrides(BuilderRef b);
    void checkForSufficientInputData(BuilderRef b, const StreamSetPort inputPort);
    void ensureSufficientOutputSpace(BuilderRef b, const StreamSetPort outputPort);
    void updatePHINodesForLoopExit(BuilderRef b);

    void calculateItemCounts(BuilderRef b);
    Value * anyInputClosed(BuilderRef b) const;
    void determineIsFinal(BuilderRef b);
    Value * hasMoreInput(BuilderRef b);
    std::pair<Value *, Value *> calculateFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems);

    void zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddresses);

    Value * allocateLocalZeroExtensionSpace(BuilderRef b, BasicBlock * const insertBefore) const;

    void writeKernelCall(BuilderRef b);
    ArgVec buildKernelCallArgumentList(BuilderRef b);
    void updateProcessedAndProducedItemCounts(BuilderRef b);
    void readReturnedOutputVirtualBaseAddresses(BuilderRef b) const;
    Value * addItemCountArg(BuilderRef b, const Binding & binding, const bool addressable, PHINode * const itemCount, ArgVec &args);
    Value * addVirtualBaseAddressArg(BuilderRef b, const StreamSetBuffer * buffer, ArgVec & args);

    void normalCompletionCheck(BuilderRef b);

    void writeInsufficientIOExit(BuilderRef b);
    void writeOnInitialTerminationJumpToNextPartitionToCheck(BuilderRef b);

    void writeCopyBackLogic(BuilderRef b);
    void writeLookAheadLogic(BuilderRef b);
    void writeLookBehindLogic(BuilderRef b);
    void writeLookBehindReflectionLogic(BuilderRef b);
    enum class CopyMode { CopyBack, LookAhead, LookBehind, LookBehindReflection };
    void copy(BuilderRef b, const CopyMode mode, Value * cond, const StreamSetPort outputPort, const StreamSetBuffer * const buffer, const unsigned itemsToCopy);


    void computeFullyProcessedItemCounts(BuilderRef b);
    void computeFullyProducedItemCounts(BuilderRef b);
    Value * computeFullyProducedItemCount(BuilderRef b, const size_t kernel, const StreamSetPort port, Value * produced, Value * const terminationSignal);

    void updatePhisAfterTermination(BuilderRef b);

    void clearUnwrittenOutputData(BuilderRef b);

    void computeMinimumConsumedItemCounts(BuilderRef b);
    void writeFinalConsumedItemCounts(BuilderRef b);
    void recordFinalProducedItemCounts(BuilderRef b);

    void writeCopyToOverflowLogic(BuilderRef b);

    void readCountableItemCountsAfterAbnormalTermination(BuilderRef b);

    enum class ItemCountSource {
        ComputedAtKernelCall
        , UpdatedItemCountsFromLoopExit
    };

    void writeUpdatedItemCounts(BuilderRef b, const ItemCountSource source);

    void writeOutputScalars(BuilderRef b, const size_t index, std::vector<Value *> & args);
    Value * getScalar(BuilderRef b, const size_t index);

// intra-kernel codegen functions

    Value * getInputStrideLength(BuilderRef b, const StreamSetPort inputPort);
    Value * getOutputStrideLength(BuilderRef b, const StreamSetPort outputPort);
    Value * calculateStrideLength(BuilderRef b, const size_t kernel, const StreamSetPort port, Value * const offset = nullptr);
    Value * calculateNumOfLinearItems(BuilderRef b, const StreamSetPort port, Value * const adjustment);
    Value * getAccessibleInputItems(BuilderRef b, const StreamSetPort inputPort, const bool useOverflow = true);
    Value * getNumOfAccessibleStrides(BuilderRef b, const StreamSetPort inputPort);
    Value * getNumOfWritableStrides(BuilderRef b, const StreamSetPort outputPort);
    Value * getWritableOutputItems(BuilderRef b, const StreamSetPort outputPort, const bool useOverflow = true);
    Value * addLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount) const;
    Value * subtractLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount);
    Constant * getLookahead(BuilderRef b, const StreamSetPort inputPort) const;
    Value * truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * const terminationSignal) const;

    void initializeLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    void updateLocallyAvailableItemCounts(BuilderRef b, BasicBlock * const entryBlock);
    Value * getLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort) const;
    void setLocallyAvailableItemCount(BuilderRef b, const StreamSetPort inputPort, Value * const available);

    Value * getPartialSumItemCount(BuilderRef b, const size_t kernel, const StreamSetPort port, Value * const offset = nullptr) const;
    Value * getMaximumNumOfPartialSumStrides(BuilderRef b, const StreamSetPort port);

// termination codegen functions

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

// consumer codegen functions

    void addConsumerKernelProperties(BuilderRef b, const unsigned producer);
    void readExternalConsumerItemCounts(BuilderRef b);
    void createConsumedPhiNodes(BuilderRef b);
    void initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced);
    void readConsumedItemCounts(BuilderRef b);
    Value * readConsumedItemCount(BuilderRef b, const size_t streamSet);
    void setConsumedItemCount(BuilderRef b, const size_t bufferVertex, not_null<Value *> consumed, const unsigned slot) const;
    void writeExternalConsumedItemCounts(BuilderRef b);

// buffer management codegen functions

    void addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index);

    void allocateOwnedBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal);
    void loadInternalStreamSetHandles(BuilderRef b, const bool nonLocal);
    void releaseOwnedBuffers(BuilderRef b, const bool nonLocal);
    void resetInternalBufferHandles();
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffers(BuilderRef b, const size_t kernelId) const;
    LLVM_READNONE bool requiresCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE bool requiresLookAhead(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getLookAhead(const unsigned bufferVertex) const;

    void prepareLinearBuffers(BuilderRef b);
    Value * getVirtualBaseAddress(BuilderRef b, const Binding & binding, const StreamSetBuffer * const buffer, Value * const position) const;
    void getInputVirtualBaseAddresses(BuilderRef b, Vec<Value *> & baseAddresses) const;
    void getZeroExtendedInputVirtualBaseAddresses(BuilderRef b, const Vec<Value *> & baseAddresses, Value * const zeroExtensionSpace, Vec<Value *> & zeroExtendedVirtualBaseAddress) const;

    LLVM_READNONE unsigned getInputPortIndex(const unsigned kernel, StreamSetPort port) const;

    LLVM_READNONE unsigned getOutputPortIndex(const unsigned kernel, StreamSetPort port) const;

// cycle counter functions

    void addCycleCounterProperties(BuilderRef b, const unsigned kernel);
    void startCycleCounter(BuilderRef b, const CycleCounter type);
    void updateCycleCounter(BuilderRef b, const CycleCounter start, const CycleCounter end) const;


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

// buffer management analysis functions

    BufferPortMap constructInputPortMappings() const;
    BufferPortMap constructOutputPortMappings() const;
    bool supportsInternalSynchronization() const;
    bool isBounded() const;
    bool canTruncateInputBuffer() const;
    void identifyPipelineInputs();

    void printBufferGraph(raw_ostream & out) const;

// synchronization functions

    void acquireSynchronizationLock(BuilderRef b, const unsigned kernelId, const CycleCounter start);
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

protected:

    Allocator									mAllocator;

    const bool                       			mCheckAssertions;
    const bool                                  mTraceProcessedProducedItemCounts;
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
    const bool                                  mHasZeroExtendedStream;

    const Partition                             KernelPartitionId;
    const std::vector<Rational>                 MinimumNumOfStrides;
    const std::vector<Rational>                 MaximumNumOfStrides;

    const RelationshipGraph                     mStreamGraph;
    const RelationshipGraph                     mScalarGraph;
    const InputTruncationGraph                  mInputTruncationGraph;
    const BufferGraph                           mBufferGraph;
    const PartitioningGraph                     mPartitioningGraph;
    const std::vector<unsigned>                 mPartitionJumpIndex;
    const PartitionJumpTree                     mPartitionJumpTree;
    const ConsumerGraph                         mConsumerGraph;
    const TerminationGraph                      mTerminationGraph;
    const IOCheckGraph                          mIOCheckGraph;

    const BufferPortMap                         mInputPortSet;
    const BufferPortMap                         mOutputPortSet;

    // pipeline state
    unsigned                                    mKernelId = 0;
    const Kernel *                              mKernel = nullptr;
    Value *                                     mKernelSharedHandle = nullptr;
    Value *                                     mKernelThreadLocalHandle = nullptr;
    Value *                                     mZeroExtendBuffer = nullptr;
    Value *                                     mZeroExtendSpace = nullptr;
    Value *                                     mSegNo = nullptr;
    Value *                                     mExhaustedInput = nullptr;
    PHINode *                                   mMadeProgressInLastSegment = nullptr;
    Value *                                     mPipelineProgress = nullptr;
    Value *                                     mCurrentThreadTerminationSignalPtr = nullptr;
    BasicBlock *                                mPipelineLoop = nullptr;
    BasicBlock *                                mKernelEntry = nullptr;
    BasicBlock *                                mKernelLoopEntry = nullptr;
    BasicBlock *                                mKernelCheckOutputSpace = nullptr;
    BasicBlock *                                mKernelLoopCall = nullptr;
    BasicBlock *                                mKernelCompletionCheck = nullptr;
    BasicBlock *                                mKernelInitiallyTerminated = nullptr;
    BasicBlock *                                mKernelInitiallyTerminatedExit = nullptr;
    BasicBlock *                                mKernelTerminated = nullptr;
    BasicBlock *                                mKernelInsufficientInput = nullptr;
    BasicBlock *                                mKernelInsufficientInputExit = nullptr;
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
    FixedVector<Value *>                        mScalarValue;

    // partition state
    Vec<BasicBlock *>                           mPartitionEntryPoint;
    unsigned                                    mCurrentPartitionId = 0;
    unsigned                                    mPartitionRootKernelId = 0;
    Value *                                     mNumOfPartitionStrides = nullptr;
    Value *                                     mPartitionRootTerminationSignal = nullptr;
    BasicBlock *                                mNextPartitionEntryPoint;

    Value *                                     mKernelIsPenultimate = nullptr;

    FixedVector<Value *>                        mPartitionTerminationSignal;

    FixedVector<PHINode *>                      mExhaustedPipelineInputAtPartitionEntry;

    FixedVector<Value *>                        mInitialConsumedItemCount;
    FixedVector<Value *>                        mOriginalBaseAddress;

    PartitionPhiNodeTable                       mPartitionProducedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionConsumedItemCountPhi;
    PartitionPhiNodeTable                       mPartitionTerminationSignalPhi;
    FixedVector<PHINode *>                      mPartitionPipelineProgressPhi;



    // kernel state
    Value *                                     mTerminatedInitially = nullptr;
    Value *                                     mTerminatedInitiallyCheck = nullptr;
    Value *                                     mMaximumNumOfStrides = nullptr;
    PHINode *                                   mCurrentNumOfStridesAtLoopEntryPhi = nullptr;
    Value *                                     mUpdatedNumOfStrides = nullptr;
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
    Value *                                     mNumOfInputStrides = nullptr;
    Value *                                     mNumOfOutputStrides = nullptr;
    Value *                                     mHasZeroExtendedInput = nullptr;
    Value *                                     mAnyRemainingInput = nullptr;
    PHINode *                                   mFixedRateFactorPhi = nullptr;
    PHINode *                                   mNumOfLinearStridesPhi = nullptr;
    PHINode *                                   mIsFinalInvocationPhi = nullptr;
    BasicBlock *                                mNextPartitionWithPotentialInput = nullptr;

    BitVector                                   mHasPipelineInput;

    Rational                                    mFixedRateLCM;
    Value *                                     mTerminatedExplicitly = nullptr;
    Value *                                     mBranchToLoopExit = nullptr;


    bool                                        mMayHaveNonLinearIO = false;
    bool                                        mIsBounded = false;
    bool                                        mKernelIsInternallySynchronized = false;
    bool                                        mKernelCanTerminateEarly = false;
    bool                                        mHasExplicitFinalPartialStride = false;
    bool                                        mCanTruncatedInput = false;
    bool                                        mIsPartitionRoot = false;
    bool                                        mNonSourceKernel = false;

    unsigned                                    mNumOfAddressableItemCount = 0;
    unsigned                                    mNumOfVirtualBaseAddresses = 0;
    unsigned                                    mNumOfTruncatedInputBuffers = 0;

    PHINode *                                   mZeroExtendBufferPhi = nullptr;

    InputPortVec<Value *>                       mInitiallyProcessedItemCount; // *before* entering the kernel
    InputPortVec<Value *>                       mInitiallyProcessedDeferredItemCount;
    InputPortVec<PHINode *>                     mAlreadyProcessedPhi; // entering the segment loop
    InputPortVec<PHINode *>                     mAlreadyProcessedDeferredPhi;

    enum { WITH_OVERFLOW = 0, WITHOUT_OVERFLOW = 1};


    InputPortVec<Value *>                       mInputEpoch;
    InputPortVec<Value *>                       mIsInputZeroExtended;
    InputPortVec<PHINode *>                     mInputVirtualBaseAddressPhi;
    InputPortVec<Value *>                       mFirstInputStrideLength;
    Vec<std::array<Value *, 2>, 8>              mAccessibleInputItems;
    InputPortVec<PHINode *>                     mLinearInputItemsPhi;
    InputPortVec<Value *>                       mReturnedProcessedItemCountPtr; // written by the kernel
    InputPortVec<Value *>                       mProcessedItemCount; // exiting the segment loop
    InputPortVec<Value *>                       mProcessedDeferredItemCount;
    InputPortVec<PHINode *>                     mFinalProcessedPhi; // exiting after termination
    InputPortVec<PHINode *>                     mInsufficientIOProcessedPhi; // exiting insufficient io
    InputPortVec<PHINode *>                     mInsufficientIOProcessedDeferredPhi;
    InputPortVec<PHINode *>                     mUpdatedProcessedPhi; // exiting the kernel
    InputPortVec<PHINode *>                     mUpdatedProcessedDeferredPhi;
    InputPortVec<Value *>                       mFullyProcessedItemCount; // *after* exiting the kernel

    FixedVector<Value *>                        mInitiallyProducedItemCount; // *before* entering the kernel
    FixedVector<Value *>                        mInitiallyProducedDeferredItemCount;
    OutputPortVec<PHINode *>                    mAlreadyProducedPhi; // entering the segment loop
    OutputPortVec<Value *>                      mAlreadyProducedDelayedPhi;
    OutputPortVec<PHINode *>                    mAlreadyProducedDeferredPhi;
    OutputPortVec<Value *>                      mFirstOutputStrideLength;
    OutputPortVec<Value *>                      mWritableOutputItems;
    OutputPortVec<PHINode *>                    mLinearOutputItemsPhi;
    OutputPortVec<Value *>                      mReturnedOutputVirtualBaseAddressPtr; // written by the kernel
    OutputPortVec<Value *>                      mReturnedProducedItemCountPtr; // written by the kernel
    OutputPortVec<Value *>                      mProducedItemCount; // exiting the segment loop
    OutputPortVec<Value *>                      mProducedDeferredItemCount;
    OutputPortVec<PHINode *>                    mFinalProducedPhi; // exiting after termination
    OutputPortVec<PHINode *>                    mInsufficientIOProducedPhi; // exiting insufficient io
    OutputPortVec<PHINode *>                    mInsufficientIOProducedDeferredPhi;
    OutputPortVec<PHINode *>                    mUpdatedProducedPhi; // exiting the kernel
    OutputPortVec<PHINode *>                    mUpdatedProducedDeferredPhi;
    OutputPortVec<PHINode *>                    mFullyProducedItemCount; // *after* exiting the kernel

    // cycle counter state
    std::array<Value *, NUM_OF_STORED_COUNTERS> mCycleCounters;

    // debug state
    Value *                                     mThreadId = nullptr;
    Value *                                     mDebugFileName = nullptr;
    Value *                                     mDebugFdPtr = nullptr;
    Value *                                     mCurrentKernelName = nullptr;
    FixedVector<Value *>                        mKernelName;

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
, mCheckAssertions(true)
#else
, mCheckAssertions(codegen::DebugOptionIsSet(codegen::EnableAsserts))
#endif
, mTraceProcessedProducedItemCounts(codegen::DebugOptionIsSet(codegen::TraceCounts))
, mTraceIndividualConsumedItemCounts(mTraceProcessedProducedItemCounts || codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))
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
, mHasZeroExtendedStream(P.mHasZeroExtendedStream)

, KernelPartitionId(std::move(P.KernelPartitionId))
, MinimumNumOfStrides(std::move(P.MinimumNumOfStrides))
, MaximumNumOfStrides(std::move(P.MaximumNumOfStrides))

, mStreamGraph(std::move(P.mStreamGraph))
, mScalarGraph(std::move(P.mScalarGraph))
, mInputTruncationGraph(std::move(P.mInputTruncationGraph))
, mBufferGraph(std::move(P.mBufferGraph))
, mPartitioningGraph(std::move(P.mPartitioningGraph))
, mPartitionJumpIndex(std::move(P.mPartitionJumpIndex))
, mPartitionJumpTree(std::move(P.mPartitionJumpTree))
, mConsumerGraph(std::move(P.mConsumerGraph))
, mTerminationGraph(std::move(P.mTerminationGraph))
, mIOCheckGraph(std::move(P.mIOCheckGraph))

, mInputPortSet(constructInputPortMappings())
, mOutputPortSet(constructOutputPortMappings())
, mInitiallyAvailableItemsPhi(FirstStreamSet, LastStreamSet, mAllocator)
, mLocallyAvailableItems(FirstStreamSet, LastStreamSet, mAllocator)
, mScalarValue(FirstKernel, LastScalar, mAllocator)

, mPartitionTerminationSignal(PartitionCount, mAllocator)
, mExhaustedPipelineInputAtPartitionEntry(PartitionCount, mAllocator)
, mInitialConsumedItemCount(FirstStreamSet, LastStreamSet, mAllocator)
, mOriginalBaseAddress(FirstStreamSet, LastStreamSet, mAllocator)

, mPartitionProducedItemCountPhi(extents[PartitionCount][LastStreamSet - FirstStreamSet + 1])
, mPartitionConsumedItemCountPhi(extents[PartitionCount][LastStreamSet - FirstStreamSet + 1])
, mPartitionTerminationSignalPhi(extents[PartitionCount][PartitionCount])
, mPartitionPipelineProgressPhi(PartitionCount, mAllocator)

, mInitiallyProcessedItemCount(this)
, mInitiallyProcessedDeferredItemCount(this)
, mAlreadyProcessedPhi(this)
, mAlreadyProcessedDeferredPhi(this)
, mInputEpoch(this)
, mIsInputZeroExtended(this)
, mInputVirtualBaseAddressPhi(this)
, mFirstInputStrideLength(this)
, mLinearInputItemsPhi(this)
, mReturnedProcessedItemCountPtr(this)
, mProcessedItemCount(this)
, mProcessedDeferredItemCount(this)
, mFinalProcessedPhi(this)
, mInsufficientIOProcessedPhi(this)
, mInsufficientIOProcessedDeferredPhi(this)
, mUpdatedProcessedPhi(this)
, mUpdatedProcessedDeferredPhi(this)
, mFullyProcessedItemCount(this)

, mInitiallyProducedItemCount(FirstStreamSet, LastStreamSet, mAllocator)
, mInitiallyProducedDeferredItemCount(FirstStreamSet, LastStreamSet, mAllocator)

, mAlreadyProducedPhi(this)
, mAlreadyProducedDelayedPhi(this)
, mAlreadyProducedDeferredPhi(this)
, mFirstOutputStrideLength(this)
, mWritableOutputItems(this)
, mLinearOutputItemsPhi(this)
, mReturnedOutputVirtualBaseAddressPtr(this)
, mReturnedProducedItemCountPtr(this)
, mProducedItemCount(this)
, mProducedDeferredItemCount(this)
, mFinalProducedPhi(this)
, mInsufficientIOProducedPhi(this)
, mInsufficientIOProducedDeferredPhi(this)
, mUpdatedProducedPhi(this)
, mUpdatedProducedDeferredPhi(this)
, mFullyProducedItemCount(this)

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
