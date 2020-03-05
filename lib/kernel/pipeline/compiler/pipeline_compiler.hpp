#ifndef PIPELINE_COMPILER_HPP
#define PIPELINE_COMPILER_HPP

#include <kernel/pipeline/pipeline_kernel.h>
#include <kernel/core/kernel_compiler.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>
#include <kernel/core/refwrapper.h>
#include <util/extended_boost_graph_containers.h>
#include <toolchain/toolchain.h>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <boost/dynamic_bitset.hpp>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Transforms/Utils/Local.h>
#include <llvm/IR/ValueMap.h>
#include <llvm/ADT/STLExtras.h>
#include <llvm/ADT/BitVector.h>
#include <util/slab_allocator.h>
#include <util/small_flat_set.hpp>
#include <algorithm>
#include <queue>
#include <z3.h>
#include <util/maxsat.hpp>
#include <assert.h>

#include "analysis/graphs.h"

#define PRINT_DEBUG_MESSAGES

#define PRINT_BUFFER_GRAPH

// #define PERMIT_THREAD_LOCAL_BUFFERS

// #define DISABLE_ZERO_EXTEND

// #define DISABLE_INPUT_ZEROING

// #define DISABLE_OUTPUT_ZEROING

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

struct PipelineGraphBundle {
    static constexpr unsigned PipelineInput = 0U;
    static constexpr unsigned FirstKernel = 1U;
    unsigned LastKernel = 0;
    unsigned PipelineOutput = 0;
    unsigned FirstStreamSet = 0;
    unsigned LastStreamSet = 0;
    unsigned FirstBinding = 0;
    unsigned LastBinding = 0;
    unsigned FirstCall = 0;
    unsigned LastCall = 0;
    unsigned FirstScalar = 0;
    unsigned LastScalar = 0;
    unsigned PartitionCount = 0;
    bool HasZeroExtendedStream = false;

    RelationshipGraph       Streams;
    RelationshipGraph       Scalars;
    OwningVector<Kernel>    InternalKernels;
    OwningVector<Binding>   InternalBindings;
    Partition               KernelPartitionId;

    PipelineGraphBundle(const unsigned n,
                        const unsigned m,
                        const unsigned numOfKernels,
                        OwningVector<Kernel> && internalKernels,
                        OwningVector<Binding> && internalBindings)
    : Streams(n), Scalars(m)
    , InternalKernels(std::move(internalKernels))
    , InternalBindings(std::move(internalBindings))
    , KernelPartitionId(numOfKernels) {

    }
};

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

const static std::string CURRENT_LOGICAL_SEGMENT_NUMBER = "ILSN";
const static std::string PIPELINE_THREAD_LOCAL_STATE = "PTL";
const static std::string KERNEL_THREAD_LOCAL_SUFFIX = ".KTL";

const static std::string ITEM_COUNT_READ_GUARD_SUFFIX = ".LRG";
const static std::string NEXT_LOGICAL_SEGMENT_SUFFIX = ".NSN";
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

template <typename T, unsigned n = 16>
using Vec = SmallVector<T, n>;

using ArgVec = Vec<Value *, 64>;

using Allocator = SlabAllocator<>;

template <typename T>
using OwningVec = std::vector<std::unique_ptr<T>>;

using BufferPortMap = flat_set<std::pair<unsigned, unsigned>>;

using PartitionJumpPhiOutMap = flat_map<std::pair<unsigned, unsigned>, Value *>;

#define BEGIN_SCOPED_REGION {
#define END_SCOPED_REGION }

class PipelineCompiler final : public KernelCompiler {

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

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel, PipelineGraphBundle && P);

// internal pipeline state construction functions

public:

    void addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex);
    void generateSingleThreadKernelMethod(BuilderRef b);
    void generateMultiThreadKernelMethod(BuilderRef b);

// main doSegment functions

    void start(BuilderRef b);
    void setActiveKernel(BuilderRef b, const unsigned index);
    void executeKernel(BuilderRef b);
    void end(BuilderRef b);

    void readPipelineIOItemCounts(BuilderRef b);
    void writeExternalProducedItemCounts(BuilderRef b);

// internal pipeline functions

    LLVM_READNONE StructType * getThreadStateType(BuilderRef b) const;
    Value * allocateThreadState(BuilderRef b, Value * const threadId);
    void readThreadState(BuilderRef b, Value * threadState);
    void deallocateThreadState(BuilderRef b, Value * const threadState);

    LLVM_READNONE StructType * getThreadLocalStateType(BuilderRef b);
    void allocateThreadLocalState(BuilderRef b, Value * const localState, Value * const threadId = nullptr);
    void bindCompilerVariablesToThreadLocalState(BuilderRef b);
    void deallocateThreadLocalState(BuilderRef b, Value * const localState);
    Value * readTerminationSignalFromLocalState(BuilderRef b, Value * const localState) const;
    inline Value * isProcessThread(BuilderRef b, Value * const threadState) const;

    void addTerminationProperties(BuilderRef b, const size_t kernel);

// partitioning codegen functions

    void addPartitionInputItemCounts(BuilderRef b, const size_t partitionId) const;
    void makePartitionEntryPoints(BuilderRef b);
    void branchToInitialPartition(BuilderRef b);
    BasicBlock * getPartitionExitPoint(BuilderRef b);
    void checkPartitionEntry(BuilderRef b);
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffersInPartition(BuilderRef b) const;
    void jumpToNextPartition(BuilderRef b);
    void checkForPartitionExit(BuilderRef b);
    void setCurrentPartitionTerminationSignal(Value * const signal);
    Value * getCurrentPartitionTerminationSignal() const;

// inter-kernel codegen functions

    void readProcessedItemCounts(BuilderRef b);
    void readProducedItemCounts(BuilderRef b);

    void initializeKernelLoopEntryPhis(BuilderRef b);
    void initializeKernelCheckOutputSpacePhis(BuilderRef b);
    void initializeKernelTerminatedPhis(BuilderRef b);
    void initializeKernelInsufficientIOExitPhis(BuilderRef b);
    void initializeKernelLoopExitPhis(BuilderRef b);
    void initializeKernelExitPhis(BuilderRef b);

    void determineNumOfLinearStrides(BuilderRef b);
    void checkForSufficientInputData(BuilderRef b, const StreamSetPort inputPort);
    void ensureSufficientOutputSpace(BuilderRef b, const StreamSetPort outputPort);
    void updatePHINodesForLoopExit(BuilderRef b);

    void calculateItemCounts(BuilderRef b);
    Value * determineIsFinal(BuilderRef b);
    std::pair<Value *, Value *> calculateFinalItemCounts(BuilderRef b, Vec<Value *> & accessibleItems, Vec<Value *> & writableItems);
    void zeroInputAfterFinalItemCount(BuilderRef b, const Vec<Value *> & accessibleItems, Vec<Value *> & inputBaseAddresses);

    void checkForLastPartialSegment(BuilderRef b, Value * isFinal);
    Value * noMoreInputData(BuilderRef b, const unsigned inputPort);
    Value * noMoreOutputData(BuilderRef b, const unsigned outputPort);

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

    void replacePhiCatchBlocksWith(BasicBlock *& from, BasicBlock * const to);

    void writeOutputScalars(BuilderRef b, const size_t index, std::vector<Value *> & args);
    Value * getScalar(BuilderRef b, const size_t index);


// intra-kernel codegen functions

    Value * getInputStrideLength(BuilderRef b, const StreamSetPort inputPort);
    Value * getOutputStrideLength(BuilderRef b, const StreamSetPort outputPort);
    Value * getFirstStrideLength(BuilderRef b, const size_t kernel, const StreamSetPort port);
    Value * calculateNumOfLinearItems(BuilderRef b, const StreamSetPort port, Value * const adjustment);
    Value * getAccessibleInputItems(BuilderRef b, const StreamSetPort inputPort, const bool useOverflow = true);
    Value * getNumOfAccessibleStrides(BuilderRef b, const StreamSetPort inputPort);
    Value * getNumOfWritableStrides(BuilderRef b, const StreamSetPort outputPort);
    Value * getWritableOutputItems(BuilderRef b, const StreamSetPort outputPort, const bool useOverflow = true);
    Value * addLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount) const;
    Value * subtractLookahead(BuilderRef b, const StreamSetPort inputPort, Value * const itemCount);
    Constant * getLookahead(BuilderRef b, const StreamSetPort inputPort) const;
    Value * truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount, Value * const terminationSignal) const;

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
    Value * readTerminationSignal(BuilderRef b);
    void writeTerminationSignal(BuilderRef b, Value * const signal);
    Value * hasPipelineTerminated(BuilderRef b) const;
    void signalAbnormalTermination(BuilderRef b);
    LLVM_READNONE static Constant * getTerminationSignal(BuilderRef b, const TerminationSignal type);

// consumer codegen functions

    ConsumerGraph makeConsumerGraph() const;
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
    void loadInternalStreamSetHandles(BuilderRef b);
    void releaseOwnedBuffers(BuilderRef b, const bool nonLocal);
    void resetInternalBufferHandles();
    void loadLastGoodVirtualBaseAddressesOfUnownedBuffers(BuilderRef b, const size_t kernelId) const;
    LLVM_READNONE bool requiresCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE bool requiresLookAhead(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getLookAhead(const unsigned bufferVertex) const;

    void prepareLinearBuffers(BuilderRef b) const;
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

    void addItemCountDeltaProperties(BuilderRef b, unsigned kernel, const StringRef suffix) const;

    void recordItemCountDeltas(BuilderRef b, const Vec<Value *> & current, const Vec<Value *> & prior, const StringRef suffix) const;

    void printItemCountDeltas(BuilderRef b, const StringRef title, const StringRef suffix) const;

// internal optimization passes

    void simplifyPhiNodes(Module * const m) const;

// pipeline analysis functions

    using KernelPartitionIds = flat_map<Relationships::vertex_descriptor, unsigned>;

    PipelineGraphBundle makePipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel);
    Relationships generateInitialPipelineGraph(BuilderRef b,
                                               PipelineKernel * const pipelineKernel,
                                               OwningVector<Kernel> & internalKernels,
                                               OwningVector<Binding> & internalBindings);

    using KernelVertexVec = SmallVector<Relationships::Vertex, 64>;

    void addRegionSelectorKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G,
                                  OwningVector<Kernel> & internalKernels, OwningVector<Binding> & internalBindings);

    void addPopCountKernels(BuilderRef b, Kernels & partition, KernelVertexVec & vertex, Relationships & G,
                            OwningVector<Kernel> &internalKernels, OwningVector<Binding> &internalBindings);

    static void combineDuplicateKernels(BuilderRef b, const Kernels & partition, Relationships & G);
    static void removeUnusedKernels(const PipelineKernel * pipelineKernel, const unsigned p_in, const unsigned p_out, const Kernels & partition, Relationships & G);

    void identifyPipelineInputs();

// partitioning analysis

    unsigned partitionIntoFixedRateRegionsWithOrderingConstraints(Relationships & G,
                                                                  KernelPartitionIds & partitionIds,
                                                                  const PipelineKernel * const pipelineKernel) const;

    unsigned identifyKernelPartitions(const Relationships &G,
                                                    const std::vector<unsigned> & orderingOfG,
                                                    const PipelineKernel * const pipelineKernel,
                                                    KernelPartitionIds & partitionIds) const;

    void addOrderingConstraintsToPartitionSubgraphs(Relationships & G,
                                                    const std::vector<unsigned> & orderingOfG,
                                                    const KernelPartitionIds & partitionIds,
                                                    const unsigned numOfPartitions) const;

    PartitioningGraph generatePartitioningGraph() const;
    Vec<unsigned> determinePartitionJumpIndices() const;
    PartitionJumpTree makePartitionJumpTree() const;


// buffer management analysis functions

    BufferGraph makeBufferGraph(BuilderRef b);
    void initializeBufferGraph(BufferGraph & G) const;
    void verifyIOStructure(const BufferGraph & G) const;
    void identifyLinearBuffers(BufferGraph & G) const;
    void identifyNonLocalBuffers(BufferGraph & G) const;
    BufferPortMap constructInputPortMappings() const;
    BufferPortMap constructOutputPortMappings() const;
    bool mayHaveNonLinearIO(const size_t kernel) const;
    bool supportsInternalSynchronization() const;
    void identifyLocalPortIds(BufferGraph & G) const;
    bool isBounded() const;

    void printBufferGraph(raw_ostream & out) const;

// dataflow analysis functions

    void computeDataFlowRates(BufferGraph & G);
    PartitionConstraintGraph identifyHardPartitionConstraints(BufferGraph & G) const;
    LengthConstraintGraph identifyLengthEqualityAssertions(BufferGraph & G) const;

// zero extension analysis function

    bool hasZeroExtendedStreams(BufferGraph & G) const;

// termination analysis functions

    TerminationGraph makeTerminationGraph() const;

// add(k) analysis functions

    AddGraph makeAddGraph() const;

// IO analysis functions

    IOCheckGraph makeKernelIOGraph() const;

// synchronization functions

    enum class LockType {
        Segment,
        ItemCheck,
    };

    void acquireSynchronizationLock(BuilderRef b, const LockType lockType, const CycleCounter start);
    void releaseSynchronizationLock(BuilderRef b, const LockType lockType);
    void verifySynchronizationLock(BuilderRef b, const LockType lockType);


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

    LLVM_READNONE RelationshipGraph::edge_descriptor getReferenceEdge(const size_t kernel, const StreamSetPort port) const;
    LLVM_READNONE unsigned getReferenceBufferVertex(const size_t kernel, const StreamSetPort port) const;
    LLVM_READNONE const StreamSetPort getReference(const size_t kernel, const StreamSetPort port) const;
    const StreamSetPort getReference(const StreamSetPort port) const;

    LLVM_READNONE unsigned getInputBufferVertex(const size_t kernelVertex, const StreamSetPort inputPort) const;
    unsigned getInputBufferVertex(const StreamSetPort inputPort) const;
    StreamSetBuffer * getInputBuffer(const StreamSetPort inputPort) const;
    LLVM_READNONE StreamSetBuffer * getInputBuffer(const size_t kernelVertex, const StreamSetPort inputPort) const;
    LLVM_READNONE const Binding & getInputBinding(const size_t kernelVertex, const StreamSetPort inputPort) const;
    const Binding & getInputBinding(const StreamSetPort inputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getInput(const size_t kernelVertex, const StreamSetPort outputPort) const;
    const Binding & getProducerOutputBinding(const StreamSetPort inputPort) const;

    LLVM_READNONE unsigned getOutputBufferVertex(const size_t kernelVertex, const StreamSetPort outputPort) const;
    unsigned getOutputBufferVertex(const StreamSetPort outputPort) const;
    StreamSetBuffer * getOutputBuffer(const StreamSetPort outputPort) const;
    LLVM_READNONE StreamSetBuffer * getOutputBuffer(const size_t kernelVertex, const StreamSetPort outputPort) const;
    LLVM_READNONE const Binding & getOutputBinding(const size_t kernelVertex, const StreamSetPort outputPort) const;
    const Binding & getOutputBinding(const StreamSetPort outputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getOutput(const size_t kernelVertex, const StreamSetPort outputPort) const;

    LLVM_READNONE unsigned getNumOfStreamInputs(const unsigned kernel) const;
    LLVM_READNONE unsigned getNumOfStreamOutputs(const unsigned kernel) const;

    LLVM_READNONE unsigned getBufferIndex(const unsigned bufferVertex) const;

    const Binding & getBinding(const StreamSetPort port) const;
    LLVM_READNONE const Binding & getBinding(const unsigned kernel, const StreamSetPort port) const;
    LLVM_READNONE const Kernel * getKernel(const unsigned index) const;

protected:

    Allocator									mAllocator;

    const bool                       			mCheckAssertions;
    const bool                                  mTraceProcessedProducedItemCounts;
    const bool                       			mTraceIndividualConsumedItemCounts;

    const unsigned								mNumOfThreads;
    const unsigned                              mNumOfSegments;

    const LengthAssertions &                    mLengthAssertions;

    // analysis state
    const RelationshipGraph                     mStreamGraph;
    const RelationshipGraph                     mScalarGraph;
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
    const Partition                             KernelPartitionId;
    const unsigned                              PartitionCount;

    std::vector<Rational>                       MinimumNumOfStrides;
    std::vector<Rational>                       MaximumNumOfStrides;

    const bool                                  ExternallySynchronized;
    const bool                                  PipelineHasTerminationSignal;

    const BufferGraph                           mBufferGraph;
    const PartitioningGraph                     mPartitioningGraph;
    const Vec<unsigned>                         mPartitionJumpIndex;
    const PartitionJumpTree                     mPartitionJumpTree;


    const BufferPortMap                         mInputPortSet;
    const BufferPortMap                         mOutputPortSet;

    const bool                                  mHasZeroExtendedStream;
    bool                                        mHasThreadLocalPipelineState;

    const ConsumerGraph                         mConsumerGraph;

    Vec<Value *>                                mScalarValue;
    const TerminationGraph                      mTerminationGraph;
    const AddGraph                              mAddGraph;
    const IOCheckGraph                          mIOCheckGraph;


    // pipeline state
    unsigned                                    mKernelId = 0;
    const Kernel *                              mKernel = nullptr;
    Value *                                     mKernelHandle = nullptr;

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
    BasicBlock *                                mKernelLoopExit = nullptr;
    BasicBlock *                                mKernelLoopExitPhiCatch = nullptr;
    BasicBlock *                                mKernelExit = nullptr;
    BasicBlock *                                mPipelineEnd = nullptr;
    BasicBlock *                                mRethrowException = nullptr;

    Vec<AllocaInst *, 16>                       mAddressableItemCountPtr;
    Vec<AllocaInst *, 16>                       mVirtualBaseAddressPtr;
    Vec<AllocaInst *, 4>                        mTruncatedInputBuffer;

    Vec<Value *, 128>                           mLocallyAvailableItems;

    // partition state
    Vec<BasicBlock *>                           mPartitionEntryPoint;
    Vec<BasicBlock *>                           mPartitionJumpExitPoint;
    Vec<BasicBlock *>                           mPartitionExitPoint;
    unsigned                                    mCurrentPartitionId = 0;
    unsigned                                    mPartitionRootKernelId = 0;
    Value *                                     mNumOfPartitionStrides = nullptr;

    Vec<PHINode *>                              mPipelineProgressAtPartitionExit;
    Vec<Value *>                                mPartitionTerminationSignalAtJumpExit;
    Vec<Value *>                                mPartitionTerminationSignal;

    Vec<PHINode *>                              mExhaustedPipelineInputAtPartitionEntry;

    Vec<Value *>                                mConsumedItemCount;

    PartitionJumpPhiOutMap                      mPartitionProducedItemCountAtJumpExit;
    PartitionJumpPhiOutMap                      mPartitionConsumedItemCountAtJumpExit;


    // kernel state
    Value *                                     mTerminatedInitially = nullptr;
    Value *                                     mTerminatedInitiallyCheck = nullptr;
    Value *                                     mMaximumNumOfStrides = nullptr;
    PHINode *                                   mCurrentNumOfStrides = nullptr;
    Value *                                     mUpdatedNumOfStrides = nullptr;
    PHINode *                                   mTotalNumOfStrides = nullptr;
    PHINode *                                   mHasProgressedPhi = nullptr;
    PHINode *                                   mAlreadyProgressedPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputAtPartitionJumpPhi = nullptr;
    PHINode *                                   mExhaustedPipelineInputAtLoopExitPhi = nullptr;
    Value *                                     mExhaustedPipelineInputAtExit = nullptr;    
    PHINode *                                   mExecutedAtLeastOncePhi = nullptr;
    PHINode *                                   mTerminatedSignalPhi = nullptr;
    PHINode *                                   mTerminatedAtLoopExitPhi = nullptr;
    PHINode *                                   mTerminatedAtExitPhi = nullptr;
    PHINode *                                   mTotalNumOfStridesAtExitPhi = nullptr;
    Value *                                     mLastPartialSegment = nullptr;
    Value *                                     mNumOfLinearStrides = nullptr;
    Value *                                     mHasZeroExtendedInput = nullptr;
    Value *                                     mAnyRemainingInput = nullptr;
    PHINode *                                   mFixedRateFactorPhi = nullptr;
    PHINode *                                   mReportedNumOfStridesPhi = nullptr;
    PHINode *                                   mNextNumOfPartitionStridesPhi = nullptr;
    PHINode *                                   mIsFinalInvocationPhi = nullptr;
    PHINode *                                   mNextNumOfPartitionStridesAtLoopExitPhi = nullptr;
    BasicBlock *                                mNextPartitionWithPotentialInput = nullptr;

    BitVector                                   mHasPipelineInput;

    Rational                                    mFixedRateLCM;
    Value *                                     mTerminatedExplicitly = nullptr;
    Value *                                     mBranchToLoopExit = nullptr;

    Value *                                     mKernelAssertionName = nullptr;

    bool                                        mMayHaveNonLinearIO = false;
    bool                                        mIsBounded = false;
    bool                                        mKernelIsInternallySynchronized = false;
    bool                                        mKernelCanTerminateEarly = false;
    bool                                        mHasExplicitFinalPartialStride = false;
    bool                                        mIsPartitionRoot = false;

    unsigned                                    mNumOfAddressableItemCount = 0;
    unsigned                                    mNumOfVirtualBaseAddresses = 0;
    unsigned                                    mNumOfTruncatedInputBuffers = 0;

    PHINode *                                   mZeroExtendBufferPhi = nullptr;

    InputPortVec<Value *>                       mInitiallyProcessedItemCount; // *before* entering the kernel
    InputPortVec<Value *>                       mInitiallyProcessedDeferredItemCount;
    InputPortVec<PHINode *>                     mAlreadyProcessedPhi; // entering the segment loop
    InputPortVec<PHINode *>                     mAlreadyProcessedDeferredPhi;
    InputPortVec<Value *>                       mInputEpoch;
    InputPortVec<Value *>                       mIsInputZeroExtended;
    InputPortVec<PHINode *>                     mInputVirtualBaseAddressPhi;
    InputPortVec<Value *>                       mFirstInputStrideLength;
    InputPortVec<Value *>                       mAccessibleInputItems;
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

    Vec<Value *>                                mInitiallyProducedItemCount; // *before* entering the kernel
    OutputPortVec<Value *>                      mInitiallyProducedDeferredItemCount;
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
    Value *                                     mThreadId;
    Value *                                     mDebugFileName;
    Value *                                     mDebugFdPtr;

    // misc.

    OwningVec<StreamSetBuffer>                  mInternalBuffers;
    OwningVec<Kernel>                           mInternalKernels;
    OwningVec<Binding>                          mInternalBindings;

};


// NOTE: these graph functions not safe for general use since they are intended for inspection of *edge-immutable* graphs.

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor first_in_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    return *in_edges(u, G).first;
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor in_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (in_degree(u, G) == 1);
    return first_in_edge(u, G);
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::vertex_descriptor parent(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    return source(in_edge(u, G), G);
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor first_out_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    return *out_edges(u, G).first;
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor out_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (out_degree(u, G) == 1);
    return first_out_edge(u, G);
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::vertex_descriptor child(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    return target(out_edge(u, G), G);
}

template <typename Graph>
LLVM_READNONE
inline bool is_parent(const typename graph_traits<Graph>::vertex_descriptor u,
                      const typename graph_traits<Graph>::vertex_descriptor v,
                      const Graph & G) {
    return parent(u, G) == v;
}

template <typename Graph>
LLVM_READNONE
inline bool has_child(const typename graph_traits<Graph>::vertex_descriptor u,
                      const typename graph_traits<Graph>::vertex_descriptor v,
                      const Graph & G) {
    for (const auto e : make_iterator_range(out_edges(u, G))) {
        if (target(e, G) == v) {
            return true;
        }
    }
    return false;
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
inline PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel)
: PipelineCompiler(b, pipelineKernel, makePipelineGraph(b, pipelineKernel)) {
    // Use a delegating constructor to compute the pipeline graph data once and pass it to
    // the compiler. Although a const function attribute ought to suffice, gcc 8.2 does not
    // resolve it correctly and clang requires -O2 or better.
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel, PipelineGraphBundle && P)
: KernelCompiler(pipelineKernel)
, mCheckAssertions(codegen::DebugOptionIsSet(codegen::EnableAsserts))
, mTraceProcessedProducedItemCounts(codegen::DebugOptionIsSet(codegen::TraceCounts))
, mTraceIndividualConsumedItemCounts(mTraceProcessedProducedItemCounts || codegen::DebugOptionIsSet(codegen::TraceDynamicBuffers))
, mNumOfThreads(pipelineKernel->getNumOfThreads())
, mNumOfSegments(pipelineKernel->getNumOfSegments())
, mLengthAssertions(pipelineKernel->getLengthAssertions())
, mStreamGraph(std::move(P.Streams))
, mScalarGraph(std::move(P.Scalars))
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
, KernelPartitionId(std::move(P.KernelPartitionId))
, PartitionCount(P.PartitionCount)

, ExternallySynchronized(pipelineKernel->hasAttribute(AttrId::InternallySynchronized))
, PipelineHasTerminationSignal(pipelineKernel->canSetTerminateSignal())

, mBufferGraph(makeBufferGraph(b))

, mPartitioningGraph(generatePartitioningGraph())
, mPartitionJumpIndex(determinePartitionJumpIndices())
, mPartitionJumpTree(makePartitionJumpTree())

, mInputPortSet(constructInputPortMappings())
, mOutputPortSet(constructOutputPortMappings())

, mConsumedItemCount(LastStreamSet + 1)

// TODO: refactor to remove the following const cast
, mHasZeroExtendedStream(hasZeroExtendedStreams(const_cast<BufferGraph &>(mBufferGraph)))

, mConsumerGraph(makeConsumerGraph())
, mScalarValue(LastScalar + 1)
, mTerminationGraph(makeTerminationGraph())
, mAddGraph(makeAddGraph())
, mIOCheckGraph(makeKernelIOGraph())

, mInitiallyProcessedItemCount(this)
, mInitiallyProcessedDeferredItemCount(this)
, mAlreadyProcessedPhi(this)
, mAlreadyProcessedDeferredPhi(this)
, mInputEpoch(this)
, mIsInputZeroExtended(this)
, mInputVirtualBaseAddressPhi(this)
, mFirstInputStrideLength(this)
, mAccessibleInputItems(this)
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


, mInitiallyProducedItemCount(LastStreamSet + 1)
, mInitiallyProducedDeferredItemCount(this)
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

, mInternalKernels(std::move(P.InternalKernels))
, mInternalBindings(std::move(P.InternalBindings))



{
    #ifdef PRINT_BUFFER_GRAPH
    printBufferGraph(errs());
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

inline size_t round_up_to(const size_t x, const size_t y) {
    assert(is_power_2(y));
    return (x + y - 1) & -y;
}

#ifndef NDEBUG
static bool isFromCurrentFunction(BuilderRef b, const Value * const value, const bool allowNull = true) {
    if (value == nullptr) {
        return allowNull;
    }
    if (LLVM_UNLIKELY(&b->getContext() != &value->getContext())) {
        return false;
    }
    if (isa<Constant>(value)) {
        return true;
    }
    const Function * const builderFunction = b->GetInsertBlock()->getParent();
    const Function * function = builderFunction;
    if (isa<Argument>(value)) {
        function = cast<Argument>(value)->getParent();
    } else if (isa<Instruction>(value)) {
        function = cast<Instruction>(value)->getParent()->getParent();
    }
    return (builderFunction == function);
}
#endif

} // end of namespace

#include "analysis/analysis.hpp"
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
