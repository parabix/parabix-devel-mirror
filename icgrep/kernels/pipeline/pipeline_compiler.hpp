#ifndef PIPELINE_COMPILER_HPP
#define PIPELINE_COMPILER_HPP

#include <kernels/pipeline_kernel.h>
#include <kernels/core/streamset.h>
#include <kernels/kernel_builder.h>
#include <kernels/pipeline/regionselectionkernel.h>
#include <toolchain/toolchain.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/range/adaptor/reversed.hpp>
//#include <boost/serialization/strong_typedef.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <boost/dynamic_bitset.hpp>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/STLExtras.h>
#include <util/slab_allocator.h>
#include <algorithm>
#include <queue>

// #define PRINT_DEBUG_MESSAGES

using namespace boost;
using namespace boost::math;
using namespace boost::adaptors;
using boost::container::flat_set;
using boost::container::flat_map;
using namespace llvm;

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 31 - __builtin_clz(v);
}

namespace kernel {

#include <util/enum_flags.hpp>

using PortType = Kernel::PortType;
using StreamPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using RateValue = ProcessingRate::RateValue;
using RateId = ProcessingRate::KindId;
using Scalars = PipelineKernel::Scalars;
using Kernels = PipelineKernel::Kernels;
using CallBinding = PipelineKernel::CallBinding;
using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

// TODO: replace ints used for port #s with the following
// BOOST_STRONG_TYPEDEF(unsigned, PortNumber)

union RelationshipNode {
    const kernel::Kernel * Kernel = nullptr;
    const kernel::Relationship * Relationship;
    llvm::Function * Callee;

    RelationshipNode() = default;
    RelationshipNode(kernel::Kernel * kernel) : Kernel(kernel) { }
    RelationshipNode(kernel::Relationship * relationship) : Relationship(relationship) { }
    RelationshipNode(llvm::Function * callee) : Callee(callee) { }
};

enum class ReasonType : unsigned {
    Explicit = 0
    , ImplicitRegionSelector = 1
};

struct RelationshipType : public StreamPort {

    ReasonType Reason;

    RelationshipType() : StreamPort(), Reason(ReasonType::Explicit) { }
    RelationshipType & operator = (const RelationshipType &) = default;
    RelationshipType(PortType type, unsigned number, ReasonType reason = ReasonType::Explicit) : StreamSetPort(type, number), Reason(reason) { }
    RelationshipType(StreamPort port) : StreamSetPort(port), Reason(ReasonType::Explicit) { }
};

using RelationshipGraph = adjacency_list<vecS, vecS, bidirectionalS, RelationshipNode, RelationshipType>;


enum class BufferType : unsigned {
    Internal = 0
    , Managed = 1
    , External = 2
};

struct BufferNode {
    StreamSetBuffer * Buffer = nullptr;
    RateValue Lower{};
    RateValue Upper{};
    unsigned Overflow = 0;
    unsigned Fasimile = 0;
    BufferType Type = BufferType::Internal;

    ~BufferNode() {
        if (LLVM_LIKELY(Type != BufferType::External)) {
            delete Buffer;
        }
    }
};

inline unsigned InputPort(const StreamPort port) {
    assert (port.Type == PortType::Input);
    return port.Number;
}

inline unsigned OutputPort(const StreamPort port) {
    assert (port.Type == PortType::Output);
    return port.Number;
}

struct BufferRateData {

    RelationshipType Port;
    BindingRef Binding;
    RateValue  Minimum;
    RateValue  Maximum;

    unsigned inputPort() const {
        return InputPort(Port);
    }

    unsigned outputPort() const {
        return OutputPort(Port);
    }

    BufferRateData() = default;

    BufferRateData(RelationshipType port, BindingRef binding, RateValue min, RateValue max)
    : Port(port), Binding(binding), Minimum(min), Maximum(max) { }

};

using BufferGraph = adjacency_list<vecS, vecS, bidirectionalS, BufferNode, BufferRateData>;

template <typename vertex_descriptor>
using RelMap = flat_map<const Relationship *, vertex_descriptor>;

using BufferMap = RelMap<BufferGraph::vertex_descriptor>;

struct ConsumerNode {
    Value * Consumed = nullptr;
    PHINode * PhiNode = nullptr;
};

using ConsumerGraph = adjacency_list<vecS, vecS, bidirectionalS, ConsumerNode, StreamPort>;

template <typename Value>
using StreamSetBufferMap = flat_map<const StreamSetBuffer *, Value>;

template <typename Value>
using KernelMap = flat_map<const Kernel *, Value>;

using TerminationGraph = adjacency_list<hash_setS, vecS, bidirectionalS, unsigned, unsigned>;

using RelationshipVertex = RelationshipGraph::vertex_descriptor;
using RelationshipMap = RelMap<RelationshipVertex>;

using ScalarGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

struct OverflowRequirement {
    unsigned copyBack;
    unsigned facsimile;
    OverflowRequirement() = default;
    OverflowRequirement(const unsigned copyBack, const unsigned copyForward)
    : copyBack(copyBack), facsimile(copyForward) { }
};

using OverflowRequirements = StreamSetBufferMap<OverflowRequirement>;

struct PopCountData {
    // compilation state
    PHINode *    PhiNode;
    Value *      Processed;
    unsigned     Encountered;
    Value *      InitialOffset;

    // analysis state
    RateValue    FieldWidth;
    bool         UsesConsumedCount;

    PopCountData() = default;
};

enum CountingType : unsigned {
    Unknown = 0
    , Positive = 1
    , Negative = 2
    , Both = Positive | Negative
};

ENABLE_ENUM_FLAGS(CountingType)

struct PopCountEdge {
    CountingType Type;
    unsigned     Port;
    PopCountEdge() : Type(Unknown), Port(0) { }
    PopCountEdge(const CountingType type, const unsigned port) : Type(type), Port(port) { }
};

using PopCountGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, PopCountEdge>;

using PipelineIOGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

struct RegionData {
    AttrId   Type;
    unsigned Stream;
    RegionData() : Type(AttrId::None), Stream(0) { }
    RegionData(const AttrId type, const unsigned stream) : Type(type), Stream(stream) { }
};

struct PipelineGraphBundle {
    static constexpr unsigned PipelineInput = 0U;
    static constexpr unsigned FirstKernel = 1U;
    unsigned LastKernel = 0;
    unsigned PipelineOutput = 0;
    unsigned FirstCall = 0;
    unsigned LastCall = 0;
    unsigned FirstScalar = 0;
    unsigned LastScalar = 0;
    unsigned FirstStreamSet = 0;
    unsigned LastStreamSet = 0;
    RelationshipGraph Graph;
    PipelineGraphBundle(const unsigned n) : Graph(n) { }
};

using ImplicitRelationships = flat_map<const Kernel *, const StreamSet *>;

const static std::string CURRENT_LOGICAL_SEGMENT_NUMBER = "ILSN";
const static std::string PIPELINE_THREAD_LOCAL_STATE = "PTL";
const static std::string KERNEL_THREAD_LOCAL_SUFFIX = ".KTL";
const static std::string LOGICAL_SEGMENT_SUFFIX = ".LSN";
const static std::string TERMINATION_PREFIX = "@TERM";
const static std::string ITEM_COUNT_SUFFIX = ".IC";
const static std::string DEFERRED_ITEM_COUNT_SUFFIX = ".DC";
const static std::string CONSUMED_ITEM_COUNT_SUFFIX = ".CON";
const static std::string CYCLE_COUNT_SUFFIX = ".CYC";

class PipelineCompiler {

    template <typename T, unsigned n = 16>
    using Vec = SmallVector<T, n>;

public:

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel);

    void addPipelineKernelProperties(BuilderRef b);
    void generateInitializeMethod(BuilderRef b);
    void generateInitializeThreadLocalMethod(BuilderRef b);
    void generateKernelMethod(BuilderRef b);
    void generateFinalizeMethod(BuilderRef b);
    void generateFinalizeThreadLocalMethod(BuilderRef b);
    std::vector<Value *> getFinalOutputScalars(BuilderRef b);

protected:

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel, PipelineGraphBundle && P);

// internal pipeline state construction functions

    void addInternalKernelProperties(BuilderRef b, const unsigned kernelIndex);
    void generateSingleThreadKernelMethod(BuilderRef b);
    void generateMultiThreadKernelMethod(BuilderRef b);
    void acquireCurrentSegment(BuilderRef b);
    void releaseCurrentSegment(BuilderRef b);
    LLVM_READNONE bool requiresSynchronization(const unsigned kernelIndex) const;

// main pipeline functions

    void start(BuilderRef b);
    void setActiveKernel(BuilderRef b, const unsigned index);
    void executeKernel(BuilderRef b);
    Value * end(BuilderRef b);

    void readPipelineIOItemCounts(BuilderRef b);
    void writePipelineIOItemCounts(BuilderRef b);

// internal pipeline functions

    LLVM_READNONE StructType * getThreadStateType(BuilderRef b) const;
    Value * allocateThreadState(BuilderRef b, Value * const threadId);
    void readThreadState(BuilderRef b, Value * threadState);
    void deallocateThreadState(BuilderRef b, Value * const threadState);

    LLVM_READNONE StructType * getThreadLocalStateType(BuilderRef b);
    void allocateThreadLocalState(BuilderRef b, Value * const localState, Value * const threadId = nullptr);
    void bindCompilerVariablesToThreadLocalState(BuilderRef b, Value * const localState);
    void deallocateThreadLocalState(BuilderRef b, Value * const localState);
    Value * readTerminationSignalFromLocalState(BuilderRef b, Value * const localState) const;
    inline Value * isProcessThread(BuilderRef b, Value * const threadState) const;

    void addTerminationProperties(BuilderRef b);

// inter-kernel functions

    void readInitialItemCounts(BuilderRef b);

    void initializeKernelLoopEntryPhis(BuilderRef b);
    void initializeKernelCallPhis(BuilderRef b);
    void initializeKernelTerminatedPhis(BuilderRef b);
    void initializeKernelLoopExitPhis(BuilderRef b);
    void initializeKernelExitPhis(BuilderRef b);

    void checkForSufficientInputDataAndOutputSpace(BuilderRef b);
    void branchToTargetOrLoopExit(BuilderRef b, Value * const cond, BasicBlock * target, Value * const halting);
    void determineNumOfLinearStrides(BuilderRef b);

    void enterRegionSpan(BuilderRef b);

    void calculateNonFinalItemCounts(BuilderRef b);
    void calculateFinalItemCounts(BuilderRef b);

    void prepareLocalZeroExtendSpace(BuilderRef b);

    void writeKernelCall(BuilderRef b);
    Value * addItemCountArg(BuilderRef b, const Binding & binding, const bool addressable, PHINode * const itemCount, Vec<Value *, 64> & args);

    void exitRegionSpan(BuilderRef b);

    void normalTerminationCheck(BuilderRef b, Value * const isFinal);

    void writeCopyBackLogic(BuilderRef b);
    void writeCopyForwardLogic(BuilderRef b);
    enum class OverflowCopy { Forwards, Backwards };
    void writeOverflowCopy(BuilderRef b, const OverflowCopy direction, Value * cond, const Binding & binding, const StreamSetBuffer * const buffer, Value * const itemsToCopy) const;


    void computeFullyProcessedItemCounts(BuilderRef b);
    void computeFullyProducedItemCounts(BuilderRef b);

    void updatePhisAfterTermination(BuilderRef b);

    void clearUnwrittenOutputData(BuilderRef b);

    void computeMinimumConsumedItemCounts(BuilderRef b);
    void writeFinalConsumedItemCounts(BuilderRef b);
    void readFinalProducedItemCounts(BuilderRef b);

    void writeCopyToOverflowLogic(BuilderRef b);
    void checkForSufficientInputData(BuilderRef b, const unsigned inputPort);
    void checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned outputPort);

    void loadItemCountsOfCountableRateStreams(BuilderRef b);

    void writeUpdatedItemCounts(BuilderRef b, const bool final);

    void writeOutputScalars(BuilderRef b, const unsigned index, std::vector<Value *> & args);
    Value * getScalar(BuilderRef b, const unsigned index);

// intra-kernel functions

    void expandOutputBuffer(BuilderRef b, const unsigned outputPort, Value * const hasEnough, BasicBlock * const target);

    Value * getInputStrideLength(BuilderRef b, const unsigned inputPort);
    Value * getOutputStrideLength(BuilderRef b, const unsigned outputPort);
    Value * getInitialStrideLength(BuilderRef b, const StreamPort port);
    Value * getMaximumStrideLength(BuilderRef b, const Binding & binding);
    Value * calculateNumOfLinearItems(BuilderRef b, const Binding & binding);
    Value * getAccessibleInputItems(BuilderRef b, const unsigned inputPort, const bool addFacsimile);
    Value * getNumOfAccessibleStrides(BuilderRef b, const unsigned inputPort);
    Value * getNumOfWritableStrides(BuilderRef b, const unsigned outputPort);
    Value * getWritableOutputItems(BuilderRef b, const unsigned outputPort, const bool addOverflow);
    Value * calculateBufferExpansionSize(BuilderRef b, const unsigned outputPort);
    Value * addLookahead(BuilderRef b, const unsigned inputPort, Value * itemCount) const;
    Value * subtractLookahead(BuilderRef b, const unsigned inputPort, Value * const itemCount);
    Constant * getLookahead(BuilderRef b, const unsigned inputPort) const;
    Value * truncateBlockSize(BuilderRef b, const Binding & binding, Value * itemCount) const;
    Value * getLocallyAvailableItemCount(BuilderRef b, const unsigned inputPort) const;
    void resetMemoizedFields();

// termination functions

    Value * hasKernelTerminated(BuilderRef b, const unsigned kernel) const;
    LLVM_READNONE Constant * getTerminationSignal(BuilderRef b, const unsigned kernel) const;
    Value * isClosed(BuilderRef b, const unsigned inputPort);
    Value * initiallyTerminated(BuilderRef b);
    void setTerminated(BuilderRef b) const;
    Value * pipelineTerminated(BuilderRef b) const;
    void updateTerminationSignal(Value * const signal);

    void loadTerminationSignals(BuilderRef b);
    void storeTerminationSignals(BuilderRef b);

// pop-count functions

    void addPopCountScalarsToPipelineKernel(BuilderRef b, const unsigned index);

    void initializePopCounts();
    void writePopCountComputationLogic(BuilderRef b);

    void initializePopCountReferenceItemCount(BuilderRef b, const unsigned bufferVertex, not_null<Value *> produced);
    void createPopCountReferenceCounts(BuilderRef b);
    void computeMinimumPopCountReferenceCounts(BuilderRef b);
    void updatePopCountReferenceCounts(BuilderRef b);
    LLVM_READNONE Value * getPopCountNextBaseOffset(BuilderRef b, const unsigned bufferVertex) const;
    LLVM_READNONE Value * getPopCountBaseOffset(BuilderRef b, const Binding & binding, const unsigned bufferVertex);

    LLVM_READNONE bool hasPositivePopCountArray(const unsigned bufferVertex) const;
    LLVM_READNONE bool hasNegativePopCountArray(const unsigned bufferVertex) const;

    Value * getMinimumNumOfLinearPopCountItems(BuilderRef b, const Binding & binding);
    Value * getMaximumNumOfPopCountStrides(BuilderRef b, const Binding & binding, not_null<Value *> sourceItemCount, not_null<Value *> peekableItemCount, Constant * const lookAhead = nullptr);
    Value * getNumOfLinearPopCountItems(BuilderRef b, const Binding & binding);

    Value * getPositivePopCountArray(BuilderRef b, const unsigned inputPort);
    Value * getNegativePopCountArray(BuilderRef b, const unsigned inputPort);
    Value * getIndividualPopCountArray(BuilderRef b, const unsigned inputPort, const unsigned index);

    LLVM_READNONE unsigned getPopCountReferencePort(const Kernel * kernel, const ProcessingRate & rate) const;
    LLVM_READNONE unsigned getPopCountReferenceBuffer(const Kernel * kernel, const ProcessingRate & rate) const;

    LLVM_READNONE StructType * getPopCountThreadLocalStateType(BuilderRef b);
    void allocatePopCountArrays(BuilderRef b, Value * base);
    void deallocatePopCountArrays(BuilderRef b, Value * base);

// pop-count analysis

    PopCountGraph makePopCountGraph() const;

    LLVM_READNONE PopCountData & getPopCountData(const unsigned bufferVertex) const;
    LLVM_READNONE PopCountData analyzePopCountReference(const unsigned bufferVertex) const;
    LLVM_READNONE RateValue popCountReferenceFieldWidth(const unsigned bufferVertex) const;
    LLVM_READNONE bool popCountReferenceCanUseConsumedItemCount(const unsigned bufferVertex) const;

    template <typename LambdaFunction>
    void forEachOutputBufferThatIsAPopCountReference(const unsigned kernelIndex, LambdaFunction func);

    template <typename LambdaFunction>
    void forEachPopCountReferenceInputPort(const unsigned kernelIndex, LambdaFunction func);

// consumer recording

    ConsumerGraph makeConsumerGraph() const;
    void addConsumerKernelProperties(BuilderRef b, const unsigned kernelIndex);
    void createConsumedPhiNodes(BuilderRef b);
    void initializeConsumedItemCount(const unsigned bufferVertex, Value * const produced);
    void readConsumedItemCounts(BuilderRef b);
    void setConsumedItemCount(BuilderRef b, const unsigned bufferVertex, not_null<Value *> consumed) const;

// buffer analysis/management functions

    BufferGraph makeBufferGraph(BuilderRef b);

    void addBufferHandlesToPipelineKernel(BuilderRef b, const unsigned index);

    void constructBuffers(BuilderRef b);
    void loadBufferHandles(BuilderRef b);
    void releaseBuffers(BuilderRef b);
    LLVM_READNONE bool requiresCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE bool requiresFacsimile(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getCopyBack(const unsigned bufferVertex) const;
    LLVM_READNONE unsigned getFacsimile(const unsigned bufferVertex) const;
    BufferType getOutputBufferType(const unsigned outputPort) const;
    Value * epoch(BuilderRef b, const Binding &binding, const StreamSetBuffer * const buffer, Value * const position, Value * const zeroExtended = nullptr) const;

// cycle counter functions

    void addCycleCounterProperties(BuilderRef b, const unsigned kernel);
    void startOptionalCycleCounter(BuilderRef b);
    void updateOptionalCycleCounter(BuilderRef b);
    void printOptionalCycleCounter(BuilderRef b);
    const Binding & selectPrincipleCycleCountBinding(const unsigned kernel) const;

// pipeline analysis functions

    static LLVM_READNONE PipelineGraphBundle makePipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel);
    static void addRegionSelectorKernels(BuilderRef b, Kernels & kernels, ImplicitRelationships & regions);
    static void combineDuplicateKernels(RelationshipGraph & G, const Kernels & kernels);
    static void removeUnusedKernels(RelationshipGraph & G, const unsigned lastKernel, const unsigned lastCall);

    bool hasZeroExtendedStream() const;

    void determineEvaluationOrderOfKernelIO();
    TerminationGraph makeTerminationGraph();
    ScalarGraph makeScalarDependencyGraph() const;
    PipelineIOGraph makePipelineIOGraph() const;

// misc. functions

    Value * getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string &suffix) const;
    Value * getInitializeFunction(BuilderRef b) const;
    Value * getInitializeThreadLocalFunction(BuilderRef b) const;
    Value * getDoSegmentFunction(BuilderRef b) const;
    Value * getFinalizeThreadLocalFunction(BuilderRef b) const;
    Value * getFinalizeFunction(BuilderRef b) const;

    LLVM_READNONE std::string makeFamilyPrefix(const unsigned kernelIndex) const;
    LLVM_READNONE std::string makeKernelName(const unsigned kernelIndex) const;
    LLVM_READNONE std::string makeBufferName(const unsigned kernelIndex, const Binding & binding) const;

    LLVM_READNONE unsigned getInputBufferVertex(const unsigned kernelVertex, const unsigned inputPort) const;
    unsigned getInputBufferVertex(const unsigned inputPort) const;
    StreamSetBuffer * getInputBuffer(const unsigned inputPort) const;
    LLVM_READNONE const Binding & getInputBinding(const unsigned kernelVertex, const unsigned inputPort) const;
    const Binding & getInputBinding(const unsigned inputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getInput(const unsigned kernelVertex, const unsigned outputPort) const;


    LLVM_READNONE unsigned getOutputBufferVertex(const unsigned kernelVertex, const unsigned outputPort) const;
    unsigned getOutputBufferVertex(const unsigned outputPort) const;
    StreamSetBuffer * getOutputBuffer(const unsigned outputPort) const;
    LLVM_READNONE const Binding & getOutputBinding(const unsigned kernelVertex, const unsigned outputPort) const;
    const Binding & getOutputBinding(const unsigned outputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getOutput(const unsigned kernelVertex, const unsigned outputPort) const;

    LLVM_READNONE unsigned getNumOfStreamInputs(const unsigned kernel) const;
    LLVM_READNONE unsigned getNumOfStreamOutputs(const unsigned kernel) const;

    LLVM_READNONE unsigned getBufferIndex(const unsigned bufferVertex) const;

    LLVM_READNONE const Binding & getBinding(const StreamPort port) const;
    LLVM_READNONE const Kernel * getKernel(const unsigned index) const;


    void printBufferGraph(const BufferGraph & G, raw_ostream & out);

    void verifyInputItemCount(BuilderRef b, Value * processed, const unsigned inputPort) const;

    void verifyOutputItemCount(BuilderRef b, Value * produced, const unsigned outputPort) const;

    void itemCountSanityCheck(BuilderRef b, const Binding & binding, const std::string & pastLabel,
                              Value * const itemCount, Value * const expected) const;


private:

    static const StreamPort FAKE_CONSUMER;

protected:

    PipelineKernel * const                      mPipelineKernel;

    unsigned                                    mKernelIndex = 0;
    const Kernel *                              mKernel = nullptr;

    // pipeline state
    Value *                                     mZeroExtendBuffer = nullptr;
    Value *                                     mZeroExtendSpace = nullptr;
    Value *                                     mSegNo = nullptr;
    Value *                                     mHalted = nullptr;
    PHINode *                                   mMadeProgressInLastSegment = nullptr;
    Value *                                     mPipelineProgress = nullptr;
    Value *                                     mPipelineTerminated = nullptr;
    BranchInst *                                mPipelineEntryBranch = nullptr;
    BasicBlock *                                mPipelineLoop = nullptr;
    BasicBlock *                                mKernelEntry = nullptr;
    BasicBlock *                                mKernelLoopEntry = nullptr;
    BasicBlock *                                mKernelRegionEntryLoop = nullptr;
    BasicBlock *                                mKernelCalculateItemCounts = nullptr;
    BasicBlock *                                mKernelLoopCall = nullptr;
    BasicBlock *                                mKernelTerminationCheck = nullptr;
    BasicBlock *                                mKernelTerminated = nullptr;
    BasicBlock *                                mKernelLoopExit = nullptr;
    BasicBlock *                                mKernelLoopExitPhiCatch = nullptr;
    BasicBlock *                                mKernelRegionExitLoopCheck = nullptr;
    BasicBlock *                                mKernelExit = nullptr;
    BasicBlock *                                mPipelineEnd = nullptr;
    Vec<AllocaInst *, 32>                       mAddressableItemCountPtr;
    Vec<Value *, 64>                            mPriorConsumedItemCount;
    Vec<Value *, 64>                            mLocallyAvailableItems;
    Vec<Value *, 16>                            mTerminationSignals;

    // kernel state
    Value *                                     mTerminatedInitially = nullptr;
    PHINode *                                   mHaltingPhi = nullptr;
    PHINode *                                   mHaltedPhi = nullptr;
    PHINode *                                   mHasProgressedPhi = nullptr;
    PHINode *                                   mAlreadyProgressedPhi = nullptr;
    PHINode *                                   mTerminatedPhi = nullptr;
    PHINode *                                   mTerminatedAtExitPhi = nullptr;
    Value *                                     mNumOfLinearStrides = nullptr;
    Value *                                     mTerminatedExplicitly = nullptr;
    Vec<unsigned, 32>                           mPortEvaluationOrder;
    unsigned                                    mNumOfAddressableItemCount = 0;

    Vec<Value *>                                mIsInputClosed;
    Vec<Value *>                                mIsInputZeroExtended;
    PHINode *                                   mZeroExtendBufferPhi = nullptr;

    Vec<Value *>                                mInitiallyProcessedItemCount; // *before* entering the kernel
    Vec<Value *>                                mInitiallyProcessedDeferredItemCount;
    Vec<PHINode *>                              mAlreadyProcessedPhi; // entering the segment loop
    Vec<PHINode *>                              mAlreadyProcessedDeferredPhi;
    Vec<Value *>                                mInputStrideLength;
    Vec<Value *>                                mAccessibleInputItems;
    Vec<PHINode *>                              mLinearInputItemsPhi;
    Vec<Value *>                                mReturnedProcessedItemCountPtr; // written by the kernel
    Vec<Value *>                                mProcessedItemCount; // exiting the segment loop
    Vec<Value *>                                mProcessedDeferredItemCount;
    Vec<PHINode *>                              mFinalProcessedPhi; // exiting after termination
    Vec<PHINode *>                              mUpdatedProcessedPhi; // exiting the kernel
    Vec<PHINode *>                              mUpdatedProcessedDeferredPhi;
    Vec<Value *>                                mFullyProcessedItemCount; // *after* exiting the kernel

    Vec<Value *>                                mInitiallyProducedItemCount; // *before* entering the kernel
    Vec<Value *>                                mInitiallyProducedDeferredItemCount;
    Vec<PHINode *>                              mAlreadyProducedPhi; // entering the segment loop
    Vec<PHINode *>                              mAlreadyProducedDeferredPhi;
    Vec<Value *>                                mOutputStrideLength;
    Vec<Value *>                                mWritableOutputItems;
    Vec<Value *>                                mConsumedItemCount;
    Vec<PHINode *>                              mLinearOutputItemsPhi;
    Vec<Value *>                                mReturnedProducedItemCountPtr; // written by the kernel
    Vec<Value *>                                mProducedItemCount; // exiting the segment loop
    Vec<Value *>                                mProducedDeferredItemCount;
    Vec<PHINode *>                              mFinalProducedPhi; // exiting after termination
    Vec<PHINode *>                              mUpdatedProducedPhi; // exiting the kernel
    Vec<PHINode *>                              mUpdatedProducedDeferredPhi;
    Vec<PHINode *>                              mFullyProducedItemCount; // *after* exiting the kernel

    // cycle counter state
    Value *                                     mCycleCountStart = nullptr;

    // popcount state
    Value *                                     mPopCountState;
    flat_map<unsigned, PopCountData>            mPopCountData;

    // analysis state
    const RelationshipGraph                     mPipelineGraph;
    static constexpr unsigned                   PipelineInput = 0;
    static constexpr unsigned                   FirstKernel = 1;
    const unsigned                              LastKernel;
    const unsigned                              PipelineOutput;
    const unsigned                              FirstCall;
    const unsigned                              LastCall;
    const unsigned                              FirstScalar;
    const unsigned                              LastScalar;
    const unsigned                              FirstStreamSet;
    const unsigned                              LastStreamSet;

    std::vector<Binding>                        ImplicitBindings; // <- change to graph property
    const BufferGraph                           mBufferGraph;

    const bool                                  mHasZeroExtendedStream;
    ConsumerGraph                               mConsumerGraph;
    ScalarGraph                                 mScalarGraph;
    Vec<Value *>                                mScalarValue;
    const PipelineIOGraph                       mPipelineIOGraph;
    const TerminationGraph                      mTerminationGraph;
    PopCountGraph                               mPopCountGraph;

};

const StreamPort PipelineCompiler::FAKE_CONSUMER{PortType::Input, std::numeric_limits<unsigned>::max()};

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
    for (const auto & e : make_iterator_range(out_edges(u, G))) {
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
inline PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel, PipelineGraphBundle && P)
: mPipelineKernel(pipelineKernel)
, mPipelineGraph(std::move(P.Graph))
, LastKernel(P.LastKernel)
, PipelineOutput(P.PipelineOutput)
, FirstCall(P.FirstCall)
, LastCall(P.LastCall)
, FirstScalar(P.FirstScalar)
, LastScalar(P.LastScalar)
, FirstStreamSet(P.FirstStreamSet)
, LastStreamSet(P.LastStreamSet)
, mBufferGraph(makeBufferGraph(b))
, mHasZeroExtendedStream(hasZeroExtendedStream())
, mConsumerGraph(makeConsumerGraph())
, mScalarGraph(makeScalarDependencyGraph())
, mScalarValue(LastScalar + 1)
, mPipelineIOGraph(makePipelineIOGraph())
, mTerminationGraph(makeTerminationGraph())
, mPopCountGraph(makePopCountGraph()) {
    initializePopCounts();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief upperBound
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue upperBound(not_null<const Kernel *> kernel, const Binding & binding) {
    assert (kernel->getStride() > 0);
    return kernel->getUpperBound(binding) * kernel->getStride();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief lowerBound
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue lowerBound(not_null<const Kernel *> kernel, const Binding & binding) {
    assert (kernel->getStride() > 0);
    return kernel->getLowerBound(binding) * kernel->getStride();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeFamilyPrefix
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeFamilyPrefix(const unsigned kernelIndex) const {
    const Kernel * k = getKernel(kernelIndex);
    const Kernels & K = mPipelineKernel->getKernels();
    const auto f = std::find(K.begin(), K.end(), k);
    assert (f != K.end());
    const auto i = std::distance(K.begin(), f);
    return "F" + std::to_string(i);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeKernelName(const unsigned kernelIndex) const {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << '@';
    out << getKernel(kernelIndex)->getName();
    out << '.';
    out << kernelIndex;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBufferName
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeBufferName(const unsigned kernelIndex, const Binding & binding) const {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << '@';
    out << getKernel(kernelIndex)->getName();
    out << '_';
    out << binding.getName();
    out << '.';
    out << kernelIndex;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLog2SizeWidth
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline Constant * getLog2SizeWidth(BuilderRef b) {
    return b->getSize(std::log2(b->getSizeTy()->getBitWidth()));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getLog2BlockWidth
 ** ------------------------------------------------------------------------------------------------------------- */
LLVM_READNONE inline Constant * getLog2BlockWidth(BuilderRef b) {
    return b->getSize(std::log2(b->getBitBlockWidth()));
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

#include "pipeline_analysis.hpp"
#include "buffer_management_logic.hpp"
#include "termination_logic.hpp"
#include "consumer_logic.hpp"
#include "core_logic.hpp"
#include "kernel_logic.hpp"
#include "cycle_counter_logic.hpp"
#include "popcount_logic.hpp"
#include "pipeline_logic.hpp"
#include "scalar_logic.hpp"

#endif // PIPELINE_COMPILER_HPP
