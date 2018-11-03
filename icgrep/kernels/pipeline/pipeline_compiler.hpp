#ifndef PIPELINE_COMPILER_HPP
#define PIPELINE_COMPILER_HPP

#include <kernels/pipeline_kernel.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
//#include <boost/graph/topological_sort.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <llvm/IR/Module.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/STLExtras.h>
#include <queue>

// #define PRINT_DEBUG_MESSAGES

using namespace boost;
using namespace boost::math;
using boost::container::flat_set;
using boost::container::flat_map;
using namespace llvm;

inline static unsigned floor_log2(const unsigned v) {
    assert ("log2(0) is undefined!" && v != 0);
    return 31 - __builtin_clz(v);
}

namespace kernel {

using Port = Kernel::Port;
using StreamPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using RateValue = ProcessingRate::RateValue;
using RateId = ProcessingRate::KindId;
using Scalars = PipelineKernel::Scalars;
using Kernels = PipelineKernel::Kernels;
using CallBinding = PipelineKernel::CallBinding;

#warning TODO: these graphs are similar; look into streamlining their generation.

struct BufferNode { // use boost::variant instead of union? std::variant is c17+
    Kernel * kernel = nullptr;
    StreamSetBuffer * buffer = nullptr;
    RateValue lower;
    RateValue upper;
};

struct BufferRateData {    

    RateValue minimum;
    RateValue maximum;
    unsigned port;

    BufferRateData(const unsigned port = 0) : port(port) { }

    BufferRateData(const unsigned port, RateValue min, RateValue max)
    : minimum(std::move(min)), maximum(std::move(max)), port(port) { }
};

using BufferGraph = adjacency_list<vecS, vecS, bidirectionalS, BufferNode, BufferRateData>; // unsigned>;

template <typename vertex_descriptor>
using RelationshipMap = flat_map<const Relationship *, vertex_descriptor>;

using BufferMap = RelationshipMap<BufferGraph::vertex_descriptor>;

struct ConsumerNode {
    Value * consumed = nullptr;
    PHINode * phiNode = nullptr;
};

using ConsumerGraph = adjacency_list<vecS, vecS, bidirectionalS, ConsumerNode, unsigned>;

template <typename Value>
using StreamSetBufferMap = flat_map<const StreamSetBuffer *, Value>;

template <typename Value>
using KernelMap = flat_map<const Kernel *, Value>;

struct Channel {
    Channel() = default;
    Channel(const RateValue & ratio, const StreamSetBuffer * const buffer = nullptr, const unsigned operand = 0)
    : ratio(ratio), buffer(buffer), portIndex(operand) {

    }
    RateValue               ratio;
    const StreamSetBuffer * buffer;
    unsigned                portIndex;
};

using ChannelGraph = adjacency_list<vecS, vecS, bidirectionalS, const Kernel *, Channel>;

using TerminationGraph = adjacency_list<hash_setS, vecS, bidirectionalS, Value *>;

using ScalarDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, Value *, unsigned>;

using PortDependencyGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, RateId>;

struct OverflowRequirement {
    unsigned copyBack;
    unsigned facsimile;
    OverflowRequirement() = default;
    OverflowRequirement(const unsigned copyBack, const unsigned copyForward)
    : copyBack(copyBack), facsimile(copyForward) { }
};

using OverflowRequirements = StreamSetBufferMap<OverflowRequirement>;

using PopCountStreamDependencyGraph = adjacency_list<vecS, vecS, directedS, Value *>;

using PopCountStreamDependencyVertex = PopCountStreamDependencyGraph::vertex_descriptor;

struct PopCountData {
    unsigned hasConstructedArray = -1;
    PopCountStreamDependencyVertex vertex = 0;
    Value * initial = nullptr;
    Value * baseIndex = nullptr;
    AllocaInst * individualCountArray = nullptr;
    AllocaInst * partialSumArray = nullptr;
    Value * strideCapacity = nullptr;
    Value * finalPartialSum = nullptr;
    Value * maximumNumOfStrides = nullptr;
};

using PopCountDataMap = flat_map<std::pair<const StreamSetBuffer *, bool>, PopCountData>;

struct PipelineCompiler {

    using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

    PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel);

    void addHandlesToPipelineKernel(BuilderRef b);
    void generateInitializeMethod(BuilderRef b);
    void generateSingleThreadKernelMethod(BuilderRef b);
    void generateMultiThreadKernelMethod(BuilderRef b, const unsigned numOfThreads);
    void generateFinalizeMethod(BuilderRef b);
    std::vector<Value *> getFinalOutputScalars(BuilderRef b);

protected:

// main pipeline functions

    void start(BuilderRef b, Value * const initialSegNo);
    void setActiveKernel(BuilderRef b, const unsigned index);
    void synchronize(BuilderRef b);
    void executeKernel(BuilderRef b);
    void end(BuilderRef b, const unsigned step);

// inter-kernel functions

    Value * checkForSufficientInputDataAndOutputSpace(BuilderRef b);
    Value * determineNumOfLinearStrides(BuilderRef b);
    void calculateNonFinalItemCounts(BuilderRef b, Value * const numOfStrides);
    void calculateFinalItemCounts(BuilderRef b);
    void provideAllInputAndOutputSpace(BuilderRef b);
    void writeKernelCall(BuilderRef b);
    void writeCopyBackLogic(BuilderRef b);
    void writeCopyForwardLogic(BuilderRef b);
    void allocateThreadLocalState(BuilderRef b, const Port port, const unsigned i);
    void deallocateThreadLocalState(BuilderRef b, const Port port, const unsigned i);

    void checkIfAllProducingKernelsHaveTerminated(BuilderRef b);
    void zeroFillPartiallyWrittenOutputStreams(BuilderRef b);
    void initializeKernelCallPhis(BuilderRef b);
    void initializeKernelExitPhis(BuilderRef b);
    void storeCopyForwardProducedItemCounts(BuilderRef b);
    void storeCopyBackProducedItemCounts(BuilderRef b);
    void computeMinimumConsumedItemCounts(BuilderRef b);
    void writeFinalConsumedItemCounts(BuilderRef b);
    void readCurrentProducedItemCounts(BuilderRef b);
    void releaseCurrentSegment(BuilderRef b);
    void writeCopyToOverflowLogic(BuilderRef b);
    void checkForSufficientInputData(BuilderRef b, const unsigned index);
    void checkForSufficientOutputSpaceOrExpand(BuilderRef b, const unsigned index);

// intra-kernel functions

    void branchToTargetOrLoopExit(BuilderRef b, Value * const cond, BasicBlock * const target);
    void expandOutputBuffer(BuilderRef b, Value * const hasEnough, const unsigned index, BasicBlock * const target);
    Value * getInputStrideLength(BuilderRef b, const unsigned index);
    Value * getOutputStrideLength(BuilderRef b, const unsigned index);
    Value * getInitialStrideLength(BuilderRef b, const Binding & binding);
    Value * calculateNumOfLinearItems(BuilderRef b, const Binding & binding, Value * const numOfStrides);
    Value * getAccessibleInputItems(BuilderRef b, const unsigned index);
    Value * getNumOfAccessibleStrides(BuilderRef b, const unsigned index);
    Value * getNumOfWritableStrides(BuilderRef b, const unsigned index);
    Value * getWritableOutputItems(BuilderRef b, const unsigned index);
    Value * calculateBufferExpansionSize(BuilderRef b, const unsigned index);
    Value * addLookahead(BuilderRef b, const unsigned index, Value * itemCount) const;
    Value * subtractLookahead(BuilderRef b, const unsigned index, Value * itemCount) const;
    Value * getFullyProcessedItemCount(BuilderRef b, const Binding & input) const;
    Value * getTotalItemCount(BuilderRef b, const StreamSetBuffer * buffer) const;
    Value * isTerminated(BuilderRef b) const;
    void setTerminated(BuilderRef b);

// pop-count functions

    void initializePopCounts(BuilderRef b);
    PopCountStreamDependencyGraph makePopCountStreamDependencyGraph(BuilderRef b);
    void addPopCountStreamDependency(BuilderRef b, const unsigned index, const Binding & binding, PopCountStreamDependencyGraph & G);

    Value * getInitialNumOfLinearPopCountItems(BuilderRef b, const ProcessingRate & rate);
    Value * getMaximumNumOfPopCountStrides(BuilderRef b, const ProcessingRate & rate);
    Value * getNumOfLinearPopCountItems(BuilderRef b, const ProcessingRate & rate, Value * const numOfStrides);
    Value * getPopCountArray(BuilderRef b, const unsigned index);
    Value * getNegatedPopCountArray(BuilderRef b, const unsigned index);
    void allocateLocalPopCountArray(BuilderRef b, const ProcessingRate & rate);
    void deallocateLocalPopCountArray(BuilderRef b, const ProcessingRate & rate);
    void storePopCountSourceItemCount(BuilderRef b, const Port port, const unsigned index, Value * const offset, Value * const processable);

    PopCountData & findOrAddPopCountData(BuilderRef b, const ProcessingRate & rate);
    PopCountData & findOrAddPopCountData(BuilderRef b, const unsigned index, const bool negated);
    Value * getInitialNumOfLinearPopCountItems(BuilderRef b, PopCountData & pc, const unsigned index, const bool negated);
    PopCountData & makePopCountArray(BuilderRef b, const ProcessingRate & rate);
    PopCountData & makePopCountArray(BuilderRef b, const unsigned index, const bool negated);
    Value * getMinimumNumOfSourceItems(BuilderRef b, const PopCountData & pc);
    Value * getSourceMarkers(BuilderRef b, PopCountData & pc, const unsigned index, Value * const offset) const;

// consumer recording

    ConsumerGraph makeConsumerGraph() const;
    void createConsumedPhiNodes(BuilderRef b);
    void initializeConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const produced);
    void setConsumedItemCount(BuilderRef b, const unsigned bufferVertex, Value * const consumed) const;
    Value * getConsumedItemCount(BuilderRef b, const unsigned index) const;

// buffer analysis/management functions

    BufferGraph makeBufferGraph(BuilderRef b);
    void enumerateBufferProducerBindings(const unsigned producer, const Bindings & bindings, BufferGraph & G, BufferMap & M);
    void enumerateBufferConsumerBindings(const unsigned consumer, const Bindings & bindings, BufferGraph & G, BufferMap & M);
    BufferRateData getBufferRateData(const unsigned index, const unsigned port, bool input);

    void constructBuffers(BuilderRef b);
    void loadBufferHandles(BuilderRef b);
    void releaseBuffers(BuilderRef b);
    LLVM_READNONE bool requiresCopyBack(const StreamSetBuffer * const buffer) const;
    LLVM_READNONE bool requiresFacsimile(const StreamSetBuffer * const buffer) const;
    LLVM_READNONE unsigned getCopyBack(const StreamSetBuffer * const buffer) const;
    LLVM_READNONE unsigned getFacsimile(const StreamSetBuffer * const buffer) const;
    LLVM_READNONE bool isPipelineIO(const StreamSetBuffer * const buffer) const;

    Value * getLogicalInputBaseAddress(BuilderRef b, const unsigned index) const;
    Value * getLogicalOutputBaseAddress(BuilderRef b, const unsigned index) const;
    Value * calculateLogicalBaseAddress(BuilderRef b, const Binding & binding, const StreamSetBuffer * const buffer, Value * const itemCount) const;

// cycle counter functions

    void startOptionalCycleCounter(BuilderRef b);
    void updateOptionalCycleCounter(BuilderRef b);
    void printOptionalCycleCounter(BuilderRef b);

// analysis functions

    PortDependencyGraph makePortDependencyGraph() const;
    TerminationGraph makeTerminationGraph() const;
    ScalarDependencyGraph makeScalarDependencyGraph() const;

// misc. functions

    Value * getFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const;

    Value * getInitializationFunction(BuilderRef b) const;

    Value * getDoSegmentFunction(BuilderRef b) const;

    Value * getFinalizeFunction(BuilderRef b) const;

    std::string makeKernelName(const unsigned kernelIndex) const;

    std::string makeBufferName(const unsigned kernelIndex, const Binding & binding) const;

    StreamSetBuffer * getInputBuffer(const unsigned index) const;

    StreamSetBuffer * getOutputBuffer(const unsigned index) const;

    const Binding & getBinding(const Port port, const unsigned i) const {
        if (port == Port::Input) {
            return mKernel->getInputStreamSetBinding(i);
        } else {
            return mKernel->getOutputStreamSetBinding(i);
        }
    }

    void printBufferGraph(const BufferGraph & G, raw_ostream & out);

    LLVM_READNONE const Binding & getInputBinding(const Kernel * const producer, const unsigned index) const;

    LLVM_READNONE const Binding & getOutputBinding(const Kernel * const consumer, const unsigned index) const;

    void writeOutputScalars(BuilderRef b, const unsigned u, std::vector<Value *> & args);

    void itemCountSanityCheck(BuilderRef b, const Binding & binding, const std::string & presentLabel, const std::string & pastLabel,
                              Value * const itemCount, Value * const expected, Value * const terminated);

protected:

    PipelineKernel * const                      mPipelineKernel;
    const Kernels &                             mPipeline;

    OwnedStreamSetBuffers                       mOwnedBuffers;
    unsigned                                    mKernelIndex = 0;
    const Kernel *                              mKernel = nullptr;

    // pipeline state
    PHINode *                                   mTerminatedPhi = nullptr;
    PHINode *                                   mSegNo = nullptr;
    BasicBlock *                                mPipelineLoop = nullptr;
    BasicBlock *                                mKernelEntry = nullptr;
    BasicBlock *                                mKernelLoopEntry = nullptr;
    BasicBlock *                                mKernelLoopCall = nullptr;
    BasicBlock *                                mKernelLoopExit = nullptr;
    BasicBlock *                                mKernelLoopExitPhiCatch = nullptr;
    BasicBlock *                                mKernelExit = nullptr;
    BasicBlock *                                mPipelineEnd = nullptr;

    // pipeline state
    StreamSetBufferMap<Value *>                 mInputConsumedItemCountPhi;
    StreamSetBufferMap<Value *>                 mTotalItemCount;
    StreamSetBufferMap<Value *>                 mConsumedItemCount;
    std::vector<Value *>                        mOutputScalars;

    // kernel state
    Value *                                     mNoMore = nullptr;
    Value *                                     mNumOfLinearStrides = nullptr;
    PHINode *                                   mNumOfLinearStridesPhi = nullptr;
    Value *                                     mNonFinal = nullptr;
    PHINode *                                   mIsFinalPhi = nullptr;

    std::vector<Value *>                        mInputStrideLength;
    std::vector<Value *>                        mAccessibleInputItems;
    std::vector<PHINode *>                      mAccessibleInputItemsPhi;
    std::vector<Value *>                        mInputStreamHandle;

    std::vector<Value *>                        mOutputStrideLength;
    std::vector<Value *>                        mWritableOutputItems;
    std::vector<Value *>                        mCopyForwardProducedOutputItems;
    std::vector<Value *>                        mAnteriorProcessedItemCount;
    std::vector<Value *>                        mCopyBackProducedOutputItems;
    std::vector<PHINode *>                      mWritableOutputItemsPhi;
    std::vector<Value *>                        mOutputStreamHandle;

    // debug + misc state
    Value *                                     mCycleCountStart = nullptr;
    PHINode *                                   mDeadLockCounter = nullptr;
    Value *                                     mPipelineProgress = nullptr;
    PHINode *                                   mHasProgressedPhi = nullptr;
    PHINode *                                   mAlreadyProgressedPhi = nullptr;

    // popcount state
    PopCountStreamDependencyGraph               mPopCountDependencyGraph;
    PopCountDataMap                             mPopCountDataMap;


    // analysis state
    flat_set<const StreamSetBuffer *>           mIsPipelineIO;
    OverflowRequirements                        mOverflowRequirements;
    BufferGraph                                 mBufferGraph;
    ConsumerGraph                               mConsumerGraph;
    TerminationGraph                            mTerminationGraph;
    ScalarDependencyGraph                       mScalarDependencyGraph;

};

inline PipelineCompiler::PipelineCompiler(BuilderRef b, PipelineKernel * const pipelineKernel)
: mPipelineKernel(pipelineKernel)
, mPipeline(pipelineKernel->mKernels)
, mBufferGraph(makeBufferGraph(b))
, mConsumerGraph(makeConsumerGraph())
, mTerminationGraph(makeTerminationGraph())
, mScalarDependencyGraph(makeScalarDependencyGraph()) {




}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getInputBuffer(const unsigned index) const {
    for (const auto e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        if (mBufferGraph[e].port == index) {
            return mBufferGraph[source(e, mBufferGraph)].buffer;
        }
    }
    llvm_unreachable("input buffer not found");
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetBuffer * PipelineCompiler::getOutputBuffer(const unsigned index) const {
    for (const auto e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        if (mBufferGraph[e].port == index) {
            return mBufferGraph[target(e, mBufferGraph)].buffer;
        }
    }
    llvm_unreachable("output buffer not found");
    return nullptr;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief storedInKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE bool storedInNestedKernel(const Binding & output) {
    return output.getRate().isUnknown() || output.hasAttribute(AttrId::ManagedBuffer);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief upperBound
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue upperBound(not_null<const Kernel *> kernel, const Binding & binding) {
    assert (kernel->getStride() > 0);
//    const auto ub = kernel->getUpperBound(binding);
//    const auto stride = kernel->getStride();
//    return (ub == 0) ? stride : ub * stride;
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
 * @brief makeKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeKernelName(const unsigned kernelIndex) const {
    return PipelineKernel::makeKernelName(mPipeline[kernelIndex], kernelIndex);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBufferName
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE std::string PipelineCompiler::makeBufferName(const unsigned kernelIndex, const Binding & binding) const {
    return PipelineKernel::makeBufferName(mPipeline[kernelIndex], kernelIndex, binding);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Relationship * getRelationship(not_null<const Relationship *> r) {
    return r.get();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Relationship * getRelationship(const Binding & b) {
    return getRelationship(b.getRelationship());
}

inline unsigned LLVM_READNONE getItemWidth(const Type * ty ) {
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

template <typename Graph>
inline typename graph_traits<Graph>::edge_descriptor in_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (in_degree(u, G) == 1);
    return *in_edges(u, G).first;
}

} // end of namespace

#endif // PIPELINE_COMPILER_HPP
