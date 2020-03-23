#ifndef OPTIMIZATIONBRANCH_COMPILER_HPP
#define OPTIMIZATIONBRANCH_COMPILER_HPP

#include <kernel/core/kernel_compiler.h>
#include <kernel/pipeline/optimizationbranch.h>
#include <kernel/pipeline/pipeline_kernel.h>
#include <kernel/core/streamset.h>
#include <kernel/core/kernel_builder.h>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/SmallVector.h>
#include <kernel/core/refwrapper.h>
#include  <array>

using namespace llvm;
using namespace boost;
using namespace boost::container;

#define PRINT_DEBUG_MESSAGES

namespace kernel {

using BindingRef = RefWrapper<Binding>;

using BuilderRef = Kernel::BuilderRef;

using StreamSetGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

struct RelationshipRef {
    unsigned Index;
    StringRef Name;
    BindingRef Binding;
    RelationshipRef() = default;
    RelationshipRef(const unsigned index, StringRef name, const kernel::Binding & binding) : Index(index), Name(name), Binding(binding) { }
};

const static std::string SHARED_PREFIX = "S";
const static std::string THREAD_LOCAL_PREFIX = "T";

const static std::string LOGICAL_SEGMENT_NUMBER = "L";

const static std::string ALL_ZERO_ACTIVE_THREADS = "AT";
const static std::string ALL_ZERO_LOGICAL_SEGMENT_NUMBER = "AS";

const static std::string NON_ZERO_ACTIVE_THREADS = "NT";
const static std::string NON_ZERO_LOGICAL_SEGMENT_NUMBER = "NS";


const static std::string SPAN_BUFFER = "SpanBuffer";
const static std::string SPAN_CAPACITY = "SpanCapacity";

using RelationshipGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, RelationshipRef>;

using RelationshipCache = flat_map<RelationshipGraph::vertex_descriptor, Value *>;


using PortType = Kernel::PortType;
using StreamSetPort = Kernel::StreamSetPort;
using BuilderRef = BuilderRef;
using AttrId = Attribute::KindId;

enum : unsigned {
    BRANCH_INPUT = 0
    , ALL_ZERO_BRANCH = 1
    , NON_ZERO_BRANCH = 2
    , BRANCH_OUTPUT = 3
    , CONDITION_VARIABLE = 4
// ----------------------
    , INITIAL_GRAPH_SIZE = 5
};

#define OTHER_BRANCH(TYPE) (TYPE ^ (ALL_ZERO_BRANCH | NON_ZERO_BRANCH))

static_assert (ALL_ZERO_BRANCH < NON_ZERO_BRANCH, "invalid branch type ordering");
static_assert (OTHER_BRANCH(ALL_ZERO_BRANCH) == NON_ZERO_BRANCH, "invalid branch type ordering");
static_assert (OTHER_BRANCH(NON_ZERO_BRANCH) == ALL_ZERO_BRANCH, "invalid branch type ordering");

using CompilerArray = std::array<std::unique_ptr<KernelCompiler>, 4>;

class OptimizationBranchCompiler final : public KernelCompiler {

    enum class RelationshipType : unsigned {
        StreamSet
        , Scalar
    };

public:

    OptimizationBranchCompiler(BuilderRef b, OptimizationBranch * const branch) noexcept;

    void addBranchProperties(BuilderRef b);
    void generateInitializeMethod(BuilderRef b);
    void generateInitializeThreadLocalMethod(BuilderRef b);
    void generateKernelMethod(BuilderRef b);
    void generateFinalizeThreadLocalMethod(BuilderRef b);
    void generateFinalizeMethod(BuilderRef b);

    std::vector<Value *> getFinalOutputScalars(BuilderRef b) override;

private:

    Value * loadHandle(BuilderRef b, const unsigned branchType) const;

    Value * getInputScalar(BuilderRef b, const unsigned scalar);

    inline unsigned getNumOfInputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getInputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    unsigned getNumOfOutputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getOutputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    void generateStreamSetBranchMethod(BuilderRef b);

    void findBranchDemarcations(BuilderRef b);

    void waitForPriorThreads(BuilderRef b);

    void executeBranches(BuilderRef b);

    void executeBranch(BuilderRef b, const unsigned branchType, Value * const firstIndex, Value * const lastIndex, Value * noMore);

    Value * enterBranch(BuilderRef b, const unsigned branchType, Value * const noMore) const;

    Value * isBranchReady(BuilderRef b, const unsigned branchType) const;

    void exitBranch(BuilderRef b, const unsigned branchType) const;

    Value * calculateAccessibleOrWritableItems(BuilderRef b, const Kernel * const kernel, const Binding & binding, Value * const first, Value * const last, Value * const defaultValue) const;

    void calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType);

    RelationshipGraph makeRelationshipGraph(const RelationshipType type) const;

private:

    const Relationship * const          mCondition;

    const std::array<const Kernel *, 4> mBranches;

    const CompilerArray                 mBranchCompiler;

    Value *                             mFirstBranchPath = nullptr;
    PHINode *                           mBranchDemarcationArray = nullptr;
    Value *                             mBranchDemarcationCount = nullptr;

    std::array<Value *, 3>              mLogicalSegmentNumber;
    std::array<Value *, 3>              mActiveThreads;

    PHINode *                           mTerminatedPhi = nullptr;

    SmallVector<Value *, 16>            mBaseInputAddress;
    SmallVector<Value *, 16>            mProcessedInputItemPtr;
    SmallVector<Value *, 16>            mProcessedInputItems;
    SmallVector<Value *, 16>            mAccessibleInputItems;
    SmallVector<Value *, 16>            mPopCountRateArray;
    SmallVector<Value *, 16>            mNegatedPopCountRateArray;

    SmallVector<Value *, 16>            mBaseOutputAddress;
    SmallVector<Value *, 16>            mProducedOutputItemPtr;
    SmallVector<Value *, 16>            mProducedOutputItems;
    SmallVector<Value *, 16>            mWritableOutputItems;

    const RelationshipGraph             mStreamSetGraph;
    const RelationshipGraph             mScalarGraph;
    RelationshipCache                   mScalarCache;

};

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor in_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (in_degree(u, G) == 1);
    return *in_edges(u, G).first;
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor preceding(const typename graph_traits<Graph>::edge_descriptor & e, const Graph & G) {
    return in_edge(source(e, G), G);
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor out_edge(const typename graph_traits<Graph>::vertex_descriptor u, const Graph & G) {
    assert (out_degree(u, G) == 1);
    return *out_edges(u, G).first;
}

template <typename Graph>
LLVM_READNONE
inline typename graph_traits<Graph>::edge_descriptor descending(const typename graph_traits<Graph>::edge_descriptor & e, const Graph & G) {
    return out_edge(target(e, G), G);
}

inline unsigned OptimizationBranchCompiler::getNumOfInputBindings(const Kernel * const kernel, const RelationshipType type) const {
    return (type == RelationshipType::StreamSet) ? kernel->getNumOfStreamInputs() : kernel->getNumOfScalarInputs();
}

inline const Binding & OptimizationBranchCompiler::getInputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const {
    return (type == RelationshipType::StreamSet) ? kernel->mInputStreamSets[i] : kernel->mInputScalars[i];
}

inline unsigned OptimizationBranchCompiler::getNumOfOutputBindings(const Kernel * const kernel, const RelationshipType type) const {
    return (type == RelationshipType::StreamSet) ? kernel->getNumOfStreamOutputs() : kernel->getNumOfScalarOutputs();
}

inline const Binding & OptimizationBranchCompiler::getOutputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const {
    return (type == RelationshipType::StreamSet) ? kernel->mOutputStreamSets[i] : kernel->mOutputScalars[i];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief concat
 ** ------------------------------------------------------------------------------------------------------------- */
inline StringRef concat(StringRef A, StringRef B, SmallVector<char, 256> & tmp) {
    Twine C = A + B;
    tmp.clear();
    return C.toStringRef(tmp);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeRelationshipGraph
 ** ------------------------------------------------------------------------------------------------------------- */
RelationshipGraph OptimizationBranchCompiler::makeRelationshipGraph(const RelationshipType type) const {

    using Vertex = RelationshipGraph::vertex_descriptor;
    using Map = flat_map<const Relationship *, Vertex>;

    RelationshipGraph G(INITIAL_GRAPH_SIZE);
    Map M;

    auto addRelationship = [&](const Relationship * const rel) {
        const auto f = M.find(rel);
        if (LLVM_UNLIKELY(f != M.end())) {
            return f->second;
        } else {
            const auto x = add_vertex(G);
            M.emplace(rel, x);
            return x;
        }
    };

    const auto numOfInputs = getNumOfInputBindings(mTarget, type);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto & input = getInputBinding(mTarget, type, i);
        const auto r = addRelationship(input.getRelationship());
        add_edge(BRANCH_INPUT, r, RelationshipRef{i, input.getName(), input}, G);
    }

    const auto numOfOutputs = getNumOfOutputBindings(mTarget, type);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto & output = getOutputBinding(mTarget, type, i);
        const auto r = addRelationship(output.getRelationship());
        add_edge(r, BRANCH_OUTPUT, RelationshipRef{i, output.getName(), output}, G);
    }

    if (type == RelationshipType::StreamSet && isa<StreamSet>(mCondition)) {
        const auto r = addRelationship(mCondition);
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    if (type == RelationshipType::Scalar && isa<Scalar>(mCondition)) {
        const auto r = addRelationship(mCondition);
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    auto linkRelationships = [&](const Kernel * const kernel, const Vertex branch) {

        auto findRelationship = [&](const Binding & binding) {
            Relationship * const rel = binding.getRelationship();
            const auto f = M.find(rel);
            if (LLVM_UNLIKELY(f == M.end())) {
                if (LLVM_LIKELY(rel->isConstant())) {
                    const auto x = add_vertex(G);
                    M.emplace(rel, x);
                    return x;
                } else {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << "Branch was not provided a ";
                    if (type == RelationshipType::StreamSet) {
                        msg << "StreamSet";
                    } else if (type == RelationshipType::Scalar) {
                        msg << "Scalar";
                    }
                    msg << " binding for "
                        << kernel->getName()
                        << '.'
                        << binding.getName();
                    report_fatal_error(msg.str());
                }
            }
            return f->second;
        };

        const auto numOfInputs = getNumOfInputBindings(kernel, type);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto & input = getInputBinding(mTarget, type, i);
            const auto r = findRelationship(input);
            add_edge(r, branch, RelationshipRef{i, input.getName(), input}, G);
        }

        const auto numOfOutputs = getNumOfOutputBindings(kernel, type);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto & output = getOutputBinding(kernel, type, i);
            const auto r = findRelationship(output);
            add_edge(branch, r, RelationshipRef{i, output.getName(), output}, G);
        }
    };

    linkRelationships(mBranches[ALL_ZERO_BRANCH], ALL_ZERO_BRANCH);
    linkRelationships(mBranches[NON_ZERO_BRANCH], NON_ZERO_BRANCH);

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::addBranchProperties(BuilderRef b) {
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (LLVM_UNLIKELY(kernel == nullptr)) continue;

        if (LLVM_LIKELY(kernel->isStateful())) {
            Type * handlePtrType = nullptr;
            if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
                handlePtrType = b->getVoidPtrTy();
            } else {
                handlePtrType = kernel->getSharedStateType()->getPointerTo();
            }
            mTarget->addInternalScalar(handlePtrType, SHARED_PREFIX + std::to_string(i));
        }

        if (kernel->hasThreadLocal()) {
            mTarget->addThreadLocalScalar(kernel->getThreadLocalStateType(), THREAD_LOCAL_PREFIX + std::to_string(i));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateInitializeMethod(BuilderRef b) {
    mScalarCache.clear();
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (kernel && kernel->isStateful() && !kernel->hasFamilyName()) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(SHARED_PREFIX + std::to_string(i), handle);
        }
    }
    std::vector<Value *> args;
    Value * terminated = b->getFalse();
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (LLVM_UNLIKELY(kernel == nullptr)) continue;
        const auto hasHandle = kernel->isStateful() ? 1U : 0U;
        args.resize(hasHandle + in_degree(i, mScalarGraph));
        if (LLVM_LIKELY(hasHandle)) {
            args[0] = b->getScalarField(SHARED_PREFIX + std::to_string(i));
        }
        for (const auto e : make_iterator_range(in_edges(i, mScalarGraph))) {
            const RelationshipRef & ref = mScalarGraph[e];
            const auto j = ref.Index + hasHandle;
            args[j] = getInputScalar(b, source(e, mScalarGraph));
        }
        Value * const terminatedOnInit = b->CreateCall(kernel->getInitializeFunction(b), args);
        terminated = b->CreateOr(terminated, terminatedOnInit);
    }
    b->CreateStore(terminated, getTerminationSignalPtr());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateInitializeThreadLocalMethod(BuilderRef b) {
    Constant * const defaultSize = b->getSize(32);
    Value * bufferPtr = b->getScalarFieldPtr(SPAN_BUFFER);
    Value * capacityPtr = b->getScalarFieldPtr(SPAN_CAPACITY);
    b->CreateStore(b->CreateCacheAlignedMalloc(b->getSizeTy(), defaultSize), bufferPtr);
    b->CreateStore(defaultSize, capacityPtr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadKernelHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::loadHandle(BuilderRef b, const unsigned branchType) const {
    const Kernel * const kernel = mBranches[branchType]; assert (kernel);
    Value * handle = nullptr;
    if (LLVM_LIKELY(kernel->isStateful())) {
        handle = b->getScalarField(SHARED_PREFIX + std::to_string(branchType));
        if (kernel->hasFamilyName()) {
            handle = b->CreatePointerCast(handle, kernel->getSharedStateType()->getPointerTo());
        }
    }
    return handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateKernelMethod(BuilderRef b) {
    if (LLVM_LIKELY(isa<StreamSet>(mCondition))) {
        generateStreamSetBranchMethod(b);
    } else {

    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getConditionRef
 ** ------------------------------------------------------------------------------------------------------------- */
inline const RelationshipRef & getConditionRef(const RelationshipGraph & G) {
    return G[preceding(in_edge(CONDITION_VARIABLE, G), G)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateStreamSetBranchMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::generateStreamSetBranchMethod(BuilderRef b) {

    findBranchDemarcations(b);
    // Two threads could be simultaneously calculating the spans and the later one
    // may finish first. So first we lock here to ensure that the one that finishes
    // first is first in the logical (i.e., single-threaded) sequence of branch calls.
    waitForPriorThreads(b);
    executeBranches(b);
}


inline bool isConstantOne(const Value * const value) {
    return (isa<ConstantInt>(value) && cast<ConstantInt>(value)->isOne());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief findConditionDemarcations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::findBranchDemarcations(BuilderRef b) {

    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    UndefValue * const unknownState = UndefValue::get(boolTy);

    Constant * const BLOCKS_PER_STRIDE = b->getSize(mTarget->getStride() / b->getBitBlockWidth());
    VectorType * const bitBlockTy = b->getBitBlockType();

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const resizeSpan = b->CreateBasicBlock("resizeSpan");
    BasicBlock * const summarizeDemarcations = b->CreateBasicBlock("summarizeDemarcations");
    BasicBlock * const checkCondition = b->CreateBasicBlock("checkCondition");
    BasicBlock * const addDemarcation = b->CreateBasicBlock("addDemarcation");
    BasicBlock * const processStrides = b->CreateBasicBlock("processStrides");


    const RelationshipRef & condRef = getConditionRef(mStreamSetGraph);
    const StreamSetBuffer * const condBuffer = getInputStreamSetBuffer(condRef.Index);

    Value * const numOfConditionStreams = condBuffer->getStreamSetCount(b);
    Value * const numOfConditionBlocks = b->CreateMul(numOfConditionStreams, BLOCKS_PER_STRIDE);

    // Store the base pointer to our condition stream prior to iterating through it incase
    // one of the pipeline branches uses the stream internally and we update the processed
    // item count.
    Value * const basePtr = b->CreatePointerCast(b->getInputStreamBlockPtr(condRef.Name, ZERO, ZERO), bitBlockTy->getPointerTo());

    Value * const spanCapacityPtr = b->getScalarFieldPtr(SPAN_CAPACITY);
    Value * const spanCapacity = b->CreateLoad(spanCapacityPtr);
    Value * const spanBufferPtr = b->getScalarFieldPtr(SPAN_BUFFER);
    Value * const initialSpanBuffer = b->CreateLoad(spanBufferPtr);
    Value * const numOfStrides = getNumOfStrides();
    Value * const largeEnough = b->CreateICmpULT(numOfStrides, spanCapacity);
    b->CreateLikelyCondBr(largeEnough, summarizeDemarcations, resizeSpan);

    b->SetInsertPoint(resizeSpan);
    Value * const newSpanCapacity = b->CreateRoundUp(numOfStrides, spanCapacity);
    b->CreateStore(newSpanCapacity, spanCapacityPtr);
    Value * const newSpanBuffer = b->CreateRealloc(sizeTy, initialSpanBuffer, newSpanCapacity);
    assert (newSpanBuffer->getType() == initialSpanBuffer->getType());
    b->CreateStore(newSpanBuffer, spanBufferPtr);
    b->CreateBr(summarizeDemarcations);

    b->SetInsertPoint(summarizeDemarcations);
    PHINode * const spanBuffer = b->CreatePHI(initialSpanBuffer->getType(), 4, "spanBuffer");
    spanBuffer->addIncoming(initialSpanBuffer, entry);
    spanBuffer->addIncoming(newSpanBuffer, resizeSpan);
    mBranchDemarcationArray = spanBuffer;
    PHINode * const spanIndex = b->CreatePHI(sizeTy, 4, "spanIndex");
    spanIndex->addIncoming(ZERO, entry);
    spanIndex->addIncoming(ZERO, resizeSpan);
    PHINode * const strideIndex = b->CreatePHI(sizeTy, 4, "lastStride");
    strideIndex->addIncoming(ZERO, entry);
    strideIndex->addIncoming(ZERO, resizeSpan);
    PHINode * const currentState = b->CreatePHI(boolTy, 4);
    currentState->addIncoming(unknownState, entry);
    currentState->addIncoming(unknownState, resizeSpan);

    Value * condition = nullptr;
    if (isConstantOne(numOfConditionBlocks)) {
        condition = b->CreateBlockAlignedLoad(basePtr, strideIndex);

        b->CreateBr(checkCondition);
    } else { // OR together every condition block in this stride
        BasicBlock * const combineCondition = b->CreateBasicBlock("combineCondition", addDemarcation);
        b->CreateBr(combineCondition);

        b->SetInsertPoint(combineCondition);
        PHINode * const iteration = b->CreatePHI(sizeTy, 2);
        iteration->addIncoming(ZERO, summarizeDemarcations);
        PHINode * const accumulated = b->CreatePHI(bitBlockTy, 2);
        accumulated->addIncoming(Constant::getNullValue(bitBlockTy), summarizeDemarcations);
        Value * offset = b->CreateMul(strideIndex, BLOCKS_PER_STRIDE);
        offset = b->CreateAdd(offset, iteration);
        condition = b->CreateBlockAlignedLoad(basePtr, offset);
        condition = b->CreateOr(condition, accumulated);
        accumulated->addIncoming(condition, combineCondition);
        Value * const nextIteration = b->CreateAdd(iteration, ONE);
        Value * const more = b->CreateICmpNE(nextIteration, numOfConditionBlocks);
        iteration->addIncoming(nextIteration, b->GetInsertBlock());
        b->CreateCondBr(more, combineCondition, checkCondition);

    }
    b->SetInsertPoint(checkCondition);

    // Check the merged value of our condition block(s); if it differs from
    // the prior value or this is our last stride, then process the strides.
    // Note, however, initially state is "indeterminate" so we silently
    // ignore the first stride unless it is also our last.
    Value * const nextState = b->bitblock_any(condition);
    Value * const sameState = b->CreateICmpEQ(nextState, currentState);
    Value * const firstStride = b->CreateICmpEQ(strideIndex, ZERO);
    Value * const attemptToExtendSpan = b->CreateOr(sameState, firstStride);
    Value * const nextStrideIndex = b->CreateAdd(strideIndex, ONE);
    Value * const notLastStride = b->CreateICmpULT(nextStrideIndex, numOfStrides);
    Value * const checkNextStride = b->CreateAnd(attemptToExtendSpan, notLastStride);

    BasicBlock * const checkConditionExit = b->GetInsertBlock();
    spanBuffer->addIncoming(spanBuffer, checkConditionExit);
    spanIndex->addIncoming(spanIndex, checkConditionExit);
    strideIndex->addIncoming(nextStrideIndex, checkConditionExit);
    currentState->addIncoming(nextState, checkConditionExit);
    b->CreateLikelyCondBr(checkNextStride, summarizeDemarcations, addDemarcation);

    // Add the demarcation
    b->SetInsertPoint(addDemarcation);
    b->CreateStore(strideIndex, b->CreateGEP(spanBuffer, spanIndex));
    Value * const nextSpanIndex = b->CreateAdd(spanIndex, ONE);
    spanBuffer->addIncoming(spanBuffer, addDemarcation);
    spanIndex->addIncoming(nextSpanIndex, addDemarcation);
    strideIndex->addIncoming(nextStrideIndex, addDemarcation);
    currentState->addIncoming(nextState, addDemarcation);
    b->CreateLikelyCondBr(notLastStride, summarizeDemarcations, processStrides);

    b->SetInsertPoint(processStrides);
    // State is initially "indeterminate" during our first stride; however if (and only if)
    // the first stride is also the last stride, then we'll end up adding a demarcation rather
    // than inspecting the second stride to see if we can continue the span.
    Value * const finalSelectedBranch = b->CreateSelect(firstStride, nextState, currentState);
    // We want to calculate the first branch choice from the last. Since we know we've added
    // spanIndex+1 path changes, we can "rewind" the state by XOR'ing the final state
    // spanIndex times, which is equivalent to:
    Value * const branchChanges = b->CreateTrunc(b->CreateAnd(spanIndex, ONE), boolTy);
    mFirstBranchPath = b->CreateXor(finalSelectedBranch, branchChanges);
    // When we reach the last (but not necessarily final) stride of this kernel,
    // we will either "append" the final stride to the current run or add one
    // additional demarcation based on whether we left the loop attempting
    // to extend the span.
    Value * finalSpanIndex = b->CreateSelect(attemptToExtendSpan, spanIndex, nextSpanIndex);
    b->CreateStore(numOfStrides, b->CreateGEP(spanBuffer, finalSpanIndex));
    mBranchDemarcationCount = b->CreateAdd(finalSpanIndex, ONE);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief waitForPriorThreads
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::waitForPriorThreads(BuilderRef b) {

    Value * const myLogicalSegmentNum = getExternalSegNo();
    mLogicalSegmentNumber[BRANCH_INPUT] = b->getScalarFieldPtr(LOGICAL_SEGMENT_NUMBER);

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("waiting", myLogicalSegmentNum);
    #endif

    // wait until we're certain the other branch has completed
    BasicBlock * const otherWait = b->CreateBasicBlock("waitForPriorThread");
    BasicBlock * const kernelStart = b->CreateBasicBlock("executeBranches");
    Value * const segNo = b->CreateAtomicLoadAcquire(mLogicalSegmentNumber[BRANCH_INPUT]);
    b->CreateCondBr(b->CreateICmpEQ(segNo, myLogicalSegmentNum), kernelStart, otherWait);

    b->SetInsertPoint(otherWait);
    b->CreatePThreadYield();
    Value * const nextSegNo = b->CreateAtomicLoadAcquire(mLogicalSegmentNumber[BRANCH_INPUT]);
    b->CreateCondBr(b->CreateICmpEQ(nextSegNo, myLogicalSegmentNum), kernelStart, otherWait);

    b->SetInsertPoint(kernelStart);

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("starting", myLogicalSegmentNum);
    #endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateStreamSetBranchMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::executeBranches(BuilderRef b) {

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    mLogicalSegmentNumber[ALL_ZERO_BRANCH] = b->getScalarFieldPtr(ALL_ZERO_LOGICAL_SEGMENT_NUMBER);
    mActiveThreads[ALL_ZERO_BRANCH] = b->getScalarFieldPtr(ALL_ZERO_ACTIVE_THREADS);
    mLogicalSegmentNumber[NON_ZERO_BRANCH] = b->getScalarFieldPtr(NON_ZERO_LOGICAL_SEGMENT_NUMBER);
    mActiveThreads[NON_ZERO_BRANCH] = b->getScalarFieldPtr(NON_ZERO_ACTIVE_THREADS);

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const allZeroBranch = b->CreateBasicBlock("allZeroBranch");
    BasicBlock * const nonZeroBranch = b->CreateBasicBlock("nonZeroBranch");
    BasicBlock * const exitLoop = b->CreateBasicBlock("exitLoop");

    b->CreateCondBr(mFirstBranchPath, allZeroBranch, nonZeroBranch);

    b->SetInsertPoint(exitLoop);
    mTerminatedPhi = nullptr;
    if (canSetTerminateSignal()) {
        mTerminatedPhi = b->CreatePHI(b->getInt1Ty(), 2);
    }

    b->SetInsertPoint(allZeroBranch);
    PHINode * const allZeroFirstIndex = b->CreatePHI(sizeTy, 2);
    allZeroFirstIndex->addIncoming(ZERO, entry);
    PHINode * const allZeroSpanIndex = b->CreatePHI(sizeTy, 2);
    allZeroSpanIndex->addIncoming(ZERO, entry);
    Value * const allZeroLastIndex = b->CreateLoad(b->CreateGEP(mBranchDemarcationArray, allZeroSpanIndex));
    Value * const nextNonZeroSpanIndex = b->CreateAdd(allZeroSpanIndex, ONE);
    Value * const allZeroNoMore = b->CreateICmpEQ(nextNonZeroSpanIndex, mBranchDemarcationCount);
    executeBranch(b, ALL_ZERO_BRANCH, allZeroFirstIndex, allZeroLastIndex, allZeroNoMore);
    BasicBlock * const allZeroBranchExit = b->GetInsertBlock();
    b->CreateCondBr(allZeroNoMore, exitLoop, nonZeroBranch);

    nonZeroBranch->moveAfter(allZeroBranchExit);

    b->SetInsertPoint(nonZeroBranch);
    PHINode * const nonZeroFirstIndex = b->CreatePHI(sizeTy, 2);
    nonZeroFirstIndex->addIncoming(ZERO, entry);
    PHINode * const nonZeroSpanIndex = b->CreatePHI(sizeTy, 2);
    nonZeroSpanIndex->addIncoming(ZERO, entry);
    Value * const nonZeroLastIndex = b->CreateLoad(b->CreateGEP(mBranchDemarcationArray, nonZeroSpanIndex));
    Value * const nextAllZeroSpanIndex = b->CreateAdd(nonZeroSpanIndex, ONE);
    Value * const nonZeroNoMore = b->CreateICmpEQ(nextAllZeroSpanIndex, mBranchDemarcationCount);
    executeBranch(b, NON_ZERO_BRANCH, nonZeroFirstIndex, nonZeroLastIndex, nonZeroNoMore);
    BasicBlock * const nonZeroBranchExit = b->GetInsertBlock();
    b->CreateCondBr(nonZeroNoMore, exitLoop, allZeroBranch);

    allZeroFirstIndex->addIncoming(nonZeroLastIndex, nonZeroBranchExit);
    nonZeroFirstIndex->addIncoming(allZeroLastIndex, allZeroBranchExit);

    allZeroSpanIndex->addIncoming(nextAllZeroSpanIndex, nonZeroBranchExit);
    nonZeroSpanIndex->addIncoming(nextNonZeroSpanIndex, allZeroBranchExit);

    exitLoop->moveAfter(nonZeroBranch);

    b->SetInsertPoint(exitLoop);
    if (mTerminatedPhi) {
        b->CreateStore(mTerminatedPhi, getTerminationSignalPtr());
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isBranchReady
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::isBranchReady(BuilderRef b, const unsigned branchType) const {
    // If this kernel is internally synchronized, we need only ensure that the other branch
    // is not currently executing. Otherwise we must wait for both to complete.
    const Kernel * const kernel = mBranches[branchType];
    Value * count = b->CreateAtomicLoadAcquire(mActiveThreads[OTHER_BRANCH(branchType)]);
    if (!kernel->hasAttribute(AttrId::InternallySynchronized)) {
        count = b->CreateOr(count, b->CreateAtomicLoadAcquire(mActiveThreads[branchType]));
    }
    return b->CreateIsNull(count);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterBranch
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::enterBranch(BuilderRef b, const unsigned branchType, Value * const noMore) const {

    assert (branchType < mActiveThreads.max_size());
    assert (OTHER_BRANCH(branchType) < mActiveThreads.max_size());
    assert (mActiveThreads[branchType]);
    assert (mActiveThreads[OTHER_BRANCH(branchType)]);
    Constant * const ONE = b->getSize(1);

    // When we enter a branch, we know that because of the "waitForPriorThreads" lock, that
    // no other thread can be here until we release the logical segment number. So we can
    // safely increment the active thread count without having to consider other threads.

    // However, we release it when we *BEGIN* the final span of work for the kernel rather
    // than waiting for it to complete (or we would block all other threads from progressing.)

    // Unfortunately, we can only rely on the kernels within the same branch path being
    // internally synchronized and must block the other threads if they're attempting to
    // work on differing branches.

    const Kernel * const kernel = mBranches[branchType];
    const auto prefix = kernel->getName();

    SmallVector<char, 256> tmp;
    BasicBlock * const otherWait = b->CreateBasicBlock(concat(prefix, "_waitForOtherBranch", tmp));
    BasicBlock * const kernelStart = b->CreateBasicBlock(concat(prefix, "_start", tmp));
    b->CreateCondBr(isBranchReady(b, branchType), kernelStart, otherWait);
    b->SetInsertPoint(otherWait);
    b->CreatePThreadYield();
    b->CreateCondBr(isBranchReady(b, branchType), kernelStart, otherWait);
    b->SetInsertPoint(kernelStart);

    // Increment the number of active threads for this branch
    b->CreateAtomicFetchAndAdd(ONE, mActiveThreads[branchType]);

    // If this is our final span, release this kernel's logical segment once we acquired the
    // logical segement number for this branch.
    Value * const branchSegmentNum = b->CreateLoad(mLogicalSegmentNumber[branchType]);
    Value * const nextBranchSegmentNum = b->CreateAdd(branchSegmentNum, ONE);
    b->CreateStore(nextBranchSegmentNum, mLogicalSegmentNumber[branchType]);

    Value * const mySegmentNum = getExternalSegNo();
    Value * const nextSegmentNum = b->CreateAdd(mySegmentNum, b->CreateZExt(noMore, b->getSizeTy()));
    b->CreateAtomicStoreRelease(nextSegmentNum, mLogicalSegmentNumber[BRANCH_INPUT]);

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("releasing", nextSegmentNum);
    #endif

    return branchSegmentNum;

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief exitBranch
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::exitBranch(BuilderRef b, const unsigned branchType) const {
    // decrement the "number of active threads" count for this branch
    Constant * const ONE = b->getSize(1);
    b->CreateAtomicFetchAndSub(ONE, mActiveThreads[branchType]);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reset
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Vec>
inline void reset(Vec & vec, const unsigned n) {
    vec.resize(n);
    std::fill_n(vec.begin(), n, nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::executeBranch(BuilderRef b,
                                               const unsigned branchType,
                                               Value * const firstIndex,
                                               Value * const lastIndex,
                                               Value * const noMore) {
    Value * const branchSegNum = enterBranch(b, branchType, noMore);

    const Kernel * const kernel = mBranches[branchType];
    if (LLVM_UNLIKELY(kernel == nullptr)) {
        report_fatal_error("empty branch not supported yet.");
    } else {

        Value * const handle = loadHandle(b, branchType);
        // Last can only be 0 if this is the branch's final stride.
        Value * const isFinal = b->CreateIsNull(lastIndex);
        Value * const numOfStrides = b->CreateSub(lastIndex, firstIndex);


        const auto numOfInputs = in_degree(branchType, mStreamSetGraph);
        reset(mBaseInputAddress, numOfInputs);
        reset(mProcessedInputItemPtr, numOfInputs);
        reset(mProcessedInputItems, numOfInputs);
        reset(mAccessibleInputItems, numOfInputs);
        reset(mPopCountRateArray, numOfInputs);
        reset(mNegatedPopCountRateArray, numOfInputs);

        for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & host = mStreamSetGraph[e];
            const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
            const Binding & input = kernel->getInputStreamSetBinding(path.Index);
            // processed input items
            Value * processed = getProcessedInputItemsPtr(host.Index);
            mProcessedInputItemPtr[path.Index] = processed;
            processed = b->CreateLoad(processed);
            mProcessedInputItems[path.Index] = processed;
            // logical base input address
            const StreamSetBuffer * const buffer = getInputStreamSetBuffer(host.Index);
            mBaseInputAddress[path.Index] = buffer->getBaseAddress(b);
            // accessible input items (after non-deferred processed item count)
            Value * const accessible = getAccessibleInputItems(path.Index);
            Value * const provided = calculateAccessibleOrWritableItems(b, kernel, input, firstIndex, lastIndex, accessible);
            mAccessibleInputItems[path.Index] = b->CreateSelect(isFinal, accessible, provided);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
                mPopCountRateArray[path.Index] = b->CreateGEP(mPopCountRateArray[host.Index], firstIndex);
            }
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                mNegatedPopCountRateArray[path.Index] = b->CreateGEP(mNegatedPopCountRateArray[host.Index], firstIndex);
            }
        }

        const auto numOfOutputs = out_degree(branchType, mStreamSetGraph);
        reset(mBaseOutputAddress, numOfOutputs);
        reset(mProducedOutputItemPtr, numOfOutputs);
        reset(mProducedOutputItems, numOfOutputs);
        reset(mWritableOutputItems, numOfOutputs);

        for (const auto & e : make_iterator_range(out_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & host = mStreamSetGraph[e];
            const RelationshipRef & path = mStreamSetGraph[descending(e, mStreamSetGraph)];
            const Binding & output = kernel->getOutputStreamSetBinding(path.Index);
            // produced output items
            Value * produced = getProducedOutputItemsPtr(host.Index);
            mProducedOutputItemPtr[path.Index] = produced;
            produced = b->CreateLoad(produced);
            mProducedOutputItems[path.Index] = produced;
            // logical base input address
            const StreamSetBuffer * const buffer = getOutputStreamSetBuffer(host.Index);
            mBaseOutputAddress[path.Index] = buffer->getBaseAddress(b);
            // writable output items
            Value * const writable = getWritableOutputItems(path.Index);
            Value * const provided = calculateAccessibleOrWritableItems(b, kernel, output, firstIndex, lastIndex, writable);
            mWritableOutputItems[path.Index] = b->CreateSelect(isFinal, writable, provided);
        }

        calculateFinalOutputItemCounts(b, isFinal, branchType);

        Function * const doSegment = kernel->getDoSegmentFunction(b);
        SmallVector<Value *, 64> args;
        args.reserve(doSegment->arg_size());
        if (handle) {
            args.push_back(handle);
        }
        if (kernel->hasThreadLocal()) {
            args.push_back(b->getScalarFieldPtr(THREAD_LOCAL_PREFIX + std::to_string(branchType)));
        }
        if (kernel->hasAttribute(AttrId::InternallySynchronized)) {
            args.push_back(branchSegNum);
        }
        args.push_back(numOfStrides);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            args.push_back(mBaseInputAddress[i]);
            const Binding & input = kernel->getInputStreamSetBinding(i);
            if (isAddressable(input)) {
                args.push_back(mProcessedInputItemPtr[i]);
            } else if (isCountable(input)) {
                args.push_back(mProcessedInputItems[i]);
            }
            args.push_back(mAccessibleInputItems[i]);
            if (mPopCountRateArray[i]) {
                args.push_back(mPopCountRateArray[i]);
            }
            if (mNegatedPopCountRateArray[i]) {
                args.push_back(mNegatedPopCountRateArray[i]);
            }
        }
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            args.push_back(mBaseOutputAddress[i]);
            const Binding & output = kernel->getOutputStreamSetBinding(i);
            if (isAddressable(output)) {
                args.push_back(mProducedOutputItemPtr[i]);
            } else if (isCountable(output)) {
                args.push_back(mProducedOutputItems[i]);
            }
            args.push_back(mWritableOutputItems[i]);
        }

        Value * const terminated = b->CreateCall(doSegment, args);

        // TODO: if either of these kernels "share" an output scalar, copy the scalar value from the
        // branch we took to the state of the branch we avoided. This requires that the branch pipeline
        // exposes them.

        BasicBlock * const incrementItemCounts = b->CreateBasicBlock("incrementItemCounts");
        BasicBlock * const kernelExit = b->CreateBasicBlock("kernelExit");
        if (kernel->canSetTerminateSignal()) {
            b->CreateUnlikelyCondBr(terminated, kernelExit, incrementItemCounts);
        } else {
            b->CreateBr(incrementItemCounts);
        }

        b->SetInsertPoint(incrementItemCounts);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = kernel->getInputStreamSetBinding(i);
            Value * processed = nullptr;
            if (isCountable(input)) {
                Value * const initiallyProcessed = mProcessedInputItems[i];
                processed = b->CreateAdd(initiallyProcessed, mAccessibleInputItems[i]);
                b->CreateStore(processed, mProcessedInputItemPtr[i]);
            } else {
                processed = b->CreateLoad(mProcessedInputItemPtr[i]);
            }
        }

        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const Binding & output = kernel->getOutputStreamSetBinding(i);
            Value * produced = nullptr;
            if (isCountable(output)) {
                Value * const initiallyProduced = mProducedOutputItems[i];
                produced = b->CreateAdd(initiallyProduced, mWritableOutputItems[i]);
                b->CreateStore(produced, mProducedOutputItemPtr[i]);
            } else {
                produced = b->CreateLoad(mProducedOutputItemPtr[i]);
            }
        }

        if (mTerminatedPhi) {
            BasicBlock * const exitBlock = b->GetInsertBlock();
            if (kernel->canSetTerminateSignal()) {
                mTerminatedPhi->addIncoming(terminated, exitBlock);
            } else {
                mTerminatedPhi->addIncoming(b->getFalse(), exitBlock);
            }
        }
        b->CreateBr(kernelExit);

        b->SetInsertPoint(kernelExit);
    }

    exitBranch(b, branchType);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType) {

    using Rational = ProcessingRate::Rational;

    const Kernel * const kernel = mBranches[branchType];

    // Record the writable item counts but when determining the number of Fixed writable items calculate:

    //   CEILING(PRINCIPAL_OR_MIN(Accessible Item Count / Fixed Input Rate) * Fixed Output Rate)

    Rational rateLCM(1);
    bool noPrincipalStream = true;

    for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
        const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
        const Binding & input = kernel->getInputStreamSetBinding(path.Index);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                rateLCM = rate.getRate();
                noPrincipalStream = false;
                break;
            }
            rateLCM = lcm(rateLCM, rate.getRate());
        }
    }

    bool hasFixedRateOutput = false;
    for (const auto & e : make_iterator_range(out_edges(branchType, mStreamSetGraph))) {
        const RelationshipRef & path = mStreamSetGraph[descending(e, mStreamSetGraph)];
        const Binding & output = kernel->getOutputStreamSetBinding(path.Index);
        const ProcessingRate & rate = output.getRate();
        if (rate.isFixed()) {
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRateOutput = true;
        }
    }

    if (hasFixedRateOutput) {

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const calculateFinalItemCounts = b->CreateBasicBlock("calculateFinalItemCounts");
        BasicBlock * const executeKernel = b->CreateBasicBlock("executeKernel");

        b->CreateUnlikelyCondBr(isFinal, calculateFinalItemCounts, executeKernel);

        b->SetInsertPoint(calculateFinalItemCounts);
        Value * minScaledInverseOfAccessibleInput = nullptr;
        for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
            const Binding & input = kernel->getInputStreamSetBinding(path.Index);
            const ProcessingRate & rate = input.getRate();
            if (rate.isFixed() && (noPrincipalStream || input.hasAttribute(AttrId::Principal))) {
                Value * const scaledInverseOfAccessibleInput =
                    b->CreateMulRate(mAccessibleInputItems[path.Index], rateLCM / rate.getRate());
                minScaledInverseOfAccessibleInput =
                    b->CreateUMin(minScaledInverseOfAccessibleInput, scaledInverseOfAccessibleInput);
            }
        }

        assert (minScaledInverseOfAccessibleInput);

        const auto numOfOutputs = out_degree(branchType, mStreamSetGraph);
        SmallVector<Value *, 16> pendingOutputItems(numOfOutputs, nullptr);
        for (const auto & e : make_iterator_range(out_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & path = mStreamSetGraph[descending(e, mStreamSetGraph)];
            const Binding & output = kernel->getOutputStreamSetBinding(path.Index);
            const ProcessingRate & rate = output.getRate();
            if (rate.isFixed()) {
                pendingOutputItems[path.Index] = b->CreateCeilUDivRate(minScaledInverseOfAccessibleInput, rateLCM / rate.getUpperBound());
            }
        }
        b->CreateBr(executeKernel);

        b->SetInsertPoint(executeKernel);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            if (pendingOutputItems[i]) {
                PHINode * const writable = b->CreatePHI(b->getSizeTy(), 2);
                writable->addIncoming(mWritableOutputItems[i], entry);
                writable->addIncoming(pendingOutputItems[i], calculateFinalItemCounts);
                mWritableOutputItems[i] = writable;
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateAccessibleOrWritableItems
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::calculateAccessibleOrWritableItems(BuilderRef b,
                                                                       const Kernel * const kernel, const Binding & binding,
                                                                       Value * const first, Value * const last,
                                                                       Value * const defaultValue) const {
    const ProcessingRate & rate = binding.getRate();
    if (LLVM_LIKELY(rate.isFixed() || rate.isBounded())) {
        Constant * const strideLength = b->getSize(ceiling(rate.getUpperBound() * kernel->getStride()));
        Value * const numOfStrides = b->CreateSub(last, first);
        return b->CreateMul(numOfStrides, strideLength);
    } else if (rate.isPopCount() || rate.isNegatedPopCount()) {
        const auto refPort = getStreamPort(rate.getReference());
        assert (refPort.Type == PortType::Input);
        Value * array = nullptr;
        if (rate.isNegatedPopCount()) {
            array = mNegatedPopCountRateArray[refPort.Number];
        } else {
            array = mPopCountRateArray[refPort.Number];
        }
        Constant * const ONE = b->getSize(1);
        Value * const currentIndex = b->CreateSub(last, ONE);
        Value * const currentSum = b->CreateLoad(b->CreateGEP(array, currentIndex));
        Value * const priorIndex = b->CreateSub(first, ONE);
        Value * const priorSum = b->CreateLoad(b->CreateGEP(array, priorIndex));
        return b->CreateSub(currentSum, priorSum);
    }
    return defaultValue;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalar
 ** ------------------------------------------------------------------------------------------------------------- */
inline Value * OptimizationBranchCompiler::getInputScalar(BuilderRef b, const unsigned scalar) {
    const auto f = mScalarCache.find(scalar);
    if (LLVM_UNLIKELY(f != mScalarCache.end())) {
        return f->second;
    }
    const auto e = in_edge(scalar, mScalarGraph);
    const RelationshipRef & ref = mScalarGraph[e];
    Value * const value = b->getScalarField(ref.Name);
    mScalarCache.emplace(scalar, value);
    return value;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {
    b->CreateFree(b->getScalarField(SPAN_BUFFER));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateFinalizeMethod(BuilderRef b) {
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (kernel) {
            kernel->finalizeInstance(b, loadHandle(b, i));
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> OptimizationBranchCompiler::getFinalOutputScalars(BuilderRef b) {
    // TODO: IMPLEMENT THIS!
    return std::vector<Value *>{};
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBranches
 ** ------------------------------------------------------------------------------------------------------------- */
inline std::array<const Kernel *, 4> makeBranches(const OptimizationBranch * const branch) {
    std::array<const Kernel *, 4> branches;
    branches[BRANCH_INPUT] = branch;
    branches[ALL_ZERO_BRANCH] = branch->getAllZeroKernel();
    branches[NON_ZERO_BRANCH] = branch->getNonZeroKernel();
    branches[BRANCH_OUTPUT] = branch;
    return branches;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBranchCompilers
 ** ------------------------------------------------------------------------------------------------------------- */
inline CompilerArray makeBranchCompilers(BuilderRef b, const OptimizationBranch * const branch) {
    CompilerArray branches;
    branches[BRANCH_INPUT] = nullptr;
    branches[ALL_ZERO_BRANCH] = branch->getAllZeroKernel()->instantiateKernelCompiler(b);
    branches[NON_ZERO_BRANCH] = branch->getNonZeroKernel()->instantiateKernelCompiler(b);
    branches[BRANCH_OUTPUT] = nullptr;
    return branches;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
OptimizationBranchCompiler::OptimizationBranchCompiler(BuilderRef b, OptimizationBranch * const branch) noexcept
: KernelCompiler(branch)
, mCondition(branch->getCondition())
, mBranches(makeBranches(branch))
, mBranchCompiler(makeBranchCompilers(b, branch))
, mStreamSetGraph(makeRelationshipGraph(RelationshipType::StreamSet))
, mScalarGraph(makeRelationshipGraph(RelationshipType::Scalar)) {

}

}

#endif // OPTIMIZATIONBRANCH_COMPILER_HPP
