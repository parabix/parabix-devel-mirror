#ifndef OPTIMIZATIONBRANCH_COMPILER_HPP
#define OPTIMIZATIONBRANCH_COMPILER_HPP

#include <kernels/optimizationbranch.h>
#include <kernels/core/streamset.h>
#include <kernels/kernel_builder.h>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <llvm/Support/raw_ostream.h>
#include <llvm/ADT/SmallVector.h>
#include  <array>

using namespace llvm;
using namespace boost;
using namespace boost::container;

// #define PRINT_DEBUG_MESSAGES

namespace kernel {

using StreamSetGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

// NOTE: std::reference_wrapper does not allow zero init, required by boost graph
struct BindingRef {
    BindingRef() noexcept : binding(nullptr) {}
    BindingRef(const Binding & ref) noexcept : binding(&ref) {}
    BindingRef(const Binding * const ref) noexcept : binding(ref) {}
    operator const Binding & () const noexcept {
        return get();
    }
    const Binding & get() const noexcept {
        assert (binding && "was not set!");
        return *binding;
    }
private:
    const Binding * binding;
};

struct RelationshipRef {
    unsigned Index;
    StringRef Name;
    BindingRef Binding;
    RelationshipRef() = default;
    RelationshipRef(const unsigned index, StringRef name, const kernel::Binding & binding) : Index(index), Name(name), Binding(binding) { }
};

const static std::string BRANCH_PREFIX = "B";
const static std::string LOGICAL_SEGMENT_PREFIX = "L";
const static std::string ALL_ZERO_ACTIVE_THREADS = "A";
const static std::string NON_ZERO_ACTIVE_THREADS = "N";

using RelationshipGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, RelationshipRef>;

using RelationshipCache = flat_map<RelationshipGraph::vertex_descriptor, Value *>;


using PortType = Kernel::PortType;
using StreamPort = Kernel::StreamSetPort;
using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;
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


class OptimizationBranchCompiler {

    enum class RelationshipType : unsigned {
        StreamSet
        , Scalar
    };

public:
    OptimizationBranchCompiler(OptimizationBranch * const branch);

    void addBranchProperties(BuilderRef b);
    void generateInitializeMethod(BuilderRef b);
    void generateKernelMethod(BuilderRef b);
    void generateFinalizeMethod(BuilderRef b);
    std::vector<Value *> getFinalOutputScalars(BuilderRef b);

private:

    Value * loadHandle(BuilderRef b, const unsigned branchType) const;

    Value * getInputScalar(BuilderRef b, const unsigned scalar);

    inline unsigned getNumOfInputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getInputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    unsigned getNumOfOutputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getOutputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    void generateStreamSetBranchMethod(BuilderRef b);

    void executeBranch(BuilderRef b, const unsigned branchType, Value * const first, Value * const last);

    void enterBranch(BuilderRef b, const unsigned branchType) const;

    Value * calculateAccessibleOrWritableItems(BuilderRef b, const Kernel * const kernel, const Binding & binding, Value * const first, Value * const last, Value * const defaultValue) const;

    void calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType);

    RelationshipGraph makeRelationshipGraph(const RelationshipType type) const;

private:


    OptimizationBranch * const          mBranch;
    const std::array<const Kernel *, 4> mBranches;

    PHINode *                           mTerminatedPhi;
    BasicBlock *                        mMergePaths;

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

    const auto numOfInputs = getNumOfInputBindings(mBranch, type);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const auto & input = getInputBinding(mBranch, type, i);
        const auto r = addRelationship(input.getRelationship());
        add_edge(BRANCH_INPUT, r, RelationshipRef{i, input.getName(), input}, G);
    }

    const auto numOfOutputs = getNumOfOutputBindings(mBranch, type);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto & output = getOutputBinding(mBranch, type, i);
        const auto r = addRelationship(output.getRelationship());
        add_edge(r, BRANCH_OUTPUT, RelationshipRef{i, output.getName(), output}, G);
    }

    if (type == RelationshipType::StreamSet && isa<StreamSet>(mBranch->getCondition())) {
        const auto r = addRelationship(mBranch->getCondition());
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    if (type == RelationshipType::Scalar && isa<Scalar>(mBranch->getCondition())) {
        const auto r = addRelationship(mBranch->getCondition());
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    auto linkRelationships = [&](const Kernel * const kernel, const Vertex branch) {

        auto findRelationship = [&](const Binding & binding) {
            const Relationship * const rel = binding.getRelationship();
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
            const auto & input = getInputBinding(mBranch, type, i);
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

    linkRelationships(mBranch->getAllZeroKernel(), ALL_ZERO_BRANCH);
    linkRelationships(mBranch->getNonZeroKernel(), NON_ZERO_BRANCH);

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
                handlePtrType = kernel->getKernelType()->getPointerTo();
            }
            mBranch->addInternalScalar(handlePtrType, BRANCH_PREFIX + std::to_string(i));
        }

        if (!kernel->hasAttribute(AttrId::SynchronizationFree)) {
            mBranch->addInternalScalar(b->getSizeTy(), LOGICAL_SEGMENT_PREFIX + std::to_string(i));
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
            b->setScalarField(BRANCH_PREFIX + std::to_string(i), handle);
        }
    }
    std::vector<Value *> args;
    Module * const m = b->getModule();
    Value * terminated = b->getFalse();
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (LLVM_UNLIKELY(kernel == nullptr)) continue;
        const auto hasHandle = kernel->isStateful() ? 1U : 0U;
        args.resize(hasHandle + in_degree(i, mScalarGraph));
        if (LLVM_LIKELY(hasHandle)) {
            args[0] = b->getScalarField(BRANCH_PREFIX + std::to_string(i));
        }
        for (const auto e : make_iterator_range(in_edges(i, mScalarGraph))) {
            const RelationshipRef & ref = mScalarGraph[e];
            const auto j = ref.Index + hasHandle;
            args[j] = getInputScalar(b, source(e, mScalarGraph));
        }
        Value * const terminatedOnInit = b->CreateCall(kernel->getInitFunction(m), args);
        terminated = b->CreateOr(terminated, terminatedOnInit);
    }
    b->CreateStore(terminated, mBranch->getTerminationSignalPtr());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadKernelHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::loadHandle(BuilderRef b, const unsigned branchType) const {
    const Kernel * const kernel = mBranches[branchType]; assert (kernel);
    Value * handle = nullptr;
    if (LLVM_LIKELY(kernel->isStateful())) {
        handle = b->getScalarField(BRANCH_PREFIX + std::to_string(branchType));
        if (kernel->hasFamilyName()) {
            handle = b->CreatePointerCast(handle, kernel->getKernelType()->getPointerTo());
        }
    }
    return handle;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateKernelMethod(BuilderRef b) {
    if (LLVM_LIKELY(isa<StreamSet>(mBranch->getCondition()))) {
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

    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);
    Constant * const BLOCKS_PER_STRIDE = b->getSize(mBranch->getStride() / b->getBitBlockWidth());
    VectorType * const bitBlockTy = b->getBitBlockType();

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const loopCond = b->CreateBasicBlock("cond");
    BasicBlock * const summarizeOneStride = b->CreateBasicBlock("summarizeOneStride");
    BasicBlock * const checkStride = b->CreateBasicBlock("checkStride");
    BasicBlock * const processStrides = b->CreateBasicBlock("processStrides");
    BasicBlock * const nonZeroPath = b->CreateBasicBlock("nonZeroPath");
    BasicBlock * const allZeroPath = b->CreateBasicBlock("allZeroPath");
    mMergePaths = b->CreateBasicBlock("mergePaths");
    BasicBlock * const exit = b->CreateBasicBlock("exit");

    const RelationshipRef & condRef = getConditionRef(mStreamSetGraph);
    const StreamSetBuffer * const condBuffer = mBranch->getInputStreamSetBuffer(condRef.Index);

    Value * const numOfConditionStreams = condBuffer->getStreamSetCount(b.get());
    Value * const numOfConditionBlocks = b->CreateMul(numOfConditionStreams, BLOCKS_PER_STRIDE);

    // Store the base pointer to our condition stream prior to iterating through it incase
    // one of the pipeline branches uses the stream internally and we update the processed
    // item count.

    Value * const basePtr = b->CreatePointerCast(b->getInputStreamBlockPtr(condRef.Name, ZERO, ZERO), bitBlockTy->getPointerTo());
    Value * const numOfStrides = mBranch->getNumOfStrides();

    #ifdef PRINT_DEBUG_MESSAGES
    b->CallPrintInt("- numOfStrides", numOfStrides);
    #endif

    b->CreateBr(loopCond);

    b->SetInsertPoint(loopCond);
    IntegerType * const sizeTy = b->getSizeTy();
    IntegerType * const boolTy = b->getInt1Ty();
    PHINode * const currentFirstIndex = b->CreatePHI(sizeTy, 3, "firstStride");
    currentFirstIndex->addIncoming(ZERO, entry);
    PHINode * const currentLastIndex = b->CreatePHI(sizeTy, 3, "lastStride");
    currentLastIndex->addIncoming(ZERO, entry);
    PHINode * const currentState = b->CreatePHI(boolTy, 3);
    currentState->addIncoming(UndefValue::get(boolTy), entry);
    b->CreateBr(summarizeOneStride);

    // Predeclare some phi nodes
    b->SetInsertPoint(allZeroPath);
    PHINode * const firstAllZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const lastAllZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const nonZeroAfterAllZero = b->CreatePHI(boolTy, 2);

    b->SetInsertPoint(nonZeroPath);
    PHINode * const firstNonZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const lastNonZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const allZeroAfterNonZero = b->CreatePHI(boolTy, 2);

    b->SetInsertPoint(mMergePaths);
    mTerminatedPhi = nullptr;
    if (mBranch->canSetTerminateSignal()) {
        mTerminatedPhi = b->CreatePHI(boolTy, 2);
    }

    // OR together every condition block in this stride
    b->SetInsertPoint(summarizeOneStride);
    PHINode * const iteration = b->CreatePHI(sizeTy, 2);
    iteration->addIncoming(ZERO, loopCond);
    PHINode * const accumulated = b->CreatePHI(bitBlockTy, 2);
    accumulated->addIncoming(Constant::getNullValue(bitBlockTy), loopCond);
    Value * offset = b->CreateMul(currentLastIndex, BLOCKS_PER_STRIDE);
    offset = b->CreateAdd(offset, iteration);
    Value * condition = b->CreateBlockAlignedLoad(basePtr, offset);
    condition = b->CreateOr(condition, accumulated);
    accumulated->addIncoming(condition, summarizeOneStride);
    Value * const nextIteration = b->CreateAdd(iteration, ONE);
    Value * const more = b->CreateICmpNE(nextIteration, numOfConditionBlocks);
    iteration->addIncoming(nextIteration, b->GetInsertBlock());
    b->CreateCondBr(more, summarizeOneStride, checkStride);

    // Check the merged value of our condition block(s); if it differs from
    // the prior value or this is our last stride, then process the strides.
    // Note, however, initially state is "indeterminate" so we silently
    // ignore the first stride unless it is also our last.
    b->SetInsertPoint(checkStride);
    Value * const nextState = b->bitblock_any(condition);
    Value * const sameState = b->CreateICmpEQ(nextState, currentState);
    Value * const firstStride = b->CreateICmpEQ(currentLastIndex, ZERO);
    Value * const continuation = b->CreateOr(sameState, firstStride);
    Value * const nextIndex = b->CreateAdd(currentLastIndex, ONE);
    Value * const notLastStride = b->CreateICmpULT(nextIndex, numOfStrides);
    Value * const checkNextStride = b->CreateAnd(continuation, notLastStride);
    currentLastIndex->addIncoming(nextIndex, checkStride);
    currentFirstIndex->addIncoming(currentFirstIndex, checkStride);
    currentState->addIncoming(nextState, checkStride);
    b->CreateLikelyCondBr(checkNextStride, loopCond, processStrides);

    // Process every stride between [first, last)
    b->SetInsertPoint(processStrides);

    // state is implicitly "indeterminate" during our first stride
    Value * const selectedPath = b->CreateSelect(firstStride, nextState, currentState);
    firstNonZeroIndex->addIncoming(currentFirstIndex, processStrides);
    firstAllZeroIndex->addIncoming(currentFirstIndex, processStrides);
    // When we reach the last (but not necessarily final) stride of this kernel,
    // we will either "append" the final stride to the current run or complete
    // the current run then perform one more iteration for the final stride, depending
    // whether it flips the branch selection state.
    Value * const nextLast = b->CreateSelect(continuation, numOfStrides, nextIndex);
    Value * const nextFirst = b->CreateSelect(continuation, numOfStrides, currentLastIndex);

    lastNonZeroIndex->addIncoming(nextFirst, processStrides);
    lastAllZeroIndex->addIncoming(nextFirst, processStrides);
    Value * finished = b->CreateNot(notLastStride);
    Value * const flipLastState = b->CreateAnd(finished, b->CreateNot(continuation));
    nonZeroAfterAllZero->addIncoming(flipLastState, processStrides);
    allZeroAfterNonZero->addIncoming(flipLastState, processStrides);
    b->CreateCondBr(selectedPath, nonZeroPath, allZeroPath);

    // make the actual calls and take any potential termination signal
    std::array<BasicBlock *, 3> branchExit;

    b->SetInsertPoint(nonZeroPath);
    executeBranch(b, NON_ZERO_BRANCH, firstNonZeroIndex, lastNonZeroIndex);
    branchExit[NON_ZERO_BRANCH] = b->GetInsertBlock();
    firstAllZeroIndex->addIncoming(nextFirst, branchExit[NON_ZERO_BRANCH]);
    lastAllZeroIndex->addIncoming(nextLast, branchExit[NON_ZERO_BRANCH]);
    nonZeroAfterAllZero->addIncoming(b->getFalse(), branchExit[NON_ZERO_BRANCH]);
    b->CreateUnlikelyCondBr(allZeroAfterNonZero, allZeroPath, mMergePaths);

    b->SetInsertPoint(allZeroPath);
    executeBranch(b, ALL_ZERO_BRANCH, firstAllZeroIndex, lastAllZeroIndex);
    branchExit[ALL_ZERO_BRANCH] = b->GetInsertBlock();
    firstNonZeroIndex->addIncoming(nextFirst, branchExit[ALL_ZERO_BRANCH]);
    lastNonZeroIndex->addIncoming(nextLast, branchExit[ALL_ZERO_BRANCH]);
    allZeroAfterNonZero->addIncoming(b->getFalse(), branchExit[ALL_ZERO_BRANCH]);
    b->CreateUnlikelyCondBr(nonZeroAfterAllZero, nonZeroPath, mMergePaths);


    b->SetInsertPoint(mMergePaths);
    currentFirstIndex->addIncoming(nextFirst, mMergePaths);
    currentLastIndex->addIncoming(nextLast, mMergePaths);
    currentState->addIncoming(nextState, mMergePaths);
    if (mTerminatedPhi) {
        finished = b->CreateOr(finished, mTerminatedPhi);
    }
    b->CreateLikelyCondBr(finished, exit, loopCond);

    b->SetInsertPoint(exit);
}

inline static const std::string & getBranchLockName(const unsigned branchType) {
    switch (branchType) {
        case ALL_ZERO_BRANCH: return ALL_ZERO_ACTIVE_THREADS;
        case NON_ZERO_BRANCH: return NON_ZERO_ACTIVE_THREADS;
    }
    llvm_unreachable("unknown branch type!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief enterBranch
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::enterBranch(BuilderRef b, const unsigned branchType) const {

    const Kernel * const kernel = mBranches[branchType];
    // If this kernel is synchronization free, we need only ensure that the other
    // branch is not currently executing.


    // wait until we're certain the other branch has completed
    const auto & name = getBranchLockName(OTHER_BRANCH(branchType));
    Value * const threadLockPtr = b->getScalarFieldPtr(name);

    const auto prefix = kernel->getName();

    BasicBlock * const kernelCheck = b->CreateBasicBlock(prefix + "_checkOtherBranch", mMergePaths);
    BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "_waitForOtherBranch", mMergePaths);
    BasicBlock * const kernelStart = b->CreateBasicBlock(prefix + "_start", mMergePaths);
    b->CreateBr(kernelCheck);

    b->SetInsertPoint(kernelCheck);
    Value * const inFlight = b->CreateAtomicLoadAcquire(threadLockPtr);
    Value * const ready = b->CreateIsNull(inFlight);
    b->CreateCondBr(ready, kernelStart, kernelWait);

    b->SetInsertPoint(kernelWait);
    b->CreatePThreadYield();
    b->CreateBr(kernelCheck);

    b->SetInsertPoint(kernelStart);

}

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief acquireCurrentSegment
 *
 * Before the segment is processed, this loads the segment number of the kernel state and ensures the previous
 * segment is complete (by checking that the acquired segment number is equal to the desired segment number).
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::acquireCurrentSegment(BuilderRef b, const unsigned branchType) {

    const Kernel * const kernel = mBranches[i];
    if (kernel == nullptr || kernel->hasAttribute(AttrId::SynchronizationFree)) {
        return;
    }

    const auto prefix = kernel->getName();
    Value * const waitingOnPtr = b->getScalarFieldPtr(LOGICAL_SEGMENT_PREFIX + std::to_string(branchType));
    BasicBlock * const kernelWait = b->CreateBasicBlock(prefix + "_Wait", mPipelineEnd);
    b->CreateBr(kernelWait);

    b->SetInsertPoint(kernelWait);
    Value * const processedSegmentCount = b->CreateAtomicLoadAcquire(waitingOnPtr);
    Value * const ready = b->CreateICmpEQ(mSegNo, processedSegmentCount);
    BasicBlock * const kernelStart = b->CreateBasicBlock(prefix + "_Start", mPipelineEnd);
    b->CreateCondBr(ready, kernelStart, kernelWait);

    b->SetInsertPoint(kernelStart);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief releaseCurrentSegment
 *
 * After executing the kernel, the segment number must be incremented to release the kernel for the next thread.
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::releaseCurrentSegment(BuilderRef b) {

    const Kernel * const kernel = mBranches[i];
    if (kernel == nullptr || kernel->hasAttribute(AttrId::SynchronizationFree)) {
        return;
    }

    Value * const nextSegNo = b->CreateAdd(mSegNo, b->getSize(1));
    const auto prefix = makeKernelName(mKernelIndex);
    Value * const waitingOnPtr = b->getScalarFieldPtr(prefix + LOGICAL_SEGMENT_SUFFIX);
    b->CreateAtomicStoreRelease(nextSegNo, waitingOnPtr);

}

#endif

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
                                               Value * const first,
                                               Value * const last) {
    // increment the active thread count for this branch
    Constant * const ONE = b->getSize(1);
    const auto & name = getBranchLockName(branchType);
    Value * const threadLockPtr = b->getScalarFieldPtr(name);
    b->CreateAtomicFetchAndAdd(ONE, threadLockPtr);

    enterBranch(b, branchType);

    const Kernel * const kernel = mBranches[branchType];
    if (LLVM_UNLIKELY(kernel == nullptr)) {
        report_fatal_error("empty branch not supported yet.");
    } else {

        Value * const handle = loadHandle(b, branchType);
        // Last can only be 0 if this is the branch's final stride.
        Value * const isFinal = b->CreateIsNull(last);
        Value * const numOfStrides = b->CreateSub(last, first);


        const auto numOfInputs = in_degree(branchType, mStreamSetGraph);
        reset(mBaseInputAddress, numOfInputs);
        reset(mProcessedInputItemPtr, numOfInputs);
        reset(mProcessedInputItems, numOfInputs);
        reset(mAccessibleInputItems, numOfInputs);
        reset(mPopCountRateArray, numOfInputs);
        reset(mNegatedPopCountRateArray, numOfInputs);

        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = kernel->getName();
        b->CallPrintInt(prefix + "_numOfStrides", numOfStrides);
        #endif

        for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & host = mStreamSetGraph[e];
            const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
            const Binding & input = kernel->getInputStreamSetBinding(path.Index);
            // processed input items
            Value * processed = mBranch->getProcessedInputItemsPtr(host.Index);
            mProcessedInputItemPtr[path.Index] = processed;
            processed = b->CreateLoad(processed);
            #ifdef PRINT_DEBUG_MESSAGES
            const auto prefix = kernel->getName() + ":" + input.getName();
            b->CallPrintInt(prefix + "_processedIn", processed);
            #endif
            mProcessedInputItems[path.Index] = processed;
            // logical base input address
            const StreamSetBuffer * const buffer = mBranch->getInputStreamSetBuffer(host.Index);
            mBaseInputAddress[path.Index] = buffer->getBaseAddress(b.get());
            // accessible input items (after non-deferred processed item count)
            Value * const accessible = mBranch->getAccessibleInputItems(path.Index);
            Value * const provided = calculateAccessibleOrWritableItems(b, kernel, input, first, last, accessible);
            mAccessibleInputItems[path.Index] = b->CreateSelect(isFinal, accessible, provided);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_accessible", mAccessibleInputItems[path.Index]);
            #endif
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
                mPopCountRateArray[path.Index] = b->CreateGEP(mBranch->mPopCountRateArray[host.Index], first);
            }
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                mNegatedPopCountRateArray[path.Index] = b->CreateGEP(mBranch->mNegatedPopCountRateArray[host.Index], first);
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
            Value * produced = mBranch->getProducedOutputItemsPtr(host.Index);
            mProducedOutputItemPtr[path.Index] = produced;
            produced = b->CreateLoad(produced);
            mProducedOutputItems[path.Index] = produced;
            #ifdef PRINT_DEBUG_MESSAGES
            const auto prefix = kernel->getName() + ":" + output.getName();
            b->CallPrintInt(prefix + "_producedIn", produced);
            #endif
            // logical base input address
            const StreamSetBuffer * const buffer = mBranch->getOutputStreamSetBuffer(host.Index);
            mBaseOutputAddress[path.Index] = buffer->getBaseAddress(b.get());
            // writable output items
            Value * const writable = mBranch->getWritableOutputItems(path.Index);
            Value * const provided = calculateAccessibleOrWritableItems(b, kernel, output, first, last, writable);
            mWritableOutputItems[path.Index] = b->CreateSelect(isFinal, writable, provided);
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(prefix + "_writable", mWritableOutputItems[path.Index]);
            #endif
        }

        calculateFinalOutputItemCounts(b, isFinal, branchType);

        Function * const doSegment = kernel->getDoSegmentFunction(b->getModule());
        SmallVector<Value *, 64> args;
        args.reserve(doSegment->arg_size());
        if (handle) {
            args.push_back(handle);
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

        BasicBlock * const incrementItemCounts = b->CreateBasicBlock("incrementItemCounts", mMergePaths);
        BasicBlock * const kernelExit = b->CreateBasicBlock("kernelExit", mMergePaths);
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
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(kernel->getName() + "." + input.getName() + "_processedOut", processed);
            #endif
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
            #ifdef PRINT_DEBUG_MESSAGES
            b->CallPrintInt(kernel->getName() + "." + output.getName() + "_producedOut", produced);
            #endif
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

    // decrement the "number of active threads" count for this branch
    b->CreateAtomicFetchAndSub(ONE, threadLockPtr);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType) {

    using RateValue = ProcessingRate::RateValue;

    const Kernel * const kernel = mBranches[branchType];

    // Record the writable item counts but when determining the number of Fixed writable items calculate:

    //   CEILING(PRINCIPAL_OR_MIN(Accessible Item Count / Fixed Input Rate) * Fixed Output Rate)

    RateValue rateLCM(1);
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
        BasicBlock * const calculateFinalItemCounts = b->CreateBasicBlock("calculateFinalItemCounts", mMergePaths);
        BasicBlock * const executeKernel = b->CreateBasicBlock("executeKernel", mMergePaths);

        b->CreateUnlikelyCondBr(isFinal, calculateFinalItemCounts, executeKernel);

        b->SetInsertPoint(calculateFinalItemCounts);
        Value * minScaledInverseOfAccessibleInput = nullptr;
        for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
            const Binding & input = kernel->getInputStreamSetBinding(path.Index);
            const ProcessingRate & rate = input.getRate();
            if (rate.isFixed() && (noPrincipalStream || input.hasAttribute(AttrId::Principal))) {
                Value * const scaledInverseOfAccessibleInput =
                    b->CreateMul2(mAccessibleInputItems[path.Index], rateLCM / rate.getRate());
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
                pendingOutputItems[path.Index] = b->CreateCeilUDiv2(minScaledInverseOfAccessibleInput, rateLCM / rate.getUpperBound());
                #ifdef PRINT_DEBUG_MESSAGES
                const auto prefix = kernel->getName() + ":" + output.getName();
                b->CallPrintInt(prefix + "_writable'", pendingOutputItems[path.Index]);
                #endif
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
        const auto refPort = mBranch->getStreamPort(rate.getReference());
        assert (refPort.Type == PortType::Input);
        Value * array = nullptr;
        if (rate.isNegatedPopCount()) {
            array = mBranch->mNegatedPopCountRateArray[refPort.Number];
        } else {
            array = mBranch->mPopCountRateArray[refPort.Number];
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

inline std::array<const Kernel *, 4> makeBranches(const OptimizationBranch * const branch) {
    std::array<const Kernel *, 4> branches;
    branches[BRANCH_INPUT] = branch;
    branches[ALL_ZERO_BRANCH] = branch->getAllZeroKernel();
    branches[NON_ZERO_BRANCH] = branch->getNonZeroKernel();
    branches[BRANCH_OUTPUT] = branch;
    return branches;
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
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
OptimizationBranchCompiler::OptimizationBranchCompiler(OptimizationBranch * const branch)
: mBranch(branch)
, mBranches(makeBranches(branch))
, mStreamSetGraph(makeRelationshipGraph(RelationshipType::StreamSet))
, mScalarGraph(makeRelationshipGraph(RelationshipType::Scalar)) {

}

}

#endif // OPTIMIZATIONBRANCH_COMPILER_HPP
