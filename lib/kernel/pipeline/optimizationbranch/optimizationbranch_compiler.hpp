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
const static std::string BUFFER_HANDLE_SUFFIX = "B";

const static std::string ALL_ZERO_LOGICAL_SEGMENT_NUMBER = "AS";
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
    void constructStreamSetBuffers(BuilderRef b) override;
    void generateInitializeMethod(BuilderRef b);
    void generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides);
    void generateInitializeThreadLocalMethod(BuilderRef b);
    void generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides);
    void generateKernelMethod(BuilderRef b);
    void generateFinalizeThreadLocalMethod(BuilderRef b);
    void generateFinalizeMethod(BuilderRef b);

    std::vector<Value *> getFinalOutputScalars(BuilderRef b) override;

private:

    Value * loadSharedHandle(BuilderRef b, const unsigned branchType) const;

    Value * loadThreadLocalHandle(BuilderRef b, const unsigned branchType) const;

    void allocateOwnedBranchBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal);

    void bindOwnedBuffers(BuilderRef b);

    Value * getInputScalar(BuilderRef b, const unsigned scalar);

    inline unsigned getNumOfInputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getInputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    unsigned getNumOfOutputBindings(const Kernel * const kernel, const RelationshipType type) const;

    const Binding & getOutputBinding(const Kernel * const kernel, const RelationshipType type, const unsigned i) const;

    void generateStreamSetBranchMethod(BuilderRef b);

    void findBranchDemarcations(BuilderRef b);

    void executeBranches(BuilderRef b);

    void executeBranch(BuilderRef b, const unsigned branchType, Value * const firstIndex, Value * const lastIndex, Value * noMore);

    void enterBranch(BuilderRef b, const unsigned branchType, Value * const noMore, const bool internallySynchronized) const;

    void exitBranch(BuilderRef b, const unsigned branchType, Value * const noMore, const bool internallySynchronized) const;

    Value * calculateAccessibleOrWritableItems(BuilderRef b, const Kernel * const kernel, const Binding & binding, Value * const first, Value * const last, Value * const defaultValue) const;

    Value *calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType);

    RelationshipGraph makeRelationshipGraph(const RelationshipType type) const;

private:

    const Relationship * const          mCondition;

    const std::array<const Kernel *, 4> mBranches;

    const CompilerArray                 mBranchCompiler;

    Value *                             mFirstBranchPath = nullptr;
    PHINode *                           mBranchDemarcationArray = nullptr;
    Value *                             mBranchDemarcationCount = nullptr;

    std::array<Value *, 3>              mLogicalSegmentNumber;

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
inline typename graph_traits<Graph>::edge_descriptor parent(const typename graph_traits<Graph>::edge_descriptor & e, const Graph & G) {
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
inline typename graph_traits<Graph>::edge_descriptor child(const typename graph_traits<Graph>::edge_descriptor & e, const Graph & G) {
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

    const auto n = mTarget->getNumOfStreamOutputs();

    for (unsigned i = 0; i != n; ++i) {
        const Binding & output = mTarget->getOutputStreamSetBinding(i);
        if (LLVM_LIKELY(Kernel::isLocalBuffer(output, false))) {
            Type * const handleTy = mStreamSetOutputBuffers[i]->getHandleType(b);
            mTarget->addInternalScalar(handleTy, output.getName() + BUFFER_HANDLE_SUFFIX);
        }
    }

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
 * @brief constructStreamSetBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::constructStreamSetBuffers(BuilderRef b) {

    mStreamSetInputBuffers.clear();
    const auto numOfInputStreams = mInputStreamSets.size();
    mStreamSetInputBuffers.resize(numOfInputStreams);
    for (unsigned i = 0; i < numOfInputStreams; ++i) {
        const Binding & input = mInputStreamSets[i];
        mStreamSetInputBuffers[i].reset(new ExternalBuffer(b, input.getType(), true, 0));
    }

    mStreamSetOutputBuffers.clear();
    const auto numOfOutputStreams = mOutputStreamSets.size();
    mStreamSetOutputBuffers.resize(numOfOutputStreams);
    for (unsigned i = 0; i < numOfOutputStreams; ++i) {
        const Binding & output = mOutputStreamSets[i];
        StreamSetBuffer * buffer = nullptr;
        if (Kernel::isLocalBuffer(output)) {
            const ProcessingRate & rate = output.getRate();
            const auto ub = rate.getUpperBound() * mTarget->getStride();
            const auto bufferSize = ceiling(ub);
            buffer = new DynamicBuffer(b, output.getType(), bufferSize, 0, 0, true, 0);
        } else {
            buffer = new ExternalBuffer(b, output.getType(), true, 0);
        }
        mStreamSetOutputBuffers[i].reset(buffer);
    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateInitializeMethod(BuilderRef b) {
    using TC = KernelBuilder::TerminationCode;

    bindOwnedBuffers(b);

    mScalarCache.clear();
    std::vector<Value *> args;
    Value * terminated = b->getFalse();
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (LLVM_UNLIKELY(kernel == nullptr)) continue;
        const bool hasSharedState = kernel->isStateful();
        const auto firstArgIndex = hasSharedState ? 1U : 0U;
        args.resize(firstArgIndex + in_degree(i, mScalarGraph));
        if (LLVM_LIKELY(hasSharedState)) {
            Value * handle = nullptr;
            if (kernel->hasFamilyName()) {
                handle = b->getScalarField(SHARED_PREFIX + std::to_string(i));
            } else {
                handle = kernel->createInstance(b);
                b->setScalarField(SHARED_PREFIX + std::to_string(i), handle);
            }
            args[0] = handle;
        }
        for (const auto e : make_iterator_range(in_edges(i, mScalarGraph))) {
            const RelationshipRef & ref = mScalarGraph[e];
            const auto j = ref.Index + firstArgIndex;
            args[j] = getInputScalar(b, source(e, mScalarGraph));
        }
        Value * const terminatedOnInit = b->CreateCall(kernel->getInitializeFunction(b), args);
        terminated = b->CreateOr(terminated, terminatedOnInit);
    }

    Value * const termSignal = b->CreateSelect(terminated, b->getSize(TC::Fatal), b->getSize(TC::None));
    b->CreateStore(termSignal, getTerminationSignalPtr());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::bindOwnedBuffers(BuilderRef b) {
    const auto n = mTarget->getNumOfStreamOutputs();
    for (unsigned i = 0; i != n; ++i) {
        const Binding & output = mTarget->getOutputStreamSetBinding(i);
        if (LLVM_LIKELY(Kernel::isLocalBuffer(output, false))) {
            Value * const handle = b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX);
            mStreamSetOutputBuffers[i]->setHandle(handle);
        }
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief allocateOwnedBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::allocateOwnedBranchBuffers(BuilderRef b, Value * const expectedNumOfStrides, const bool nonLocal) {
    assert (expectedNumOfStrides);
    // recursively allocate any internal buffers for the nested kernels, giving them the correct
    // num of strides it should expect to perform
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernelObj = mBranches[i];
        if (LLVM_UNLIKELY(kernelObj == nullptr)) continue;
        if (LLVM_UNLIKELY(kernelObj->allocatesInternalStreamSets())) {
            if (nonLocal || kernelObj->hasThreadLocal()) {
                SmallVector<Value *, 3> params;
                if (LLVM_LIKELY(kernelObj->isStateful())) {
                    params.push_back(loadSharedHandle(b, i));
                }
                Function * func = nullptr;
                if (nonLocal) {
                    func = kernelObj->getAllocateSharedInternalStreamSetsFunction(b, false);
                } else {
                    func = kernelObj->getAllocateThreadLocalInternalStreamSetsFunction(b, false);
                    params.push_back(loadThreadLocalHandle(b, i));
                }
                params.push_back(expectedNumOfStrides);
                b->CreateCall(func, params);
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    allocateOwnedBranchBuffers(b, expectedNumOfStrides, true);
    // allocate any owned output buffers
    const auto n = mTarget->getNumOfStreamOutputs();
    for (unsigned i = 0; i != n; ++i) {
        const Binding & output = mTarget->getOutputStreamSetBinding(i);
        if (LLVM_LIKELY(Kernel::isLocalBuffer(output, false))) {
            auto & buffer = mStreamSetOutputBuffers[i];
            buffer->setHandle(b->getScalarFieldPtr(output.getName() + BUFFER_HANDLE_SUFFIX));
            buffer->allocateBuffer(b, expectedNumOfStrides);
        }
    }
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
 * @brief generateAllocateThreadLocalInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    allocateOwnedBranchBuffers(b, expectedNumOfStrides, false);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief loadSharedHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::loadSharedHandle(BuilderRef b, const unsigned branchType) const {
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
 * @brief loadThreadLocalHandle
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::loadThreadLocalHandle(BuilderRef b, const unsigned branchType) const {
    const Kernel * const kernel = mBranches[branchType]; assert (kernel);
    Value * handle = nullptr;
    if (LLVM_LIKELY(kernel->hasThreadLocal())) {
        handle = b->getScalarField(THREAD_LOCAL_PREFIX + std::to_string(branchType));
        if (kernel->hasFamilyName()) {
            handle = b->CreatePointerCast(handle, kernel->getThreadLocalStateType()->getPointerTo());
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
    return G[parent(in_edge(CONDITION_VARIABLE, G), G)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateStreamSetBranchMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::generateStreamSetBranchMethod(BuilderRef b) {

    findBranchDemarcations(b);
    // Two threads could be simultaneously calculating the spans and the later one
    // may finish first. So first we lock here to ensure that the one that finishes
    // first is first in the logical (i.e., single-threaded) sequence of branch calls.
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

    Value * const accessible = getAccessibleInputItems(condRef.Name);
    const Binding & binding = condRef.Binding;
    const auto condRate = binding.getRate().getLowerBound() * mTarget->getStride();
    Value * const numOfStrides = b->CreateUDivRate(accessible, condRate);
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
 * @brief generateStreamSetBranchMethod
 ** ------------------------------------------------------------------------------------------------------------- */
inline void OptimizationBranchCompiler::executeBranches(BuilderRef b) {

    IntegerType * const sizeTy = b->getSizeTy();
    Constant * const ZERO = b->getSize(0);
    Constant * const ONE = b->getSize(1);

    mLogicalSegmentNumber[ALL_ZERO_BRANCH] = b->getScalarFieldPtr(ALL_ZERO_LOGICAL_SEGMENT_NUMBER);
    mLogicalSegmentNumber[NON_ZERO_BRANCH] = b->getScalarFieldPtr(NON_ZERO_LOGICAL_SEGMENT_NUMBER);

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
 * @brief enterBranch
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::enterBranch(BuilderRef b, const unsigned branchType, Value * const noMore, const bool internallySynchronized) const {

    assert (mLogicalSegmentNumber[branchType]);
    assert (mLogicalSegmentNumber[OTHER_BRANCH(branchType)]);

    const Kernel * const kernel = mBranches[branchType];
    const auto prefix = kernel->getName();

    SmallVector<char, 256> tmp;
    BasicBlock * const acquire = b->CreateBasicBlock(concat(prefix, "_acquire", tmp));
    BasicBlock * const acquired = b->CreateBasicBlock(concat(prefix, "_acquired", tmp));

    b->CreateBr(acquire);

    b->SetInsertPoint(acquire);
    Value * const currentSegNo = b->CreateAtomicLoadAcquire(mLogicalSegmentNumber[branchType]);
    Value * const ready = b->CreateICmpEQ(getExternalSegNo(), currentSegNo);
    b->CreateLikelyCondBr(ready, acquired, acquire);

    b->SetInsertPoint(acquired);
    // If this kernel is internally synchronized and there are no more strides after this,
    // we can immediately release the lock *of the current branch* and rely on the its
    // synchronization for safety. The other branch must wait for this kernel to finish.
    if (internallySynchronized) {

        BasicBlock * const releaseCurrentLock = b->CreateBasicBlock(concat(prefix, "_releaseCurrentLock", tmp));
        BasicBlock * const executeKernel = b->CreateBasicBlock(concat(prefix, "_executeKernel", tmp));

        Constant * const ONE = b->getSize(1);

        b->CreateCondBr(noMore, releaseCurrentLock, executeKernel);

        b->SetInsertPoint(releaseCurrentLock);
        Value * const nextSegNo = b->CreateAdd(currentSegNo, ONE);
        b->CreateAtomicStoreRelease(nextSegNo, mLogicalSegmentNumber[branchType]);
        b->CreateBr(executeKernel);

        b->SetInsertPoint(executeKernel);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief exitBranch
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::exitBranch(BuilderRef b, const unsigned branchType, Value * const noMore,
                                            const bool internallySynchronized) const {
    // decrement the "number of active threads" count for this branch
    Constant * const ONE = b->getSize(1);

    const Kernel * const kernel = mBranches[branchType];
    const auto prefix = kernel->getName();

    SmallVector<char, 256> tmp;
    BasicBlock * const releaseLock = b->CreateBasicBlock(concat(prefix, "_releaseLock", tmp));
    BasicBlock * const exitBranch = b->CreateBasicBlock(concat(prefix, "_exitBranch", tmp));

    b->CreateCondBr(noMore, releaseLock, exitBranch);
    // If there are no more strides then once we finish executing the kernel, we can release the
    // synchronization lock of the other branch as we now no longer have to worry data corruption
    // due to multiple unsynchronized writers.
    b->SetInsertPoint(releaseLock);
    Value * const currentSegNo = getExternalSegNo();
    Value * const nextSegNo = b->CreateAdd(currentSegNo, ONE);
    if (!internallySynchronized) {
        b->CreateAtomicStoreRelease(nextSegNo, mLogicalSegmentNumber[branchType]);
    }
    b->CreateAtomicStoreRelease(nextSegNo, mLogicalSegmentNumber[OTHER_BRANCH(branchType)]);
    b->CreateBr(exitBranch);

    b->SetInsertPoint(exitBranch);
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

    const Kernel * const kernel = mBranches[branchType];

    if (LLVM_UNLIKELY(kernel == nullptr)) {
        report_fatal_error("empty branch not supported yet.");
    }

    const auto internallySynchronized = kernel->hasAttribute(AttrId::InternallySynchronized);
    const auto greedy = kernel->isGreedy();

    enterBranch(b, branchType, noMore, internallySynchronized);

    Value * const handle = loadSharedHandle(b, branchType);
    Value * const isFinal = b->CreateAnd(mIsFinal, noMore);
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
        const RelationshipRef & path = mStreamSetGraph[parent(e, mStreamSetGraph)];
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
        const RelationshipRef & path = mStreamSetGraph[child(e, mStreamSetGraph)];
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
        Value * const writable = getWritableOutputItems(path.Index); assert (writable);
        Value * const provided = calculateAccessibleOrWritableItems(b, kernel, output, firstIndex, lastIndex, writable); assert (provided);
        mWritableOutputItems[path.Index] = b->CreateSelect(isFinal, writable, provided);
    }

    Function * const doSegment = kernel->getDoSegmentFunction(b);
    SmallVector<Value *, 64> args;
    args.reserve(doSegment->arg_size());

    FunctionType * const funcType = cast<FunctionType>(doSegment->getType()->getPointerElementType());


    auto addNextArg = [&](Value * arg) {
        assert ("null argument" && arg);
        errs() << "ARG: "; arg->getType()->print(errs()); errs() << "\n";
        errs() << "EXP: "; funcType->getParamType(args.size())->print(errs()); errs() << "\n";

        assert ("too many arguments?" && args.size() < funcType->getNumParams());
        assert ("invalid argument type" && (funcType->getParamType(args.size()) == arg->getType()));
        args.push_back(arg);
    };


    if (handle) {
        addNextArg(handle);
    }
    if (kernel->hasThreadLocal()) {
        addNextArg(loadThreadLocalHandle(b, branchType));
    }

    if (internallySynchronized || greedy) {
        if (internallySynchronized) {
            addNextArg(getExternalSegNo());
            addNextArg(mStrideRateFactor);
        }
        addNextArg(isFinal);
    } else {
        Value * const fixedRateFactor = calculateFinalOutputItemCounts(b, isFinal, branchType);
        addNextArg(numOfStrides);
        if (fixedRateFactor) {
            addNextArg(fixedRateFactor);
        }
    }

    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = kernel->getInputStreamSetBinding(i);
        addNextArg(mBaseInputAddress[i]);
        if (internallySynchronized || isAddressable(input)) {
            addNextArg(mProcessedInputItemPtr[i]);
        } else if (isCountable(input)) {
            addNextArg(mProcessedInputItems[i]);
        }
        if (internallySynchronized || requiresItemCount(input)) {
            addNextArg(mAccessibleInputItems[i]);
        }
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        const auto isLocal = internallySynchronized || Kernel::isLocalBuffer(output);
        if (isLocal) {
            addNextArg(mUpdatableOutputBaseVirtualAddressPtr[i]);
        } else {
            addNextArg(mBaseOutputAddress[i]);
        }
        if (internallySynchronized || isAddressable(output)) {
            addNextArg(mProducedOutputItemPtr[i]);
        } else if (isCountable(output)) {
            addNextArg(mProducedOutputItems[i]);
        }
        if (isLocal) {
            addNextArg(mConsumedOutputItems[i]);
        } else {
            addNextArg(mWritableOutputItems[i]);
        }
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
        }
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = kernel->getOutputStreamSetBinding(i);
        Value * produced = nullptr;
        if (isCountable(output)) {
            Value * const initiallyProduced = mProducedOutputItems[i];
            produced = b->CreateAdd(initiallyProduced, mWritableOutputItems[i]);
            b->CreateStore(produced, mProducedOutputItemPtr[i]);
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

    exitBranch(b, branchType, noMore, internallySynchronized);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief calculateFinalItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
Value * OptimizationBranchCompiler::calculateFinalOutputItemCounts(BuilderRef b, Value * const isFinal, const unsigned branchType) {

    using Rational = ProcessingRate::Rational;

    const Kernel * const kernel = mBranches[branchType];

    // Record the writable item counts but when determining the number of Fixed writable items calculate:

    //   CEILING(PRINCIPAL_OR_MIN(Accessible Item Count / Fixed Input Rate) * Fixed Output Rate)

    Rational rateLCM(1);
    bool noPrincipalStream = true;
    bool hasFixedRateInput = false;
    for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
        const RelationshipRef & path = mStreamSetGraph[parent(e, mStreamSetGraph)];
        const Binding & input = kernel->getInputStreamSetBinding(path.Index);
        const ProcessingRate & rate = input.getRate();
        if (rate.isFixed()) {
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                rateLCM = rate.getRate();
                noPrincipalStream = false;
                break;
            }
            rateLCM = lcm(rateLCM, rate.getRate());
            hasFixedRateInput = true;
        }
    }

    Value * minScaledInverseOfAccessibleInput = nullptr;

    if (hasFixedRateInput) {

        BasicBlock * const entry = b->GetInsertBlock();
        BasicBlock * const calculateFinalItemCounts = b->CreateBasicBlock("calculateFinalItemCounts");
        BasicBlock * const executeKernel = b->CreateBasicBlock("executeKernel");

        b->CreateUnlikelyCondBr(isFinal, calculateFinalItemCounts, executeKernel);

        b->SetInsertPoint(calculateFinalItemCounts);
        for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
            const RelationshipRef & path = mStreamSetGraph[parent(e, mStreamSetGraph)];
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
            const RelationshipRef & path = mStreamSetGraph[child(e, mStreamSetGraph)];
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

    return minScaledInverseOfAccessibleInput;
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

    errs() << ref.Name << "\n";

    Value * const value = b->getScalarField(ref.Name);
    mScalarCache.emplace(scalar, value);
    return value;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateFinalizeThreadLocalMethod(BuilderRef b) {
    b->CreateFree(b->getScalarField(SPAN_BUFFER));
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (kernel->hasThreadLocal()) {
            SmallVector<Value *, 2> args;
            if (LLVM_LIKELY(kernel->isStateful())) {
                args.push_back(loadSharedHandle(b, i));
            }
            args.push_back(loadThreadLocalHandle(b, i));
            kernel->finalizeThreadLocalInstance(b, args);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::generateFinalizeMethod(BuilderRef b) {
    bindOwnedBuffers(b);
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
        if (LLVM_LIKELY(kernel->isStateful())) {
            kernel->finalizeInstance(b, loadSharedHandle(b, i));
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
