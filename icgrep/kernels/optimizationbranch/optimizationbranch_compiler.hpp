#ifndef OPTIMIZATIONBRANCH_COMPILER_HPP
#define OPTIMIZATIONBRANCH_COMPILER_HPP

#include <kernels/optimizationbranch.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>
#include <boost/container/flat_map.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;
using namespace boost;
using namespace boost::container;

using StreamSetGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

struct RelationshipRef {
    unsigned Index;
    StringRef Name;
    RelationshipRef() : Index(0), Name() { }
    RelationshipRef(const unsigned index, StringRef name) : Index(index), Name(name) { }
};

const static std::string BRANCH_PREFIX = "@B";

using RelationshipGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, RelationshipRef>;

using RelationshipCache = flat_map<RelationshipGraph::vertex_descriptor, Value *>;

namespace kernel {

using Port = Kernel::Port;
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

static_assert (ALL_ZERO_BRANCH < NON_ZERO_BRANCH, "invalid branch type ordering");

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

    Value * calculateAccessibleOrWritableItems(BuilderRef b, const Kernel * const kernel, const Binding & binding, Value * const first, Value * const last, Value * const defaultValue) const;

    RelationshipGraph makeRelationshipGraph(const RelationshipType type) const;

private:

    OptimizationBranch * const          mBranch;
    const std::vector<const Kernel *>   mBranches;

    PHINode *                           mTerminatedPhi;




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
        add_edge(BRANCH_INPUT, r, RelationshipRef{i, input.getName()}, G);
    }

    const auto numOfOutputs = getNumOfOutputBindings(mBranch, type);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const auto & output = getOutputBinding(mBranch, type, i);
        const auto r = addRelationship(output.getRelationship());
        add_edge(r, BRANCH_OUTPUT, RelationshipRef{i, output.getName()}, G);
    }

    if (type == RelationshipType::StreamSet && isa<StreamSet>(mBranch->getCondition())) {
        const auto r = addRelationship(mBranch->getCondition());
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    if (type == RelationshipType::Scalar && isa<Scalar>(mBranch->getCondition())) {
        const auto r = addRelationship(mBranch->getCondition());
        add_edge(r, CONDITION_VARIABLE, RelationshipRef{}, G);
    }

    auto findRelationship = [&](const Kernel * kernel, const Binding & binding) {
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

    auto linkRelationships = [&](const Kernel * const kernel, const Vertex branch) {

        const auto numOfInputs = getNumOfInputBindings(kernel, type);
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto & input = getInputBinding(mBranch, type, i);
            const auto r = findRelationship(kernel, input);
            add_edge(r, branch, RelationshipRef{i, input.getName()}, G);
        }

        const auto numOfOutputs = getNumOfOutputBindings(kernel, type);
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto & output = getOutputBinding(kernel, type, i);
            const auto r = findRelationship(kernel, output);
            add_edge(branch, r, RelationshipRef{i, output.getName()}, G);
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
        if (LLVM_LIKELY(kernel->isStateful())) {
            Type * handlePtrType = nullptr;
            if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
                handlePtrType = b->getVoidPtrTy();
            } else {
                handlePtrType = kernel->getKernelType()->getPointerTo();
            }
            mBranch->addInternalScalar(handlePtrType, BRANCH_PREFIX + std::to_string(i));
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
        if (kernel->isStateful() && !kernel->hasFamilyName()) {
            Value * const handle = kernel->createInstance(b);
            b->setScalarField(BRANCH_PREFIX + std::to_string(i), handle);
        }
    }
    std::vector<Value *> args;
    Module * const m = b->getModule();
    Value * terminated = b->getFalse();
    for (unsigned i = ALL_ZERO_BRANCH; i <= NON_ZERO_BRANCH; ++i) {
        const Kernel * const kernel = mBranches[i];
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
    const Kernel * const kernel = mBranches[branchType];
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

    BasicBlock * const entry = b->GetInsertBlock();
    BasicBlock * const loopCond = b->CreateBasicBlock("cond");
    BasicBlock * const summarizeOneStride = b->CreateBasicBlock("summarizeOneStride");
    BasicBlock * const checkStride = b->CreateBasicBlock("checkStride");
    BasicBlock * const processStrides = b->CreateBasicBlock("processStrides");
    BasicBlock * const mergePaths = b->CreateBasicBlock("mergePaths");
    BasicBlock * const nonZeroPath = b->CreateBasicBlock("nonZeroPath");
    BasicBlock * const allZeroPath = b->CreateBasicBlock("allZeroPath");
    BasicBlock * const exit = b->CreateBasicBlock("exit");
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

    const RelationshipRef & condRef = getConditionRef(mStreamSetGraph);
    Value * const numOfConditionStreams = b->getInputStreamSetCount(condRef.Name);
    Value * const numOfConditionBlocks = b->CreateMul(numOfConditionStreams, BLOCKS_PER_STRIDE);

    Value * const offset = b->CreateMul(currentLastIndex, BLOCKS_PER_STRIDE);
    Value * basePtr = b->getInputStreamBlockPtr(condRef.Name, ZERO, offset);
    Type * const BitBlockTy = b->getBitBlockType();
    basePtr = b->CreatePointerCast(basePtr, BitBlockTy->getPointerTo());
    b->CreateBr(summarizeOneStride);

    // Predeclare some phi nodes

    b->SetInsertPoint(nonZeroPath);
    PHINode * const firstNonZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const lastNonZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const allZeroAfterNonZero = b->CreatePHI(boolTy, 2);

    b->SetInsertPoint(allZeroPath);
    PHINode * const firstAllZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const lastAllZeroIndex = b->CreatePHI(sizeTy, 2);
    PHINode * const nonZeroAfterAllZero = b->CreatePHI(boolTy, 2);

    mTerminatedPhi = nullptr;
    if (mBranch->canSetTerminateSignal()) {
        b->SetInsertPoint(mergePaths);
        mTerminatedPhi = b->CreatePHI(boolTy, 2);
    }

    Value * const numOfStrides = mBranch->mNumOfStrides;

    // OR together every condition block in this stride
    b->SetInsertPoint(summarizeOneStride);
    PHINode * const iteration = b->CreatePHI(sizeTy, 2);
    iteration->addIncoming(ZERO, loopCond);
    PHINode * const merged = b->CreatePHI(BitBlockTy, 2);
    merged->addIncoming(Constant::getNullValue(BitBlockTy), loopCond);
    Value * value = b->CreateBlockAlignedLoad(basePtr, iteration);
    value = b->CreateOr(value, merged);
    merged->addIncoming(value, summarizeOneStride);
    Value * const nextIteration = b->CreateAdd(iteration, ONE);
    Value * const more = b->CreateICmpNE(nextIteration, numOfConditionBlocks);
    iteration->addIncoming(nextIteration, b->GetInsertBlock());
    b->CreateCondBr(more, summarizeOneStride, checkStride);

    // Check the merged value of our condition block(s); if it differs from
    // the prior value or this is our last stride, then process the strides.
    // Note, however, initially state is "indeterminate" so we silently
    // ignore the first stride unless it is also our last.
    b->SetInsertPoint(checkStride);
    Value * const nextState = b->bitblock_any(value);
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
    b->SetInsertPoint(nonZeroPath);
    executeBranch(b, NON_ZERO_BRANCH, firstNonZeroIndex, lastNonZeroIndex);
    BasicBlock * const nonZeroPathExit = b->GetInsertBlock();
    firstAllZeroIndex->addIncoming(nextFirst, nonZeroPathExit);
    lastAllZeroIndex->addIncoming(nextLast, nonZeroPathExit);
    nonZeroAfterAllZero->addIncoming(b->getFalse(), nonZeroPathExit);
    b->CreateUnlikelyCondBr(allZeroAfterNonZero, allZeroPath, mergePaths);

    b->SetInsertPoint(allZeroPath);
    executeBranch(b, ALL_ZERO_BRANCH, firstAllZeroIndex, lastAllZeroIndex);
    BasicBlock * const allZeroPathExit = b->GetInsertBlock();
    firstNonZeroIndex->addIncoming(nextFirst, allZeroPathExit);
    lastNonZeroIndex->addIncoming(nextLast, allZeroPathExit);
    allZeroAfterNonZero->addIncoming(b->getFalse(), allZeroPathExit);
    b->CreateUnlikelyCondBr(nonZeroAfterAllZero, nonZeroPath, mergePaths);

    b->SetInsertPoint(mergePaths);
    currentFirstIndex->addIncoming(nextFirst, mergePaths);
    currentLastIndex->addIncoming(nextLast, mergePaths);
    currentState->addIncoming(nextState, mergePaths);
    if (mTerminatedPhi) {
        finished = b->CreateOr(finished, mTerminatedPhi);
    }
    b->CreateLikelyCondBr(finished, exit, loopCond);

    b->SetInsertPoint(exit);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief callKernel
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranchCompiler::executeBranch(BuilderRef b,
                                               const unsigned branchType,
                                               Value * const first,
                                               Value * const last) {


    const Kernel * const kernel = mBranches[branchType];

    Function * const doSegment = kernel->getDoSegmentFunction(b->getModule());

    BasicBlock * incrementItemCounts = nullptr;
    BasicBlock * kernelExit = nullptr;
    if (kernel->canSetTerminateSignal()) {
        incrementItemCounts = b->CreateBasicBlock("incrementItemCounts");
        kernelExit = b->CreateBasicBlock("kernelExit");
    }

    Value * const handle = loadHandle(b, branchType);
    // Last can only be 0 if this is the branch's final stride.
    Value * const isFinal = b->CreateIsNull(last);

    const auto numOfInputs = in_degree(branchType, mStreamSetGraph);

    std::vector<Value *> baseInputAddress(numOfInputs, nullptr);
    std::vector<Value *> processedInputItemPtr(numOfInputs, nullptr);
    std::vector<Value *> processedInputItem(numOfInputs, nullptr);
    std::vector<Value *> accessibleInputItem(numOfInputs, nullptr);
    std::vector<Value *> popCountRateArray(numOfInputs, nullptr);
    std::vector<Value *> negatedPopCountRateArray(numOfInputs, nullptr);

    for (const auto & e : make_iterator_range(in_edges(branchType, mStreamSetGraph))) {
        const RelationshipRef & host = mStreamSetGraph[e];
        const auto & buffer = mBranch->getInputStreamSetBuffer(host.Index);
        const Binding & input = kernel->getInputStreamSetBinding(host.Index);
        const RelationshipRef & path = mStreamSetGraph[preceding(e, mStreamSetGraph)];
        // logical base input address
        baseInputAddress[path.Index] = buffer->getBaseAddress(b.get());
        // processed input items
        Value * processed = mBranch->getProcessedInputItemsPtr(path.Index);
        if (kernel->isCountable(input)) {
            processedInputItemPtr[path.Index] = processed;
            processed = b->CreateLoad(processed);
        }
        processedInputItem[path.Index] = processed;

        // accessible input items (after non-deferred processed item count)
        Value * const accessible = mBranch->getAccessibleInputItems(path.Index);
        Value * const provided = calculateAccessibleOrWritableItems(b, kernel, input, first, last, accessible);
        accessibleInputItem[path.Index] = b->CreateSelect(isFinal, accessible, provided);

        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresPopCountArray))) {
            popCountRateArray[path.Index] = b->CreateGEP(mBranch->mPopCountRateArray[host.Index], first);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
            negatedPopCountRateArray[path.Index] = b->CreateGEP(mBranch->mNegatedPopCountRateArray[host.Index], first);
        }
    }

    const auto numOfOutputs = out_degree(branchType, mStreamSetGraph);

    std::vector<Value *> baseOutputAddress(numOfOutputs, nullptr);
    std::vector<Value *> producedOutputItemPtr(numOfOutputs, nullptr);
    std::vector<Value *> producedOutputItem(numOfOutputs, nullptr);
    std::vector<Value *> writableOutputItem(numOfInputs, nullptr);

    for (const auto & e : make_iterator_range(out_edges(branchType, mStreamSetGraph))) {
        const RelationshipRef & host = mStreamSetGraph[e];
        const auto & buffer = mBranch->getOutputStreamSetBuffer(host.Index);
        const Binding & output = kernel->getOutputStreamSetBinding(host.Index);
        const RelationshipRef & path = mStreamSetGraph[descending(e, mStreamSetGraph)];
        // logical base input address
        baseOutputAddress[path.Index] = buffer->getBaseAddress(b.get());
        // produced output items
        Value * produced = mBranch->getProducedOutputItemsPtr(path.Index);
        if (kernel->isCountable(output)) {
            producedOutputItemPtr[path.Index] = produced;
            produced = b->CreateLoad(produced);
        }
        producedOutputItem[path.Index] = produced;
        Value * const writable = mBranch->getWritableOutputItems(path.Index);
        Value * const provided = calculateAccessibleOrWritableItems(b, kernel, output, first, last, writable);
        writableOutputItem[path.Index] = b->CreateSelect(isFinal, writable, provided);
    }

    std::vector<Value *> args;
    args.reserve(doSegment->arg_size());
    if (handle) {
        args.push_back(handle);
    }
    args.push_back(b->CreateSub(last, first)); // numOfStrides
    for (unsigned i = 0; i < numOfInputs; ++i) {
        args.push_back(baseInputAddress[i]);
        args.push_back(processedInputItem[i]);
        args.push_back(accessibleInputItem[i]);
        if (popCountRateArray[i]) {
            args.push_back(popCountRateArray[i]);
        }
        if (negatedPopCountRateArray[i]) {
            args.push_back(negatedPopCountRateArray[i]);
        }
    }
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        args.push_back(baseOutputAddress[i]);
        args.push_back(producedOutputItem[i]);
        args.push_back(writableOutputItem[i]);
    }

    Value * const terminated = b->CreateCall(doSegment, args);

    // TODO: if either of these kernels "share" an output scalar, copy the scalar value from the
    // branch we took to the state of the branch we avoided. This requires that the branch pipeline
    // exposes them.


    if (incrementItemCounts) {
        b->CreateUnlikelyCondBr(terminated, kernelExit, incrementItemCounts);

        b->SetInsertPoint(incrementItemCounts);
    }

    for (unsigned i = 0; i < numOfInputs; ++i) {
        if (processedInputItemPtr[i]) {
            Value * const processed = processedInputItem[i];
            Value * const itemCount = accessibleInputItem[i];
            Value * const updatedInputCount = b->CreateAdd(processed, itemCount);
            b->CreateStore(updatedInputCount, processedInputItemPtr[i]);
        }
    }

    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (producedOutputItemPtr[i]) {
            Value * const produced = producedOutputItem[i];
            Value * const itemCount = writableOutputItem[i];
            Value * const updatedOutputCount = b->CreateAdd(produced, itemCount);
            b->CreateStore(updatedOutputCount, producedOutputItemPtr[i]);
        }
    }

    if (incrementItemCounts) {
        mTerminatedPhi->addIncoming(terminated, b->GetInsertBlock());
        b->CreateBr(kernelExit);
        b->SetInsertPoint(kernelExit);
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
        Port refPort;
        unsigned refIndex = 0;
        std::tie(refPort, refIndex) = mBranch->getStreamPort(rate.getReference());
        assert (refPort == Port::Input);
        Value * array = nullptr;
        if (rate.isNegatedPopCount()) {
            array = mBranch->mNegatedPopCountRateArray[refIndex];
        } else {
            array = mBranch->mPopCountRateArray[refIndex];
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

inline std::vector<const Kernel *> makeBranches(const OptimizationBranch * const branch) {
    std::vector<const Kernel *> branches(4);
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
        kernel->finalizeInstance(b, loadHandle(b, i));
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
