#ifndef COMMON_PIPELINE_GRAPH_FUNCTIONS_HPP
#define COMMON_PIPELINE_GRAPH_FUNCTIONS_HPP

#include "graphs.h"

namespace kernel {

template <typename T, unsigned n = 16>
using Vec = SmallVector<T, n>;

using Allocator = SlabAllocator<>;

template <typename T>
struct FixedVector {
    FixedVector(const size_t First, const size_t Last, Allocator & A)
    : mArray(A.allocate<T>(Last - First + 1U) - First)
    #ifndef NDEBUG
    , mFirst(First)
    , mLast(Last)
    #endif
    {
        reset(First, Last);
    }

    FixedVector(const size_t Size, Allocator & A)
    : mArray(A.allocate<T>(Size))
    #ifndef NDEBUG
    , mFirst(0)
    , mLast(Size - 1U)
    #endif
    {
        reset(0, Size - 1U);
    }

    inline T operator[](const size_t index) const {
        assert ("index exceeds allocated bounds!" && index >= mFirst && index <= mLast);
        return mArray[index];
    }

    inline T & operator[](const size_t index) {
        assert ("index exceeds allocated bounds!" && index >= mFirst && index <= mLast);
        return mArray[index];
    }

    inline void reset(const size_t First, const size_t Last) {
        assert ("invalid range!" && First <= Last);
        assert ("range exceeds allocated bounds!" && mFirst <= First && mLast >= Last);
        std::fill_n(mArray + First, (Last - First) + 1U, T{});
    }

private:
    T * const mArray;
    #ifndef NDEBUG
    const size_t mFirst;
    const size_t mLast;
    #endif
};

template <typename T>
struct InputPortVector {
    inline InputPortVector(const size_t n, Allocator & A)
    : mArray(n, A) {
    }
    inline T operator[](const StreamSetInputPort port) const {
        return mArray[port.Number];
    }
    inline T & operator[](const StreamSetInputPort port) {
        return mArray[port.Number];
    }
    inline void reset(const size_t n) {
        mArray.reset(0, n);
    }
private:
    FixedVector<T> mArray;
};

template <typename T>
struct OutputPortVector {
    inline OutputPortVector(const size_t n, Allocator & A)
    : mArray(n, A) {
    }
    inline T operator[](const StreamSetOutputPort port) const {
        return mArray[port.Number];
    }
    inline T & operator[](const StreamSetOutputPort port) {
        return mArray[port.Number];
    }
    inline void reset(const size_t n) {
        mArray.reset(0, n);
    }
private:
    FixedVector<T> mArray;
};


template <typename T>
using OwningVec = std::vector<std::unique_ptr<T>>;

#ifndef NDEBUG
static bool isFromCurrentFunction(BuilderRef b, const Value * const value, const bool allowNull = true) {
    if (value == nullptr) {
        return allowNull;
    }
    BasicBlock * const ip = b->GetInsertBlock();
    assert (ip);
    if (isa<Constant>(value)) {
        return true;
    }
    const Function * const builderFunction = ip->getParent();
    assert (builderFunction);
    const Function * function = builderFunction;
    if (isa<Argument>(value)) {
        function = cast<Argument>(value)->getParent();
    } else if (isa<Instruction>(value)) {
        function = cast<Instruction>(value)->getParent()->getParent();
    }
    assert (function);
    if (LLVM_UNLIKELY(&b->getContext() != &value->getContext())) {
        return false;
    }
    return (builderFunction == function);
}
#endif

class PipelineCommonGraphFunctions {
public:

    PipelineCommonGraphFunctions(const RelationshipGraph & sg, const BufferGraph & bg)
    : mStreamGraphRef(sg)
    , mBufferGraphRef(bg) {

    }

    LLVM_READNONE RelationshipGraph::edge_descriptor getReferenceEdge(const size_t kernel, const StreamSetPort port) const;
    LLVM_READNONE unsigned getReferenceBufferVertex(const size_t kernel, const StreamSetPort port) const;
    LLVM_READNONE const StreamSetPort getReference(const size_t kernel, const StreamSetPort port) const;

    LLVM_READNONE unsigned getInputBufferVertex(const size_t kernel, const StreamSetPort inputPort) const;
    LLVM_READNONE StreamSetBuffer * getInputBuffer(const size_t kernel, const StreamSetPort inputPort) const;
    LLVM_READNONE const Binding & getInputBinding(const size_t kernel, const StreamSetPort inputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getInput(const size_t kernel, const StreamSetPort outputPort) const;


    LLVM_READNONE unsigned getOutputBufferVertex(const size_t kernel, const StreamSetPort outputPort) const;
    LLVM_READNONE StreamSetBuffer * getOutputBuffer(const size_t kernel, const StreamSetPort outputPort) const;
    LLVM_READNONE const Binding & getOutputBinding(const size_t kernel, const StreamSetPort outputPort) const;
    LLVM_READNONE const BufferGraph::edge_descriptor getOutput(const size_t kernel, const StreamSetPort outputPort) const;


    LLVM_READNONE unsigned numOfStreamInputs(const unsigned kernel) const;
    LLVM_READNONE unsigned numOfStreamOutputs(const unsigned kernel) const;

    LLVM_READNONE const Binding & getBinding(const unsigned kernel, const StreamSetPort port) const;
    LLVM_READNONE const Kernel * getKernel(const unsigned index) const;

    LLVM_READNONE bool mayHaveNonLinearIO(const size_t kernel) const;


private:

    const RelationshipGraph &   mStreamGraphRef;
    const BufferGraph &         mBufferGraphRef;

};

#define BEGIN_SCOPED_REGION {
#define END_SCOPED_REGION }

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

inline size_t round_up_to(const size_t x, const size_t y) {
    assert(is_power_2(y));
    return (x + y - 1) & -y;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReferenceVertex
 ** ------------------------------------------------------------------------------------------------------------- */
BOOST_NOINLINE
RelationshipGraph::edge_descriptor PipelineCommonGraphFunctions::getReferenceEdge(const size_t kernel, const StreamSetPort port) const {
    using InEdgeIterator = graph_traits<RelationshipGraph>::in_edge_iterator;
    using OutEdgeIterator = graph_traits<RelationshipGraph>::out_edge_iterator;
    RelationshipGraph::vertex_descriptor binding = 0;
    if (port.Type == PortType::Input) {
        InEdgeIterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(kernel, mStreamGraphRef);
        assert (port.Number < std::distance(ei, ei_end));
        const auto e = *(ei + port.Number);
        assert (mStreamGraphRef[e].Number == port.Number);
        binding = source(e, mStreamGraphRef);
    } else { // if (port.Type == PortType::Output) {
        OutEdgeIterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(kernel, mStreamGraphRef);
        assert (port.Number < std::distance(ei, ei_end));
        const auto e = *(ei + port.Number);
        assert (mStreamGraphRef[e].Number == port.Number);
        binding = target(e, mStreamGraphRef);
    }
    assert (mStreamGraphRef[binding].Type == RelationshipNode::IsBinding);
    assert (in_degree(binding, mStreamGraphRef) == 2);

    InEdgeIterator ei, ei_end;
    std::tie(ei, ei_end) = in_edges(binding, mStreamGraphRef);
    assert (std::distance(ei, ei_end) == 2);
    const auto e = *(ei + 1);
    assert (mStreamGraphRef[e].Reason == ReasonType::Reference);
    return e;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReferenceBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCommonGraphFunctions::getReferenceBufferVertex(const size_t kernel, const StreamSetPort port) const {
    return parent(source(getReferenceEdge(kernel, port), mStreamGraphRef), mStreamGraphRef);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReference
 ** ------------------------------------------------------------------------------------------------------------- */
inline const StreamSetPort PipelineCommonGraphFunctions::getReference(const size_t kernel, const StreamSetPort port) const {
    return mStreamGraphRef[getReferenceEdge(kernel, port)];
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCommonGraphFunctions::getInputBufferVertex(const size_t kernel, const StreamSetPort inputPort) const {
    return source(getInput(kernel, inputPort), mBufferGraphRef);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSetBuffer * PipelineCommonGraphFunctions::getInputBuffer(const size_t kernel, const StreamSetPort inputPort) const {
    return mBufferGraphRef[getInputBufferVertex(kernel, inputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCommonGraphFunctions::getInputBinding(const size_t kernel, const StreamSetPort inputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;

    graph_traits<RelationshipGraph>::in_edge_iterator ei, ei_end;
    std::tie(ei, ei_end) = in_edges(kernel, mStreamGraphRef);
    assert (inputPort.Number < static_cast<size_t>(std::distance(ei, ei_end)));
    e = *(ei + inputPort.Number);
    v = source(e, mStreamGraphRef);

    assert (static_cast<StreamSetPort>(mStreamGraphRef[e]) == inputPort);
    const RelationshipNode & rn = mStreamGraphRef[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCommonGraphFunctions::getInput(const size_t kernel, const StreamSetPort inputPort) const {
    assert (inputPort.Type == PortType::Input);
    assert (inputPort.Number < in_degree(kernel, mBufferGraphRef));
    for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraphRef))) {
        const BufferRateData & br = mBufferGraphRef[e];
        if (br.Port.Number == inputPort.Number) {
            return e;
        }
    }
    llvm_unreachable("could not find input port");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBufferVertex
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCommonGraphFunctions::getOutputBufferVertex(const size_t kernel, const StreamSetPort outputPort) const {
    return target(getOutput(kernel, outputPort), mBufferGraphRef);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCommonGraphFunctions::getOutputBinding(const size_t kernel, const StreamSetPort outputPort) const {

    RelationshipGraph::vertex_descriptor v;
    RelationshipGraph::edge_descriptor e;

    graph_traits<RelationshipGraph>::out_edge_iterator ei, ei_end;
    std::tie(ei, ei_end) = out_edges(kernel, mStreamGraphRef);
    assert (outputPort.Number < static_cast<size_t>(std::distance(ei, ei_end)));
    e = *(ei + outputPort.Number);
    v = target(e, mStreamGraphRef);

    assert (static_cast<StreamSetPort>(mStreamGraphRef[e]) == outputPort);

    const RelationshipNode & rn = mStreamGraphRef[v];
    assert (rn.Type == RelationshipNode::IsBinding);
    return rn.Binding;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getOutputBuffer
 ** ------------------------------------------------------------------------------------------------------------- */
StreamSetBuffer * PipelineCommonGraphFunctions::getOutputBuffer(const size_t kernel, const StreamSetPort outputPort) const {
    return mBufferGraphRef[getOutputBufferVertex(kernel, outputPort)].Buffer;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInput
 ** ------------------------------------------------------------------------------------------------------------- */
inline const BufferGraph::edge_descriptor PipelineCommonGraphFunctions::getOutput(const size_t kernel, const StreamSetPort outputPort) const {
    assert (outputPort.Type == PortType::Output);
    assert (outputPort.Number < out_degree(kernel, mBufferGraphRef));
    for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraphRef))) {
        const BufferRateData & br = mBufferGraphRef[e];
        if (br.Port.Number == outputPort.Number) {
            return e;
        }
    }
    llvm_unreachable("could not find output port");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfStreamInputs
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCommonGraphFunctions::numOfStreamInputs(const unsigned kernel) const {
    return in_degree(kernel, mStreamGraphRef);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getNumOfStreamOutputs
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned PipelineCommonGraphFunctions::numOfStreamOutputs(const unsigned kernel) const {
    return out_degree(kernel, mStreamGraphRef);
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getBinding
 ** ------------------------------------------------------------------------------------------------------------- */
const Binding & PipelineCommonGraphFunctions::getBinding(const unsigned kernel, const StreamSetPort port) const {
    if (port.Type == PortType::Input) {
        return getInputBinding(kernel, port);
    } else if (port.Type == PortType::Output) {
        return getOutputBinding(kernel, port);
    }
    llvm_unreachable("unknown port binding type!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernel
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Kernel * PipelineCommonGraphFunctions::getKernel(const unsigned index) const {
    // assert (PipelineInput <= index && index <= PipelineOutput);
    return mStreamGraphRef[index].Kernel;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief mayHaveNonLinearIO
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCommonGraphFunctions::mayHaveNonLinearIO(const size_t kernel) const {

    // If this kernel has I/O that crosses a partition boundary and the
    // buffer itself is not guaranteed to be linear then this kernel
    // may have non-linear I/O. A kernel with non-linear I/O may not be
    // able to execute its full segment without splitting the work across
    // two or more linear sub-segments.

    for (const auto input : make_iterator_range(in_edges(kernel, mBufferGraphRef))) {
        const auto streamSet = source(input, mBufferGraphRef);
        const BufferNode & node = mBufferGraphRef[streamSet];
        if (node.NonLinear) {
            return true;
        }
    }
    for (const auto output : make_iterator_range(out_edges(kernel, mBufferGraphRef))) {
        const auto streamSet = target(output, mBufferGraphRef);
        const BufferNode & node = mBufferGraphRef[streamSet];
        if (node.NonLinear) {
            return true;
        }
    }
    return false;
}

}

#endif // COMMON_HPP
