#ifndef PIPELINE_KERNEL_HEADER_GRAPHS_H
#define PIPELINE_KERNEL_HEADER_GRAPHS_H

#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/container/flat_set.hpp>
#include <boost/container/flat_map.hpp>
#include <kernel/core/refwrapper.h>
#include <util/extended_boost_graph_containers.h>
#include <toolchain/toolchain.h>
#include <boost/range/adaptor/reversed.hpp>
#include <boost/math/common_factor_rt.hpp>
#include <boost/dynamic_bitset.hpp>
#include <llvm/IR/ValueMap.h>
#include <llvm/ADT/BitVector.h>
#include <llvm/Support/raw_ostream.h>
#include <util/small_flat_set.hpp>

using namespace boost;
using namespace llvm;
using boost::container::flat_set;
using boost::container::flat_map;

namespace kernel {

#include <util/enum_flags.hpp>

using BindingRef = RefWrapper<Binding>;
using PortType = Kernel::PortType;
using StreamSetPort = Kernel::StreamSetPort;
using AttrId = Attribute::KindId;
using Rational = ProcessingRate::Rational;
using RateId = ProcessingRate::KindId;
using Scalars = PipelineKernel::Scalars;
using Kernels = PipelineKernel::Kernels;
using CallBinding = PipelineKernel::CallBinding;
using CallRef = RefWrapper<CallBinding>;
using LengthAssertion = PipelineKernel::LengthAssertion;
using LengthAssertions = PipelineKernel::LengthAssertions;
using BuilderRef = KernelCompiler::BuilderRef;
using ArgIterator = KernelCompiler::ArgIterator;
using InitArgTypes = KernelCompiler::InitArgTypes;

struct RelationshipNode {

    enum RelationshipNodeType : unsigned {
        IsNil
        , IsKernel
        , IsRelationship
        , IsCallee
        , IsBinding
    } Type;

    union {

        const kernel::Kernel * Kernel;
        kernel::Relationship * Relationship;
        CallRef                Callee;
        BindingRef             Binding;

    };

    bool operator == (const RelationshipNode & rn) const {
        return (Type == rn.Type) && (Kernel == rn.Kernel);
    }

    bool operator < (const RelationshipNode & rn) const {
        if(static_cast<unsigned>(Type) < static_cast<unsigned>(rn.Type)) {
            return true;
        }
        if (Type == rn.Type && Kernel < rn.Kernel) {
            return true;
        }
        return false;
    }

    static_assert(sizeof(Kernel) == sizeof(Relationship), "pointer size inequality?");
    static_assert(sizeof(Kernel) == sizeof(Callee), "pointer size inequality?");
    static_assert(sizeof(Kernel) == sizeof(Binding), "pointer size inequality?");

    explicit RelationshipNode() noexcept : Type(IsNil), Kernel(nullptr) { }
    explicit RelationshipNode(std::nullptr_t) noexcept : Type(IsNil), Kernel(nullptr) { }
    explicit RelationshipNode(not_null<const kernel::Kernel *> kernel) noexcept : Type(IsKernel), Kernel(kernel) { }
    explicit RelationshipNode(not_null<kernel::Relationship *> relationship) noexcept : Type(IsRelationship), Relationship(relationship) { }
    explicit RelationshipNode(not_null<const CallBinding *> callee) noexcept : Type(IsCallee), Callee(callee) { }
    explicit RelationshipNode(not_null<const kernel::Binding *> ref) noexcept : Type(IsBinding), Binding(ref) { }
    explicit RelationshipNode(const RelationshipNode & rn) noexcept : Type(rn.Type), Kernel(rn.Kernel) { }

    RelationshipNode & operator = (const RelationshipNode & other) {
        Type = other.Type;
        Kernel = other.Kernel;
        return *this;
    }

};

enum class ReasonType : unsigned {
    None
    // -----------------------------
    , Explicit
    // -----------------------------
    , ImplicitRegionSelector
    , ImplicitPopCount
    // -----------------------------
    , Reference
    // -----------------------------
    , OrderingConstraint
};

struct RelationshipType : public StreamSetPort {
    ReasonType Reason;

    explicit RelationshipType(PortType type, unsigned number, ReasonType reason = ReasonType::Explicit)
    : StreamSetPort(type, number), Reason(reason) { }

    explicit RelationshipType(StreamSetPort port, ReasonType reason = ReasonType::Explicit)
    : StreamSetPort(port), Reason(reason) { }

    explicit RelationshipType(ReasonType reason = ReasonType::None)
    : StreamSetPort(), Reason(reason) { }

    RelationshipType & operator = (const RelationshipType &) = default;

    bool operator == (const RelationshipType & rn) const {
        return (Number == rn.Number) && (Reason == rn.Reason) && (Type == rn.Type);
    }

    bool operator < (const RelationshipType & rn) const {
        if (LLVM_LIKELY(Reason == rn.Reason)) {
            if (LLVM_LIKELY(Type == rn.Type)) {
                return Number < rn.Number;
            }
            return static_cast<unsigned>(Type) < static_cast<unsigned>(rn.Type);
        }
        return static_cast<unsigned>(Reason) < static_cast<unsigned>(rn.Reason);
    }

};

using RelationshipGraph = adjacency_list<vecS, vecS, bidirectionalS, RelationshipNode, RelationshipType, no_property>;

struct Relationships : public RelationshipGraph {
    using Vertex = RelationshipGraph::vertex_descriptor;

    template <typename T>
    inline Vertex add(T key) {
        RelationshipNode k(key);
        return __add(k);
    }

    template <typename T>
    inline Vertex set(T key, Vertex v) {
        RelationshipNode k(key);
        return __set(k, v);
    }

    template <typename T>
    inline Vertex find(T key) {
        RelationshipNode k(key);
        return __find(k);
    }

    template <typename T>
    inline Vertex addOrFind(T key, const bool permitAdd = true) {
        RelationshipNode k(key);
        return __addOrFind(k, permitAdd);
    }

    RelationshipGraph & Graph() {
        return static_cast<RelationshipGraph &>(*this);
    }

private:

    BOOST_NOINLINE Vertex __add(const RelationshipNode & key) {
        assert ("adding an existing relationship key!" && mMap.find(key.Kernel) == mMap.end());
        const auto v = add_vertex(key, *this);
        mMap.emplace(key.Kernel, v);
        assert ((*this)[v] == key);
        assert (__find(key) == v);
        return v;
    }

    BOOST_NOINLINE Vertex __set(const RelationshipNode & key, const Vertex v) {
        auto f = mMap.find(key.Kernel);
        if (LLVM_UNLIKELY(f == mMap.end())) {
            mMap.emplace(key.Kernel, v);
        } else {
            f->second = v;
        }
        assert ((*this)[v] == key);
        assert (__find(key) == v);
        return v;
    }

    BOOST_NOINLINE Vertex __find(const RelationshipNode & key) const {
        const auto f = mMap.find(key.Kernel);
        if (LLVM_LIKELY(f != mMap.end())) {
            const auto v = f->second;
            assert ((*this)[v] == key);
            return v;
        }
        llvm_unreachable("could not find node in relationship graph");
    }

    BOOST_NOINLINE Vertex __addOrFind(const RelationshipNode & key, const bool permitAdd) {
        const auto f = mMap.find(key.Kernel);
        if (f != mMap.end()) {
            const auto v = f->second;
            assert ((*this)[v] == key);
            return v;
        }
        if (LLVM_LIKELY(permitAdd)) {
            return __add(key);
        }
        llvm_unreachable("could not find node in relationship graph");
    }

    flat_map<const void *, Vertex> mMap;
};

enum BufferType : unsigned {
    External = 1
    , Unowned = 2
    , Shared = 4
};

ENABLE_ENUM_FLAGS(BufferType)

struct BufferNode {
    StreamSetBuffer * Buffer = nullptr;
    unsigned Type = 0;
    bool NonLocal = false;
    bool NonLinear = false;

    unsigned CopyBack = 0;
    unsigned CopyBackReflection = 0;

    unsigned LookAhead = 0;
    unsigned LookBehind = 0;
    unsigned MaxAdd = 0;

    bool isOwned() const {
        return (Type & BufferType::Unowned) == 0;
    }

    bool isUnowned() const {
        return (Type & BufferType::Unowned) != 0;
    }

    bool isInternal() const {
        return (Type & BufferType::External) == 0;
    }

    bool isExternal() const {
        return (Type & BufferType::External) != 0;
    }

    bool isShared() const {
        return (Type & BufferType::Shared) != 0;
    }



};

struct BufferPort {

    RelationshipType Port;
    BindingRef Binding;
    Rational Minimum;
    Rational Maximum;

    unsigned LocalPortId = 0U;
    unsigned GlobalPortId = 0U;

    // binding attributes
    unsigned Add = 0;
    unsigned Truncate = 0;
    unsigned Delay = 0;
    unsigned LookAhead = 0;
    unsigned LookBehind = 0;

    bool IsPrincipal = false;
    bool IsZeroExtended = false;
    bool IsDeferred = false;
    bool IsShared = false;
    bool IsManaged = false;

    int TransitiveAdd = 0;

    bool operator < (const BufferPort & rn) const {
        if (LLVM_LIKELY(Port.Type == rn.Port.Type)) {
            return Port.Number < rn.Port.Number;
        }
        return static_cast<unsigned>(Port.Type) < static_cast<unsigned>(rn.Port.Type);
    }

    BufferPort() = default;

    BufferPort(RelationshipType port, const struct Binding & binding,
               Rational minRate, Rational maxRate)
    : Port(port), Binding(binding)
    , Minimum(minRate), Maximum(maxRate) {

    }

};

using BufferGraph = adjacency_list<vecS, vecS, bidirectionalS, BufferNode, BufferPort>;

using BufferVertexSet = SmallFlatSet<BufferGraph::vertex_descriptor, 32>;

struct ConsumerNode {
    mutable Value * Consumed = nullptr;
    mutable PHINode * PhiNode = nullptr;
};

struct ConsumerEdge {

    enum ConsumerTypeFlags : unsigned {
        None = 0
        , UpdatePhi = 1
        , WriteConsumedCount = 2
        , UpdateExternalCount = 4
    };

    unsigned Port = 0;
    unsigned Index = 0;
    unsigned Flags = ConsumerEdge::None;

    ConsumerEdge() = default;

    ConsumerEdge(const StreamSetPort port, const unsigned index, const unsigned flags)
    : Port(port.Number), Index(index), Flags(flags) { }
};

using ConsumerGraph = adjacency_list<vecS, vecS, bidirectionalS, ConsumerNode, ConsumerEdge>;

enum TerminationSignal : unsigned {
    None = KernelBuilder::TerminationCode::None
    , Aborted = KernelBuilder::TerminationCode::Terminated
    , Fatal = KernelBuilder::TerminationCode::Fatal
    , Completed = Aborted | Fatal
};

enum TerminationCheckFlag : unsigned {
    Soft = 1
    , Hard = 2
};

using TerminationChecks = std::vector<unsigned>;

using TerminationPropagationGraph = adjacency_list<vecS, vecS, bidirectionalS>;

enum CountingType : unsigned {
    Unknown = 0
    , Positive = 1
    , Negative = 2
    , Both = Positive | Negative
};

ENABLE_ENUM_FLAGS(CountingType)

using PipelineIOGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, unsigned>;

template <typename T>
using OwningVector = std::vector<std::unique_ptr<T>>;

using Partition = std::vector<unsigned>;

using PartitionConstraintGraph = adjacency_matrix<undirectedS>;

struct PartitioningGraphNode {
    enum TypeId {
        Partition = 0
        , Variable
        , Fixed
        , Greedy
        , PartialSum
        , Relative
    };

    TypeId Type = TypeId::Partition;
    unsigned StreamSet = 0;
    unsigned Delay = 0;

    PartitioningGraphNode() = default;
};

struct PartitioningGraphEdge {
    enum TypeId : unsigned {
        IOPort = 0
        , Channel
        , Reference
    };

    TypeId          Type;
    unsigned        Kernel;
    StreamSetPort   Port;

    PartitioningGraphEdge(unsigned kernel, StreamSetPort port) : Type(IOPort), Kernel(kernel), Port(port) { }
    PartitioningGraphEdge(TypeId type = IOPort, unsigned kernel = 0, StreamSetPort port = StreamSetPort{}) : Type(type), Kernel(kernel), Port(port) { }
};

bool operator < (const PartitioningGraphEdge &A, const PartitioningGraphEdge & B) {
    assert (A.Type == B.Type);
    if (A.Kernel < B.Kernel) {
        return true;
    }
    return (A.Port < B.Port);
}

using PartitioningGraph = adjacency_list<vecS, vecS, bidirectionalS, PartitioningGraphNode, PartitioningGraphEdge>;

using PartitionJumpTree = adjacency_list<vecS, vecS, bidirectionalS, no_property, no_property, no_property>;

struct IOCheckEdge {
    unsigned        Kernel = 0;
    StreamSetPort   Port;

    IOCheckEdge() = default;
    IOCheckEdge(unsigned kernel, StreamSetPort port) : Kernel(kernel), Port(port) { }
};

using IOCheckGraph = adjacency_list<vecS, vecS, bidirectionalS, no_property, IOCheckEdge>;

using LengthConstraintGraph = adjacency_list<vecS, vecS, undirectedS>;

}

#endif // PIPELINE_KERNEL_HEADER_GRAPHS_H
