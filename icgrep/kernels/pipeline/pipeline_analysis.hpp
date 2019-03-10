#include "pipeline_compiler.hpp"
#include "lexographic_ordering.hpp"
#include <boost/graph/topological_sort.hpp>
#include <util/extended_boost_graph_containers.h>

namespace kernel {

// TODO: support call bindings that produce output that are inputs of
// other call bindings or become scalar outputs of the pipeline

// TODO: with a better model of stride rates, we could determine whether
// being unable to execute a kernel implies we won't be able to execute
// another and "skip" over the unnecessary kernels.

#if 1

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph>
void printGraph(const Graph & G, raw_ostream & out, const StringRef name = "G") {

    out << "digraph " << name << " {\n";
    for (auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << "\"];\n";
    }
    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << ";\n";
    }

    out << "}\n\n";
    out.flush();
}

#endif

namespace { // start of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
inline Relationship * getRelationship(not_null<Relationship *> r) {
    return r.get();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
inline Relationship * getRelationship(const Binding & b) {
    return getRelationship(b.getRelationship());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProcessingRate
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename T>
inline ProcessingRate getProcessingRate(const T &) {
    return FixedRate();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getProcessingRate
 ** ------------------------------------------------------------------------------------------------------------- */
template <>
inline ProcessingRate getProcessingRate<Binding>(const Binding & b) {
    return b.getRate();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOrFindRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
inline unsigned addOrFindRelationship(Relationship * const value, Graph & G, Map & M) {
    const auto f = M.find(value);
    if (LLVM_LIKELY(f == M.end())) {
        const auto v = add_vertex(G);
        G[v].Relationship = value;
        M.emplace(value, v);
        return v;
    } else {
        return f->second;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addProducerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Array, typename Graph, typename Map>
void addProducerRelationships(const PortType portType, const unsigned producer, const Array & array, Graph & G, Map & M) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        const auto relationship = addOrFindRelationship(getRelationship(array[i]), G, M);
        add_edge(producer, relationship, RelationshipType{StreamPort{portType, i},
                                                          ReasonType::Explicit,
                                                          getProcessingRate(array[i])}, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addIfConstantOrFindRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
inline unsigned addIfConstantOrFindRelationship(Relationship * const value, Graph & G, Map & M) {
    const auto f = M.find(value);
    if (LLVM_LIKELY(f != M.end())) {
        return f->second;
    } else if (value->isConstant()) {
        return addOrFindRelationship(value, G, M);
    }
    llvm_unreachable("consumer has no producer!");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Array, typename Graph, typename Map>
void addConsumerRelationships(const PortType portType, const unsigned consumer, const Array & array, Graph & G, Map & M) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        const auto relationship = addIfConstantOrFindRelationship(getRelationship(array[i]), G, M);
        add_edge(relationship, consumer, RelationshipType{StreamPort{portType, i},
                                                          ReasonType::Explicit,
                                                          getProcessingRate(array[i])}, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addImplicitConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
void addImplicitConsumerRelationships(const unsigned consumer, const Relationships & R, Graph & G, Map & M) {
    const Kernel * const kernel = G[consumer].Kernel; assert (kernel);
    const auto f = R.find(kernel);
    if (LLVM_UNLIKELY(f != R.end())) {
        for (const auto & e : make_iterator_range(out_edges(f->second, R))) {
            const auto relationship = addIfConstantOrFindRelationship(getRelationship(R[target(e, R)].Relationship), G, M);
            add_edge(relationship, consumer, R[e], G);
        }
    }
}

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGraphBundle PipelineCompiler::makePipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel) {

    enum : unsigned {
        SCALAR,
        STREAM,
        RELATIONSHIP_COUNT
    };

    // copy the list of internal kernels and add in any implicit kernels
    Kernels kernels(pipelineKernel->getKernels());

    Relationships implicitRelationships;
    addRegionSelectorKernels(b, kernels, implicitRelationships);
    addPopCountKernels(b, kernels, implicitRelationships);

    const auto pipelineInput = 0U;
    const auto firstKernel = 1U;
    const auto lastKernel = kernels.size();
    const auto pipelineOutput = lastKernel + 1;

    const auto & call = pipelineKernel->getCallBindings();
    const auto numOfCalls = call.size();
    const auto firstCall = pipelineOutput + 1;
    const auto lastCall = pipelineOutput + numOfCalls;
    const auto initialSize = lastCall + 1;

    /// ------------------------------------------------------------------------------------------
    /// Construct the initial relationship graph G for this pipeline
    /// ------------------------------------------------------------------------------------------

    RelationshipGraph G(initialSize);
    RelationshipMap M;

    // Enumerate all scalars first to make it easy to decide whether the relationship
    // is a scalar or a streamset.
    unsigned firstRelationship[RELATIONSHIP_COUNT];
    unsigned lastRelationship[RELATIONSHIP_COUNT];

    firstRelationship[SCALAR] = initialSize;
    addProducerRelationships(PortType::Input, pipelineInput, pipelineKernel->getInputScalarBindings(), G, M);
    G[pipelineInput].Kernel = pipelineKernel;
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        Kernel * kernel = kernels[i - firstKernel];
        G[i].Kernel = kernel;
        addProducerRelationships(PortType::Output, i, kernel->getOutputScalarBindings(), G, M);
    }
    G[pipelineOutput].Kernel = pipelineKernel;
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        Kernel * kernel = kernels[i - firstKernel];
        addConsumerRelationships(PortType::Input, i, kernel->getInputScalarBindings(), G, M);
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        G[firstCall + i].Callee = cast<Function>(call[i].Callee);
        addConsumerRelationships(PortType::Input, firstCall + i, call[i].Args, G, M);
    }
    addConsumerRelationships(PortType::Output, pipelineOutput, pipelineKernel->getOutputScalarBindings(), G, M);
    lastRelationship[SCALAR] = num_vertices(G) - 1;
    firstRelationship[STREAM] = lastRelationship[SCALAR] + 1;

    // Now enumerate all streamsets
    addProducerRelationships(PortType::Input, pipelineInput, pipelineKernel->getInputStreamSetBindings(), G, M);
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        Kernel * kernel = kernels[i - firstKernel];
        addProducerRelationships(PortType::Output, i, kernel->getOutputStreamSetBindings(), G, M);
    }
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        Kernel * kernel = kernels[i - firstKernel];
        addConsumerRelationships(PortType::Input, i, kernel->getInputStreamSetBindings(), G, M);
        addImplicitConsumerRelationships(i, implicitRelationships, G, M);
    }
    addConsumerRelationships(PortType::Output, pipelineOutput, pipelineKernel->getOutputStreamSetBindings(), G, M);
    lastRelationship[STREAM] = num_vertices(G) - 1;

    /// ------------------------------------------------------------------------------------------
    /// Pipeline optimizations
    /// ------------------------------------------------------------------------------------------

    combineDuplicateKernels(G, kernels);
    removeUnusedKernels(G, lastKernel, lastCall);

    /// ------------------------------------------------------------------------------------------
    /// Compute the lexographical ordering of G
    /// ------------------------------------------------------------------------------------------

    std::vector<unsigned> O;

    lexical_ordering(G, O, "pipeline contains a cycle!");

    Vec<unsigned, 64> ordering;
    for (const auto i : O) {
        if (firstKernel <= i && i <= lastKernel && G[i].Kernel) {
            ordering.push_back(i);
        }
    }

    assert (ordering.size() >= 1);
    assert (ordering.size() <= kernels.size());

    /// ------------------------------------------------------------------------------------------
    /// Transcribe the pipeline graph based on the lexical ordering, accounting for any auxillary
    /// kernels and subsituted kernels/relationships.
    /// ------------------------------------------------------------------------------------------

    const auto n = ordering.size();
    PipelineGraphBundle P(n + 2 + numOfCalls);
    P.LastKernel = P.PipelineInput + n;
    P.PipelineOutput = P.LastKernel + 1;
    P.FirstCall = P.PipelineOutput + 1;
    P.LastCall = P.PipelineOutput + numOfCalls;

    SmallVector<unsigned, 64> subsitution(initialSize, 0);

    // record the new numbering of our active kernels
    subsitution[pipelineInput] = P.PipelineInput;
    P.Graph[P.PipelineInput].Kernel = pipelineKernel;
    for (unsigned i = 0; i < n; ++i) {
        const auto j = ordering[i]; assert (G[j].Kernel);
        assert ("duplicate subsitution?" && j > 0 && subsitution[j] == 0);
        subsitution[j] = P.FirstKernel + i;
        assert (subsitution[j] > P.PipelineInput && subsitution[j] < P.PipelineOutput);
        P.Graph[P.FirstKernel + i].Kernel = G[j].Kernel;
    }
    subsitution[pipelineOutput] = P.PipelineOutput;
    P.Graph[P.PipelineOutput].Kernel = pipelineKernel;

    // record the new numbering of our function calls
    for (unsigned i = 0; i < numOfCalls; ++i) {
        subsitution[firstCall + i] = P.FirstCall + i;
        P.Graph[P.FirstCall + i].Callee = G[firstCall + i].Callee;
    }
    unsigned numOfActiveRelationships[RELATIONSHIP_COUNT] = {0, 0};
    for (unsigned k = 0; k < RELATIONSHIP_COUNT; ++k) {
        for (unsigned i = firstRelationship[k]; i <= lastRelationship[k]; ++i) {
            if (LLVM_LIKELY(G[i].Relationship != nullptr)) {
                numOfActiveRelationships[k]++;
                const auto relationship = add_vertex(P.Graph);
                P.Graph[relationship].Relationship = G[i].Relationship;
                if (LLVM_LIKELY(in_degree(i, G) > 0)) {
                    const auto e = in_edge(i, G);
                    const auto producer = subsitution[source(e, G)];
                    add_edge(producer, relationship, G[e], P.Graph);
                }
                for (const auto & e : make_iterator_range(out_edges(i, G))) {
                    const auto consumer = subsitution[target(e, G)];
                    add_edge(relationship, consumer, G[e], P.Graph);
                }
            }
        }
    }
    P.FirstScalar = P.LastCall + 1;
    P.LastScalar = P.LastCall + numOfActiveRelationships[SCALAR];
    P.FirstStreamSet = P.LastScalar + 1;
    P.LastStreamSet = P.LastScalar + numOfActiveRelationships[STREAM];
    return P;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addRegionSelectorKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addRegionSelectorKernels(BuilderRef b, Kernels & kernels, Relationships & regions) {

    enum : unsigned {
        REGION_START = 0
        , REGION_END = 1
        , SELECTOR = 2
    };

    using Condition = std::array<std::pair<StreamSet *, unsigned>, 3>; // {selector, start, end} x {streamset, streamIndex}

    using RSK = RegionSelectionKernel;
    using Demarcators = RSK::Demarcators;
    using Starts = RSK::Starts;
    using Ends = RSK::Ends;
    using Selectors = RSK::Selectors;

    // TODO: when we support sequentially dependent regions, make sure to test that the start/end are
    // of the same type.

    auto hasSelector = [](const Condition & c) {
        return std::get<0>(c[SELECTOR]) != nullptr;
    };

    auto hasIndependentStartEndStreams = [](const Condition & c) {
        return (c[REGION_START] != c[REGION_END]);
    };

    auto missingRegionStartOrEnd = [](const Condition & c) {
        return std::get<0>(c[REGION_START]) == nullptr || std::get<0>(c[REGION_END]) == nullptr;
    };

    BaseDriver & driver = b->getDriver();

    const auto numOfKernels = kernels.size();

    flat_map<Condition, StreamSet *> alreadyCreated;

    for (unsigned i = 0; i < numOfKernels; ++i) {
        Kernel * const kernel = kernels[i];
        Condition cond{};
        bool hasRegions = false;
        const Bindings & inputs = kernel->getInputStreamSetBindings();
        for (unsigned j = 0; j < inputs.size(); ++j) {
            const Binding & input = inputs[j];
            auto setIfAttributeExists = [&](const AttrId attrId, const unsigned index) {
                if (LLVM_UNLIKELY(input.hasAttribute(attrId))) {
                    const ProcessingRate & rate = input.getRate();
                    if (LLVM_UNLIKELY(!rate.isFixed() || rate.getRate() != RateValue(1))) {
                        report_fatal_error(kernel->getName() + ": region streams must be FixedRate(1).");
                    }
                    if (LLVM_UNLIKELY(std::get<0>(cond[index]) != nullptr)) {
                        std::string tmp;
                        raw_string_ostream msg(tmp);
                        msg << kernel->getName()
                            << " cannot have multiple region ";
                        switch (attrId) {
                            case AttrId::RegionSelector:
                                msg << "selector"; break;
                            case AttrId::IndependentRegionBegin:
                                msg << "start"; break;
                            case AttrId::IndependentRegionEnd:
                                msg << "end"; break;
                            default: llvm_unreachable("unknown region attribute type");
                        }
                        msg << " attributes";
                        report_fatal_error(msg.str());
                    }
                    const Attribute & region = input.findAttribute(attrId);
                    Relationship * const rel = input.getRelationship();
                    cond[index] = std::make_pair(cast<StreamSet>(rel), region.amount());
                    hasRegions = true;
                }
            };
            setIfAttributeExists(AttrId::RegionSelector, SELECTOR);
            setIfAttributeExists(AttrId::IndependentRegionBegin, REGION_START);
            setIfAttributeExists(AttrId::IndependentRegionEnd, REGION_END);
        }

        if (LLVM_UNLIKELY(hasRegions)) {
            const auto f = alreadyCreated.find(cond);
            StreamSet * regionSpans = nullptr;
            if (LLVM_LIKELY(f == alreadyCreated.end())) {
                if (missingRegionStartOrEnd(cond)) {
                    report_fatal_error(kernel->getName() + " must have both a region start and end");
                }
                RSK * selector = nullptr;
                if (hasSelector(cond)) {
                    regionSpans = driver.CreateStreamSet();
                    if (hasIndependentStartEndStreams(cond)) {
                        selector = new RSK(b, Starts{cond[REGION_START]}, Ends{cond[REGION_END]}, Selectors{cond[SELECTOR]}, regionSpans);
                    } else {
                        selector = new RSK(b, Demarcators{cond[REGION_START]}, Selectors{cond[SELECTOR]}, regionSpans);
                    }
                } else if (hasIndependentStartEndStreams(cond)) {
                    regionSpans = driver.CreateStreamSet();
                    selector = new RSK(b, Starts{cond[REGION_START]}, Ends{cond[REGION_END]}, regionSpans);
                } else { // regions span the entire input space; ignore this one
                    continue;
                }
                // Add the kernel to the pipeline
                driver.addKernel(selector);
                kernels.push_back(selector);
                // Mark the region selectors for this kernel
                alreadyCreated.emplace(cond, regionSpans);
            } else { // we've already created the correct region span
                regionSpans = f->second; assert (regionSpans);
            }
            // insert the implicit relationships
            const auto K = regions.addOrFind(kernel);
            const auto R = regions.addOrFind(regionSpans);
            add_edge(K, R, RelationshipType{PortType::Input, -1U, ReasonType::ImplicitRegionSelector}, regions);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPopCountKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPopCountKernels(BuilderRef b, Kernels & kernels, Relationships & implicit) {



    struct Edge {
        CountingType Type;
        StreamPort   Port;
        Edge() : Type(Unknown), Port() { }
        Edge(const CountingType type, const StreamPort port) : Type(type), Port(port) { }
    };

    using Graph = adjacency_list<vecS, vecS, bidirectionalS, Relationship *, Edge>;
    using Vertex = Graph::vertex_descriptor;
    using Map = flat_map<Relationship *, Vertex>;



    const auto numOfKernels = kernels.size();

    Graph G(numOfKernels);
    Map M;

    for (unsigned i = 0; i < numOfKernels; ++i) {

        const Kernel * const kernel = kernels[i];
        const auto numOfInputs = kernel->getNumOfStreamInputs();
        const auto numOfOutputs = kernel->getNumOfStreamOutputs();

        auto addPopCountDependency = [&](
            const StreamPort port,
            const Binding & binding) {
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
                // determine which port this I/O port refers to
                const StreamPort refPort = kernel->getStreamPort(rate.getReference());
                // verify the reference stream is an input port
                if (LLVM_UNLIKELY(refPort.Type != PortType::Input)) {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << kernel->getName();
                    msg << ": pop count rate for ";
                    msg << binding.getName();
                    msg << " cannot refer to an output stream";
                    report_fatal_error(msg.str());
                }
                const Binding & refBinding = kernel->getInputStreamSetBinding(refPort.Number);
                Relationship * const refStream = refBinding.getRelationship();
                const auto f = M.find(refStream);
                Vertex refVertex = 0;
                if (LLVM_UNLIKELY(f != M.end())) {
                    refVertex = f->second;
                } else {
                    if (LLVM_UNLIKELY(refBinding.isDeferred() || !refBinding.getRate().isFixed())) {
                        std::string tmp;
                        raw_string_ostream msg(tmp);
                        msg << kernel->getName();
                        msg << ": pop count reference ";
                        msg << refBinding.getName();
                        msg << " must be a non-deferred Fixed rate stream";
                        report_fatal_error(msg.str());
                    }
                    refVertex = add_vertex(refStream, G);
                }

                // NOTE: self-references will be detected later. Ignore the possibility for now.

                const auto type = rate.isPopCount() ? CountingType::Positive : CountingType::Negative;

                add_edge(i, refVertex, Edge{type, port}, G);
            }
        };

        for (unsigned j = 0; j < numOfInputs; ++j) {
            const auto & input = kernel->getInputStreamSetBinding(j);
            addPopCountDependency(StreamPort{PortType::Input, j}, input);
        }
        for (unsigned j = 0; j < numOfOutputs; ++j) {
            const auto & output = kernel->getOutputStreamSetBinding(j);
            addPopCountDependency(StreamPort{PortType::Output, j}, output);
        }
    }

    const auto n = num_vertices(G);
    if (LLVM_LIKELY(n == numOfKernels)) {
        return;
    }

    BaseDriver & driver = b->getDriver();

    IntegerType * sizeTy = b->getSizeTy();

    for (auto i = numOfKernels + 1; i < n; ++i) {

        CountingType type = CountingType::Unknown;
        for (const auto & e : make_iterator_range(in_edges(i, G))) {
            const Edge & ed = G[e];
            type |= ed.Type;
        }
        assert (type != CountingType::Unknown);

        StreamSet * const input = cast<StreamSet>(G[i]); assert (input);

        Vertex p = 0;
        StreamSet * positive = nullptr;
        if (type & CountingType::Positive) {
            positive = driver.CreateStreamSet(1, sizeTy->getBitWidth());
            p = implicit.addOrFind(positive);
        }

        Vertex n = 0;
        StreamSet * negative = nullptr;
        if (type & CountingType::Negative) {
            negative = driver.CreateStreamSet(1, sizeTy->getBitWidth());
            n = implicit.addOrFind(positive);
        }

        PopCountKernel * popCount = nullptr;
        switch (type) {
            case CountingType::Positive:
                popCount = new PopCountKernel(b, PopCountKernel::POSITIVE, input, positive);
                break;
            case CountingType::Negative:
                popCount = new PopCountKernel(b, PopCountKernel::NEGATIVE, input, negative);
                break;
            case CountingType::Both:
                popCount = new PopCountKernel(b, PopCountKernel::BOTH, input, positive, negative);
                break;
            default: llvm_unreachable("unknown counting type?");
        }
        // Add the kernel to the pipeline
        driver.addKernel(popCount);
        kernels.push_back(popCount);

        // insert the implicit relationships
        for (const auto & e : make_iterator_range(in_edges(i, G))) {
            const Edge & ed = G[e];
            const auto consumer = implicit.addOrFind(kernels[source(e, G)]);
            assert (ed.Type == CountingType::Positive || ed.Type == CountingType::Negative);
            const auto popCount = ed.Type == CountingType::Positive ? p : n;
            add_edge(consumer, popCount, RelationshipType{ed.Port, ReasonType::ImplicitPopCount}, implicit);
        }
    }

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief combineDuplicateKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::combineDuplicateKernels(RelationshipGraph & G, const Kernels & kernels) {

    using Vector = std::vector<unsigned>;

    struct KernelId {
        const std::string Id;
        const Vector Scalars;
        const Vector Streams;

        KernelId(const std::string && id, const Vector & streams, const Vector & scalars)
        : Id(id), Scalars(scalars), Streams(streams) {

        }
        bool operator<(const KernelId & other) const {
            const auto diff = Id.compare(other.Id);
            if (LLVM_LIKELY(diff != 0)) {
                return diff < 0;
            } else {
                return (Scalars < other.Scalars) || (Streams < other.Streams);
            }
        }
    };

    std::map<KernelId, unsigned> Ids;

    Vector scalars;
    Vector streams;

    const auto firstKernel = 1U;
    const auto lastKernel = kernels.size();

    for (;;) {
        bool unmodified = true;
        Ids.clear();
        for (unsigned i = firstKernel; i <= lastKernel; ++i) {
            const Kernel * k = kernels[i - firstKernel];
            // We cannot reason about a family of kernels
            if (k->hasFamilyName()) {
                continue;
            }

            const auto n = in_degree(i, G);
            streams.resize(n);
            scalars.resize(n);
            unsigned numOfStreams = 0;

            for (const auto & e : make_iterator_range(in_edges(i, G))) {
                const RelationshipType & port = G[e];
                const auto relationship = source(e, G);
                if (isa<StreamSet>(G[relationship])) {
                    streams[port.Number] = relationship;
                    ++numOfStreams;
                } else {
                    scalars[port.Number] = relationship;
                }
            }
            streams.resize(numOfStreams);
            scalars.resize(n - numOfStreams);

            KernelId id(k->getName(), streams, scalars);

            const auto f = Ids.emplace(std::move(id), i);
            if (LLVM_UNLIKELY(!f.second)) {
                // We already have an identical kernel; replace kernel i with kernel j
                bool error = false;
                const auto j = f.first->second;
                const auto m = out_degree(j, G);

                if (LLVM_UNLIKELY(out_degree(i, G) != m)) {
                    error = true;
                } else {
                    // Collect all of the output information from kernel j.
                    streams.resize(m);
                    scalars.resize(m);
                    unsigned numOfStreams = 0;
                    for (const auto & e : make_iterator_range(out_edges(j, G))) {
                        const RelationshipType & port = G[e];
                        const auto relationship = target(e, G);
                        if (isa<StreamSet>(G[relationship].Relationship)) {
                            streams[port.Number] = relationship;
                            ++numOfStreams;
                        } else {
                            scalars[port.Number] = relationship;
                        }
                    }
                    streams.resize(numOfStreams);
                    const auto numOfScalars = n - numOfStreams;
                    scalars.resize(numOfScalars);

                    // Replace the consumers of kernel i's outputs with j's.
                    for (const auto & e : make_iterator_range(out_edges(i, G))) {
                        const StreamPort & port = G[e];
                        const auto original = target(e, G);
                        Relationship * const a = G[original].Relationship;
                        unsigned replacement = 0;
                        if (isa<StreamSet>(a)) {
                            replacement = streams[port.Number];
                        } else {
                            replacement = scalars[port.Number];
                        }
                        Relationship * const b = G[replacement].Relationship;
                        if (LLVM_UNLIKELY(a->getType() != b->getType())) {
                            error = true;
                            break;
                        }
                        for (const auto & e : make_iterator_range(out_edges(original, G))) {
                            const auto inputPort = G[e];
                            const auto consumer = target(e, G);
                            add_edge(replacement, consumer, inputPort, G);
                        }
                        clear_out_edges(original, G);
                    }
                    clear_vertex(i, G);
                    G[i].Kernel = nullptr;
                    unmodified = false;
                }
                if (LLVM_UNLIKELY(error)) {
                    report_fatal_error(k->getName() + " is ambiguous: multiple I/O layouts have the same name");
                }
            }
        }
        if (unmodified) {
            break;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief removeUnusedKernels
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::removeUnusedKernels(RelationshipGraph & G, const unsigned lastKernel, const unsigned lastCall) {

    const auto firstKernel = 1U;
    const auto pipelineOutput = lastKernel + 1;
    const auto firstCall = pipelineOutput + 1;

    adjacency_matrix<directedS> M(num_vertices(G));
    for (const auto & e : make_iterator_range(edges(G))) {
        add_edge(source(e, G), target(e, G), M);
    }

    for (auto i = firstKernel; i <= lastKernel; ++i) {
        const Kernel * const kernel = G[i].Kernel;
        if (kernel->hasAttribute(AttrId::SideEffecting)) {
            add_edge(i, pipelineOutput, M);
        }
    }

    for (unsigned i = firstCall; i <= lastCall; ++i) {
        add_edge(i, pipelineOutput, M);
    }

    transitive_closure_dag(M);

    dynamic_bitset<> active(pipelineOutput, false);
    for (const auto & e : make_iterator_range(in_edges(pipelineOutput, M))) {
        const auto k = source(e, M);
        if (k <= lastKernel) {
            active.set(k);
        }
    }

    // Remove every inactive kernel by removing the Kernel pointer and
    // deleting all I/O relationships, including its produced outputs.
    for (auto i = firstKernel; i <= lastKernel; ++i) {
        if (LLVM_UNLIKELY(!active.test(i))) {
            G[i].Kernel = nullptr;
            for (const auto & e : make_iterator_range(out_edges(i, G))) {
                G[target(e, G)].Relationship = nullptr;
            }
            clear_vertex(i, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasZeroExtendedStream
 *
 * Determine whether there are any zero extend attributes on any kernel and verify that every kernel with
 * zero extend attributes have at least one input that is not transitively dependent on a zero extended
 * input stream.
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::hasZeroExtendedStream() const {

    using Graph = adjacency_list<vecS, vecS, bidirectionalS>;

    bool hasZeroExtendedStream = false;

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = mPipelineGraph[i].Kernel; assert (kernel);
        const auto numOfInputs = getNumOfStreamInputs(i);

        Graph G(numOfInputs + 1);

        // enumerate the input relations
        for (unsigned j = 0; j < numOfInputs; ++j) {
            const Binding & input = getInputBinding(i, j);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
                add_edge(j, numOfInputs, G);
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                    report_fatal_error(kernel->getName() + "." + input.getName() +
                                       " cannot have both ZeroExtend and Principal attributes");
                }
            }
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.hasReference())) {
                const StreamPort port = kernel->getStreamPort(rate.getReference());
                assert ("input stream cannot refer to an output stream" && port.Type == PortType::Input);
                add_edge(port.Number, j, G);
            }
        }

        if (LLVM_LIKELY(in_degree(numOfInputs, G) == 0)) {
            continue;
        }

        if (LLVM_UNLIKELY(in_degree(numOfInputs, G) == numOfInputs)) {
            report_fatal_error(kernel->getName() + " requires at least one non-zero-extended input");
        }

        // Identify all transitive dependencies on zero-extended inputs
        transitive_closure_dag(G);

        if (LLVM_UNLIKELY(in_degree(numOfInputs, G) == numOfInputs)) {
            report_fatal_error(kernel->getName() + " requires at least one non-zero-extended input"
                                                   " that does not refer to a zero-extended input");
        }

        hasZeroExtendedStream = true;
    }

    return hasZeroExtendedStream;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineEvaluationOrderOfKernelIO
 *
 * Returns a lexographically sorted list of ports s.t. the inputs will be ordered as close as possible (baring
 * any constraints) to the kernel's original I/O ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determineEvaluationOrderOfKernelIO() {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS>;

    const auto numOfInputs = in_degree(mKernelIndex, mBufferGraph);
    const auto numOfOutputs = out_degree(mKernelIndex, mBufferGraph);
    const auto firstOutput = numOfInputs;
    const auto numOfPorts = numOfInputs + numOfOutputs;


    Graph G(numOfPorts);

    // enumerate the input relations

    SmallVector<unsigned, 16> zext(numOfInputs);

    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & bd = mBufferGraph[e];
        const Binding & input = bd.Binding;
        const ProcessingRate & rate = input.getRate();
        if (rate.hasReference()) {
            const StreamPort ref = mKernel->getStreamPort(rate.getReference());
            assert ("input stream cannot refer to an output stream" && ref.Type == PortType::Input);
            add_edge(ref.Number, bd.Port.Number, G);
        }
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
            zext.push_back(bd.Port.Number);
        }
    }
    assert (std::is_sorted(zext.begin(), zext.end()));

    // and then enumerate the output relations
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mBufferGraph))) {
        const BufferRateData & bd = mBufferGraph[e];
        const Binding & output = bd.Binding;
        const ProcessingRate & rate = output.getRate();
        if (rate.hasReference()) {
            StreamPort ref = mKernel->getStreamPort(rate.getReference());
            if (LLVM_UNLIKELY(ref.Type == PortType::Output)) {
                ref.Number += firstOutput;
            }
            add_edge(ref.Number, bd.Port.Number + firstOutput, G);
        }
    }

    // check any zeroextended inputs last
    const auto n = zext.size();
    if (LLVM_UNLIKELY(n > 0)) {
        zext.push_back(numOfInputs); // sentinal
        for (unsigned i = 0, j = 0; i < numOfInputs; ++i) {
            if (zext[j] == i) {
                ++j;
                continue;
            } else {
                for (unsigned k = 0; k < n; ++k) {
                    const auto t = zext[k];
                    if (i == t) continue;
                    add_edge_if_no_induced_cycle(i, zext[k], G);
                }
            }
        }
    }

    // check any pipeline input first
    if (out_degree(PipelineInput, mBufferGraph)) {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto buffer = getInputBufferVertex(i);
            if (LLVM_UNLIKELY(is_parent(buffer, PipelineInput, mBufferGraph))) {
                for (unsigned j = 0; j < i; ++j) {
                    add_edge_if_no_induced_cycle(i, j, G);
                }
                for (unsigned j = i + 1; j < numOfPorts; ++j) {
                    add_edge_if_no_induced_cycle(i, j, G);
                }
            }
        }
    }

    // ... and check any pipeline output first
    if (in_degree(PipelineOutput, mBufferGraph)) {
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto buffer = getOutputBufferVertex(i);
            if (LLVM_UNLIKELY(has_child(buffer, PipelineOutput, mBufferGraph))) {
                const auto k = firstOutput + i;
                for (unsigned j = 0; j < k; ++j) {
                    add_edge_if_no_induced_cycle(k, j, G);
                }
                for (unsigned j = k + 1; j < numOfPorts; ++j) {
                    add_edge_if_no_induced_cycle(k, j, G);
                }
            }
        }
    }

    // check any dynamic buffer last
    std::vector<unsigned> D;
    D.reserve(numOfOutputs);
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        if (LLVM_UNLIKELY(isa<DynamicBuffer>(getOutputBuffer(i)))) {
            D.push_back(i);
        }
    }

    for (const auto i : D) {
        for (unsigned j = 0; j < numOfInputs; ++j) {
            add_edge_if_no_induced_cycle(j, firstOutput + i, G);
        }
        auto Dj = D.begin();
        for (unsigned j = 0; j < numOfOutputs; ++j) {
            if (*Dj == j) {
                ++Dj;
            } else {
                add_edge_if_no_induced_cycle(firstOutput + j, firstOutput + i, G);
            }
        }
        assert (Dj == D.end());
    }

    // TODO: add additional constraints on input ports to indicate the ones
    // likely to have fewest number of strides?

    mPortEvaluationOrder.clear();

    lexical_ordering(G, mPortEvaluationOrder, Twine{mKernel->getName(), " has cyclic port dependencies."});
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineIOGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineIOGraph PipelineCompiler::makePipelineIOGraph() const {

    PipelineIOGraph G((PipelineOutput - PipelineInput) + 1);
    for (const auto & p : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto buffer = target(p, mBufferGraph);
        for (const auto & c : make_iterator_range(out_edges(buffer, mBufferGraph))) {
            const auto consumer = target(c, mBufferGraph);
            const BufferRateData & cr = mBufferGraph[c];
            add_edge(PipelineInput, consumer, cr.Port.Number, G);
        }
    }
    for (const auto & c : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(c, mBufferGraph);
        const auto & p = in_edge(buffer, mBufferGraph);
        const auto producer = source(p, mBufferGraph);
        const BufferRateData & pr = mBufferGraph[p];
        add_edge(producer, PipelineOutput, pr.Port.Number, G);
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isPipelineInput
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isPipelineInput(const unsigned inputPort) const {
    if (LLVM_LIKELY(in_degree(mKernelIndex, mPipelineIOGraph) == 0)) {
        return false;
    }
    for (const auto & e : make_iterator_range(in_edges(mKernelIndex, mPipelineIOGraph))) {
        if (LLVM_LIKELY(mPipelineIOGraph[e] == inputPort)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isPipelineOutput
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineCompiler::isPipelineOutput(const unsigned outputPort) const {
    if (LLVM_LIKELY(out_degree(mKernelIndex, mPipelineIOGraph) == 0)) {
        return false;
    }
    for (const auto & e : make_iterator_range(out_edges(mKernelIndex, mPipelineIOGraph))) {
        if (LLVM_LIKELY(mPipelineIOGraph[e] == outputPort)) {
            return true;
        }
    }
    return false;
}

} // end of namespace
