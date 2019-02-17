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
void printGraph(const Graph & G, raw_ostream & out) {

    out << "digraph G {\n";
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOrFindRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
inline unsigned addOrFindRelationship(const Relationship * const value, Graph & G, Map & M) {
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
void addProducerRelationships(const unsigned producer, const Array & array, Graph & G, Map & M) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        const auto relationship = addOrFindRelationship(getRelationship(array[i]), G, M);
        add_edge(producer, relationship, i, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addIfConstantOrFindRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
inline unsigned addIfConstantOrFindRelationship(const Relationship * const value, Graph & G, Map & M) {
    const auto f = M.find(value);
    if (LLVM_LIKELY(f != M.end())) {
        return f->second;
    } else if (value->isConstant()) {
        const auto relationship = addOrFindRelationship(value, G, M);
        add_edge(0, relationship, SCALAR_CONSTANT, G);
        return relationship;
    } else {
        llvm_unreachable("consumer has no producer!");
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Array, typename Graph, typename Map>
void addConsumerRelationships(const unsigned consumer, const Array & array, Graph & G, Map & M) {
    const auto n = array.size();
    for (unsigned i = 0; i < n; ++i) {
        const auto relationship = addIfConstantOrFindRelationship(getRelationship(array[i]), G, M);
        add_edge(relationship, consumer, i, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addImplicitConsumerRelationship
 ** ------------------------------------------------------------------------------------------------------------- */
template <typename Graph, typename Map>
void addImplicitConsumerRelationship(const unsigned codeId, const unsigned consumer, const ImplicitRelationships & R, Graph & G, Map & M) {
    const Kernel * const kernel = G[consumer].Kernel; assert (kernel);
    const auto f = R.find(kernel);
    if (LLVM_UNLIKELY(f != R.end())) {
        const auto relationship = addIfConstantOrFindRelationship(getRelationship(f->second), G, M);
        add_edge(relationship, consumer, codeId, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGraph PipelineCompiler::makePipelineGraph(BuilderRef b) const {

    // copy the list of internal kernels and add in any implicit kernels
    Kernels kernels(mPipelineKernel->getKernels());

    ImplicitRelationships regions;
    addRegionSelectorKernels(b, kernels, regions);

    const auto pipelineInput = 0U;
    const auto firstKernel = 1U;
    const auto lastKernel = kernels.size();
    const auto pipelineOutput = lastKernel + 1;

    const auto & call = mPipelineKernel->getCallBindings();
    const auto numOfCalls = call.size();
    const auto firstCall = pipelineOutput + 1;
    const auto lastCall = firstCall + numOfCalls;
    const auto initialSize = lastCall + 1;

    /// ------------------------------------------------------------------------------------------
    /// Construct the initial relationship graph G for this pipeline
    /// ------------------------------------------------------------------------------------------

    RelationshipGraph G(initialSize);

    G[pipelineInput].Kernel = mPipelineKernel;
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        G[i].Kernel = kernels[i - firstKernel];
    }
    G[pipelineOutput].Kernel = mPipelineKernel;

    RelationshipMap M;
    addProducerRelationships(pipelineInput, mPipelineKernel->getInputScalarBindings(), G, M);
    addProducerRelationships(pipelineInput, mPipelineKernel->getInputStreamSetBindings(), G, M);
    for (unsigned i = firstKernel; i <= lastKernel; ++i) {
        const Kernel * k = kernels[i - firstKernel];
        addConsumerRelationships(i, k->getInputStreamSetBindings(), G, M);
        addConsumerRelationships(i, k->getInputScalarBindings(), G, M);
        addImplicitConsumerRelationship(IMPLICIT_REGION_SELECTOR, i, regions, G, M);
        addProducerRelationships(i, k->getOutputStreamSetBindings(), G, M);
        addProducerRelationships(i, k->getOutputScalarBindings(), G, M);
    }
    for (unsigned i = 0; i < numOfCalls; ++i) {
        addConsumerRelationships(firstCall + i, call[i].Args, G, M);
    }
    addConsumerRelationships(pipelineOutput, mPipelineKernel->getOutputScalarBindings(), G, M);
    addConsumerRelationships(pipelineOutput, mPipelineKernel->getOutputStreamSetBindings(), G, M);

    /// ------------------------------------------------------------------------------------------
    /// Pipeline optimizations
    /// ------------------------------------------------------------------------------------------

    combineDuplicateKernels(G, kernels);
    removeUnusedKernels(G, kernels);

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

    /// ------------------------------------------------------------------------------------------
    /// Transcribe the pipeline graph based on the lexical ordering, accounting for any auxillary
    /// kernels and subsituted kernels/relationships.
    /// ------------------------------------------------------------------------------------------

    const auto n = ordering.size();
    PipelineGraph P(n + numOfCalls + 3);
    P.LastKernel = n;
    P.PipelineOutput = n + 1;
    P.FirstCall = numOfCalls ? (n + 2) : 0;
    P.LastCall = P.FirstCall + numOfCalls;
    P.FirstRelationship = n + numOfCalls + 3;

    SmallVector<unsigned, 64> remapping(initialSize);

    // record the new numbering of our active kernels
    remapping[pipelineInput] = P.PipelineInput;
    P[P.PipelineInput].Kernel = mPipelineKernel;
    for (unsigned i = 0; i < n; ++i) {
        const auto j = ordering[i];
        remapping[j] = firstKernel + i;
        P[firstKernel + i].Kernel = G[j].Kernel;
    }
    remapping[pipelineOutput] = P.PipelineOutput;
    P[P.PipelineOutput].Kernel = mPipelineKernel;

    // record the new numbering of our function calls
    for (unsigned i = 0; i < numOfCalls; ++i) {
        remapping[firstCall + i] = P.FirstCall + i;
    }
    const auto firstRelationship = initialSize;
    const auto lastRelationship = num_vertices(G);
    for (unsigned i = firstRelationship; i < lastRelationship; ++i) {
        if (LLVM_LIKELY(in_degree(i, G) != 0)) {
            const auto e = in_edge(i, G);
            const auto producer = remapping[source(e, G)];
            const auto relationship = add_vertex(P);
            P[relationship].Relationship = G[i].Relationship;
            add_edge(producer, relationship, G[e], P);
            for (const auto & e : make_iterator_range(out_edges(i, G))) {
                const auto consumer = remapping[target(e, G)];
                add_edge(relationship, consumer, G[e], P);
            }
        }
    }
    P.LastRelationship = num_vertices(P);
    return P;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineList
 ** ------------------------------------------------------------------------------------------------------------- */
Kernels PipelineCompiler::makePipelineList() const {
    // temporary refactoring step
    const auto n = mPipelineGraph.PipelineOutput;
    Kernels kernels(mPipelineGraph.PipelineOutput + 1);
    for (unsigned i = mPipelineGraph.PipelineInput; i <= n; ++i) {
        kernels[i] = const_cast<Kernel *>(mPipelineGraph[i].Kernel);
    }
    return kernels;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addRegionSelectorKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addRegionSelectorKernels(BuilderRef b, Kernels & kernels, ImplicitRelationships & regions) const {

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
                regions.emplace(kernel, regionSpans);
                alreadyCreated.emplace(cond, regionSpans);
            } else { // we've already created the correct region span
                regionSpans = f->second; assert (regionSpans);
            }
            regions.emplace(kernel, regionSpans);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief combineDuplicateKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::combineDuplicateKernels(RelationshipGraph & G, const Kernels & kernels) const {

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
                unsigned port = G[e];
                const auto relationship = source(e, G);
                if (isa<StreamSet>(G[relationship])) {
                    streams[port] = relationship;
                    ++numOfStreams;
                } else {
                    scalars[port] = relationship;
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
                        unsigned port = G[e];
                        const auto relationship = target(e, G);
                        if (isa<StreamSet>(G[relationship].Relationship)) {
                            streams[port] = relationship;
                            ++numOfStreams;
                        } else {
                            scalars[port] = relationship;
                        }
                    }
                    streams.resize(numOfStreams);
                    const auto numOfScalars = n - numOfStreams;
                    scalars.resize(numOfScalars);

                    // Replace the consumers of kernel i's outputs with j's.
                    for (const auto & e : make_iterator_range(out_edges(i, G))) {
                        unsigned port = G[e];
                        const auto original = target(e, G);
                        const Relationship * const a = G[original].Relationship;
                        unsigned replacement = 0;
                        if (isa<StreamSet>(a)) {
                            replacement = streams[port];
                        } else {
                            replacement = scalars[port];
                        }
                        const Relationship * const b = G[replacement].Relationship;
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
inline void PipelineCompiler::removeUnusedKernels(RelationshipGraph & G, const Kernels & kernels) const {

    const auto firstKernel = 1U;
    const auto lastKernel = kernels.size();
    const auto pipelineOutput = lastKernel + 1;

    const auto & call = mPipelineKernel->getCallBindings();
    const auto numOfCalls = call.size();
    const auto firstCall = pipelineOutput + 1;
    const auto lastCall = firstCall + numOfCalls;

    RelationshipGraph T(G);
    const auto sink = add_vertex(T);
    for (auto i = firstKernel; i <= lastKernel; ++i) {
        const Kernel * const kernel = G[i].Kernel;
        if (kernel->hasAttribute(AttrId::SideEffecting)) {
            add_edge(i, sink, 0, T);
        }
    }
    for (unsigned i = pipelineOutput; i <= lastCall; ++i) {
        add_edge(i, sink, 0, T);
    }
    transitive_closure_dag(T);
    dynamic_bitset<> active(lastKernel + 1, false);
    for (const auto & e : make_iterator_range(in_edges(sink, T))) {
        const auto k = source(e, G);
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
                clear_out_edges(target(e, G), G);
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

    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        const Kernel * const kernel = mPipeline[i];
        const auto numOfInputs = kernel->getNumOfStreamInputs();

        Graph G(numOfInputs + 1);

        // enumerate the input relations
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const Binding & input = kernel->getInputStreamSetBinding(i);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
                add_edge(i, numOfInputs, G);
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                    report_fatal_error(kernel->getName() + "." + input.getName() +
                                       " cannot have both ZeroExtend and Principal attributes");
                }
            }
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.hasReference())) {
                Port port; unsigned j;
                std::tie(port, j) = kernel->getStreamPort(rate.getReference());
                assert ("input stream cannot refer to an output stream" && port == Port::Input);
                add_edge(j, i, G);
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
 * @brief makeScalarDependencyGraph
 *
 * producer -> buffer/scalar -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
RelationshipGraph PipelineCompiler::makeScalarDependencyGraph() const {
    const auto & call = mPipelineKernel->getCallBindings();
    const auto numOfCalls = call.size();
    const auto firstCall = mPipelineOutput + 1;
    const auto initialSize = firstCall + numOfCalls;

    RelationshipGraph G(initialSize);
    RelationshipMap M;

    addProducerRelationships(mPipelineInput, mPipelineKernel->getInputScalarBindings(), G, M);
    // verify each scalar input of the kernel is an input to the pipeline
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        addConsumerRelationships(i, mPipeline[i]->getInputScalarBindings(), G, M);
    }
    // enumerate the output scalars
    for (unsigned i = mFirstKernel; i < mLastKernel; ++i) {
        addProducerRelationships(i, mPipeline[i]->getOutputScalarBindings(), G, M);
    }
    // enumerate the call bindings
    for (unsigned i = 0; i < numOfCalls; ++i) {
        addConsumerRelationships(firstCall + i, call[i].Args, G, M);
    }
    // enumerate the pipeline outputs
    addConsumerRelationships(mPipelineOutput, mPipelineKernel->getOutputScalarBindings(), G, M);

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief determineEvaluationOrderOfKernelIO
 *
 * Returns a lexographically sorted list of ports s.t. the inputs will be ordered as close as possible (baring
 * any constraints) to the kernel's original I/O ordering.
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::determineEvaluationOrderOfKernelIO() {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS>;

    const auto numOfInputs = mKernel->getNumOfStreamInputs();
    const auto numOfOutputs = mKernel->getNumOfStreamOutputs();
    const auto firstOutput = numOfInputs;
    const auto numOfPorts = numOfInputs + numOfOutputs;

    Graph G(numOfPorts);

    // enumerate the input relations
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.hasReference()) {
            Port port; unsigned j;
            std::tie(port, j) = mKernel->getStreamPort(rate.getReference());
            assert ("input stream cannot refer to an output stream" && port == Port::Input);
            add_edge(j, i, G);
        }
    }
    // and then enumerate the output relations
    for (unsigned i = 0; i < numOfOutputs; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.hasReference()) {
            Port port; unsigned j;
            std::tie(port, j) = mKernel->getStreamPort(rate.getReference());
            add_edge(j + ((port == Port::Output) ? numOfInputs : 0), (i + numOfInputs), G);
        }
    }

    // check any zeroextended inputs last
    SmallVector<unsigned, 16> zext(numOfInputs);
    for (unsigned i = 0; i < numOfInputs; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
            zext.push_back(i);
        }
    }
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
    if (out_degree(mPipelineInput, mBufferGraph)) {
        for (unsigned i = 0; i < numOfInputs; ++i) {
            const auto buffer = getInputBufferVertex(i);
            if (LLVM_UNLIKELY(is_parent(buffer, mPipelineInput, mBufferGraph))) {
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
    if (in_degree(mPipelineOutput, mBufferGraph)) {
        for (unsigned i = 0; i < numOfOutputs; ++i) {
            const auto buffer = getOutputBufferVertex(i);
            if (LLVM_UNLIKELY(has_child(buffer, mPipelineOutput, mBufferGraph))) {
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

namespace {

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief minimumConsumed
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue minimumConsumed(const Kernel * const kernel, const Binding & binding) {
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::Deferred))) {
        return RateValue{0};
    }
    return lowerBound(kernel, binding);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief maximumConsumed
 ** ------------------------------------------------------------------------------------------------------------- */
inline LLVM_READNONE RateValue maximumConsumed(const Kernel * const kernel, const Binding & binding) {
    auto ub = upperBound(kernel, binding);
    if (binding.hasLookahead()) {
        ub += binding.getLookahead();
    }
    return ub;
}

#endif

}




/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    const auto firstBuffer = mPipelineOutput + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    ConsumerGraph G(lastBuffer);

#if 0

    #warning TODO: ConsumerGraph assumes the dataflow is transitively bounded by the same initial source

    #warning REVISIT: ConsumerGraph is not optimal for handling relative rate inputs

    struct ConsumerData {
        unsigned Kernel{0};
        unsigned Port{0};
        RateValue Minimum{0};
        RateValue Maximum{0};

        inline bool operator < (const ConsumerData & other) const {
            return (Kernel < other.Kernel) || (Port < other.Port);
        }
    };

    std::vector<ConsumerData> consumers;
#endif

    for (auto bufferVertex = firstBuffer; bufferVertex < lastBuffer; ++bufferVertex) {
        // copy the producing edge
        const auto pe = in_edge(bufferVertex, mBufferGraph);
        add_edge(source(pe, mBufferGraph), bufferVertex, mBufferGraph[pe].Port, G);

        for (const auto ce : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            add_edge(bufferVertex, target(ce, mBufferGraph), mBufferGraph[ce].Port, G);
        }

#if 0

        // collect the consumers of the i-th buffer
        consumers.clear();
        for (const auto e : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            ConsumerData cd;
            cd.Kernel = target(e, mBufferGraph);
            const BufferNode & bn = mBufferGraph[cd.Kernel];
            const BufferRateData & rd = mBufferGraph[e];
            cd.Port = rd.Port;
            const Kernel * const kernel = mPipeline[cd.Kernel];
            const Binding & input = kernel->getInputStreamSetBinding(cd.Port);
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                cd.Minimum = RateValue{0};
            } else {
                cd.Minimum = bn.Lower * rd.Minimum;
            }
            cd.Maximum = bn.Upper * rd.Maximum;
            if (LLVM_UNLIKELY(input.hasLookahead())) {
                cd.Maximum += input.getLookahead();
            }
            consumers.emplace_back(cd);
        }

        // If the minimum input rate of the j-th consumer is greater than or equal to the maximum input
        // rate of the k-th consumer, we do not need to test the j-th consumer. This also ensures that
        // for each *FIXED* rate stream, keep only the minimum such rate. However, we may need to insert
        // a "fake" edge to mark the last consumer otherwise we'll set it too soon.

        // NOTE: here we need to consider the impact of lookahead on the use of a buffer since it may
        // limit how much work we can perform when nearing the end of the buffer.

        // TODO: this takes too narrow of a view of the problem. By considering a buffer's consumers
        // in isolation, it does not take into account that a particular kernel may be executed fewer
        // times than another because of I/O constraints independent of the buffer we're considering.
        // Essentially, to make this optimization safe we need to prove that if a consumer has performed
        // k strides, all other consumers performed k.

        if (LLVM_LIKELY(consumers.size() > 1)) {

            std::sort(consumers.begin(), consumers.end());

            const auto finalConsumer = consumers.back().first;

            for (auto j = consumers.begin() + 1; j != consumers.end(); ) {

                const ConsumerData & Cj = *j;
                for (auto k = consumers.begin(); k != j; ++k) {
                    const ConsumerData & Ck = *k;
                    if (LLVM_UNLIKELY(Cj.Minimum >= Ck.Maximum)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                for (auto k = j + 1; k != consumers.end(); ++k) {
                    const ConsumerData & Ck = *k;
                    if (LLVM_UNLIKELY(Cj.Minimum >= Ck.Maximum)) {
                        j = consumers.erase(j);
                        goto next;
                    }
                }

                ++j;
next:           continue;
            }
            if (LLVM_UNLIKELY(consumers.back().first != finalConsumer)) {
                consumers.emplace_back(finalConsumer, FAKE_CONSUMER);
            }
        }

        for (const auto & consumer : consumers) {
            add_edge(bufferVertex, consumer.first, consumer.second, G);
        }
#endif

    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePopCountGraph
 *
 * Kernel -> Port -> Buffer ordering. Edge between a Kernel and a port indicates the port # of the source
 * items stream. Edges between a port and buffer state the ref port #. The kernel and buffer vertices are
 * aligned with the BufferGraph. Any buffer vertex with an in-degree > 0 is the reference of a pop count
 * rate stream.
 ** ------------------------------------------------------------------------------------------------------------- */
PopCountGraph PipelineCompiler::makePopCountGraph() const {

    using Vertex = PopCountGraph::vertex_descriptor;
    using Map = flat_map<unsigned, Vertex>;

    PopCountGraph G(num_vertices(mBufferGraph));
    Map M;

    for (unsigned kernelVertex = mFirstKernel; kernelVertex < mLastKernel; ++kernelVertex) {
        const Kernel * const kernel = mPipeline[kernelVertex];
        const auto numOfInputs = kernel->getNumOfStreamInputs();
        const auto numOfOutputs = kernel->getNumOfStreamOutputs();

        auto insertPopCountDependency = [&](
                const CountingType type,
                const unsigned refPort,
                const unsigned srcPort) {
            // check if we've already created a vertex for the ref port ...
            const auto f = M.find(refPort);
            if (LLVM_UNLIKELY(f != M.end())) {
                const auto refPortVertex = f->second;
                add_edge(kernelVertex, refPortVertex, PopCountEdge{type, srcPort}, G);
                // update the ref -> buffer edge with the counting type
                G[out_edge(refPortVertex, G)].Type |= type;
            } else { // ... otherwise map a new vertex to it.

                // verify the reference stream is a Fixed rate stream
                if (LLVM_LIKELY(srcPort != refPort)) {
                    const Binding & refBinding = kernel->getInputStreamSetBinding(refPort);
                    if (LLVM_UNLIKELY(refBinding.isDeferred() || !refBinding.getRate().isFixed())) {
                        std::string tmp;
                        raw_string_ostream msg(tmp);
                        msg << kernel->getName();
                        msg << ": pop count reference ";
                        msg << refBinding.getName();
                        msg << " must be a non-deferred Fixed rate stream";
                        report_fatal_error(msg.str());
                    }
                }

                const auto refPortVertex = add_vertex(G);
                M.emplace(refPort, refPortVertex);
                add_edge(kernelVertex, refPortVertex, PopCountEdge{type, srcPort}, G);

                // determine which buffer this port refers to by inspecting the buffer graph
                const auto bufferVertex = getInputBufferVertex(kernelVertex, refPort);
                add_edge(refPortVertex, bufferVertex, PopCountEdge{type, refPort}, G);
            }
        };

        auto addPopCountDependency = [&](
            const unsigned portIndex,
            const Binding & binding) {
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
                // determine which port this I/O port refers to
                Port portType; unsigned refPort;
                std::tie(portType, refPort) = kernel->getStreamPort(rate.getReference());
                // verify the reference stream is an input port
                if (LLVM_UNLIKELY(portType != Port::Input)) {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << kernel->getName();
                    msg << ": pop count rate for ";
                    msg << binding.getName();
                    msg << " cannot refer to an output stream";
                    report_fatal_error(msg.str());
                }
                const auto type = rate.isPopCount() ? CountingType::Positive : CountingType::Negative;
                insertPopCountDependency(type, refPort, portIndex);
            }
        };

        auto checkRequiredArray = [&](
            const unsigned portIndex,
            const Binding & binding) {

            CountingType type = CountingType::Unknown;
            if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::RequiresPopCountArray))) {
                type = CountingType::Positive;
            }
            if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::RequiresNegatedPopCountArray))) {
                type |= CountingType::Negative;
            }
            if (LLVM_UNLIKELY(type != CountingType::Unknown)) {
                insertPopCountDependency(type, portIndex, portIndex);
            }
        };

        for (unsigned j = 0; j < numOfInputs; ++j) {
            const auto & input = kernel->getInputStreamSetBinding(j);
            addPopCountDependency(j, input);
            checkRequiredArray(j, input);
        }
        const auto firstOutput = numOfInputs;
        for (unsigned j = 0; j < numOfOutputs; ++j) {
            const auto & output = kernel->getOutputStreamSetBinding(j);
            addPopCountDependency(firstOutput + j, output);
        }
        M.clear();
    }

    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineIOGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineIOGraph PipelineCompiler::makePipelineIOGraph() const {

    PipelineIOGraph G((mPipelineOutput - mPipelineInput) + 1);
    for (const auto e : make_iterator_range(out_edges(mPipelineInput, mBufferGraph))) {
        const auto buffer = target(e, mBufferGraph);
        for (const auto e : make_iterator_range(out_edges(buffer, mBufferGraph))) {
            const auto consumer = target(e, mBufferGraph);
            add_edge(mPipelineInput, consumer, mBufferGraph[e].inputPort(), G);
        }
    }
    for (const auto e : make_iterator_range(in_edges(mPipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        for (const auto e : make_iterator_range(in_edges(buffer, mBufferGraph))) {
            const auto producer = source(e, mBufferGraph);
            add_edge(producer, mPipelineOutput, mBufferGraph[e].outputPort(), G);
        }
    }
    return G;
}


} // end of namespace
