#include "pipeline_compiler.hpp"
#include "lexographic_ordering.hpp"
#include <boost/graph/topological_sort.hpp>

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


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printRelationshipGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void printRelationshipGraph(const RelationshipGraph & G, raw_ostream & out, const StringRef name = "G") {


    auto write = [](const RateValue & v, llvm::raw_ostream & out)  {
        if (LLVM_LIKELY(v.denominator() == 1)) {
            out << v.numerator();
        } else {
            out << '(' << v.numerator() << '/' << v.denominator() << ')';
        }
    };

    out << "digraph " << name << " {\n";
    for (const auto v : make_iterator_range(vertices(G))) {
        out << "v" << v << " [label=\"" << v << ' ';
        const RelationshipNode & rn = G[v];
        switch (rn.Type) {
            case RelationshipNode::IsNil:
                out << "<nil>";
                break;
            case RelationshipNode::IsKernel:
                out << "K:";
                if (rn.Kernel) {
                    out << rn.Kernel->getName();
                }
                break;
            case RelationshipNode::IsBinding: {
                    const Binding & binding = rn.Binding;
                    out << "B:";
                    using KindId = ProcessingRate::KindId;
                    const ProcessingRate & rate = binding.getRate();
                    switch (rate.getKind()) {
                        case KindId::Fixed:
                            out << 'F';
                            write(rate.getLowerBound(), out);
                            break;
                        case KindId::Bounded:
                            out << 'B';
                            write(rate.getLowerBound(), out);
                            out << '-';
                            write(rate.getUpperBound(), out);
                            break;
                        case KindId::Unknown:
                            out << 'U';
                            write(rate.getLowerBound(), out);
                            break;
                        case KindId::PopCount:
                            out << 'P';
                            break;
                        case KindId::NegatedPopCount:
                            out << 'N';
                            break;
                        case KindId::Relative:
                            out << 'R';
                            break;
                        case KindId::PartialSum:
                            out << 'S';
                            break;
                    }
                    out << "  " << binding.getName();
                }
                break;
            case RelationshipNode::IsCallee:
                assert (&rn.Callee);
                out << "C:" << rn.Callee.get().Name;
                break;
            case RelationshipNode::IsRelationship:
                assert (rn.Relationship);
                out << "R:"; rn.Relationship->getType()->print(errs());
                break;
        }
        out << "\"];\n";
        out.flush();
    }



    for (const auto & e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t << " [label=\"";
        const RelationshipType & rt = G[e];
        switch (rt.Type) {
            case PortType::Input:
                out << 'I';
                break;
            case PortType::Output:
                out << 'O';
                break;
        }
        out << ':' << rt.Number;
        switch (rt.Reason) {
            case ReasonType::None:
                assert (!"reason cannot be None!");
            case ReasonType::Explicit:
                break;
            case ReasonType::ImplicitPopCount:
                out << " (popcount)";
                break;
            case ReasonType::ImplicitRegionSelector:
                out << " (region)";
                break;
            case ReasonType::Reference:
                out << " (ref)";
                break;
        }
        out << "\"];\n";
        out.flush();
    }
    out << "}\n\n";
    out.flush();
}


#endif


namespace { // start of anonymous namespace

using RefVector = SmallVector<Relationships::Vertex, 4>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addProducerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addProducerRelationships(const PortType portType, const unsigned producer, const Bindings & array, Relationships & G) {
    const auto n = array.size();
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    if (isa<StreamSet>(array[0].getRelationship())) {
        for (unsigned i = 0; i < n; ++i) {
            const Binding & item = array[i];
            assert (isa<StreamSet>(item.getRelationship()));
            const auto binding = G.add(&item);
            add_edge(producer, binding, RelationshipType{portType, i}, G);
            const auto relationship = G.addOrFind(item.getRelationship());
            add_edge(binding, relationship, RelationshipType{portType, i}, G);
        }
    } else if (isa<Scalar>(array[0].getRelationship())) {
        for (unsigned i = 0; i < n; ++i) {
            assert (isa<Scalar>(array[i].getRelationship()));
            const auto relationship = G.addOrFind(array[i].getRelationship());
            add_edge(producer, relationship, RelationshipType{portType, i}, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addConsumerRelationships(const PortType portType, const unsigned consumer, const Bindings & array, Relationships & G) {
    const auto n = array.size();
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    if (isa<StreamSet>(array[0].getRelationship())) {
        for (unsigned i = 0; i < n; ++i) {
            const Binding & item = array[i];
            assert (isa<StreamSet>(item.getRelationship()));
            const auto binding = G.add(&item);
            add_edge(binding, consumer, RelationshipType{portType, i}, G);
            const auto relationship = G.find(item.getRelationship());
            add_edge(relationship, binding, RelationshipType{portType, i}, G);
        }
    } else if (isa<Scalar>(array[0].getRelationship())) {
        for (unsigned i = 0; i < n; ++i) {
            assert (isa<Scalar>(array[i].getRelationship()));
            const auto relationship = G.addOrFind(array[i].getRelationship());
            add_edge(relationship, consumer, RelationshipType{portType, i}, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addConsumerRelationships(const PortType portType, const CallBinding & call, Relationships & G) {
    const auto & array = call.Args;
    const auto n = array.size();
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    const auto consumer = G.addOrFind(&call);
    for (unsigned i = 0; i < n; ++i) {
        const auto relationship = G.addOrFind(array[i]);
        add_edge(relationship, consumer, RelationshipType{portType, i}, G);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
inline void addReferenceRelationships(const PortType portType, const unsigned index, const Bindings & array, Relationships & G) {
    const auto n = array.size();
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    for (unsigned i = 0; i < n; ++i) {
        const Binding & item = array[i];
        const ProcessingRate & rate = item.getRate();
        if (LLVM_UNLIKELY(rate.hasReference())) {
            const Kernel * const kernel = G[index].Kernel;
            const StreamPort refPort = kernel->getStreamPort(rate.getReference());
            if (LLVM_UNLIKELY(portType == PortType::Input && refPort.Type == PortType::Output)) {
                std::string tmp;
                raw_string_ostream msg(tmp);
                msg << "input stream "
                    << kernel->getName()
                    << "."
                    << item.getName()
                    << " cannot refer to an output stream";
                report_fatal_error(msg.str());
            }
            const Binding & ref = kernel->getStreamBinding(refPort);
            assert (isa<StreamSet>(ref.getRelationship()));
            // To preserve acyclicity, reference bindings always point to the binding that refers to it.
            // To simplify later I/O lookup, the edge stores the info of the reference port.
            add_edge(G.find(&ref), G.find(&item), RelationshipType{refPort, ReasonType::Reference}, G);
        }
    }
}

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineGraphBundle PipelineCompiler::makePipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel) {

    const auto construction_error_msg = "error constructing internal pipeline graph";

    using Vertices = Vec<unsigned, 64>;

    OwningVector<Kernel> internalKernels;
    OwningVector<Binding> internalBindings;

    const auto G = generateInitialPipelineGraph(b, pipelineKernel, internalKernels, internalBindings);

    // Compute the lexographical ordering of H

    std::vector<unsigned> O;
    if (LLVM_UNLIKELY(!lexical_ordering(G, O))) {
        // TODO: inspect G to determine what type of cycle. E.g., do we have circular references in the binding of
        // a kernel or is it a problem with the I/O relationships?
        report_fatal_error("Pipeline contains a cycle");
    }

    // TODO: in u32u8, calls to the StreamExpand/FieldDeposit kernels could be "merged" if we had the ability to
    // "re-execute" pipeline code with a different input kernel & I/O state. However, we may not necessarily want
    // to just re-execute the same kernel and may instead want to do the full sequence before repeating.

    Vertices kernels;
    Vertices bindings;
    Vertices streamSets;
    Vertices callees;
    Vertices scalars;

    for (const auto i : O) {
        const RelationshipNode & rn = G[i];
        switch (rn.Type) {
            case RelationshipNode::IsKernel:
                assert (rn.Kernel);
                kernels.push_back(i);
                break;
            case RelationshipNode::IsRelationship:
                assert (rn.Relationship);
                if (isa<StreamSet>(rn.Relationship)) {
                    streamSets.push_back(i);
                } else {
                    scalars.push_back(i);
                }
                break;
            case RelationshipNode::IsCallee:
                assert (&rn.Callee);
                callees.push_back(i);
                break;
            case RelationshipNode::IsBinding:
                assert (&rn.Binding);
                bindings.push_back(i);
                break;
            default:
                break;
        }
    }

    // Transcribe the pipeline graph based on the lexical ordering, accounting for any auxillary
    // kernels and subsituted kernels/relationships.

    const auto numOfKernels = kernels.size();
    const auto numOfStreamSets = streamSets.size();
    const auto numOfBindings = bindings.size();
    const auto numOfCallees = callees.size();
    const auto numOfScalars = scalars.size();

    if (LLVM_UNLIKELY(G[kernels[0]].Kernel != pipelineKernel ||
                      G[kernels[numOfKernels - 1]].Kernel != pipelineKernel)) {
        report_fatal_error(construction_error_msg);
    }

    const auto n = numOfKernels + numOfBindings + numOfStreamSets;
    const auto m = numOfKernels + numOfCallees + numOfScalars;

    PipelineGraphBundle P(n, m, std::move(internalKernels), std::move(internalBindings));
    P.LastKernel = P.PipelineInput + numOfKernels - 2;
    P.PipelineOutput = P.LastKernel + 1;

    P.FirstStreamSet = P.PipelineOutput + 1;
    P.LastStreamSet = P.PipelineOutput + numOfStreamSets;
    P.FirstBinding = P.LastStreamSet + 1;
    P.LastBinding = P.LastStreamSet + numOfBindings;

    P.FirstCall = P.PipelineOutput + 1;
    P.LastCall = P.PipelineOutput + numOfCallees;
    P.FirstScalar = P.LastCall + 1;
    P.LastScalar = P.LastCall + numOfScalars;

    SmallVector<unsigned, 256> subsitution(num_vertices(G), -1U);

    for (unsigned i = 0; i < numOfKernels; ++i) {
        subsitution[kernels[i]] = i;
    }

    assert (subsitution[kernels[0]] == P.PipelineInput);
    assert (subsitution[kernels[numOfKernels - 1]] == P.PipelineOutput);

    for (unsigned i = 0; i < numOfStreamSets; ++i) {
        subsitution[streamSets[i]] = P.FirstStreamSet + i;
    }
    for (unsigned i = 0; i < numOfBindings; ++i) {
        subsitution[bindings[i]] = P.FirstBinding  + i;
    }
    for (unsigned i = 0; i < numOfCallees; ++i) {
        subsitution[callees[i]] = P.FirstCall + i;
    }
    for (unsigned i = 0; i < numOfScalars; ++i) {
        subsitution[scalars[i]] = P.FirstScalar + i;
    }

    SmallVector<std::pair<RelationshipType, unsigned>, 64> temp;

    auto transcribe = [&](const Vertices & V, RelationshipGraph & H) {
        for (const auto j : V) {
            assert (j < subsitution.size());
            const auto v = subsitution[j];
            assert (j < num_vertices(G));
            assert (v < num_vertices(H));
            H[v] = G[j];
        }
    };

    auto copy_in_edges = [&](const Vertices & V, RelationshipGraph & H,
            const RelationshipNode::RelationshipNodeType type) {
        for (const auto j : V) {
            const auto v = subsitution[j];
            for (const auto & e : make_iterator_range(in_edges(j, G))) {
                const auto i = source(e, G);
                if (G[i].Type == type) {
                    const auto u = subsitution[i];
                    if (LLVM_UNLIKELY(u >= num_vertices(H))) {
                        report_fatal_error(construction_error_msg);
                    }
                    temp.emplace_back(G[e], u);
                }
            }
            std::sort(temp.begin(), temp.end());
            for (const auto & e : temp) {
                add_edge(e.second, v, e.first, H);
            }
            temp.clear();
        }
    };

    auto copy_out_edges = [&](const Vertices & V, RelationshipGraph & H,
            const RelationshipNode::RelationshipNodeType type) {
        for (const auto j : V) {
            const auto v = subsitution[j];
            for (const auto & e : make_iterator_range(out_edges(j, G))) {
                const auto i = target(e, G);
                if (G[i].Type == type) {
                    const auto w = subsitution[i];
                    if (LLVM_UNLIKELY(w >= num_vertices(H))) {
                        report_fatal_error(construction_error_msg);
                    }
                    temp.emplace_back(G[e], w);
                }
            }
            std::sort(temp.begin(), temp.end());
            for (const auto & e : temp) {
                add_edge(v, e.second, e.first, H);
            }
            temp.clear();
        }
    };

    // create the stream graph
    transcribe(kernels, P.Streams);
    copy_in_edges(kernels, P.Streams, RelationshipNode::IsBinding);
    copy_out_edges(kernels, P.Streams, RelationshipNode::IsBinding);

    transcribe(streamSets, P.Streams);
    copy_in_edges(streamSets, P.Streams, RelationshipNode::IsBinding);
    copy_out_edges(streamSets, P.Streams, RelationshipNode::IsBinding);

    transcribe(bindings, P.Streams);
    copy_out_edges(bindings, P.Streams, RelationshipNode::IsBinding);

    // create the scalar graph
    transcribe(kernels, P.Scalars);
    copy_in_edges(kernels, P.Scalars, RelationshipNode::IsRelationship);
    copy_out_edges(kernels, P.Scalars, RelationshipNode::IsRelationship);

    transcribe(callees, P.Scalars);
    copy_in_edges(callees, P.Scalars, RelationshipNode::IsRelationship);
    copy_out_edges(callees, P.Scalars, RelationshipNode::IsRelationship);

    transcribe(scalars, P.Scalars);

    // printRelationshipGraph(P.Streams, errs(), "Streams");
    // printRelationshipGraph(P.Scalars, errs(), "Scalars");

    return P;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitialPipelineGraph
 ** ------------------------------------------------------------------------------------------------------------- */
Relationships PipelineCompiler::generateInitialPipelineGraph(BuilderRef b, PipelineKernel * const pipelineKernel,
                                                             OwningVector<Kernel> & internalKernels,
                                                             OwningVector<Binding> & internalBindings) {


    // Copy the list of kernels and add in any internal kernels
    Relationships G;
    Kernels kernels(pipelineKernel->getKernels());

    const auto p_in = add_vertex(RelationshipNode(pipelineKernel), G);
    addProducerRelationships(PortType::Input, p_in, pipelineKernel->getInputStreamSetBindings(), G);
    const auto n = kernels.size();
    SmallVector<Relationships::Vertex, 64> vertex(n);
    for (unsigned i = 0; i < n; ++i) {
        const Kernel * K = kernels[i];
        if (LLVM_UNLIKELY(K == pipelineKernel)) {
            std::string tmp;
            raw_string_ostream msg(tmp);
            msg << pipelineKernel->getName()
                << " contains itself in its pipeline";
            report_fatal_error(msg.str());
        }
        vertex[i] = G.add(K);
        addProducerRelationships(PortType::Output, vertex[i], K->getOutputStreamSetBindings(), G);
    }
    for (unsigned i = 0; i < n; ++i) {
        addConsumerRelationships(PortType::Input, vertex[i], kernels[i]->getInputStreamSetBindings(), G);
    }
    for (unsigned i = 0; i < n; ++i) {
        addReferenceRelationships(PortType::Input, vertex[i], kernels[i]->getInputStreamSetBindings(), G);
        addReferenceRelationships(PortType::Output, vertex[i], kernels[i]->getOutputStreamSetBindings(), G);
    }

    // addRegionSelectorKernels(b, kernels, G, internalKernels, internalBindings);
    addPopCountKernels(b, kernels, G, internalKernels, internalBindings);
    const auto p_out = add_vertex(RelationshipNode(pipelineKernel), G);
    addConsumerRelationships(PortType::Output, p_out, pipelineKernel->getOutputStreamSetBindings(), G);

    addProducerRelationships(PortType::Input, p_in, pipelineKernel->getInputScalarBindings(), G);
    for (unsigned i = 0; i < n; ++i) {
        addProducerRelationships(PortType::Output, vertex[i], kernels[i]->getOutputScalarBindings(), G);
    }
    for (unsigned i = 0; i < n; ++i) {
        addConsumerRelationships(PortType::Input, vertex[i], kernels[i]->getInputScalarBindings(), G);
    }
    for (const CallBinding & C : pipelineKernel->getCallBindings()) {
        addConsumerRelationships(PortType::Input, C, G);
    }
    addConsumerRelationships(PortType::Output, p_out, pipelineKernel->getOutputScalarBindings(), G);

    // Pipeline optimizations
    // combineDuplicateKernels(b, kernels, G);
    removeUnusedKernels(pipelineKernel, p_in, p_out, kernels, G);
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addRegionSelectorKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addRegionSelectorKernels(BuilderRef b, Kernels & kernels, Relationships & G,
                                                OwningVector<Kernel> & internalKernels, OwningVector<Binding> & internalBindings) {

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
                kernels.push_back(selector);
                internalKernels.emplace_back(selector);
                // Mark the region selectors for this kernel
                alreadyCreated.emplace(cond, regionSpans);
            } else { // we've already created the correct region span
                regionSpans = f->second; assert (regionSpans);
            }
            // insert the implicit relationships
            const auto K = G.addOrFind(kernel);
            Binding * const binding = new Binding("#regionselector", regionSpans);
            internalBindings.emplace_back(binding);
            const auto B = G.addOrFind(binding);
            add_edge(B, K, RelationshipType{PortType::Input, -1U, ReasonType::ImplicitRegionSelector}, G);
            const auto R = G.addOrFind(regionSpans);
            add_edge(R, B, RelationshipType{PortType::Input, -1U, ReasonType::ImplicitRegionSelector}, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPopCountKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addPopCountKernels(BuilderRef b, Kernels & kernels, Relationships & G,
                                          OwningVector<Kernel> & internalKernels, OwningVector<Binding> & internalBindings) {

    struct Edge {
        CountingType Type;
        StreamPort   Port;
        Edge() : Type(Unknown), Port() { }
        Edge(const CountingType type, const StreamPort port) : Type(type), Port(port) { }
    };

    using Graph = adjacency_list<vecS, vecS, directedS, Relationship *, Edge>;
    using Vertex = Graph::vertex_descriptor;
    using Map = flat_map<Relationship *, Vertex>;

    const auto numOfKernels = kernels.size();

    Graph H(numOfKernels);
    Map M;

    for (unsigned i = 0; i < numOfKernels; ++i) {

        const Kernel * const kernel = kernels[i];

        auto addPopCountDependency = [&](const Relationships::vertex_descriptor v,
                                         const RelationshipType & port) {

            const RelationshipNode & rn = G[v];
            assert (rn.Type == RelationshipNode::IsBinding);
            const Binding & binding = rn.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
                // determine which port this I/O port refers to
                for (const auto & e : make_iterator_range(in_edges(v, G))) {
                    const RelationshipType & rt = G[e];
                    if (rt.Reason == ReasonType::Reference) {
                        const auto ref = source(e, G);
                        const RelationshipNode & rn = G[ref];
                        assert (rn.Type == RelationshipNode::IsBinding);
                        const Binding & refBinding = rn.Binding;


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
                                msg << " must refer to a non-deferred Fixed rate stream";
                                report_fatal_error(msg.str());
                            }
                            refVertex = add_vertex(refStream, H);
                        }
                        const auto type = rate.isPopCount() ? CountingType::Positive : CountingType::Negative;
                        add_edge(refVertex, i, Edge{type, port}, H);
                        return;
                    }
                }
                llvm_unreachable("could not find reference for popcount rate?");
            }
        };

        const auto j = G.find(kernel);

        for (const auto & e : make_iterator_range(in_edges(j, G))) {
            addPopCountDependency(source(e, G), G[e]);
        }
        for (const auto & e : make_iterator_range(out_edges(j, G))) {
            addPopCountDependency(target(e, G), G[e]);
        }
    }

    const auto n = num_vertices(H);
    if (LLVM_LIKELY(n == numOfKernels)) {
        return;
    }

    BaseDriver & driver = b->getDriver();

    IntegerType * const sizeTy = b->getSizeTy();

    kernels.resize(n, nullptr);

    for (auto i = numOfKernels; i < n; ++i) {

        CountingType type = CountingType::Unknown;
        for (const auto & e : make_iterator_range(out_edges(i, H))) {
            const Edge & ed = H[e];
            type |= ed.Type;
        }
        assert (type != CountingType::Unknown);

        StreamSet * positive = nullptr;
        if (LLVM_LIKELY(type & CountingType::Positive)) {
            positive = driver.CreateStreamSet(1, sizeTy->getBitWidth());
        }

        StreamSet * negative = nullptr;
        if (LLVM_UNLIKELY(type & CountingType::Negative)) {
            negative = driver.CreateStreamSet(1, sizeTy->getBitWidth());
        }

        StreamSet * const input = cast<StreamSet>(H[i]); assert (input);

        PopCountKernel * popCountKernel = nullptr;
        switch (type) {
            case CountingType::Positive:
                popCountKernel = new PopCountKernel(b, PopCountKernel::POSITIVE, input, positive);
                break;
            case CountingType::Negative:
                popCountKernel = new PopCountKernel(b, PopCountKernel::NEGATIVE, input, negative);
                break;
            case CountingType::Both:
                popCountKernel = new PopCountKernel(b, PopCountKernel::BOTH, input, positive, negative);
                break;
            default: llvm_unreachable("unknown counting type?");
        }
        // Add the popcount kernel to the pipeline
        assert (i < kernels.size());
        kernels[i] = popCountKernel;
        internalKernels.emplace_back(popCountKernel);

        const auto k = G.add(popCountKernel);
        addConsumerRelationships(PortType::Input, k, popCountKernel->getInputStreamSetBindings(), G);
        addProducerRelationships(PortType::Output, k, popCountKernel->getOutputStreamSetBindings(), G);

        // subsitute the popcount relationships
        for (const auto & e : make_iterator_range(out_edges(i, H))) {
            const Edge & ed = H[e];
            const Kernel * const kernel = kernels[target(e, H)];
            const auto consumer = G.find(kernel);
            assert (ed.Type == CountingType::Positive || ed.Type == CountingType::Negative);
            StreamSet * const stream = ed.Type == CountingType::Positive ? positive : negative; assert (stream);
            const auto streamVertex = G.find(stream);

            // append the popcount rate stream to the kernel
            Binding * const popCount = new Binding("#popcount" + std::to_string(ed.Port.Number), stream, FixedRate({1, kernel->getStride()}));
            internalBindings.emplace_back(popCount);
            const auto popCountBinding = G.add(popCount);

            const unsigned portNum = in_degree(consumer, G);
            add_edge(streamVertex, popCountBinding, RelationshipType{PortType::Input, portNum, ReasonType::ImplicitPopCount}, G);
            add_edge(popCountBinding, consumer, RelationshipType{PortType::Input, portNum, ReasonType::ImplicitPopCount}, G);


            auto rebind_reference = [&](const unsigned binding) {

                RelationshipNode & rn = G[binding];
                assert (rn.Type == RelationshipNode::IsBinding);

                graph_traits<Relationships>::in_edge_iterator ei, ei_end;
                std::tie(ei, ei_end) = in_edges(binding, G);
                assert (std::distance(ei, ei_end) == 2);

                for (;;) {
                    const RelationshipType & type = G[*ei];
                    if (type.Reason == ReasonType::Reference) {
                        remove_edge(*ei, G);
                        break;
                    }
                    ++ei;
                    assert (ei != ei_end);
                }

                // create a new binding with the partial sum rate.
                const Binding & orig = rn.Binding;
                assert (orig.getRate().isPopCount() || orig.getRate().isNegatedPopCount());
                Binding * const replacement = new Binding(orig, PartialSum(popCount->getName()));
                internalBindings.emplace_back(replacement);
                rn.Binding = replacement;

                add_edge(popCountBinding, binding, RelationshipType{PortType::Input, portNum, ReasonType::Reference}, G);

            };

            bool notFound = true;
            if (ed.Port.Type == PortType::Input) {
                for (const auto & e : make_iterator_range(in_edges(consumer, G))) {
                    const RelationshipType & type = G[e];
                    if (type.Number == ed.Port.Number) {
                        assert (type.Type == PortType::Input);
                        rebind_reference(source(e, G));
                        notFound = false;
                        break;
                    }
                }
            } else { // if (ed.Port.Type == PortType::Output) {
                for (const auto & e : make_iterator_range(out_edges(consumer, G))) {
                    const RelationshipType & type = G[e];
                    if (type.Number == ed.Port.Number) {
                        assert (type.Type == PortType::Output);
                        rebind_reference(target(e, G));
                        notFound = false;
                        break;
                    }
                }
            }
            if (LLVM_UNLIKELY(notFound)) {
                report_fatal_error("Internal error: failed to locate PopCount binding.");
            }
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief combineDuplicateKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::combineDuplicateKernels(BuilderRef b, const Kernels & kernels, Relationships & G) {

    using StreamSetVector = std::vector<std::pair<unsigned, StreamPort>>;
    using ScalarVector = std::vector<unsigned>;

    struct KernelId {
        const std::string Id;
        const StreamSetVector Streams;
        const ScalarVector Scalars;

        KernelId(const std::string && id, const StreamSetVector & streams, const ScalarVector & scalars)
        : Id(id), Streams(streams), Scalars(scalars) {

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

    std::vector<unsigned> kernelList;
    for (const Kernel * kernel : kernels) {
        kernelList.push_back(G.addOrFind(kernel));
    }

    std::map<KernelId, unsigned> Ids;

    ScalarVector scalars;
    StreamSetVector inputs;
    ScalarVector outputs;

    for (;;) {
        bool unmodified = true;
        Ids.clear();

        for (const auto i : kernelList) {

            RelationshipNode & bn = G[i];
            if (bn.Type == RelationshipNode::IsKernel) {
                const Kernel * const kernel = bn.Kernel;
                // We cannot reason about a family of kernels nor safely combine two
                // side-effecting kernels.
                if (kernel->hasFamilyName() || kernel->hasAttribute(AttrId::SideEffecting)) {
                    continue;
                }

                const auto n = in_degree(i, G);
                inputs.resize(n);
                scalars.resize(n);
                unsigned numOfStreams = 0;

                for (const auto & e : make_iterator_range(in_edges(i, G))) {
                    const RelationshipType & port = G[e];
                    const auto input = source(e, G);
                    const RelationshipNode & node = G[input];
                    if (node.Type == RelationshipNode::IsBinding) {
                        unsigned relationship = 0;
                        StreamPort ref{};
                        for (const auto & e : make_iterator_range(in_edges(input, G))) {
                            RelationshipType & rt = G[e];
                            if (rt.Reason == ReasonType::Reference) {
                                ref = rt;
                                assert (G[source(e, G)].Type == RelationshipNode::IsBinding);
                            } else {
                                relationship = source(e, G);
                                assert (G[relationship].Type == RelationshipNode::IsRelationship);
                                assert (isa<StreamSet>(G[relationship].Relationship));
                            }
                        }
                        inputs[port.Number] = std::make_pair(relationship, ref);
                        ++numOfStreams;
                    } else if (node.Type == RelationshipNode::IsRelationship) {
                        assert (isa<Scalar>(G[input].Relationship));
                        scalars[port.Number] = input;
                    }
                }

                inputs.resize(numOfStreams);
                scalars.resize(n - numOfStreams);

                KernelId id(kernel->makeSignature(b), inputs, scalars);

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
                        outputs.resize(m);
                        scalars.resize(m);
                        unsigned numOfStreams = 0;
                        for (const auto & e : make_iterator_range(out_edges(j, G))) {

                            const RelationshipType & port = G[e];
                            const auto output = target(e, G);
                            const RelationshipNode & node = G[output];
                            if (node.Type == RelationshipNode::IsBinding) {
                                const auto relationship = child(output, G);
                                assert (G[relationship].Type == RelationshipNode::IsRelationship);
                                assert (isa<StreamSet>(G[relationship].Relationship));
                                outputs[port.Number] = relationship;
                                ++numOfStreams;
                            } else if (node.Type == RelationshipNode::IsRelationship) {
                                assert (isa<Scalar>(G[output].Relationship));
                                scalars[port.Number] = output;
                            }
                        }
                        outputs.resize(numOfStreams);
                        scalars.resize(m - numOfStreams);

                        // Replace the consumers of kernel i's outputs with j's.
                        for (const auto & e : make_iterator_range(out_edges(i, G))) {
                            const StreamPort & port = G[e];
                            const auto output = target(e, G);
                            const RelationshipNode & node = G[output];
                            unsigned original = 0;
                            if (node.Type == RelationshipNode::IsBinding) {
                                const auto relationship = child(output, G);
                                assert (G[relationship].Type == RelationshipNode::IsRelationship);
                                assert (isa<StreamSet>(G[relationship].Relationship));
                                original = relationship;
                            } else if (node.Type == RelationshipNode::IsRelationship) {
                                assert (isa<Scalar>(G[output].Relationship));
                                original = output;
                            }
                            assert (G[original].Type == RelationshipNode::IsRelationship);

                            unsigned replacement = 0;
                            if (node.Type == RelationshipNode::IsBinding) {
                                assert (port.Number < outputs.size());
                                replacement = outputs[port.Number];
                            } else {
                                assert (port.Number < scalars.size());
                                replacement = scalars[port.Number];
                            }
                            assert (G[replacement].Type == RelationshipNode::IsRelationship);

                            Relationship * const a = G[original].Relationship;
                            Relationship * const b = G[replacement].Relationship;
                            if (LLVM_UNLIKELY(a->getType() != b->getType())) {
                                error = true;
                                break;
                            }

                            for (const auto & e : make_iterator_range(out_edges(original, G))) {
                                add_edge(replacement, target(e, G), G[e], G);
                            }
                            clear_out_edges(original, G);
                        }
                        clear_vertex(i, G);
                        RelationshipNode & rn = G[i];
                        rn.Type = RelationshipNode::IsNil;
                        rn.Kernel = nullptr;
                        unmodified = false;
                    }
                    if (LLVM_UNLIKELY(error)) {
                        report_fatal_error(kernel->getName() + " is ambiguous: multiple I/O layouts have the same signature");
                    }
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
inline void PipelineCompiler::removeUnusedKernels(const PipelineKernel * pipelineKernel,
                                                  const unsigned p_in, const unsigned p_out,
                                                  const Kernels & kernels, Relationships & G) {

    flat_set<unsigned> visited;
    std::queue<unsigned> pending;
    pending.push(p_out);
    assert (p_in < p_out);
    visited.insert_unique(p_in);
    visited.insert_unique(p_out);

    // identify all nodes that must be in the final pipeline
    for (const Binding & output : pipelineKernel->getOutputScalarBindings()) {
        const auto p = G.find(output.getRelationship());
        pending.push(p);
        visited.insert_unique(p);
    }
    const auto & calls = pipelineKernel->getCallBindings();
    for (const CallBinding & C : calls) {
        const auto c = G.find(&C);
        pending.push(c);
        visited.insert_unique(c);
    }
    for (const Kernel * kernel : kernels) {
        if (LLVM_UNLIKELY(kernel->hasAttribute(AttrId::SideEffecting))) {
            const auto k = G.find(kernel);
            pending.push(k);
            visited.insert_unique(k);
        }
    }

    // determine the inputs for each of the required nodes
    for (;;) {
        const auto v = pending.front(); pending.pop();
        for (const auto & e : make_iterator_range(in_edges(v, G))) {
            const auto input = source(e, G);
            if (visited.insert(input).second) {
                pending.push(input);
            }
        }
        if (pending.empty()) {
            break;
        }
    }

    // To cut any non-required kernel from G, we cannot simply
    // remove every unvisited node as we still need to keep the
    // unused outputs of a kernel in G. Instead we queue all
    // unused kernels and sweep through the graph to eliminate
    // their immediate descendents.
    for (const auto v : make_iterator_range(vertices(G))) {
        const RelationshipNode & rn = G[v];
        if (rn.Type == RelationshipNode::IsKernel) {
            if (LLVM_UNLIKELY(visited.count(v) == 0)) {
                pending.push(v);
            }
        }
    }

    if (LLVM_LIKELY(pending.empty())) {
        return;
    }

    for (;;) {
        const auto v = pending.front(); pending.pop();
        RelationshipNode & rn = G[v];
        assert (rn.Type != RelationshipNode::IsNil);
        assert (rn.Type != RelationshipNode::IsCallee);
        assert (visited.count(v) == 0);
        for (const auto & e : make_iterator_range(out_edges(v, G))) {
            pending.push(target(e, G));
        }
        clear_vertex(v, G);
        rn.Type = RelationshipNode::IsNil;
        rn.Kernel = nullptr;
        if (pending.empty()) {
            break;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReferenceVertex
 ** ------------------------------------------------------------------------------------------------------------- */
const StreamPort PipelineCompiler::getReference(const StreamPort port) const {
    using InEdgeIterator = graph_traits<RelationshipGraph>::in_edge_iterator;
    using OutEdgeIterator = graph_traits<RelationshipGraph>::out_edge_iterator;


    RelationshipGraph::vertex_descriptor binding = 0;
    if (port.Type == PortType::Input) {
        InEdgeIterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(mKernelIndex, mStreamGraph);
        assert (port.Number < std::distance(ei, ei_end));
        const auto e = *(ei + port.Number);
        assert (mStreamGraph[e].Number == port.Number);
        binding = source(e, mStreamGraph);
    } else { // if (port.Type == PortType::Output) {
        OutEdgeIterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(mKernelIndex, mStreamGraph);
        assert (port.Number < std::distance(ei, ei_end));
        const auto e = *(ei + port.Number);
        assert (mStreamGraph[e].Number == port.Number);
        binding = target(e, mStreamGraph);
    }
    assert (mStreamGraph[binding].Type == RelationshipNode::IsBinding);
    assert (in_degree(binding, mStreamGraph) == 2);

    InEdgeIterator ei, ei_end;
    std::tie(ei, ei_end) = in_edges(binding, mStreamGraph);
    assert (std::distance(ei, ei_end) == 2);
    const auto e = *(ei + 1);
    const RelationshipType & rt = mStreamGraph[e];
    assert (rt.Reason == ReasonType::Reference);
    return rt;
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
        const RelationshipNode & rn = mStreamGraph[i];
        assert (rn.Type == RelationshipNode::IsKernel);
        const Kernel * const kernel = rn.Kernel; assert (kernel);
        const auto numOfInputs = in_degree(i, mStreamGraph);

        Graph G(numOfInputs + 1);

        // enumerate the input relations
        for (const auto & e : make_iterator_range(in_edges(i, mStreamGraph))) {
            const auto k = source(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[k];
            assert (rn.Type == RelationshipNode::IsBinding);
            const Binding & input = rn.Binding;
            const RelationshipType & port = mStreamGraph[e];

            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::ZeroExtended))) {
                add_edge(port.Number, numOfInputs, G);
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Principal))) {
                    report_fatal_error(kernel->getName() + "." + input.getName() +
                                       " cannot have both ZeroExtend and Principal attributes");
                }
            }
            if (LLVM_UNLIKELY(in_degree(k, mStreamGraph) != 1)) {
                graph_traits<RelationshipGraph>::in_edge_iterator ei, ei_end;
                std::tie(ei, ei_end) = in_edges(k, mStreamGraph);
                assert (std::distance(ei, ei_end) == 2);
                const auto f = *(ei + 1);
                const RelationshipType & ref = mStreamGraph[f];
                assert (ref.Reason == ReasonType::Reference);
                add_edge(ref.Number, port.Number, G);
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
            const StreamPort ref = getReference(bd.Port);
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
            StreamPort ref = getReference(bd.Port);
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
    if (LLVM_UNLIKELY(!lexical_ordering(G, mPortEvaluationOrder))) {
        report_fatal_error(mKernel->getName() + " has cyclic port dependencies.");
    }
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
