#ifndef PIPELINE_ANALYSIS_HPP
#define PIPELINE_ANALYSIS_HPP

#include "pipeline_analysis.hpp"
#include "lexographic_ordering.hpp"
#include "../internal/popcount_kernel.h"
#include "../internal/regionselectionkernel.h"
#include <boost/graph/topological_sort.hpp>
#include <llvm/Support/ErrorHandling.h>

namespace kernel {

// TODO: support call bindings that produce output that are inputs of
// other call bindings or become scalar outputs of the pipeline

// TODO: with a better model of stride rates, we could determine whether
// being unable to execute a kernel implies we won't be able to execute
// another and "skip" over the unnecessary kernels.

namespace { // start of anonymous namespace

using RefVector = SmallVector<ProgramGraph::Vertex, 4>;

#warning change enum tag to distinguish relationships and streamsets

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addProducerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addProducerRelationships(const PortType portType, const unsigned producer, const Bindings & array, ProgramGraph & G) {
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
void addConsumerRelationships(const PortType portType, const unsigned consumer, const Bindings & array, ProgramGraph & G, const bool addRelationship) {
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
            const auto rel = item.getRelationship();
            const auto relationship = G.addOrFind(rel, addRelationship);
            add_edge(relationship, binding, RelationshipType{portType, i}, G);
        }
    } else if (isa<Scalar>(array[0].getRelationship())) {
        for (unsigned i = 0; i < n; ++i) {
            assert (isa<Scalar>(array[i].getRelationship()));
            const auto rel = array[i].getRelationship();
            const auto relationship = G.addOrFind(rel, addRelationship);
            add_edge(relationship, consumer, RelationshipType{portType, i}, G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addConsumerRelationships(const PortType portType, const CallBinding & call, ProgramGraph & G) {
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
 * @brief getReferencePort
 ** ------------------------------------------------------------------------------------------------------------- */
inline StreamSetPort getReferencePort(const Kernel * const kernel, const StringRef ref) {
    const Bindings & inputs = kernel->getInputStreamSetBindings();
    const auto n = inputs.size();
    for (unsigned i = 0; i != n; ++i) {
        if (ref.equals(inputs[i].getName())) {
            return StreamSetPort{PortType::Input, i};
        }
    }
    const Bindings & outputs = kernel->getInputStreamSetBindings();
    const auto m = outputs.size();
    for (unsigned i = 0; i != m; ++i) {
        if (ref.equals(outputs[i].getName())) {
            return StreamSetPort{PortType::Output, i};
        }
    }
    SmallVector<char, 256> tmp;
    raw_svector_ostream msg(tmp);
    msg << "Invalid reference name: "
        << kernel->getName()
        << " does not contain a StreamSet called "
        << ref;
    report_fatal_error(msg.str());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getReferenceBinding
 ** ------------------------------------------------------------------------------------------------------------- */
inline const Binding & getReferenceBinding(const Kernel * const kernel, const StreamSetPort port) {
    if (port.Type == PortType::Input) {
        return kernel->getInputStreamSetBinding(port.Number);
    } else if (port.Type == PortType::Output) {
        return kernel->getOutputStreamSetBinding(port.Number);
    }
    llvm_unreachable("unknown port type?");
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addReferenceRelationships
 ** ------------------------------------------------------------------------------------------------------------- */
void addReferenceRelationships(const PortType portType, const unsigned index, const Bindings & array, ProgramGraph & G) {
    const auto n = array.size();
    if (LLVM_UNLIKELY(n == 0)) {
        return;
    }
    for (unsigned i = 0; i != n; ++i) {
        const Binding & item = array[i];
        const ProcessingRate & rate = item.getRate();
        if (LLVM_UNLIKELY(rate.hasReference())) {
            const Kernel * const kernel = G[index].Kernel;
            const StreamSetPort refPort = getReferencePort(kernel, rate.getReference());
            if (LLVM_UNLIKELY(portType == PortType::Input && refPort.Type == PortType::Output)) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << "Reference of input stream "
                    << kernel->getName()
                    << "."
                    << item.getName()
                    << " cannot refer to an output stream";
                report_fatal_error(msg.str());
            }
            const Binding & ref = getReferenceBinding(kernel, refPort);
            assert (isa<StreamSet>(ref.getRelationship()));
            if (LLVM_UNLIKELY(rate.isRelative() && ref.getRate().isFixed())) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << "Reference of a Relative-rate stream "
                    << kernel->getName()
                    << "."
                    << item.getName()
                    << " cannot refer to a Fixed-rate stream";
                report_fatal_error(msg.str());
            }
            // To preserve acyclicity, reference bindings always point to the binding that refers to it.
            // To simplify later I/O lookup, the edge stores the info of the reference port.
            add_edge(G.find(&ref), G.find(&item), RelationshipType{refPort, ReasonType::Reference}, G);
        }
    }
}

} // end of anonymous namespace

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief transcribeRelationshipGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::transcribeRelationshipGraph(const PartitionGraph & partitionGraph) {

    using Vertices = Vec<unsigned, 64>;

    // Compute the lexographical ordering of G
    std::vector<unsigned> O;
    if (LLVM_UNLIKELY(!lexical_ordering(Relationships, O))) {
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
        const RelationshipNode & rn = Relationships[i];
        switch (rn.Type) {
            case RelationshipNode::IsKernel:
                assert (rn.Kernel);
                kernels.push_back(i);
                break;
            case RelationshipNode::IsRelationship:
                assert (rn.Relationship);
                if (isa<StreamSet>(rn.Relationship)) {
                    streamSets.push_back(i);
                } else { assert (isa<Scalar>(rn.Relationship));
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

    const unsigned numOfKernels = kernels.size();
    const unsigned numOfStreamSets = streamSets.size();
    const unsigned numOfBindings = bindings.size();
    const unsigned numOfCallees = callees.size();
    const unsigned numOfScalars = scalars.size();

    SmallVector<unsigned, 256> subsitution(num_vertices(Relationships), -1U);

    LastKernel = PipelineInput + numOfKernels - 2;

    // Now fill in all of the remaining kernels subsitute position
    KernelPartitionId.resize(numOfKernels);
    ExpectedNumOfStrides.resize(numOfKernels);

    KernelPartitionId[PipelineInput] = 0;

    auto currentOrigPartitionId = -1U;
    auto outputPartitionId = -1U;

    assert (kernels[0] == PipelineInput);
    assert (kernels[numOfKernels - 1] == PipelineOutput);

    const auto origPartitionCount = num_vertices(partitionGraph);

    auto calculateExpectedNumOfStrides = [&](const unsigned kernelId, const unsigned partitionId) {
        assert (partitionId < origPartitionCount);
        const PartitionData & P = partitionGraph[partitionId];
        const auto & R = P.Repetitions;
        if (R.empty()) {
            return Rational{0};
        }
        const Partition & K = P.Kernels;
        assert (P.Repetitions.size() == K.size());
        const auto k = std::find(K.begin(), K.end(), kernelId);
        assert (k != K.end());
        const auto j = std::distance(K.begin(), k);
        return R[j] * P.ExpectedRepetitions;
    };

    SmallVector<unsigned, 64> remappedPartitionId(origPartitionCount);
    remappedPartitionId[0] = 0;

    for (unsigned i = 0; i < (numOfKernels - 1); ++i) {
        const auto in = kernels[i];
        assert (subsitution[in] == -1U);
        const auto out = PipelineInput + i;
        subsitution[in] = out;

        const auto f = PartitionIds.find(in);
        assert (f != PartitionIds.end());
        const auto origPartitionId = f->second;

        ExpectedNumOfStrides[i] = calculateExpectedNumOfStrides(in, origPartitionId);

        // renumber the partitions to reflect the selected ordering
        #ifndef FORCE_EACH_KERNEL_INTO_UNIQUE_PARTITION
        if (origPartitionId != currentOrigPartitionId) {
        #endif
            ++outputPartitionId;
            currentOrigPartitionId = origPartitionId;
            remappedPartitionId[currentOrigPartitionId] = outputPartitionId;
        #ifndef FORCE_EACH_KERNEL_INTO_UNIQUE_PARTITION
        }
        #endif
        KernelPartitionId[out] = outputPartitionId;
    }

    const auto newPipelineOutput = LastKernel + 1U;
    KernelPartitionId[newPipelineOutput] = ++outputPartitionId;
    subsitution[PipelineOutput] = newPipelineOutput;

    BEGIN_SCOPED_REGION

    const auto f = PartitionIds.find(PipelineOutput);
    assert (f != PartitionIds.end());
    const auto origPartitionId = f->second;
    ExpectedNumOfStrides[newPipelineOutput] = calculateExpectedNumOfStrides(PipelineOutput, origPartitionId);

    END_SCOPED_REGION

    // our new partition count can exceed the original one by at most one
    PartitionCount = outputPartitionId + 1U;
    assert (origPartitionCount <= PartitionCount);
    assert ((origPartitionCount + 1) >= PartitionCount);

    // Retain how much memory each partition required
    RequiredPartitionMemory.resize(PartitionCount);
    for (unsigned i = 0; i < origPartitionCount; ++i) {
        const PartitionData & P = partitionGraph[i];
        const auto m = ceiling(P.RequiredMemory * P.ExpectedRepetitions);
        const auto j = remappedPartitionId[i];
        RequiredPartitionMemory[j] = m;
    }

    // Originally, if the pipeline kernel does not have external I/O, both the pipeline in/out
    // nodes would be placed into the same (ignored) set but this won't be true after scheduling.
    // Similarly, if we disable partitioning, every kernel will be placed into its own partition.
    // Accept whatever the prior loops determines is the new partition count.

    PipelineOutput = newPipelineOutput;

    FirstStreamSet = PipelineOutput + 1U;
    LastStreamSet = PipelineOutput + numOfStreamSets;
    FirstBinding = LastStreamSet + 1U;
    LastBinding = LastStreamSet + numOfBindings;

    FirstCall = PipelineOutput + 1U;
    LastCall = PipelineOutput + numOfCallees;
    FirstScalar = LastCall + 1U;
    LastScalar = LastCall + numOfScalars;

    assert (KernelPartitionId[PipelineInput] == 0);

    assert (Relationships[kernels[PipelineInput]].Kernel == mPipelineKernel);
    assert (Relationships[kernels[PipelineOutput]].Kernel == mPipelineKernel);

    for (unsigned i = 0; i < numOfStreamSets; ++i) {
        assert (subsitution[streamSets[i]] == -1U);
        subsitution[streamSets[i]] = FirstStreamSet + i;
    }
    for (unsigned i = 0; i < numOfBindings; ++i) {
        assert (subsitution[bindings[i]] == -1U);
        subsitution[bindings[i]] = FirstBinding  + i;
    }
    for (unsigned i = 0; i < numOfCallees; ++i) {
        assert (subsitution[callees[i]] == -1U);
        subsitution[callees[i]] = FirstCall + i;
    }
    for (unsigned i = 0; i < numOfScalars; ++i) {
        assert (subsitution[scalars[i]] == -1U);
        subsitution[scalars[i]] = FirstScalar + i;
    }

    SmallVector<std::pair<RelationshipType, unsigned>, 64> temp;

    auto transcribe = [&](const Vertices & V, RelationshipGraph & H) {
        for (const auto j : V) {
            assert (j < subsitution.size());
            const auto v = subsitution[j];
            assert (j < num_vertices(Relationships));
            assert (v < num_vertices(H));
            H[v] = Relationships[j];
        }
    };

    auto copy_in_edges = [&](const Vertices & V, RelationshipGraph & H,
            const RelationshipNode::RelationshipNodeType type) {
        for (const auto j : V) {
            const auto v = subsitution[j];
            for (const auto e : make_iterator_range(in_edges(j, Relationships))) {
                const auto i = source(e, Relationships);
                if (Relationships[i].Type == type) {
                    assert (Relationships[e].Reason != ReasonType::OrderingConstraint);
                    const auto u = subsitution[i];
                    assert (u < num_vertices(H));
                    temp.emplace_back(Relationships[e], u);
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
            for (const auto e : make_iterator_range(out_edges(j, Relationships))) {
                const auto i = target(e, Relationships);
                if (Relationships[i].Type == type) {
                    assert (Relationships[e].Reason != ReasonType::OrderingConstraint);
                    const auto w = subsitution[i];
                    assert (w < num_vertices(H));
                    temp.emplace_back(Relationships[e], w);
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
    mStreamGraph = RelationshipGraph{numOfKernels + numOfBindings + numOfStreamSets};

    transcribe(kernels, mStreamGraph);
    copy_in_edges(kernels, mStreamGraph, RelationshipNode::IsBinding);
    copy_out_edges(kernels, mStreamGraph, RelationshipNode::IsBinding);

    transcribe(streamSets, mStreamGraph);
    copy_in_edges(streamSets, mStreamGraph, RelationshipNode::IsBinding);
    copy_out_edges(streamSets, mStreamGraph, RelationshipNode::IsBinding);

    transcribe(bindings, mStreamGraph);
    copy_out_edges(bindings, mStreamGraph, RelationshipNode::IsBinding);

     // create the scalar graph
    mScalarGraph = RelationshipGraph{numOfKernels + numOfCallees + numOfScalars};

    transcribe(kernels, mScalarGraph);
    copy_in_edges(kernels, mScalarGraph, RelationshipNode::IsRelationship);
    copy_out_edges(kernels, mScalarGraph, RelationshipNode::IsRelationship);

    transcribe(callees, mScalarGraph);
    copy_in_edges(callees, mScalarGraph, RelationshipNode::IsRelationship);
    copy_out_edges(callees, mScalarGraph, RelationshipNode::IsRelationship);

    transcribe(scalars, mScalarGraph);

//    printRelationshipGraph(mStreamGraph, errs(), "Streams");
//    printRelationshipGraph(mScalarGraph, errs(), "Scalars");

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitialPipelineGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::generateInitialPipelineGraph(BuilderRef b) {


    // Copy the list of kernels and add in any internal kernels
    Kernels kernels(mPipelineKernel->getKernels());
    assert (num_vertices(Relationships) == 0);
    const unsigned p_in = add_vertex(RelationshipNode(mPipelineKernel), Relationships);
    assert (p_in == PipelineInput);
    const auto n = kernels.size();
    KernelVertexVec vertex(n);
    for (unsigned i = 0; i < n; ++i) {
        const Kernel * K = kernels[i];
        if (LLVM_UNLIKELY(K == mPipelineKernel)) {
            std::string tmp;
            raw_string_ostream msg(tmp);
            msg << mPipelineKernel->getName()
                << " contains itself in its pipeline";
            report_fatal_error(msg.str());
        }
        vertex[i] = Relationships.add(K);
    }
    const unsigned p_out = add_vertex(RelationshipNode(mPipelineKernel), Relationships);
    PipelineOutput = p_out;

    // From the pipeline's perspective, a pipeline input node "produces" the inputs of the pipeline and a
    // pipeline output node "consumes" its outputs. Internally this means the inputs and outputs of the
    // pipeline are inverted from its external view but this change simplifies the analysis considerably
    // by permitting the compiler's internal graphs to acyclic.

    addProducerRelationships(PortType::Output, p_in, mPipelineKernel->getInputStreamSetBindings(), Relationships);
    addConsumerRelationships(PortType::Input, p_out, mPipelineKernel->getOutputStreamSetBindings(), Relationships, true);

    for (unsigned i = 0; i < n; ++i) {
        addProducerRelationships(PortType::Output, vertex[i], kernels[i]->getOutputStreamSetBindings(), Relationships);
    }
    for (unsigned i = 0; i < n; ++i) {
        addConsumerRelationships(PortType::Input, vertex[i], kernels[i]->getInputStreamSetBindings(), Relationships, false);
    }

    for (unsigned i = 0; i < n; ++i) {
        addReferenceRelationships(PortType::Input, vertex[i], kernels[i]->getInputStreamSetBindings(), Relationships);
    }
    for (unsigned i = 0; i < n; ++i) {
        addReferenceRelationships(PortType::Output, vertex[i], kernels[i]->getOutputStreamSetBindings(), Relationships);
    }

    addPopCountKernels(b, kernels, vertex, Relationships);
    // addRegionSelectorKernels(b, kernels, vertex, G, internalKernels, internalBindings);

    addProducerRelationships(PortType::Output, p_in, mPipelineKernel->getInputScalarBindings(), Relationships);
    addConsumerRelationships(PortType::Input, p_out, mPipelineKernel->getOutputScalarBindings(), Relationships, true);
    for (unsigned i = 0; i < n; ++i) {
        addProducerRelationships(PortType::Output, vertex[i], kernels[i]->getOutputScalarBindings(), Relationships);
    }

    for (unsigned i = 0; i < n; ++i) {
        addConsumerRelationships(PortType::Input, vertex[i], kernels[i]->getInputScalarBindings(), Relationships, true);
    }

    for (const CallBinding & C : mPipelineKernel->getCallBindings()) {
        addConsumerRelationships(PortType::Input, C, Relationships);
    }

    // Pipeline optimizations
    combineDuplicateKernels(b, kernels, Relationships);
    removeUnusedKernels(p_in, p_out, kernels, Relationships);

    #ifndef EXPERIMENTAL_SCHEDULING
    // Add ordering constraints to ensure the input must be before all kernel invocations
    // and the invocations must come before the output.
    for (const auto v : vertex) {
        add_edge(p_in, v, RelationshipType{PortType::Input, 0, ReasonType::OrderingConstraint}, Relationships);
        add_edge(v, p_out, RelationshipType{PortType::Input, 0, ReasonType::OrderingConstraint}, Relationships);
    }
    #endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addRegionSelectorKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineAnalysis::addRegionSelectorKernels(BuilderRef b, Kernels & kernels, KernelVertexVec & vertex, ProgramGraph & G) {

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

    BaseDriver & driver = reinterpret_cast<BaseDriver &>(b->getDriver());

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
                    if (LLVM_UNLIKELY(!rate.isFixed() || rate.getRate() != Rational(1))) {
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
                mInternalKernels.emplace_back(selector);
                // Mark the region selectors for this kernel
                alreadyCreated.emplace(cond, regionSpans);
            } else { // we've already created the correct region span
                regionSpans = f->second; assert (regionSpans);
            }
            // insert the implicit relationships
            const auto K = G.addOrFind(kernel);
            vertex.push_back(K);
            Binding * const binding = new Binding("#regionselector", regionSpans);
            mInternalBindings.emplace_back(binding);
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
void PipelineAnalysis::addPopCountKernels(BuilderRef b, Kernels & kernels, KernelVertexVec & vertex, ProgramGraph & G) {

    struct Edge {
        CountingType    Type;
        StreamSetPort   Port;
        size_t          StrideLength;

        Edge() : Type(Unknown), Port(), StrideLength() { }
        Edge(const CountingType type, const StreamSetPort port, size_t stepFactor) : Type(type), Port(port), StrideLength(stepFactor) { }
    };

    using Graph = adjacency_list<vecS, vecS, directedS, Relationship *, Edge>;
    using Vertex = Graph::vertex_descriptor;
    using Map = flat_map<Relationship *, Vertex>;

    const auto numOfKernels = kernels.size();

    Graph H(numOfKernels);
    Map M;

    for (unsigned i = 0; i < numOfKernels; ++i) {

        const Kernel * const kernel = kernels[i];

        auto addPopCountDependency = [&](const ProgramGraph::vertex_descriptor bindingVertex,
                                         const RelationshipType & port) {

            const RelationshipNode & rn = G[bindingVertex];
            assert (rn.Type == RelationshipNode::IsBinding);
            const Binding & binding = rn.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_UNLIKELY(rate.isPopCount() || rate.isNegatedPopCount())) {
                // determine which port this I/O port refers to
                for (const auto e : make_iterator_range(in_edges(bindingVertex, G))) {
                    const RelationshipType & rt = G[e];
                    if (rt.Reason == ReasonType::Reference) {
                        const auto refStreamVertex = source(e, G);
                        const RelationshipNode & rn = G[refStreamVertex];
                        assert (rn.Type == RelationshipNode::IsBinding);
                        const Binding & refBinding = rn.Binding;
                        const ProcessingRate & refRate = refBinding.getRate();
                        Relationship * const refStream = refBinding.getRelationship();
                        const auto f = M.find(refStream);
                        Vertex refVertex = 0;
                        if (LLVM_UNLIKELY(f != M.end())) {
                            refVertex = f->second;
                        } else {
                            if (LLVM_UNLIKELY(refBinding.isDeferred() || !refRate.isFixed())) {
                                SmallVector<char, 0> tmp;
                                raw_svector_ostream msg(tmp);
                                msg << kernel->getName();
                                msg << ": pop count reference ";
                                msg << refBinding.getName();
                                msg << " must refer to a non-deferred Fixed rate stream";
                                report_fatal_error(msg.str());
                            }
                            refVertex = add_vertex(refStream, H);
                            M.emplace(refStream, refVertex);
                        }
                        const Rational strideLength = refRate.getRate() * kernel->getStride();
                        if (LLVM_UNLIKELY(strideLength.denominator() != 1)) {
                            SmallVector<char, 0> tmp;
                            raw_svector_ostream msg(tmp);
                            msg << kernel->getName();
                            msg << ": pop count reference ";
                            msg << refBinding.getName();
                            msg << " cannot have a rational rate";
                            report_fatal_error(msg.str());
                        }
                        const auto type = rate.isPopCount() ? CountingType::Positive : CountingType::Negative;
                        add_edge(refVertex, i, Edge{type, port, strideLength.numerator()}, H);
                        return;
                    }
                }
                llvm_unreachable("could not find reference for popcount rate?");
            }
        };

        const auto j = G.find(kernel);

        for (const auto e : make_iterator_range(in_edges(j, G))) {
            addPopCountDependency(source(e, G), G[e]);
        }
        for (const auto e : make_iterator_range(out_edges(j, G))) {
            addPopCountDependency(target(e, G), G[e]);
        }
    }

    const auto n = num_vertices(H);
    if (LLVM_LIKELY(n == numOfKernels)) {
        return;
    }

    BaseDriver & driver = reinterpret_cast<BaseDriver &>(b->getDriver());

    IntegerType * const sizeTy = b->getSizeTy();

    kernels.resize(n, nullptr);

    for (auto i = numOfKernels; i < n; ++i) {

        size_t strideLength = 0;
        CountingType type = CountingType::Unknown;
        for (const auto e : make_iterator_range(out_edges(i, H))) {
            const Edge & ed = H[e];
            type |= ed.Type;
            if (strideLength == 0) {
                strideLength = ed.StrideLength;
            } else {
                strideLength = boost::gcd(strideLength, ed.StrideLength);
            }
        }
        assert (strideLength != 1);
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
                popCountKernel = new PopCountKernel(b, PopCountKernel::POSITIVE, strideLength, input, positive);
                break;
            case CountingType::Negative:
                popCountKernel = new PopCountKernel(b, PopCountKernel::NEGATIVE, strideLength, input, negative);
                break;
            case CountingType::Both:
                popCountKernel = new PopCountKernel(b, PopCountKernel::BOTH, strideLength, input, positive, negative);
                break;
            default: llvm_unreachable("unknown counting type?");
        }
        // Add the popcount kernel to the pipeline
        assert (i < kernels.size());
        kernels[i] = popCountKernel;
        mInternalKernels.emplace_back(popCountKernel);

        const auto k = G.add(popCountKernel);
        vertex.push_back(k);
        addConsumerRelationships(PortType::Input, k, popCountKernel->getInputStreamSetBindings(), G, false);
        addProducerRelationships(PortType::Output, k, popCountKernel->getOutputStreamSetBindings(), G);

        // subsitute the popcount relationships
        for (const auto e : make_iterator_range(out_edges(i, H))) {
            const Edge & ed = H[e];
            const Kernel * const kernel = kernels[target(e, H)];
            const auto consumer = G.find(kernel);
            assert (ed.Type == CountingType::Positive || ed.Type == CountingType::Negative);
            StreamSet * const stream = ed.Type == CountingType::Positive ? positive : negative; assert (stream);
            const auto streamVertex = G.find(stream);

            // append the popcount rate stream to the kernel
            Rational stepRate{ed.StrideLength, strideLength * kernel->getStride()};
            Binding * const popCount = new Binding("#popcount" + std::to_string(ed.Port.Number), stream, FixedRate(stepRate));
            mInternalBindings.emplace_back(popCount);
            const auto popCountBinding = G.add(popCount);

            const unsigned portNum = in_degree(consumer, G);
            add_edge(streamVertex, popCountBinding, RelationshipType{PortType::Input, portNum, ReasonType::ImplicitPopCount}, G);
            add_edge(popCountBinding, consumer, RelationshipType{PortType::Input, portNum, ReasonType::ImplicitPopCount}, G);

            auto rebind_reference = [&](const unsigned binding) {

                RelationshipNode & rn = G[binding];
                assert (rn.Type == RelationshipNode::IsBinding);

                graph_traits<ProgramGraph>::in_edge_iterator ei, ei_end;
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
                mInternalBindings.emplace_back(replacement);
                rn.Binding = replacement;

                add_edge(popCountBinding, binding, RelationshipType{PortType::Input, portNum, ReasonType::Reference}, G);

            };

            bool notFound = true;
            if (ed.Port.Type == PortType::Input) {
                for (const auto e : make_iterator_range(in_edges(consumer, G))) {
                    const RelationshipType & type = G[e];
                    if (type.Number == ed.Port.Number) {
                        assert (type.Type == PortType::Input);
                        rebind_reference(source(e, G));
                        notFound = false;
                        break;
                    }
                }
            } else { // if (ed.Port.Type == PortType::Output) {
                for (const auto e : make_iterator_range(out_edges(consumer, G))) {
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
void PipelineAnalysis::combineDuplicateKernels(BuilderRef b, const Kernels & kernels, ProgramGraph & G) /*static*/ {

    using StreamSetVector = std::vector<std::pair<unsigned, StreamSetPort>>;
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
                if (kernel->externallyInitialized() || kernel->hasAttribute(AttrId::SideEffecting)) {
                    continue;
                }

                const auto n = in_degree(i, G);
                inputs.resize(n);
                scalars.resize(n);
                unsigned numOfStreams = 0;

                for (const auto e : make_iterator_range(in_edges(i, G))) {
                    const RelationshipType & port = G[e];
                    const auto input = source(e, G);
                    const RelationshipNode & node = G[input];
                    if (node.Type == RelationshipNode::IsBinding) {
                        unsigned relationship = 0;
                        StreamSetPort ref{};
                        for (const auto e : make_iterator_range(in_edges(input, G))) {
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

                KernelId id(kernel->getName(), inputs, scalars);

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
                        for (const auto e : make_iterator_range(out_edges(j, G))) {

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
                        for (const auto e : make_iterator_range(out_edges(i, G))) {
                            const StreamSetPort & port = G[e];
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

                            for (const auto e : make_iterator_range(out_edges(original, G))) {
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
inline void PipelineAnalysis::removeUnusedKernels(const unsigned p_in, const unsigned p_out,
                                                  const Kernels & kernels, ProgramGraph & G) /*static*/ {

    flat_set<unsigned> visited;
    std::queue<unsigned> pending;
    pending.push(p_out);
    assert (p_in < p_out);
    visited.insert_unique(p_in);
    visited.insert_unique(p_out);

    // identify all nodes that must be in the final pipeline
    for (const Binding & output : mPipelineKernel->getOutputScalarBindings()) {
        const auto p = G.find(output.getRelationship());
        pending.push(p);
        visited.insert_unique(p);
    }
    const auto & calls = mPipelineKernel->getCallBindings();
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
        for (const auto e : make_iterator_range(in_edges(v, G))) {
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
    // unused outputs of a kernel in G. Instead we make two
    // passes: (1) marks the outputs of all used kernels as
    // live. (2) deletes every dead node.

    for (const auto v : make_iterator_range(vertices(G))) {
        const RelationshipNode & rn = G[v];
        if (rn.Type == RelationshipNode::IsKernel) {
            if (LLVM_LIKELY(visited.count(v) != 0)) {
                for (const auto e : make_iterator_range(out_edges(v, G))) {
                    const auto b = target(e, G);
                    const RelationshipNode & rb = G[b];
                    assert (rb.Type == RelationshipNode::IsBinding || rb.Type == RelationshipNode::IsRelationship);
                    visited.insert(b); // output binding/scalar
                    if (LLVM_LIKELY(rb.Type == RelationshipNode::IsBinding)) {
                        visited.insert(child(b, G)); // output stream
                    }
                }
            }
        }
    }

    for (const auto v : make_iterator_range(vertices(G))) {
        if (LLVM_UNLIKELY(visited.count(v) == 0)) {
            RelationshipNode & rn = G[v];
            clear_vertex(v, G);
            rn.Type = RelationshipNode::IsNil;
            rn.Kernel = nullptr;
        }
    }

}

} // end of namespace

#endif
