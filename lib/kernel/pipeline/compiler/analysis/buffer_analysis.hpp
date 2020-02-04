#ifndef BUFFER_ANALYSIS_HPP
#define BUFFER_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include <boost/algorithm/string/replace.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <tuple>
#include <llvm/Support/ErrorHandling.h>

// TODO: any buffers that exist only to satisfy the output dependencies are unnecessary.
// We could prune away kernels if none of their outputs are needed but we'd want some
// form of "fake" buffer for output streams in which only some are unnecessary. Making a
// single static thread local buffer thats large enough for one segment.

// TODO: can we "combine" static stream sets that are used together and use fixed offsets
// from the first set? Would this improve data locality or prefetching?

// TODO: generate thread local buffers when we can guarantee all produced data is consumed
// within the same segment "iteration"? We can eliminate synchronization for kernels that
// consume purely local data.

// TODO: if we can prove the liveness of two streams never overlaps, can we reuse the
// memory space.

namespace kernel {

#warning TODO: if an external buffer is marked as managed, have it allocate and manage the buffer but not deallocate it.

inline Rational mod(const Rational & a, const Rational & b) {
    Rational n(a.numerator() * b.denominator(), b.numerator() * a.denominator());
    return a - Rational{floor(n)} * b;
}

bool requiresLinearAccess(const Binding & binding) {
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::Linear))) {
        return true;
    }
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::LookBehind))) {
        const auto & lookBehind = binding.findAttribute(AttrId::LookBehind);
        if (LLVM_UNLIKELY(lookBehind.amount() == 0)) {
            return true;
        }
    }
    return false;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePipelineBufferGraph
 *
 * Return an acyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    auto roundUpTo = [](const Rational a, const Rational b) {
        // m = mod(a, b)
        Rational n(a.numerator() * b.denominator(), b.numerator() * a.denominator());
        const auto m = a - Rational{floor(n)} * b;
        if (LLVM_UNLIKELY(m.numerator() != 0)) {
            const auto r = (a - m) + b;
            assert (r.denominator() == 1);
            return r.numerator();
        }
        assert (a.denominator() == 1);
        return a.numerator();
    };

    BufferGraph G(LastStreamSet + 1);

    initializeBufferGraph(G);
    computeDataFlowRates(G);

    SmallFlatSet<BufferGraph::vertex_descriptor, 16> E;
    // mark all external I/O
    E.reserve(out_degree(PipelineInput, G) + in_degree(PipelineOutput, G));
    for (const auto e : make_iterator_range(out_edges(PipelineInput, G))) {
        E.insert(target(e, G));
    }
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, G))) {
        E.insert(source(e, G));
    }

    auto internalOrExternal = [&E](BufferGraph::vertex_descriptor streamSet) -> BufferType {
        if (LLVM_LIKELY(E.count(streamSet) == 0)) {
            return BufferType::Internal;
        } else {
            return BufferType::External;
        }
    };

    // fill in any known managed buffers
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & producerRate = G[e];
            const Binding & output = producerRate.Binding;
            if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) {
                const auto streamSet = target(e, G);
                BufferNode & bn = G[streamSet];
                const auto linear = output.hasAttribute(AttrId::Linear);
                bn.Buffer = new ExternalBuffer(b, output.getType(), linear, 0);
                bn.Type = BufferType::Unowned | internalOrExternal(streamSet);
            }
        }
    }

    // fill in any unmanaged pipeline input buffers
    for (const auto e : make_iterator_range(out_edges(PipelineInput, G))) {
        const auto streamSet = target(e, G);
        BufferNode & bn = G[streamSet];
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferRateData & rate = G[e];
            const Binding & input = rate.Binding;
            bn.Buffer = new ExternalBuffer(b, input.getType(), true, 0);
            bn.Type = BufferType::UnownedExternal;
        }
    }

    // and pipeline output buffers ...
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, G))) {
        const auto streamSet = source(e, G);
        BufferNode & bn = G[streamSet];
        if (LLVM_LIKELY(bn.Buffer == nullptr)) {
            const BufferRateData & rate = G[e];
            const Binding & output = rate.Binding;
            if (LLVM_UNLIKELY(Kernel::isLocalBuffer(output))) continue;
            bn.Buffer = new ExternalBuffer(b, output.getType(), true, 0);
            bn.Type = BufferType::UnownedExternal;
        }
    }

    const auto blockWidth = b->getBitBlockWidth();
    const auto numOfSegments = std::max(mNumOfSegments, mNumOfThreads);

    // then construct the rest
    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {

        BufferNode & bn = G[streamSet];
        const auto pe = in_edge(streamSet, G);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;

        if (LLVM_LIKELY(bn.Buffer == nullptr)) { // is internal buffer

            // TODO: If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. During initialization, we could pass a "suggestion"
            // argument to indicate what the outer pipeline believes its I/O rates will be.

            const BufferType bufferType = internalOrExternal(streamSet);

            bn.Type = bufferType;

            // If this buffer is externally used, we cannot analyze the dataflow rate of
            // external consumers. Default to dynamic for such buffers.

            // Similarly if any internal consumer has a deferred rate, we cannot analyze
            // any consumption rates.

            bool dynamic = (bufferType == BufferType::External) || output.hasAttribute(AttrId::Deferred);

            const auto in = in_edge(streamSet, G);
            const BufferRateData & producerRate = G[in];
            const auto producer = source(pe, G);

            unsigned lookAhead = 0;
            unsigned lookBehind = 0;
            unsigned reflection = 0;

            const Binding & outputBinding = producerRate.Binding;
            for (const Attribute & attr : outputBinding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::LookBehind:
                        lookBehind = std::max(lookBehind, attr.amount());
                        break;
                    case AttrId::Delayed:
                        reflection = std::max(reflection, attr.amount());
                        break;
                    default: break;
                }
            }

            const auto pMax = producerRate.Maximum * MaximumNumOfStrides[producer];

            Rational requiredSizeFactor{1};
            if (producerRate.Maximum == producerRate.Minimum) {
                requiredSizeFactor = producerRate.Maximum;
            }

            assert (producerRate.Maximum >= producerRate.Minimum);

            Rational consumeMin(std::numeric_limits<unsigned>::max());
            Rational consumeMax(std::numeric_limits<unsigned>::min());

            const auto producerPartitionId = KernelPartitionId[producer];

            bool linear = requiresLinearAccess(output);
            for (const auto ce : make_iterator_range(out_edges(streamSet, G))) {

                const BufferRateData & consumerRate = G[ce];
                const Binding & input = consumerRate.Binding;

                const auto consumer = target(ce, G);

                const auto cMin = consumerRate.Minimum * MinimumNumOfStrides[consumer];
                const auto cMax = consumerRate.Maximum * MaximumNumOfStrides[consumer];

                if (!dynamic) {
                    // Does this stream cross a partition boundary?
                    if (producerPartitionId != KernelPartitionId[consumer]) {
                        dynamic = true;
                    // Could we consume less data than we produce?
                    } else if (consumerRate.Minimum < consumerRate.Maximum) {
                        dynamic = true;
                    // Or is the data consumption rate unpredictable despite its type?
                    } else if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                        dynamic = true;
                    }
                }

                if (LLVM_UNLIKELY(linear || requiresLinearAccess(input))) {
                    linear = true;
                }

                assert (consumerRate.Maximum >= consumerRate.Minimum);

                consumeMin = std::min(consumeMin, cMin);
                consumeMax = std::max(consumeMax, cMax);

                assert (consumeMax >= consumeMin);

                if (consumerRate.Maximum == consumerRate.Minimum) {
                    requiredSizeFactor = lcm(requiredSizeFactor, consumerRate.Maximum);
                }

                const Binding & inputBinding = consumerRate.Binding;
                // Get output overflow size
                unsigned tmpLookAhead = 0;
                for (const Attribute & attr : inputBinding.getAttributes()) {
                    switch (attr.getKind()) {
                        case AttrId::LookAhead:
                            tmpLookAhead = std::max(tmpLookAhead, attr.amount());
                            break;
                        case AttrId::LookBehind:
                            lookBehind = std::max(lookBehind, attr.amount());
                            break;
                        default: break;
                    }
                }
                if (consumerRate.Maximum > consumerRate.Minimum) {
                    tmpLookAhead += ceiling(consumerRate.Maximum - consumerRate.Minimum);
                }
                lookAhead = std::max(tmpLookAhead, lookAhead);
            }

            #ifdef PERMIT_THREAD_LOCAL_BUFFERS
            const auto nonLocal = dynamic;
            #else
            const auto nonLocal = true;
            #endif

            bn.NonLocal = nonLocal;

            const auto copyBack = ceiling(producerRate.Maximum - producerRate.Minimum);

            bn.LookBehind = lookBehind;
            bn.LookBehindReflection = reflection;
            bn.CopyBack = copyBack;
            bn.LookAhead = lookAhead;

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto overflowSize = round_up_to(std::max(copyBack, lookAhead), blockWidth);
            const auto underflowSize = round_up_to(std::max(lookBehind, reflection), blockWidth);
            const auto reqSize0 = std::max(pMax, Rational{2} * consumeMax - consumeMin);
            const Rational reqSize1{2 * (overflowSize + underflowSize), 1};
            const auto reqSize2 = std::max(reqSize0, reqSize1);
            const auto requiredSize = roundUpTo(reqSize2, requiredSizeFactor);

            #ifdef PERMIT_THREAD_LOCAL_BUFFERS
            const auto bufferFactor = nonLocal ? numOfSegments : 1U;
            #else
            const auto bufferFactor = numOfSegments;
            #endif

            const auto bufferSize = round_up_to(requiredSize, blockWidth) * bufferFactor;

            Type * const baseType = output.getType();

            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            StreamSetBuffer * buffer = nullptr;
            if (dynamic) {
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, linear, 0U);
            } else {
                buffer = new StaticBuffer(b, baseType, bufferSize, overflowSize, underflowSize, linear, 0U);
            }
            bn.Buffer = buffer;
        }
    }

    #ifdef PRINT_BUFFER_GRAPH
    printBufferGraph(G, errs());
    #endif

    verifyIOStructure(G);

    return G;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::initializeBufferGraph(BufferGraph & G) const {

    for (unsigned i = PipelineInput; i <= PipelineOutput; ++i) {
        const RelationshipNode & node = mStreamGraph[i];
        const Kernel * const kernel = node.Kernel; assert (kernel);

        auto computeBufferRateBounds = [&](const RelationshipType port,
                                           const RelationshipNode & bindingNode,
                                           const unsigned streamSet) {
            assert (bindingNode.Type == RelationshipNode::IsBinding);
            const Binding & binding = bindingNode.Binding;
            const ProcessingRate & rate = binding.getRate();
            Rational lb{rate.getLowerBound()};
            Rational ub{rate.getUpperBound()};
            if (LLVM_UNLIKELY(rate.isGreedy())) {
                if (LLVM_UNLIKELY(port.Type == PortType::Output)) {
                    SmallVector<char, 0> tmp;
                    raw_svector_ostream out(tmp);
                    out << "Greedy rate cannot be applied an output port: "
                        << kernel->getName() << "." << binding.getName();
                    report_fatal_error(out.str());
                }
                const auto e = in_edge(streamSet, G);
                const BufferRateData & producerBr = G[e];
                ub = std::max(lb, producerBr.Maximum);
            } else {
                const auto strideLength = kernel->getStride();
                if (LLVM_UNLIKELY(rate.isRelative())) {
                    const Binding & ref = getBinding(getReference(i, port));
                    const ProcessingRate & refRate = ref.getRate();
                    lb *= refRate.getLowerBound();
                    ub *= refRate.getUpperBound();
                }
                lb *= strideLength;
                ub *= strideLength;
            }
            return BufferRateData{port, binding, lb, ub};
        };

        // add in any inputs
        RelationshipType prior_in{};
        for (const auto e : make_iterator_range(in_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            assert (prior_in < port);
            prior_in = port;
            const auto binding = source(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const auto f = first_in_edge(binding, mStreamGraph);
            assert (mStreamGraph[f].Reason != ReasonType::Reference);
            unsigned streamSet = source(f, mStreamGraph);
            assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
            add_edge(streamSet, i, computeBufferRateBounds(port, rn, streamSet), G);
        }

        // and any outputs
        RelationshipType prior_out{};
        for (const auto e : make_iterator_range(out_edges(i, mStreamGraph))) {
            const RelationshipType & port = mStreamGraph[e];
            assert (prior_out < port);
            prior_out = port;
            const auto binding = target(e, mStreamGraph);
            const RelationshipNode & rn = mStreamGraph[binding];
            assert (rn.Type == RelationshipNode::IsBinding);
            const auto f = out_edge(binding, mStreamGraph);
            assert (mStreamGraph[f].Reason != ReasonType::Reference);
            unsigned streamSet = target(f, mStreamGraph);
            assert (mStreamGraph[streamSet].Type == RelationshipNode::IsRelationship);
            assert (isa<StreamSet>(mStreamGraph[streamSet].Relationship));
            add_edge(i, streamSet, computeBufferRateBounds(port, rn, streamSet), G);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief verifyIOStructure
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyIOStructure(const BufferGraph & G) const {


#if 0

    // verify that the buffer config is valid
    for (unsigned i = FirstStreamSet; i <= LastStreamSet; ++i) {

        const BufferNode & bn = G[i];
        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;




        // Type check stream set I/O types.
        Type * const baseType = output.getType();
        for (const auto e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & consumerRate = G[e];
            const Binding & input = consumerRate.Binding;
            if (LLVM_UNLIKELY(baseType != input.getType())) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream msg(tmp);
                msg << producer->getName() << ':' << output.getName()
                    << " produces a ";
                baseType->print(msg);
                const Kernel * const consumer = getKernel(target(e, G));
                msg << " but "
                    << consumer->getName() << ':' << input.getName()
                    << " expects ";
                input.getType()->print(msg);
                report_fatal_error(msg.str());
            }
        }

        for (const auto ce : make_iterator_range(out_edges(i, G))) {
            const Binding & input = G[ce].Binding;
            if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                const auto consumer = target(ce, G);
                out << getKernel(consumer)->getName()
                    << '.' << input.getName()
                    << " requires that "
                    << producer->getName()
                    << '.' << output.getName()
                    << " is a Linear buffer.";
                report_fatal_error(out.str());
            }
        }


    }

#endif

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) const {

    using BufferId = StreamSetBuffer::BufferKind;

    auto print_rational = [&out](const Rational & r) -> raw_ostream & {
        if (r.denominator() > 1) {
            const auto n = r.numerator() / r.denominator();
            const auto p = r.numerator() % r.denominator();
            out << n << '+' << p << '/' << r.denominator();
        } else {
            out << r.numerator();
        }
        return out;
    };

    auto rate_range = [&out, print_rational](const Rational & a, const Rational & b) -> raw_ostream & {
        print_rational(a);
        out << ",";
        print_rational(b);
        return out;
    };

    auto printStreamSet = [&](const unsigned streamSet) {
        out << "v" << streamSet << " [shape=record, label=\""
               << streamSet << "|{";

        const BufferNode & bn = G[streamSet];
        const StreamSetBuffer * const buffer = bn.Buffer;
        if (buffer == nullptr) {
            out << '?';
        } else {
            char bufferType = '?';
            switch (bn.Type) {
                case BufferType::Internal:
                    switch (buffer->getBufferKind()) {
                        case BufferId::StaticBuffer:
                            bufferType = 'S'; break;
                        case BufferId::DynamicBuffer:
                            bufferType = 'D'; break;
                        default: llvm_unreachable("unknown streamset type");
                    }
                    break;
                case BufferType::ManagedByKernel:
                    bufferType = 'M';
                    break;
                case BufferType::External:
                    bufferType = 'E';
                    break;
                case BufferType::UnownedExternal:
                    bufferType = 'U';
                    break;
                default: llvm_unreachable("unknown buffer type id");
            }
            out << bufferType;
            if (buffer->isLinear()) {
                out << 'L';
            }
            Type * ty = buffer->getBaseType();
            out << ':'
                << ty->getArrayNumElements() << 'x';
            ty = ty->getArrayElementType();
            ty = ty->getVectorElementType();
            out << ty->getIntegerBitWidth();
        }

        out << "|{";

        if (buffer && buffer->getBufferKind() != BufferId::ExternalBuffer) {
            switch (buffer->getBufferKind()) {
                case BufferId::StaticBuffer:
                    out << cast<StaticBuffer>(buffer)->getCapacity();
                    break;
                case BufferId::DynamicBuffer:
                    out << cast<DynamicBuffer>(buffer)->getInitialCapacity();
                    break;
                default: llvm_unreachable("unknown buffer type");
            }
        }

        if (bn.LookBehind) {
            out << "|LB:" << bn.LookBehind;
        }
        if (bn.CopyBack) {
            out << "|CB:" << bn.CopyBack;
        }
        if (bn.LookAhead) {
            out << "|LA:" << bn.LookAhead;
        }

        out << "}}\"];\n";

    };

    auto currentPartition = PartitionCount;
    bool closePartition = false;

    auto checkClosePartitionLabel = [&]() {
        if (closePartition) {
            out << "}\n";
            closePartition = false;
        }
    };

    auto checkOpenPartitionLabel = [&](const unsigned kernel) {
        const auto partitionId = KernelPartitionId[kernel];
        if (partitionId != currentPartition) {
            checkClosePartitionLabel();
            if (LLVM_LIKELY(partitionId != -1U)) {
                out << "subgraph cluster_" << partitionId << " {\n"
                       "label=\"Partition #" << partitionId  << "\";"
                       "fontcolor=\"red\";"
                       "style=\"rounded,dashed,bold\";"
                       "color=\"red\";"
                       "\n";
                closePartition = true;
            }            
        }        
        currentPartition = partitionId;
    };

    auto printKernel = [&](const unsigned kernel, const StringRef name) {
        checkOpenPartitionLabel(kernel);
        out << "v" << kernel << " [label=\"[" <<
                kernel << "] " << name << "\\n"
                //" Partition: " << KernelPartitionId[v] << "\\n"
                " Expected:  ["; print_rational(MinimumNumOfStrides[kernel]) << ',';
                                print_rational(MaximumNumOfStrides[kernel]) << "]\\n"
                "\" shape=rect,style=rounded,peripheries=2"
                "];\n";

        for (const auto e : make_iterator_range(out_edges(kernel, G))) {
            const auto streamSet = target(e, G);
            printStreamSet(streamSet);
        }
    };

    out << "digraph \"" << mTarget->getName() << "\" {\n"
           "rankdir=tb;"
           "nodesep=0.25;"
           "ranksep=0.5;"
           "newrank=true;"
           "\n";

    printKernel(PipelineInput, "P_{in}");
    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        auto name = kernel->getName().str();
        boost::replace_all(name, "\"", "\\\"");       
        printKernel(i, name);
    }
    printKernel(PipelineOutput, "P_{out}");
    checkClosePartitionLabel();


    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);

        bool isLocal = true;
        if (s >= FirstStreamSet) {
            const auto p = parent(s, G);
            isLocal = KernelPartitionId[p] == KernelPartitionId[t];
        }

        out << "v" << s << " -> v" << t <<
               " [";
        const BufferRateData & pd = G[e];
        out << "label=\"#" << pd.Port.Number << ": ";
        const Binding & binding = pd.Binding;
        const ProcessingRate & rate = binding.getRate();
        switch (rate.getKind()) {
            case RateId::Fixed:
                out << "F(";
                print_rational(pd.Minimum);
                out << ")";
                break;
            case RateId::Bounded:
                out << "B(";
                rate_range(pd.Minimum, pd.Maximum);
                out << ")";
                break;
            case RateId::Greedy:
                out << "G(";
                print_rational(rate.getLowerBound());
                out << ",*)";
                break;
            case RateId::PartialSum:
                out << "P(";
                print_rational(rate.getUpperBound());
                out << ")";
                break;
            default: llvm_unreachable("unknown or unhandled rate type in buffer graph");
        }
        if (binding.hasAttribute(AttrId::Principal)) {
            out << " [P]";
        }
        if (binding.hasAttribute(AttrId::ZeroExtended)) {
            out << " [Z]";
        }
        std::string name = binding.getName();
        boost::replace_all(name, "\"", "\\\"");
        out << "\\n" << name << "\"";
        if (isLocal) {
            out << " style=dashed";
        } else if (!isCountable(binding)) {
            out << " style=bold";
        }
        out << "];\n";
    }

    out << "}\n\n";
    out.flush();
}


}

#endif
