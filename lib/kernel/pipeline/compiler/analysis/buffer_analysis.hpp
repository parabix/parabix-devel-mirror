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
 * @brief verifyIOStructure
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::verifyIOStructure() const {


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
 * @brief makePipelineBufferGraph
 *
 * Return an acyclic bi-partite graph indicating the I/O relationships between the kernels and their buffers.
 *
 * Ordering: producer -> buffer -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
BufferGraph PipelineCompiler::makeBufferGraph(BuilderRef b) {

    BufferGraph G(LastStreamSet + 1);

    initializeBufferGraph(G);
    computeDataFlowRates(b, G);

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

            const auto producer = source(pe, G);
            const auto producerPartitionId = KernelPartitionId[producer];

            bool linear = requiresLinearAccess(output);
            for (const auto ce : make_iterator_range(out_edges(streamSet, G))) {
                const BufferRateData & consumerRate = G[ce];
                const Binding & input = consumerRate.Binding;

                // Does this stream cross a partition boundary?
                const auto consumer = target(ce, G);
                if (producerPartitionId != KernelPartitionId[consumer]) {
                    dynamic = true;
                }
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                    dynamic = true;
                }

                if (LLVM_UNLIKELY(linear || requiresLinearAccess(input))) {
                    linear = true;
                }
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            const auto overflowSize = std::max(bn.CopyBack, bn.LookAhead);
            const auto underflowSize = bn.LookBehind;
            const auto bufferSize = bn.RequiredSpace;
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
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) const {

    using BufferId = StreamSetBuffer::BufferKind;

    auto print_rational = [&out](const Rational & r) {
        if (r.denominator() > 1) {
            const auto n = r.numerator() / r.denominator();
            const auto p = r.numerator() % r.denominator();
            out << n << '+' << p << '/' << r.denominator();
        } else {
            out << r.numerator();
        }
    };

    auto rate_range = [&out, print_rational](const Rational & a, const Rational & b) {
        print_rational(a);
        out << " - ";
        print_rational(b);
    };

    out << "digraph \"" << mTarget->getName() << "\" {\n"
           "v" << PipelineInput << " [label=\"[" <<
           PipelineInput << "] P_{in}\\n"
           " Partition: " << KernelPartitionId[PipelineInput] << "\\n"
           " Expected: " << ExpectedNumOfStrides[PipelineInput] << "\\n";
    out << "\" shape=rect, style=rounded, peripheries=2];\n";

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");
        out << "v" << i <<
               " [label=\"[" << i << "] " << name << "\\n"
               " Partition: " << KernelPartitionId[i] << "\\n"
               " Expected: " << ExpectedNumOfStrides[i] << "\\n";
        out << "\" shape=rect, style=rounded, peripheries=2];\n";
    }

    out << "v" << PipelineOutput << " [label=\"[" <<
           PipelineOutput << "] P_{out}\\n"
           " Partition: " << KernelPartitionId[PipelineOutput] << "\\n"
           " Expected: " << ExpectedNumOfStrides[PipelineOutput] << "\\n";
    out << "\" shape=rect, style=rounded, peripheries=2];\n";

    for (auto i = FirstStreamSet; i <= LastStreamSet; ++i) {
        out << "v" << i << " [shape=record, label=\""
               << i << "|{";

        const BufferNode & bn = G[i];
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
    }

    for (auto e : make_iterator_range(edges(G))) {
        const auto s = source(e, G);
        const auto t = target(e, G);
        out << "v" << s << " -> v" << t;
        const BufferRateData & pd = G[e];
        out << " [label=\"#" << pd.Port.Number << ": ";
        rate_range(pd.Minimum, pd.Maximum);
        const Binding & binding = pd.Binding;
        if (binding.hasAttribute(AttrId::Principal)) {
            out << " [P]";
        }
        if (binding.hasAttribute(AttrId::ZeroExtended)) {
            out << " [Z]";
        }
        std::string name = binding.getName();
        boost::replace_all(name, "\"", "\\\"");
        out << ")\\n" << name << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}


}

#endif
