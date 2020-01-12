#ifndef BUFFER_ANALYSIS_HPP
#define BUFFER_ANALYSIS_HPP

#include "../pipeline_compiler.hpp"
#include <boost/algorithm/string/replace.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <tuple>
#include <llvm/Support/ErrorHandling.h>
#include <boost/dynamic_bitset.hpp>
#include "rate_math.hpp"

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
    partitionIntoFixedRateSubgraphs(G);


    identifySymbolicRates(G);
    identifyThreadLocalBuffers(G);
    computeDataFlow(G);

    const auto numOfSegments = std::max(mNumOfSegments, mNumOfThreads);
    const auto blockWidth = b->getBitBlockWidth();


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

            auto roundUpTo = [&](const Rational & num, const Rational & dom) {
                const Rational m = mod(num, dom);
                if (LLVM_UNLIKELY(m.numerator() != 0)) {
                    const auto r = (num - m) + dom;
                    assert (r.denominator() == 1);
                    return r.numerator();
                }
                assert (num.denominator() == 1);
                return num.numerator();
            };

            auto maxOf = [&](const Binding & output, const AttrId type, Rational & value) {
                if (LLVM_UNLIKELY(output.hasAttribute(type))) {
                    const auto & attr = output.findAttribute(type);
                    value = std::max(value, Rational{attr.amount(), blockWidth} );
                }
            };

            const Rational BLOCK_WIDTH{blockWidth};
            Rational requiredSpace{producerRate.MaximumSpace};

            Rational copyBackSpace{producerRate.Maximum - producerRate.Minimum};

            Rational requiredSizeFactor{BLOCK_WIDTH};
            if (producerRate.Maximum == producerRate.Minimum) {
                requiredSizeFactor = lcm(BLOCK_WIDTH, producerRate.Maximum);
            }

            Rational lookAheadSpace{0};
            Rational lookBehindSpace{0};
            maxOf(output, AttrId::LookBehind, lookBehindSpace);
            Rational reflectionSpace{0};
            maxOf(output, AttrId::Delayed, reflectionSpace);

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
            Rational minConsumptionSpace{producerRate.MinimumSpace};

            bool linear = requiresLinearAccess(output);
            for (const auto ce : make_iterator_range(out_edges(streamSet, G))) {
                const BufferRateData & consumerRate = G[ce];
                const Binding & input = consumerRate.Binding;
                requiredSpace = std::max(requiredSpace, producerRate.MaximumSpace);
                if (consumerRate.Maximum == consumerRate.Minimum) {
                    requiredSizeFactor = lcm(requiredSizeFactor, consumerRate.Maximum);
                }
                // Get output overflow size
                Rational lookAhead{consumerRate.Maximum - consumerRate.Minimum};
                if (LLVM_UNLIKELY(input.hasLookahead())) {
                    lookAhead += input.getLookahead();
                }
                lookAheadSpace = std::max(lookAheadSpace, lookAhead);

                minConsumptionSpace = std::min(minConsumptionSpace, consumerRate.MinimumSpace);

                // Are all symbolic consumption rates the same as its production rate?
                if (consumerRate.SymbolicRate != producerRate.SymbolicRate) {
                    dynamic = true;
                }
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                    dynamic = true;
                }

                if (LLVM_UNLIKELY(linear || requiresLinearAccess(input))) {
                    linear = true;
                }
                // If we have a lookbehind attribute, make sure we have enough underflow
                // space to satisfy the processing rate.
                maxOf(input, AttrId::LookBehind, lookBehindSpace);
            }

            lookBehindSpace = std::max(lookBehindSpace, reflectionSpace);

            // calculate overflow (copyback) and fascimile (copyforward) space


            const auto overflowSpace = std::max(copyBackSpace, lookAheadSpace);

            // If the minimum consumption space is less than the minimum production space,
            // there is an implicit delay in the dataflow that impede (or outright block)
            // progress once the buffer is full.

            // TODO: this requires a bit more work and analysis if we want to reduce
            // splitting a segment across two iterations.

            const auto delay = producerRate.MaximumSpace - minConsumptionSpace;
            if (LLVM_UNLIKELY(delay > overflowSpace)) {
                requiredSpace += delay - overflowSpace;
            }

            const auto overflowSize = roundUpTo(overflowSpace, BLOCK_WIDTH);
            const auto underflowSize = roundUpTo(lookBehindSpace, BLOCK_WIDTH);
            const Rational minRequiredSize{std::max(underflowSize, overflowSize) * 2};
            assert (requiredSizeFactor >= BLOCK_WIDTH);
            const auto requiredSize = roundUpTo(std::max(requiredSpace, minRequiredSize), requiredSizeFactor);
            const auto bufferSize = requiredSize * numOfSegments;

            Type * const baseType = output.getType();

            const auto itemWidth = getItemWidth(baseType);
            const Rational BLOCK_SIZE{blockWidth, itemWidth};

            bn.LookBehind = roundUpTo(lookBehindSpace, BLOCK_SIZE);
            bn.LookBehindReflection = roundUpTo(reflectionSpace, BLOCK_SIZE);
            bn.CopyBack = roundUpTo(copyBackSpace, BLOCK_SIZE);
            bn.LookAhead = roundUpTo(lookAheadSpace, BLOCK_SIZE);

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
        const Kernel * const kernel = mStreamGraph[i].Kernel; assert (kernel);

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
 * @brief partitionIntoFixedRateSubgraphs
 ** ------------------------------------------------------------------------------------------------------------- */
unsigned PipelineCompiler::partitionIntoFixedRateSubgraphs(BufferGraph & G) const {

    // LLVM BitVector does not have a built-in < comparator
    using BV = dynamic_bitset<>;
    using BVIds = std::map<const BV, unsigned>;
    using InEdgeIterator = graph_traits<BufferGraph>::in_edge_iterator;
    std::vector<BV> rateSet(LastStreamSet + 1);

    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    unsigned currentRateId = 0;

    for (auto kernel = first; kernel <= last; ++kernel) {

        BV & kernelRateSet = rateSet[kernel];

        // combine all incoming buffer rates sets
        const auto numOfInputs = in_degree(kernel, G);

        if (LLVM_LIKELY(numOfInputs != 0)) {
            InEdgeIterator ei, ei_end;
            std::tie(ei, ei_end) = in_edges(kernel, G);
            kernelRateSet = rateSet[source(*ei, G)];
            while (++ei != ei_end) {
                kernelRateSet |=  rateSet[source(*ei, G)];
            }
        }

        auto taintWithNewRate = [&]() {
            // A source kernel produces data without input thus is the root of a
            // new partition.
            if (numOfInputs == 0) {
                return true;
            }
            // If this kernel has any bounded or unknown input rates, there is a
            // potential change of rate of dataflow through the kernel.
            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

                switch (rate.getKind()) {
                    // countable
                    case RateId::Fixed:
                    case RateId::PartialSum:
                    case RateId::Greedy:
                        // If a countable rate has a deferred attribute, we cannot infer
                        // how many items will actually be consumed.
                        if (LLVM_UNLIKELY(binding.isDeferred())) {
                            return true;
                        }
                        break;
                    // non-countable
                    case RateId::Bounded:
                        return true;
                    default: break;
                }
            }
            return false;
        };

        auto addRateId = [](BV & bv, const unsigned rateId) {
            if (LLVM_UNLIKELY(rateId > bv.capacity())) {
                bv.resize(round_up_to(rateId, BV::bits_per_block));
            }
            bv.set(rateId);
        };

        if (LLVM_UNLIKELY(taintWithNewRate())) {
            addRateId(kernelRateSet, ++currentRateId);
        }


        // If this kernel is a PopCount kernel, then it will consume/produce data
        // at a fixed rate. However, its output will influence the number of items
        // consumed/produced by any of its descendents. Rather than reasoning about
        // it at the consumer node, simply mark its output buffer as having a new rate.

        const Kernel * const kernelObj = getKernel(kernel);

        unsigned popCountRateId = 0;
        if (LLVM_UNLIKELY(isa<PopCountKernel>(kernelObj))) {
            popCountRateId = ++currentRateId;
            // We need at least one source kernel for a popcount rate to exist
            assert (popCountRateId > 0);
        }

        SmallVector<unsigned, 32> newRateIds(out_degree(kernel, G), 0);
        bool hasRelativeRate = false;

        for (const auto e : make_iterator_range(out_edges(kernel, G))) {

            BV & outgoingRateSet = rateSet[target(e, G)];
            // inherit all of the kernel rates
            outgoingRateSet = kernelRateSet;

            if (LLVM_UNLIKELY(popCountRateId != 0)) {
                addRateId(outgoingRateSet, popCountRateId);
            }

            const BufferRateData & outputRate = G[e];
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            bool requiresNewRateId = false;

            switch (rate.getKind()) {
                case RateId::Bounded:
                case RateId::Unknown:
                    requiresNewRateId = true;
                    break;
                case RateId::Fixed:
                case RateId::PartialSum:
                    requiresNewRateId = binding.isDeferred();
                    break;
                case RateId::Relative:
                    hasRelativeRate = true;
                    break;
                default: break;
            }

            if (requiresNewRateId) {
                const auto newRateId = ++currentRateId;
                assert (newRateId > 0);
                newRateIds[outputRate.Port.Number] = newRateId;
                addRateId(outgoingRateSet, newRateId);
            }

        }

        if (LLVM_UNLIKELY(hasRelativeRate)) {
            for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[e];
                const Binding & binding = outputRate.Binding;
                const ProcessingRate & rate = binding.getRate();
                if (LLVM_UNLIKELY(rate.isRelative())) {
                    const StreamSetPort refPort = getReference(kernel, outputRate.Port);

                    // If an output rate is relative to the input rate, we can
                    // ignore it because a relative rate must refer to a variable
                    // rate and any kernel with a variable input rate is tainted
                    // with a new rateId.

                    if (refPort.Type == PortType::Output) {
                        const auto refRateId = newRateIds[refPort.Number];
                        if (refRateId) {
                            // if refRateId is 0 then it must be relative to a partialsum rate
                            // (or some other rate that is already accounted for)
                            BV & outgoingRateSet = rateSet[target(e, G)];
                            addRateId(outgoingRateSet, refRateId);
                        }
                    }
                }
            }
        }

    }

    // now that we've tainted the kernels with any influencing rate ids, convert each unique set
    // to a unique fixed number.

    BVIds partitionIds;
    unsigned nextPartitionId = 0;
    for (auto kernel = first; kernel <= last; ++kernel) {
        const BV & kernelRateSet = rateSet[kernel];
        auto f = partitionIds.find(kernelRateSet);
        unsigned partitionId;
        if (f == partitionIds.end()) {
            partitionId = ++nextPartitionId;
            partitionIds.emplace(kernelRateSet, partitionId);
        } else {
            partitionId = f->second;
        }
        BufferNode & kernelData = G[kernel];
        kernelData.PartitionId = partitionId;
    }

    return nextPartitionId; // total # of partitions
}


#ifdef USE_Z3

BufferRateData & getRateDataForPortNum(const unsigned kernel, const unsigned portNum, BufferGraph & G) {
    const auto numOfInputs = in_degree(kernel, G);
    BufferGraph::edge_descriptor e;
    if (portNum < numOfInputs) { // Input
        graph_traits<BufferGraph>::in_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = in_edges(kernel, G);
        e = *(ei + portNum);
    } else { // Output
        graph_traits<BufferGraph>::out_edge_iterator ei, ei_end;
        std::tie(ei, ei_end) = out_edges(kernel, G);
        e = *(ei + (portNum - numOfInputs));
    }
    BufferRateData & result = G[e];
    assert (result.Port ==
            StreamSetPort{
                portNum < numOfInputs ? PortType::Input : PortType::Output
                , portNum < numOfInputs ? portNum : portNum - numOfInputs });
    return result;
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifySymbolicRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifySymbolicRates(BufferGraph & G) const {

    // Identify the symbolic rates of each kernel, where Fixed rates inherit the
    // symbolic rate of the kernel, popcount/partial sum/relative rates share the
    // same rate when referencing the same source, and bounded/unknown rates are
    // given a new rate. When a kernel has inputs whose rate differ, the kernel is
    // assigned a new rate otherwise the kernel inherits the rate of its inputs.

    using ReferenceGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    using CommonPartialSumRateMap = flat_map<std::pair<unsigned, unsigned>, Expr>;

    CommonPartialSumRateMap P;


    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;


    // With the assumption that we have an unbounded input from any sources and infinite length buffers,
    // calculate the upper/lower bound for the number of items/strides we can process in a single segment.

    for (auto kernel = first; kernel <= last; ++kernel) {

        // Determine whether we have any relative or partialsum bindings.
        // Relative rates will be temporarily ignored and then filled in
        // after we determine the reference rates.


        const auto numOfInputs = in_degree(kernel, G);
        const auto numOfOutputs = out_degree(kernel, G);
        const Kernel * const kernelObj = getKernel(kernel);
        const auto strideSize = kernelObj->getStride();


#if 0

        for (const auto i : mPortEvaluationOrder) {




            const Binding & binding = (i < numOfInputs) ? getInputBinding(kernel, i) : getOutputBinding(kernel, i - numOfInputs);
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_UNLIKELY(rate.isRelative() || rate.isPartialSum())) {
                StreamSetPort port{(i < numOfInputs) ? PortType::Input : PortType::Output,  };
                if (i < numOfInputs) {
                    port.Type = PortType::Input;
                    port.Number = i;
                } else {
                    port.Type = PortType::Output;
                    port.Number = i - numOfInputs;
                }

                const StreamSetPort ref = getReference(kernel, port);
                auto H = rate.isRelative() ? R : S;
                add_edge(i, ref.Number, H);
            }
        }

#endif

        auto toPortIndex = [&](const StreamSetPort port) {
            if (port.Type == PortType::Input) {
                return port.Number;
            } else {
                return port.Number + numOfInputs;
            }
        };

        SmallVector<Expr, 32> symbolicRates(numOfInputs + numOfOutputs);

        auto calculateSymbolicRate = [&](const BufferRateData & ioRate) -> Expr {

            const Binding & binding = ioRate.Binding;
            const ProcessingRate & rate = binding.getRate();

            using RateId = ProcessingRate::KindId;

            Expr symRate = nullptr;

            // calculate base rate

            switch (rate.getKind()) {
                case RateId::Fixed:
                    symRate = constant(rate.getRate() * strideSize);
                    break;
                case RateId::Bounded:
                    symRate = bounded_variable(rate.getLowerBound() * strideSize, rate.getUpperBound() * strideSize);
                    break;
                case RateId::PartialSum:
                    BEGIN_SCOPED_REGION
                    const auto ref = getReferenceBufferVertex(kernel, ioRate.Port);
                    const auto key = std::make_pair(ref, refSymRate);
                    const auto f = P.find(key);
                    if (LLVM_LIKELY(f == P.end())) {
                        Expr const popCountRate = bounded_variable(constant(0), refSymRate);
                        P.emplace(key, popCountRate);
                        symRate = popCountRate;
                    } else {
                        symRate = f->second;
                    }
                    END_SCOPED_REGION
                    break;
                case RateId::Greedy:
                    assert (refSymRate);
                    symRate = refSymRate;
                    break;
                case RateId::Relative:
                    BEGIN_SCOPED_REGION
                    const StreamSetPort ref = getReference(ioRate.Port);
                    symRate = symbolicRates[toPortIndex(ref)];
                    END_SCOPED_REGION
                case RateId::Unknown:
                    symRate = free_variable();
                    break;
                default: llvm_unreachable("invalid or unhandled rate type");
            }

            // apply rate attributes

            bool isDeferred = false;
            unsigned sub_lb = 0;

            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Deferred:
                        // TODO: check if its necessary to flag this. if the lb is already
                        // zero then this is unnecessary.
                        break;
                    case AttrId::Delayed:
                        sub_lb += attr.amount();
                        break;
                    case AttrId::LookAhead:
                        sub_lb += attr.amount();
                        break;
                    default: break;
                }
            }

            if (LLVM_UNLIKELY(isDeferred || sub_lb)) {
                Expr lb = symRate;
                Expr ub = symRate;
                if (isDeferred) {
                    lb = constant(0);
                }
                if (sub_lb) {
                    lb = subtract(symRate, constant(sub_lb));
                }
                symRate = bounded_variable(lb, ub);
            }

            return symRate;
        };

        // Any kernel without an input must be a source kernel
        if (LLVM_UNLIKELY(numOfInputs == 0)) {

            for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[e];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number, R) != 0)) {
                    continue;
                }
                outputRate.SymbolicRate = calculateSymbolicRate(outputRate, nullptr);
            }

        } else { // non-source kernel

            /* mPortEvaluationOrder = */ determineEvaluationOrderOfKernelIO(kernel, G);

            // Identify the input rates

            ExprSet dataFlowRates;

            for (const auto i : mPortEvaluationOrder) {
                if (i < numOfInputs) {

                } else {

                }
            }


            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(inputRate.Port.Number, R) != 0)) {
                    continue;
                }

                const auto output = in_edge(source(input, G), G);
                assert (out_degree(inputRate.Port.Number, R) == 0);
                const BufferRateData & outputRate = G[output];
                auto sr = calculateSymbolicRate(inputRate, outputRate.SymbolicRate);
                inputRate.SymbolicRate = sr;
                symbolicRates[inputRate.Port.Number] = sr;

                // check if we already saw that num/denom pair?

                Expr relativeDataFlow = divide(outputRate.SymbolicRate, sr);
                dataFlowRates.insert(relativeDataFlow);
            }

            // Characterize the expected number of strides
            auto expected = bounded_variable(mk_min(dataFlowRates), mk_max(dataFlowRates));

            for (const auto e : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[e];
                // is this a relative rate?
                if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number, R) != 0)) {
                    continue;
                }

                auto symrate = calculateSymbolicRate(outputRate, outputRate);
                outputRate.SymbolicRate = symrate;
            }



        }



        // Correct the input rate to be relative to the newly computed symbolic rate.
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                inputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                inputRate.SymbolicRate = getReferenceSymbolicRate(inputRate, symbolicRate);
            }
        }

        // Determine the output rates
        for (const auto e : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[e];
            // is this a relative rate?
            if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number + numOfInputs, R) != 0)) {
                continue;
            }
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                outputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                outputRate.SymbolicRate = getReferenceSymbolicRate(outputRate, symbolicRate);
            } else { // if (rate.isBounded() || rate.isUnknown()) {
                outputRate.SymbolicRate = add_vertex(M);
                rateIsBoundedBy(outputRate.SymbolicRate, symbolicRate);
            }
        }


    }
}

#else

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifySymbolicRates
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifySymbolicRates(BufferGraph & G) const {


    // Identify the symbolic rates of each kernel, where Fixed rates inherit the
    // symbolic rate of the kernel, popcount/partial sum/relative rates share the
    // same rate when referencing the same source, and bounded/unknown rates are
    // given a new rate. When a kernel has inputs whose rate differ, the kernel is
    // assigned a new rate otherwise the kernel inherits the rate of its inputs.

    using SymbolicRateMinGraph = adjacency_list<hash_setS, vecS, directedS, unsigned>;
    using ReferenceGraph = adjacency_list<vecS, vecS, bidirectionalS>;

    using CommonPartialSumRateMap = flat_map<std::pair<unsigned, unsigned>, unsigned>;
//    using CommonDelayedRateMap = flat_map<std::pair<unsigned, unsigned>, unsigned>;
    using CommonFixedRateChangeMap = flat_map<std::tuple<Rational, unsigned, unsigned>, unsigned>;

    CommonPartialSumRateMap P;
    CommonFixedRateChangeMap F;
//    CommonDelayedRateMap D;
    SymbolicRateMinGraph M;

    flat_set<unsigned> rates;

    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    for (auto kernel = first; kernel <= last; ++kernel) {

        auto rateIsBoundedBy = [&](const unsigned a, const unsigned b) {
            // maintain a transitive closure of the rates
            add_edge(a, b, M);
            for (const auto e : make_iterator_range(out_edges(b, M))) {
                add_edge(a, target(e, M), M);
            }
        };

        auto getReferenceSymbolicRate = [&](const BufferRateData & bd, const unsigned refSymRate) -> unsigned {
            const auto ref = getReferenceBufferVertex(kernel, bd.Port);
            const auto key = std::make_pair(ref, refSymRate);
            const auto f = P.find(key);
            if (LLVM_LIKELY(f == P.end())) {
                const auto symbolicRate = add_vertex(M);
                rateIsBoundedBy(symbolicRate, refSymRate);
                P.emplace(key, symbolicRate);
                return symbolicRate;
            } else {
                return f->second;
            }
        };

//        auto getDelayedSymbolicRate = [&](const unsigned symRate, const unsigned delay) -> unsigned {
//            const auto key = std::make_pair(symRate, delay);
//            const auto f = D.find(key);
//            if (LLVM_LIKELY(f == D.end())) {
//                const auto symbolicRate = add_vertex(M);
//                rateIsBoundedBy(symbolicRate, symRate);
//                D.emplace(key, symbolicRate);
//                return symbolicRate;
//            } else {
//                return f->second;
//            }
//        };

        rates.clear();

        // Determine whether we have any relative bindings. These will be temporarily ignored and then
        // filled in after we determine the reference rates.

        const auto numOfInputs = in_degree(kernel, G);
        const auto numOfOutputs = out_degree(kernel, G);

        ReferenceGraph R(numOfInputs + numOfOutputs);

        for (const auto e : make_iterator_range(in_edges(kernel, G))) {
            const BufferRateData & bd = G[e];
            const Binding & input = bd.Binding;
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.isRelative())) {
                const StreamSetPort ref = getReference(bd.Port);
                assert ("input stream cannot refer to an output stream" && ref.Type == PortType::Input);
                add_edge(bd.Port.Number, ref.Number, R);
            }
        }
        for (const auto e : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & bd = G[e];
            const Binding & input = bd.Binding;
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.isRelative())) {
                StreamSetPort ref = getReference(bd.Port);
                if (LLVM_UNLIKELY(ref.Type == PortType::Output)) {
                    ref.Number += numOfInputs;
                }
                add_edge(bd.Port.Number + numOfInputs, ref.Number, R);
            }
        }

        // Identify the input rates
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            // is this a relative rate?
            if (LLVM_UNLIKELY(out_degree(inputRate.Port.Number, R) != 0)) {
                continue;
            }

            const auto output = in_edge(source(input, G), G);
            assert (out_degree(inputRate.Port.Number, R) == 0);
            const BufferRateData & outputRate = G[output];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & inputProcessingRate = binding.getRate();

            if (LLVM_LIKELY(inputProcessingRate.isFixed())) {
                // When going from a non-Fixed production rate to Fixed consumption rate
                // or streaming fixed rate data at "unaligned" rates, we are potentially
                // "truncating" the flow of information.

                auto compatibleFixedRates = [&]() {
                    if (LLVM_UNLIKELY(binding.hasLookahead())) {
                        return false;
                    }
                    if (outputRate.Minimum != outputRate.Maximum) {
                        return false;
                    }
                    const auto g = lcm(outputRate.Minimum, inputRate.Minimum);
                    const auto m = std::max(outputRate.Minimum, inputRate.Minimum);
                    if (LLVM_UNLIKELY(g != m)) {
                        return false;
                    }
                    return true;
                };

                // TODO: The truncated buffer need not need to be dynamic.
                if (compatibleFixedRates()) {
//                    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::Delayed))) {
//                        const auto & delay = binding.findAttribute(AttrId::Delayed);
//                        inputRate.SymbolicRate = getDelayedSymbolicRate(outputRate.SymbolicRate, delay.amount());
//                    } else {
                        inputRate.SymbolicRate = outputRate.SymbolicRate;
//                    }
                } else {
                    unsigned la = 0;
                    if (LLVM_UNLIKELY(binding.hasLookahead())) {
                        la = binding.getLookahead();
                    }
                    const auto key = std::make_tuple(inputRate.Maximum, outputRate.SymbolicRate, la);
                    const auto f = F.find(key);
                    if (f == F.end()) {
                        inputRate.SymbolicRate = add_vertex(M);
                        rateIsBoundedBy(inputRate.SymbolicRate, outputRate.SymbolicRate);
                        F.emplace(key, inputRate.SymbolicRate);
                    } else {
                        inputRate.SymbolicRate = f->second;
                        assert (inputRate.SymbolicRate != outputRate.SymbolicRate);
                    }
                }
            } else if (LLVM_UNLIKELY(inputProcessingRate.isGreedy())) {
                inputRate.SymbolicRate = outputRate.SymbolicRate;
            } else if (LLVM_UNLIKELY(inputProcessingRate.isPartialSum())) {
                inputRate.SymbolicRate = getReferenceSymbolicRate(inputRate, outputRate.SymbolicRate);
            } else if (inputProcessingRate.isBounded() || inputProcessingRate.isUnknown()) {
                inputRate.SymbolicRate = add_vertex(M);
                rateIsBoundedBy(inputRate.SymbolicRate, outputRate.SymbolicRate);
            } else {
                llvm_unreachable("unknown rate type");
            }
        }


        // Do we have a principal input stream?
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            if (binding.hasAttribute(AttrId::Principal)) {
                assert (out_degree(inputRate.Port.Number, R) == 0);
                rates.insert(inputRate.SymbolicRate);
                goto has_principal_rate;
            }
        }

        // No; just insert all of the possible rates
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            const BufferRateData & inputRate = G[input];
            rates.insert(inputRate.SymbolicRate);
        }

has_principal_rate:

        assert ("independent sources must have unique symbolic rates." && (in_degree(kernel, G) != 0 || rates.empty()));

        // Determine the symbolic rate of this kernel
        unsigned symbolicRate = 0;
        if (LLVM_UNLIKELY(rates.empty())) {
            symbolicRate = add_vertex(M);
        } else if (rates.size() == 1) {
            symbolicRate = *rates.begin();
        } else if (rates.size() > 1) {
            symbolicRate = [&]() -> unsigned {
                // If one of the rates dominates all other rates, we can inherit it.
                // Otherwise we'll have to create a new one.
                const auto begin = rates.begin();
                const auto end = rates.end();

                for (auto i = begin; i != end; ) {
                    const auto s = *i;
                    for (auto j = begin; j != i; ++j) {
                        if (!edge(s, *j, M).second) {
                            goto does_not_dominate;
                        }
                    }
                    for (auto j = i + 1; j != end; ++j) {
                        if (!edge(s, *j, M).second) {
                            goto does_not_dominate;
                        }
                    }
                    return s;
    does_not_dominate:
                    ++i;
                }
                const auto s = add_vertex(M);
                for (auto i = begin; i != end; ++i) {
                    rateIsBoundedBy(s, *i);
                }
                return s;
            }();
        }

        auto forceNewRate = [&]() {

            // Is it safe to ignore this?

//            const Kernel * const K = getKernel(kernel);
//            for (const Attribute & attr : K->getAttributes()) {
//                switch (attr.getKind()) {
//                    case AttrId::CanTerminateEarly:
//                    case AttrId::MustExplicitlyTerminate:
//                        return true;
//                    default: continue;
//                }
//            }
            return false;
        };

        if (LLVM_UNLIKELY(forceNewRate())) {
            const auto newSymbolicRate = add_vertex(M);
            rateIsBoundedBy(newSymbolicRate, symbolicRate);
            symbolicRate = newSymbolicRate;
        }

        // Correct the input rate to be relative to the newly computed symbolic rate.
        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            const Binding & binding = inputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                inputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                inputRate.SymbolicRate = getReferenceSymbolicRate(inputRate, symbolicRate);
            }
        }

        // Determine the output rates
        for (const auto e : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[e];
            // is this a relative rate?
            if (LLVM_UNLIKELY(out_degree(outputRate.Port.Number + numOfInputs, R) != 0)) {
                continue;
            }
            const Binding & binding = outputRate.Binding;
            const ProcessingRate & rate = binding.getRate();
            if (LLVM_LIKELY(rate.isFixed())) {
                outputRate.SymbolicRate = symbolicRate;
            } else if (LLVM_UNLIKELY(rate.isPartialSum())) {
                outputRate.SymbolicRate = getReferenceSymbolicRate(outputRate, symbolicRate);
            } else { // if (rate.isBounded() || rate.isUnknown()) {
                outputRate.SymbolicRate = add_vertex(M);
                rateIsBoundedBy(outputRate.SymbolicRate, symbolicRate);
            }
        }

        // Fill in any relative rates
        for (const auto e : make_iterator_range(edges(R))) {

            auto getBufferRate = [&](const unsigned v) -> BufferRateData & {
                BufferGraph::edge_descriptor f;
                if (v < numOfInputs) {
                    graph_traits<BufferGraph>::in_edge_iterator ei, ei_end;
                    std::tie(ei, ei_end) = in_edges(kernel, G);
                    assert (std::distance(ei, ei_end) < v);
                    f = *(ei + v);
                } else {
                    const unsigned w = v - numOfInputs;
                    graph_traits<BufferGraph>::out_edge_iterator ei, ei_end;
                    std::tie(ei, ei_end) = out_edges(kernel, G);
                    assert (std::distance(ei, ei_end) < w);
                    f = *(ei + w);
                }
                BufferRateData & bd = G[f];
                assert (bd.Port.Number == (v < numOfInputs ? v : (v - numOfInputs)));
                assert (bd.Port.Type == (v < numOfInputs ? PortType::Input : PortType::Output));
                return bd;
            };

            BufferRateData & relativePort = getBufferRate(source(e, R));
            const BufferRateData & referencePort = getBufferRate(target(e, R));
            relativePort.SymbolicRate = referencePort.SymbolicRate;
        }
    }
}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief identifyThreadLocalBuffers
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::identifyThreadLocalBuffers(BufferGraph & G) const {
    for (auto stream = FirstStreamSet; stream <= LastStreamSet; ++stream) {
        bool threadLocal = true;
        auto shares_data_across_thread = [](const Binding & binding) {
            for (const Attribute & attr : binding.getAttributes()) {
                switch (attr.getKind()) {
                    case AttrId::Deferred:
                    case AttrId::LookBehind:
                    case AttrId::LookAhead:
                        return true;
                    default: continue;
                }
            }
            return false;
        };
        const auto e = in_edge(stream, G);
        const BufferRateData & input = G[e];
        if (shares_data_across_thread(input.Binding)) {
            threadLocal = false;
        } else {
            const auto symRate = input.SymbolicRate;
            for (const auto e : make_iterator_range(out_edges(stream, G))) {
                const BufferRateData & output = G[e];
                if (output.SymbolicRate != symRate || shares_data_across_thread(output.Binding)) {
                    threadLocal = false;
                    break;
                }
            }
        }
        BufferNode & bn = G[stream];
        bn.ThreadLocal = threadLocal;
    }
}

#if 1

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDataFlow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlow(BufferGraph & G) const {

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    auto div_by_non_zero = [](const Rational & num, const Rational & denom) -> Rational {
        return  (denom.numerator() == 0) ? num : (num / denom);
    };

    auto saturating_subtract = [](const Rational & a, const Rational & b) -> Rational {
        if (a < b) {
            return Rational{0};
        } else {
            return a - b;
        }
    };

    auto check_attribute = [](const AttrId id, const BufferRateData & rateData, Rational & accum) {
        const Binding & binding = rateData.Binding;
        if (LLVM_UNLIKELY(binding.hasAttribute(id))) {
            const Rational & rv = rateData.Maximum;
            const auto k = binding.findAttribute(id).amount();
            Rational amount{k * rv.denominator(), rv.numerator()};
            accum = std::max(accum, amount);
        }
    };

    const auto MAX_RATE = std::numeric_limits<unsigned>::max();

    const auto first = out_degree(PipelineInput, G) == 0 ? FirstKernel : PipelineInput;
    const auto last = in_degree(PipelineOutput, G) == 0 ? LastKernel : PipelineOutput;

    // Compute how much data each kernel could process/produce per segment
    for (auto kernel = first; kernel <= last; ++kernel) {


        Rational lower_absolute{MAX_RATE};
        Rational upper_absolute{MAX_RATE};
        Rational lower_expected{MAX_RATE};
        Rational upper_expected{MAX_RATE};

        BufferNode & kn = G[kernel];
        if (LLVM_UNLIKELY(in_degree(kernel, G) == 0)) {
            // source kernels are executed only once per segment
            lower_absolute = Rational{1};
            upper_absolute = Rational{1};
            lower_expected = Rational{1};
            upper_expected = Rational{1};

        } else {

            // for each input ...
            for (const auto kernelInput : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[kernelInput];
                const auto inputBuffer = source(kernelInput, G);
                if (LLVM_UNLIKELY(inputRate.Maximum == Rational{0})) {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << getKernel(kernel)->getName() << "."
                        << inputRate.Binding.get().getName()
                        << " may not have an unknown input rate.";
                    report_fatal_error(msg.str());
                }

                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

                // for each producer of the input ...
                for (const auto producer : make_iterator_range(in_edges(inputBuffer, G))) {
                    const BufferRateData & outputRate = G[producer];
                    // If the input rate is greedy, base the calculations off the production rate
                    // not the consumption. Note outputRate.Maximum instead of inputRate.Minimum.
                    if (LLVM_UNLIKELY(rate.isGreedy())) {
                        const auto min_expected = outputRate.MinimumExpectedFlow / outputRate.Maximum;
                        lower_expected = std::min(lower_expected, min_expected);
                        const auto min_absolute = outputRate.MinimumSpace / outputRate.Maximum;
                        lower_absolute = std::min(lower_absolute, min_absolute);
                        const auto max_expected = div_by_non_zero(outputRate.MaximumExpectedFlow, outputRate.Maximum);
                        upper_expected = std::min(upper_expected, max_expected);
                        const auto max_absolute = div_by_non_zero(outputRate.MaximumSpace, outputRate.Maximum);
                        upper_absolute = std::min(upper_absolute, max_absolute);
                    } else {
                        const auto min_expected = outputRate.MinimumExpectedFlow / inputRate.Maximum;
                        lower_expected = std::min(lower_expected, min_expected);
                        const auto min_absolute = outputRate.MinimumSpace / inputRate.Maximum;
                        lower_absolute = std::min(lower_absolute, min_absolute);
                        const auto max_expected = div_by_non_zero(outputRate.MaximumExpectedFlow, inputRate.Minimum);
                        upper_expected = std::min(upper_expected, max_expected);
                        const auto max_absolute = div_by_non_zero(outputRate.MaximumSpace, inputRate.Minimum);
                        upper_absolute = std::min(upper_absolute, max_absolute);
                    }

                }

                // Fixed rate input streams may potentially truncate the input passed into them.
                // Make sure the truncation is taken into account when computing the bounds.
                if (LLVM_LIKELY(rate.isFixed())) {
                    const auto flower_expected = floor(lower_expected);
                    const auto dlower_expected = lower_expected - flower_expected;
                    lower_expected = flower_expected;

                    const auto flower_absolute = floor(lower_absolute);
                    const auto dlower_absolute = lower_absolute - flower_absolute;
                    lower_absolute = flower_absolute;

                    upper_expected = ceiling(upper_expected + dlower_expected);
                    upper_absolute = ceiling(upper_absolute + dlower_absolute);
                }

            }

            auto mod_one = [](const Rational & m) -> Rational {
                return Rational{m.numerator() % m.denominator(), m.denominator()};
            };

            auto compute2 = [div_by_non_zero, mod_one](const Rational & a, const Rational & b) {
                if (lcm(a, b) == a) {
                    return Rational{0};
                }
                return mod_one(div_by_non_zero(a, b));
            };

            auto compute = [compute2](const Rational & a, const Rational & b) {
                if (LLVM_LIKELY(a >= b)) {
                    return compute2(a, b);
                } else {
                    return compute2(b, a);
                }
            };

            Rational variance{0};

            auto update_variance = [&](const BufferRateData & A, const BufferRateData & B) {
                const auto a = compute(A.Maximum, B.Minimum);
                const auto b = compute(B.Maximum, A.Minimum);
                variance = std::max(variance, std::max(a, b));
            };

            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[input];
                const auto producer = in_edge(source(input, G), G);
                const BufferRateData & producerRate = G[producer];
                update_variance(producerRate, inputRate);
            }

            for (const auto output : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[output];
                for (const auto consumer : make_iterator_range(out_edges(target(output, G), G))) {
                    const BufferRateData & consumptionRate = G[consumer];
                    update_variance(consumptionRate, outputRate);
                }
            }

            lower_expected = saturating_subtract(lower_expected, variance);
            lower_absolute = saturating_subtract(lower_absolute, variance);

            upper_expected += variance;
            upper_absolute += variance;

            // since we cannot compute a partial stride, adjust the lower and upper
            // bound accordingly.

            upper_expected = ceiling(upper_expected + mod_one(lower_expected));
            lower_expected = floor(lower_expected);

            lower_absolute = std::min(lower_absolute, lower_expected);
            upper_absolute = std::max(upper_absolute, upper_expected);

            Rational lookAhead{0};
            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                check_attribute(AttrId::LookAhead, G[input], lookAhead);
            }
            lower_absolute = std::min(lower_expected, saturating_subtract(lower_absolute, lookAhead));

            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                inputRate.MinimumExpectedFlow = lower_expected * inputRate.Minimum;
                inputRate.MaximumExpectedFlow = upper_expected * inputRate.Maximum;
                inputRate.MinimumSpace = lower_absolute * inputRate.Minimum;
                Rational lookAhead{0};
                check_attribute(AttrId::LookAhead, inputRate, lookAhead);
                const auto f = upper_absolute + lookAhead;
                inputRate.MaximumSpace = f * inputRate.Maximum;
                assert (inputRate.MinimumSpace <= inputRate.MinimumExpectedFlow);
                assert (inputRate.MaximumSpace >= inputRate.MaximumExpectedFlow);
            }

        }

        kn.Lower = lower_absolute;
        kn.Upper = upper_absolute;

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[output];
            outputRate.MinimumExpectedFlow = lower_expected * outputRate.Minimum;
            outputRate.MaximumExpectedFlow = upper_expected * outputRate.Maximum;

            Rational add{0};
            check_attribute(AttrId::Add, outputRate, add);
            const auto buffer = target(output, G);
            Rational add2{0};
            for (const auto consumer_input : make_iterator_range(out_edges(buffer, G))) {
                BufferRateData & inputRate = G[consumer_input];
                check_attribute(AttrId::Add, inputRate, add2);
            }
            Rational roundUp{0};
            const Binding & binding = outputRate.Binding;
            if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::RoundUpTo))) {
                const auto & roundUpTo = binding.findAttribute(AttrId::RoundUpTo);
                Rational r(roundUpTo.amount());
                const auto a = mod(outputRate.Minimum, r);
                const auto b = mod(outputRate.Maximum + add, r);
                roundUp = std::max(roundUp, std::max(a, b));
            }
            outputRate.MinimumSpace = lower_absolute * outputRate.Minimum;
            Rational delayed{0};
            check_attribute(AttrId::Delayed, outputRate, delayed);
            outputRate.MinimumSpace = saturating_subtract(outputRate.MinimumSpace, delayed);

            outputRate.MaximumSpace = upper_absolute * outputRate.Maximum + std::max(add + add2, roundUp);
            assert (outputRate.MinimumSpace <= outputRate.MinimumExpectedFlow);
            assert (outputRate.MaximumSpace >= outputRate.MaximumExpectedFlow);
        }
    }

}

#else

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDataFlow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlow(BufferGraph & G) const {

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    auto div_by_non_zero = [](const Rational & num, const Rational & denom) -> Rational {
        return  (denom.numerator() == 0) ? num : (num / denom);
    };

    auto saturating_subtract = [](const Rational & a, const Rational & b) -> Rational {
        if (a < b) {
            return Rational{0};
        } else {
            return a - b;
        }
    };

    auto check_attribute = [](const AttrId id, const BufferRateData & rateData, Rational & accum) {
        const Binding & binding = rateData.Binding;
        if (LLVM_UNLIKELY(binding.hasAttribute(id))) {
            const Rational & rv = rateData.Maximum;
            const auto k = binding.findAttribute(id).amount();
            Rational amount{k * rv.denominator(), rv.numerator()};
            accum = std::max(accum, amount);
        }
    };

    auto clamp = [](const Rational & n) -> Rational {
        return std::max(n, Rational{0});
    };

    const auto MAX_RATE = std::numeric_limits<unsigned>::max();

    // Compute how many segments each kernel can process/produce
    for (unsigned kernel = FirstKernel; kernel <= LastKernel; ++kernel) {


        Rational lower_expected{MAX_RATE};
        Rational upper_expected{MAX_RATE};

        BufferNode & K = G[kernel];
        if (LLVM_UNLIKELY(in_degree(kernel, G) == 0)) {
            // source kernels are executed only once per segment
            lower_expected = Rational{1};
            upper_expected = Rational{1};

        } else {

            // for each input ...
            for (const auto kernelInput : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[kernelInput];
                const auto inputBuffer = source(kernelInput, G);

                if (LLVM_UNLIKELY(inputRate.Maximum == Rational{0})) {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << getKernel(kernel)->getName() << "."
                        << inputRate.Binding.get().getName()
                        << " may not have an unknown input rate.";
                    report_fatal_error(msg.str());
                }

                const Binding & binding = inputRate.Binding;
                const ProcessingRate & rate = binding.getRate();

                // for each producer of the input ...
                for (const auto producer : make_iterator_range(in_edges(inputBuffer, G))) {
                    const BufferRateData & outputRate = G[producer];
                    // If the input rate is greedy, base the calculations off the production rate
                    // not the consumption. Note outputRate.Maximum instead of inputRate.Minimum.
                    if (LLVM_UNLIKELY(rate.isGreedy())) {
                        const auto min_expected = outputRate.MinimumExpectedFlow / outputRate.Maximum;
                        lower_expected = std::min(lower_expected, min_expected);
                        const auto max_expected = div_by_non_zero(outputRate.MaximumExpectedFlow, outputRate.Maximum);
                        upper_expected = std::min(upper_expected, max_expected);
                    } else {
                        const auto min_expected = outputRate.MinimumExpectedFlow / inputRate.Maximum;
                        lower_expected = std::min(lower_expected, min_expected);
                        const auto max_expected = div_by_non_zero(outputRate.MaximumExpectedFlow, inputRate.Minimum);
                        upper_expected = std::min(upper_expected, max_expected);
                    }
                }

                Rational lookAhead{0};
                check_attribute(AttrId::LookAhead, inputRate, lookAhead);
                lower_expected = clamp(saturating_subtract(lower_expected, lookAhead));

                // Fixed rate input streams may potentially truncate the input passed into them.
                // Make sure the truncation is taken into account when computing the bounds.
                if (LLVM_LIKELY(rate.isFixed())) {
                    const auto flower_expected = floor(lower_expected);
                    const auto dlower_expected = lower_expected - flower_expected;
                    lower_expected = flower_expected;
                    upper_expected = ceiling(upper_expected + dlower_expected);
                }

            }

            auto mod_one = [](const Rational & m) -> Rational {
                return Rational{m.numerator() % m.denominator(), m.denominator()};
            };

            auto compute2 = [div_by_non_zero, mod_one](const Rational & a, const Rational & b) {
                if (lcm(a, b) == a) {
                    return Rational{0};
                }
                return mod_one(div_by_non_zero(a, b));
            };

            auto compute = [compute2](const Rational & a, const Rational & b) {
                if (LLVM_LIKELY(a >= b)) {
                    return compute2(a, b);
                } else {
                    return compute2(b, a);
                }
            };

            Rational variance{0};

            auto update_variance = [&](const BufferRateData & A, const BufferRateData & B) {
                const auto a = compute(A.Maximum, B.Minimum);
                const auto b = compute(B.Maximum, A.Minimum);
                variance = std::max(variance, std::max(a, b));
            };

            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[input];
                const auto producer = in_edge(source(input, G), G);
                const BufferRateData & producerRate = G[producer];
                update_variance(producerRate, inputRate);
            }

            for (const auto output : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[output];
                for (const auto consumer : make_iterator_range(out_edges(target(output, G), G))) {
                    const BufferRateData & consumptionRate = G[consumer];
                    update_variance(consumptionRate, outputRate);
                }
            }

            lower_expected = saturating_subtract(lower_expected, variance);

            upper_expected += variance;

            // since we cannot compute a partial stride, adjust the lower and upper
            // bound accordingly.

            upper_expected = ceiling(upper_expected + mod_one(lower_expected));
            lower_expected = floor(lower_expected);

            for (const auto input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                inputRate.MinimumExpectedFlow = lower_expected * inputRate.Minimum;
                inputRate.MaximumExpectedFlow = upper_expected * inputRate.Maximum;
            }

        }

        K.Lower = lower_expected;
        K.Upper = upper_expected;

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[output];
            outputRate.MinimumExpectedFlow = lower_expected * outputRate.Minimum;
            outputRate.MaximumExpectedFlow = upper_expected * outputRate.Maximum;
        }
    }

    #ifdef PRINT_BUFFER_GRAPH
    printBufferGraph(G, errs());
    #endif

    // Determine how much data may run through this straem in one full pipeline loop
    for (unsigned streamSet = LastStreamSet; streamSet >= FirstStreamSet; --streamSet) {

        Rational minInput{MAX_RATE};
        Rational maxInput{0};

        for (const auto port : make_iterator_range(out_edges(streamSet, G))) {
            const BufferRateData & p = G[port];
            const auto consumer =  target(port, G);
            const BufferNode & C = G[consumer];
            Rational lookAhead{0};
            check_attribute(AttrId::LookAhead, p, lookAhead);
            minInput = std::min(minInput, clamp((p.Minimum - lookAhead) * C.Lower));
            maxInput = std::max(maxInput, p.Maximum * C.Upper);
        }

        const auto port = in_edge(streamSet, G);
        const BufferRateData & p = G[port];
        const auto producer = source(port, G);
        const BufferNode & P = G[producer];

        Rational delayed{0};
        check_attribute(AttrId::Delayed, p, delayed);
        const auto minOutput = clamp((p.Minimum - delayed) * P.Lower);
        const auto maxOutput = p.Maximum * P.Upper;

        const auto A = div_by_non_zero(minOutput, maxInput);
        const auto B = div_by_non_zero(minInput, maxOutput);

        BufferNode & S = G[streamSet];
        S.Lower = std::min(A, B);
        S.Upper = std::max(A, B);
    }

    // Compute how much data each kernel could process/produce per segment
    for (unsigned kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        BufferNode & K = G[kernel];

        Rational min{MAX_RATE};

        for (const auto port : make_iterator_range(out_edges(kernel, G))) {
            const BufferNode & S = G[target(port, G)];
            min = std::min(min, S.Lower);
        }
        K.Lower *= min;

        for (const auto input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            inputRate.MinimumExpectedFlow = K.Lower * inputRate.Minimum;
            inputRate.MaximumExpectedFlow = K.Upper * inputRate.Maximum;
        }

        for (const auto output : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[output];
            outputRate.MinimumExpectedFlow = K.Lower * outputRate.Minimum;
            outputRate.MaximumExpectedFlow = K.Upper * outputRate.Maximum;
        }
    }


    // Determine how much data may run through this straem in one full pipeline loop
    for (unsigned streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        const auto port = in_edge(streamSet, G);
        const BufferRateData & output = G[port];
        Rational add{0};
        check_attribute(AttrId::Add, output, add);
        Rational roundUp{0};
        const Binding & binding = output.Binding;
        if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::RoundUpTo))) {
            const auto & roundUpTo = binding.findAttribute(AttrId::RoundUpTo);
            Rational r(roundUpTo.amount());
            const auto a = mod(output.Minimum, r);
            const auto b = mod(output.Maximum + add, r);
            roundUp = std::max(roundUp, std::max(a, b));
        }
        Rational delayed{0};
        check_attribute(AttrId::Delayed, output, delayed);
        auto overflow = std::max(add, roundUp) + delayed;
        auto requiredSpace = output.MaximumExpectedFlow + overflow;
        for (const auto consumer_input : make_iterator_range(out_edges(streamSet, G))) {
            BufferRateData & input = G[consumer_input];
            Rational add2{0};
            check_attribute(AttrId::Add, input, add2);
            Rational lookAhead{0};
            check_attribute(AttrId::LookAhead, input, lookAhead);
            const auto overflow = std::max(add + add2, lookAhead);
            requiredSpace = std::max(requiredSpace, input.MaximumExpectedFlow + overflow);
        }
        BufferNode & S = G[streamSet];
        S.RequiredSpace = ceiling(requiredSpace);
    }

    #ifdef PRINT_BUFFER_GRAPH
    printBufferGraph(G, errs());
    #endif

}

#endif

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) const {

    using KindId = StreamSetBuffer::BufferKind;

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

    const BufferNode & sn = G[PipelineInput];
    out << "digraph \"" << mTarget->getName() << "\" {\n"
           "v" << PipelineInput << " [label=\"[" <<
           PipelineInput << "] P_{in}\\n"
           " Partition: " << sn.PartitionId << "\\n";
           rate_range(sn.Lower, sn.Upper);
    out << "\" shape=rect, style=rounded, peripheries=2];\n";

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");
        const BufferNode & bn = G[i];
        out << "v" << i <<
               " [label=\"[" << i << "] " << name << "\\n"
               " Partition: " << bn.PartitionId << "\\n";
        rate_range(bn.Lower, bn.Upper);

        out << "\" shape=rect, style=rounded, peripheries=2];\n";
    }

    const BufferNode & tn = G[PipelineInput];

    out << "v" << PipelineOutput << " [label=\"[" <<
           PipelineOutput << "] P_{out}\\n"
           " Partition: " << tn.PartitionId << "\\n";
           rate_range(tn.Lower, tn.Upper);
    out << "\" shape=rect, style=rounded, peripheries=2];\n";

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(G);

    for (unsigned i = firstBuffer; i != lastBuffer; ++i) {
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
                        case KindId::StaticBuffer:
                            bufferType = 'S'; break;
                        case KindId::DynamicBuffer:
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

        if (buffer && buffer->getBufferKind() != KindId::ExternalBuffer) {
            switch (buffer->getBufferKind()) {
                case KindId::StaticBuffer:
                    out << cast<StaticBuffer>(buffer)->getCapacity();
                    break;
                case KindId::DynamicBuffer:
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
        out << " {" << pd.SymbolicRate << "}";
        const Binding & binding = pd.Binding;
        if (binding.hasAttribute(AttrId::Principal)) {
            out << " [P]";
        }
        if (binding.hasAttribute(AttrId::ZeroExtended)) {
            out << " [Z]";
        }
        out << "\\n(EXP:";
        rate_range(pd.MinimumExpectedFlow, pd.MaximumExpectedFlow);
        out << ")\\n(SPC:";
        rate_range(pd.MinimumSpace, pd.MaximumSpace);
        std::string name = binding.getName();
        boost::replace_all(name, "\"", "\\\"");
        out << ")\\n" << name << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}


}

#endif
