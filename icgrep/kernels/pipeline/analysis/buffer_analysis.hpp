#ifndef BUFFER_ANALYSIS_HPP
#define BUFFER_ANALYSIS_HPP

#include <kernels/pipeline/pipeline_compiler.hpp>
#include <boost/algorithm/string/replace.hpp>
#include <boost/interprocess/mapped_region.hpp>
#include <tuple>

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


inline RateValue mod(const RateValue & a, const RateValue & b) {
    RateValue n(a.numerator() * b.denominator(), b.numerator() * a.denominator());
    return a - RateValue{floor(n)} * b;
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

    BufferGraph G(LastStreamSet + 1);

    initializeBufferGraph(G);
    identifySymbolicRates(G);
    identifyThreadLocalBuffers(G);
    computeDataFlow(G);

    // fill in any known pipeline I/O buffers
    for (const auto e : make_iterator_range(out_edges(PipelineInput, G))) {
        const auto bufferVertex = target(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getInputStreamSetBuffer(G[e].inputPort());
        bn.Type = BufferType::External;
    }

    for (const auto e : make_iterator_range(in_edges(PipelineOutput, G))) {
        const auto bufferVertex = source(e, G);
        BufferNode & bn = G[bufferVertex];
        assert (bn.Buffer == nullptr);
        bn.Buffer = mPipelineKernel->getOutputStreamSetBuffer(G[e].outputPort());
        bn.Type = BufferType::External;
    }

    assert (codegen::BufferSegments > 0);
    assert (codegen::ThreadNum > 0);

    const auto requiredThreadSegments = ((codegen::ThreadNum > 1U) ? codegen::ThreadNum + 1U : 1U);
    const auto numOfSegments = std::max(codegen::BufferSegments, requiredThreadSegments);
    const auto blockWidth = b->getBitBlockWidth();

    // then construct the rest
    for (unsigned i = FirstStreamSet; i <= LastStreamSet; ++i) {

        BufferNode & bn = G[i];
        const auto pe = in_edge(i, G);
        const auto producerVertex = source(pe, G);
        const Kernel * const producer = getKernel(producerVertex);
        const BufferRateData & producerRate = G[pe];
        const Binding & output = producerRate.Binding;

        // Verify all consumers expect the same input type as the produced output type.
        Type * const baseType = output.getType();
        for (const auto & e : make_iterator_range(out_edges(i, G))) {
            const BufferRateData & consumerRate = G[e];
            const Binding & input = consumerRate.Binding;
            if (LLVM_UNLIKELY(baseType != input.getType())) {
                std::string tmp;
                raw_string_ostream msg(tmp);
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
        const auto itemWidth = getItemWidth(baseType);

        // Is this a pipeline I/O buffer?
        assert ((bn.Buffer == nullptr) ^ (bn.Type == BufferType::External));
        if (bn.Type == BufferType::External) {
            continue;
        }

        const ProcessingRate & rate = output.getRate();
        const auto isUnknown = rate.isUnknown();
        const auto isManaged = output.hasAttribute(AttrId::ManagedBuffer);

        StreamSetBuffer * buffer = nullptr;
        BufferType bufferType = BufferType::Internal;



        if (LLVM_UNLIKELY(isUnknown || isManaged)) {
            const auto linear = output.hasAttribute(AttrId::Linear);
            buffer = new ExternalBuffer(b, baseType, linear, 0);
            bufferType = BufferType::Managed;

            if (!linear) {
                for (const auto & ce : make_iterator_range(out_edges(i, G))) {
                    const Binding & input = G[ce].Binding;
                    if (LLVM_UNLIKELY(requiresLinearAccess(input))) {
                        SmallVector<char, 0> tmp;
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

            // TODO: verify no buffer requires this to be linear?
        } else {

            RateValue rounding{producerRate.Maximum};
            RateValue requiredSpace{producerRate.MaximumExpectedFlow};
            RateValue copyBackSpace{producerRate.Maximum - producerRate.Minimum};
            RateValue lookAheadSpace{0};

            RateValue underflowSpace{0};
            RateValue overflowSpace{producerRate.MaximumSpace - producerRate.MaximumExpectedFlow};

            if (LLVM_UNLIKELY(output.hasAttribute(AttrId::LookBehind))) {
                const auto & lookBehind = output.findAttribute(AttrId::LookBehind);
                const auto amount = lookBehind.amount();
                underflowSpace = RateValue{itemWidth * amount, blockWidth};
            }

            // TODO: If we have an open system, then the input rate to this pipeline cannot
            // be bounded a priori. During initialization, we could pass a "suggestion"
            // argument to indicate what the outer pipeline believes its I/O rates will be.

            bool dynamic = output.hasAttribute(AttrId::Deferred);

            bool linear = requiresLinearAccess(output);
            for (const auto ce : make_iterator_range(out_edges(i, G))) {
                const BufferRateData & consumerRate = G[ce];
                const Binding & input = consumerRate.Binding;

                rounding = lcm(rounding, consumerRate.Maximum);
                const auto S = producerRate.MaximumExpectedFlow + consumerRate.MaximumExpectedFlow;
                const auto D = gcd(producerRate.MaximumExpectedFlow, consumerRate.MaximumExpectedFlow);
                requiredSpace = std::max(requiredSpace, S - D);

                const auto diff = consumerRate.MaximumSpace - consumerRate.MaximumExpectedFlow;
                overflowSpace = std::max(overflowSpace, diff);

                // Get output overflow size
                RateValue lookAhead{consumerRate.Maximum - consumerRate.Minimum};
                if (LLVM_UNLIKELY(input.hasLookahead())) {
                    lookAhead += input.getLookahead();
                }
                lookAheadSpace = std::max(lookAheadSpace, lookAhead);

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
                if (LLVM_UNLIKELY(input.hasAttribute(AttrId::LookBehind))) {
                    const auto & lookBehind = input.findAttribute(AttrId::LookBehind);
                    const auto amount = lookBehind.amount();
                    RateValue u{itemWidth * amount, blockWidth};
                    underflowSpace = std::max(underflowSpace, u);
                }
            }

            // calculate overflow (copyback) and fascimile (copyforward) space
            rounding = lcm(rounding, RateValue{blockWidth});

            auto round_up = [&](RateValue & r) {
                const RateValue m = mod(r, rounding);
                if (LLVM_UNLIKELY(m.numerator() != 0)) {
                    r = (r - m) + rounding;
                }
            };

            round_up(copyBackSpace);
            overflowSpace = std::max(overflowSpace, copyBackSpace);
            round_up(lookAheadSpace);
            overflowSpace = std::max(overflowSpace, lookAheadSpace);
            round_up(overflowSpace);
            const auto overflowSize = ceiling(overflowSpace);
            round_up(underflowSpace);
            const auto underflowSize = ceiling(underflowSpace);
            const auto minRequiredSize = std::max(underflowSize, overflowSize) * 2;
            round_up(requiredSpace);
            const auto requiredSize = std::max(ceiling(requiredSpace), minRequiredSize);
            const auto bufferSize = requiredSize * numOfSegments;

            bn.LookBehind = underflowSize;
            bn.CopyBack = ceiling(copyBackSpace);
            bn.LookAhead = ceiling(lookAheadSpace);

            const unsigned addressSpace = 0U;
            // A DynamicBuffer is necessary when we cannot bound the amount of unconsumed data a priori.
            if (dynamic) {
                buffer = new DynamicBuffer(b, baseType, bufferSize, overflowSize, underflowSize, linear, addressSpace);
            } else {
                buffer = new StaticBuffer(b, baseType, bufferSize, overflowSize, underflowSize, linear, addressSpace);
            }
        }

        bn.Buffer = buffer;
        bn.Type = bufferType;
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
            RateValue lb{rate.getLowerBound()};
            RateValue ub{rate.getUpperBound()};
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

                auto strideLength = kernel->getStride();
                if (LLVM_UNLIKELY(i == PipelineInput || i == PipelineOutput)) {
                    strideLength = boost::lcm(codegen::SegmentSize, strideLength);
                }

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
        for (const auto & e : make_iterator_range(in_edges(i, mStreamGraph))) {
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
        for (const auto & e : make_iterator_range(out_edges(i, mStreamGraph))) {
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
    using CommonFixedRateChangeMap = flat_map<std::tuple<RateValue, unsigned, unsigned>, unsigned>;

    CommonPartialSumRateMap P;
    CommonFixedRateChangeMap F;
    SymbolicRateMinGraph M;

    flat_set<unsigned> rates;

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        auto rateIsBoundedBy = [&](const unsigned a, const unsigned b) {
            // maintain a transitive closure of the rates
            add_edge(a, b, M);
            for (const auto & e : make_iterator_range(out_edges(b, M))) {
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

        rates.clear();

        // Determine whether we have any relative bindings. These will be temporarily ignored and then
        // filled in after we determine the reference rates.

        const auto numOfInputs = in_degree(kernel, G);
        const auto numOfOutputs = out_degree(kernel, G);

        ReferenceGraph R(numOfInputs + numOfOutputs);

        for (const auto & e : make_iterator_range(in_edges(kernel, G))) {
            const BufferRateData & bd = G[e];
            const Binding & input = bd.Binding;
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.isRelative())) {
                const StreamPort ref = getReference(bd.Port);
                assert ("input stream cannot refer to an output stream" && ref.Type == PortType::Input);
                add_edge(bd.Port.Number, ref.Number, R);
            }
        }
        for (const auto & e : make_iterator_range(out_edges(kernel, G))) {
            const BufferRateData & bd = G[e];
            const Binding & input = bd.Binding;
            const ProcessingRate & rate = input.getRate();
            if (LLVM_UNLIKELY(rate.isRelative())) {
                StreamPort ref = getReference(bd.Port);
                if (LLVM_UNLIKELY(ref.Type == PortType::Output)) {
                    ref.Number += numOfInputs;
                }
                add_edge(bd.Port.Number + numOfInputs, ref.Number, R);
            }
        }

        // Identify the input rates
        for (const auto & input : make_iterator_range(in_edges(kernel, G))) {
            BufferRateData & inputRate = G[input];
            // is this a relative rate?
            if (LLVM_UNLIKELY(out_degree(inputRate.Port.Number, R) != 0)) {
                continue;
            }
            const auto output = in_edge(source(input, G), G);
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
                    inputRate.SymbolicRate = outputRate.SymbolicRate;
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
            rates.insert(inputRate.SymbolicRate);
        }

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
        for (const auto & input : make_iterator_range(in_edges(kernel, G))) {
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
        for (const auto & e : make_iterator_range(out_edges(kernel, G))) {
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
        for (const auto & e : make_iterator_range(edges(R))) {

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
        const auto & e = in_edge(stream, G);
        const BufferRateData & input = G[e];
        if (shares_data_across_thread(input.Binding)) {
            threadLocal = false;
        } else {
            const auto symRate = input.SymbolicRate;
            for (const auto & e : make_iterator_range(out_edges(stream, G))) {
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

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeDataFlow
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::computeDataFlow(BufferGraph & G) const {

    // Since we do not want to create an artifical bottleneck by constructing output buffers that
    // cannot accommodate the full amount of data we could produce given the expected inputs, the
    // next loop will resize them accordingly.

    auto div_by_non_zero = [](const RateValue & num, const RateValue & denom) -> RateValue {
        return  (denom.numerator() == 0) ? num : (num / denom);
    };

    auto saturating_subtract = [](const RateValue & a, const RateValue & b) -> RateValue {
        if (a < b) {
            return RateValue{0};
        } else {
            return a - b;
        }
    };

    auto check_attribute = [](const AttrId id, const BufferRateData & rateData, RateValue & accum) {
        const Binding & binding = rateData.Binding;
        if (LLVM_UNLIKELY(binding.hasAttribute(id))) {
            const RateValue & rv = rateData.Maximum;
            const auto k = binding.findAttribute(id).amount();
            RateValue amount{k * rv.denominator(), rv.numerator()};
            accum = std::max(accum, amount);
        }
    };

    const auto MAX_RATE = std::numeric_limits<unsigned>::max();

    // Compute how much data each kernel could process/produce per segment
    for (unsigned kernel = FirstKernel; kernel <= LastKernel; ++kernel) {


        RateValue lower_absolute{MAX_RATE};
        RateValue upper_absolute{MAX_RATE};
        RateValue lower_expected{MAX_RATE};
        RateValue upper_expected{MAX_RATE};

        BufferNode & kn = G[kernel];
        if (LLVM_UNLIKELY(in_degree(kernel, G) == 0)) {
            // source kernels are executed only once per segment
            lower_absolute = RateValue{1};
            upper_absolute = RateValue{1};
            lower_expected = RateValue{1};
            upper_expected = RateValue{1};

        } else {

            // for each input ...
            for (const auto & kernelInput : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[kernelInput];
                const auto inputBuffer = source(kernelInput, G);
                if (LLVM_UNLIKELY(inputRate.Maximum == RateValue{0})) {
                    std::string tmp;
                    raw_string_ostream msg(tmp);
                    msg << getKernel(kernel)->getName() << "."
                        << inputRate.Binding.get().getName()
                        << " may not have an unknown input rate.";
                    report_fatal_error(msg.str());
                }

                // for each producer of the input ...
                for (const auto & producer : make_iterator_range(in_edges(inputBuffer, G))) {
                    const BufferRateData & outputRate = G[producer];
                    const Binding & binding = inputRate.Binding;
                    const ProcessingRate & rate = binding.getRate();
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
            }

            auto mod_one = [](const RateValue & m) -> RateValue {
                return RateValue{m.numerator() % m.denominator(), m.denominator()};
            };

            auto compute2 = [div_by_non_zero, mod_one](const RateValue & a, const RateValue & b) {
                if (lcm(a, b) == a) {
                    return RateValue{0};
                }
                return mod_one(div_by_non_zero(a, b));
            };

            auto compute = [compute2](const RateValue & a, const RateValue & b) {
                if (LLVM_LIKELY(a >= b)) {
                    return compute2(a, b);
                } else {
                    return compute2(b, a);
                }
            };

            RateValue variance{0};

            auto update_variance = [&](const BufferRateData & A, const BufferRateData & B) {
                const auto a = compute(A.Maximum, B.Minimum);
                const auto b = compute(B.Maximum, A.Minimum);
                variance = std::max(variance, std::max(a, b));
            };

            for (const auto & input : make_iterator_range(in_edges(kernel, G))) {
                const BufferRateData & inputRate = G[input];
                const auto producer = in_edge(source(input, G), G);
                const BufferRateData & producerRate = G[producer];
                update_variance(producerRate, inputRate);
            }

            for (const auto & output : make_iterator_range(out_edges(kernel, G))) {
                const BufferRateData & outputRate = G[output];
                for (const auto & consumer : make_iterator_range(out_edges(target(output, G), G))) {
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

            RateValue lookAhead{0};
            for (const auto & input : make_iterator_range(in_edges(kernel, G))) {
                check_attribute(AttrId::LookAhead, G[input], lookAhead);
            }
            lower_absolute = std::min(lower_expected, saturating_subtract(lower_absolute, lookAhead));

            for (const auto & input : make_iterator_range(in_edges(kernel, G))) {
                BufferRateData & inputRate = G[input];
                inputRate.MinimumExpectedFlow = lower_expected * inputRate.Minimum;
                inputRate.MaximumExpectedFlow = upper_expected * inputRate.Maximum;
                inputRate.MinimumSpace = lower_absolute * inputRate.Minimum;
                RateValue lookAhead{0};
                check_attribute(AttrId::LookAhead, inputRate, lookAhead);
                const auto f = upper_absolute + lookAhead;
                inputRate.MaximumSpace = f * inputRate.Maximum;
                assert (inputRate.MinimumSpace <= inputRate.MinimumExpectedFlow);
                assert (inputRate.MaximumSpace >= inputRate.MaximumExpectedFlow);
            }

        }

        kn.Lower = lower_absolute;
        kn.Upper = upper_absolute;

        for (const auto & output : make_iterator_range(out_edges(kernel, G))) {
            BufferRateData & outputRate = G[output];
            outputRate.MinimumExpectedFlow = lower_expected * outputRate.Minimum;
            outputRate.MaximumExpectedFlow = upper_expected * outputRate.Maximum;
            RateValue add{0};
            check_attribute(AttrId::Add, outputRate, add);
            RateValue roundUp{0};
            const Binding & binding = outputRate.Binding;
            if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::RoundUpTo))) {
                const auto & roundUpTo = binding.findAttribute(AttrId::RoundUpTo);
                RateValue r(roundUpTo.amount());
                const auto a = mod(outputRate.Minimum, r);
                const auto b = mod(outputRate.Maximum + add, r);
                roundUp = std::max(roundUp, std::max(a, b));
            }
            outputRate.MinimumSpace = lower_absolute * outputRate.Minimum;
            const auto f = upper_absolute + std::max(add, roundUp);
            outputRate.MaximumSpace = f * outputRate.Maximum;
            assert (outputRate.MinimumSpace <= outputRate.MinimumExpectedFlow);
            assert (outputRate.MaximumSpace >= outputRate.MaximumExpectedFlow);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief printBufferGraph
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::printBufferGraph(const BufferGraph & G, raw_ostream & out) const {

    using KindId = StreamSetBuffer::BufferKind;

    auto print_rational = [&out](const RateValue & r) {
        if (r.denominator() > 1) {
            const auto n = r.numerator() / r.denominator();
            const auto p = r.numerator() % r.denominator();
            out << n << '+' << p << '/' << r.denominator();
        } else {
            out << r.numerator();
        }
    };

    auto rate_range = [&out, print_rational](const RateValue & a, const RateValue & b) {
        print_rational(a);
        out << " - ";
        print_rational(b);
    };

    const BufferNode & sn = G[PipelineInput];
    out << "digraph G {\n"
           "v" << PipelineInput << " [label=\"[" <<
           PipelineInput << "] P_{in}\\n";
           rate_range(sn.Lower, sn.Upper);
    out << "\" shape=rect, style=rounded, peripheries=2];\n";

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        std::string name = kernel->getName();
        boost::replace_all(name, "\"", "\\\"");
        const BufferNode & bn = G[i];
        out << "v" << i <<
               " [label=\"[" << i << "] " << name << "\\n";
        rate_range(bn.Lower, bn.Upper);
        out << "\" shape=rect, style=rounded, peripheries=2];\n";
    }

    const BufferNode & tn = G[PipelineInput];

    out << "v" << PipelineOutput << " [label=\"[" <<
           PipelineOutput << "] P_{out}\\n";
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
            switch (buffer->getBufferKind()) {
                case KindId::ExternalBuffer: bufferType = 'E'; break;
                case KindId::StaticBuffer: bufferType = 'S'; break;
                case KindId::DynamicBuffer: bufferType = 'D'; break;
                default: llvm_unreachable("unknown buffer type");
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
        out << " {" << pd.SymbolicRate << "}"
               "\\n(";
        rate_range(pd.MinimumExpectedFlow, pd.MaximumExpectedFlow);

        out << ")\\n(";
        rate_range(pd.MinimumSpace, pd.MaximumSpace);


        std::string name = pd.Binding.get().getName();
        boost::replace_all(name, "\"", "\\\"");
        out << ")\\n" << name << "\"];\n";
    }

    out << "}\n\n";
    out.flush();
}


}

#endif
