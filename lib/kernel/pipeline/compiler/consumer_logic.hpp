#ifndef CONSUMER_LOGIC_HPP
#define CONSUMER_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

#if 0

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    using Graph = adjacency_list<hash_setS, vecS, bidirectionalS, no_property, unsigned>;
    using Map = flat_map<unsigned, Graph::vertex_descriptor>;

    Graph S(PipelineOutput + 1);

    if (LLVM_LIKELY(!mTrackIndividualConsumedItemCounts)) {

        Map Rin;
        Map Rout;

        auto getSymbolicRateVertex = [&](const unsigned symbolicRate, Map & R) {
            const auto f = R.find(symbolicRate);
            if (f == R.end()) {
                const auto v = add_vertex(S);
                R.emplace(symbolicRate, v);
                return v;
            } else {
                return f->second;
            }
        };

        // First compute a graph depicting the symbolic flow of information
        // with the appropriate port nums for later use.

        for (auto stream = FirstStreamSet; stream <= LastStreamSet; ++stream) {

            const auto pe = in_edge(stream, mBufferGraph);
            const BufferRateData & Ir = mBufferGraph[pe];

            const auto p_out = source(pe, mBufferGraph);
            const auto s_in = getSymbolicRateVertex(Ir.SymbolicRate, Rin);
            add_edge(p_out, s_in, Ir.Port.Number + 1U, S);

            for (const auto ce : make_iterator_range(out_edges(stream, mBufferGraph))) {
                const BufferRateData & Or = mBufferGraph[ce];
                auto s_out = getSymbolicRateVertex(Or.SymbolicRate, Rout);
                // add_edge(s_in, s_out, 0, S);
                const auto c_in = target(ce, mBufferGraph);
                add_edge(s_out, c_in, Or.Port.Number + 1U, S);
            }

        }

        printGraph(S, errs(), "S");

        // Take a transitive reduction of S to eliminate any relationships
        // whose information flow is guaranteed to be less than some other
        // input.

        transitive_reduction_dag(S);

        // Cut out any edges that exist purely to compute S'
        remove_edge_if([&S](const Graph::edge_descriptor & e) {
            return S[e] == 0;
        }, S);

        printGraph(S, errs(), "S'");

    }

    // Now compute the consumer graph C using S' as a filter for which
    // ports we wish to include in the final graph.
    ConsumerGraph C(LastStreamSet + 1);

    for (auto kernel = FirstKernel; kernel <= LastKernel; ++kernel) {

        BitVector input_filter(in_degree(kernel, mBufferGraph), false);
        if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
            input_filter.set(0, input_filter.size());
        } else {
            for (const auto e : make_iterator_range(in_edges(kernel, S))) {
                const auto p = S[e]; assert (p > 0);
                input_filter.set(p - 1);
            }
        }

        for (const auto e : make_iterator_range(in_edges(kernel, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[e];
            if (input_filter.test(br.Port.Number)) {
                const auto buffer = source(e, mBufferGraph);
                add_edge(buffer, kernel, ConsumerEdge{br.Port, 0}, C);
            }
        }

        BitVector output_filter(out_degree(kernel, mBufferGraph), false);
        if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {
            output_filter.set(0, output_filter.size());
        } else {
            for (const auto e : make_iterator_range(out_edges(kernel, S))) {
                const auto p = S[e]; assert (p > 0);
                output_filter.set(p - 1);
            }
        }

        for (const auto e : make_iterator_range(out_edges(kernel, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[e];
            if (output_filter.test(br.Port.Number)) {
                const auto buffer = target(e, mBufferGraph);
                add_edge(kernel, buffer, ConsumerEdge{br.Port, 0}, C);
            }
        }

    }

    if (LLVM_UNLIKELY(mTrackIndividualConsumedItemCounts)) {

        for (auto stream = FirstStreamSet; stream <= LastStreamSet; ++stream) {
            unsigned index = 0;
            for (const auto e : make_iterator_range(out_edges(stream, C))) {
                ConsumerEdge & c = C[e];
                c.Index = index++;
            }
        }

    }

    return C;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    const auto firstBuffer = PipelineOutput + 1;
    const auto lastBuffer = num_vertices(mBufferGraph);
    ConsumerGraph G(lastBuffer);

#if 0

    #warning TODO: ConsumerGraph assumes the dataflow is transitively bounded by the same initial source

    #warning REVISIT: ConsumerGraph is not optimal for handling relative rate inputs

    struct ConsumerData {
        unsigned Kernel{0};
        unsigned Port{0};
        Rational Minimum{0};
        Rational Maximum{0};

        inline bool operator < (const ConsumerData & other) const {
            return (Kernel < other.Kernel) || (Port < other.Port);
        }
    };

    std::vector<ConsumerData> consumers;
#endif

    for (auto bufferVertex = firstBuffer; bufferVertex < lastBuffer; ++bufferVertex) {
        // copy the producing edge
        const auto pe = in_edge(bufferVertex, mBufferGraph);
        const BufferRateData & br = mBufferGraph[pe];
        add_edge(source(pe, mBufferGraph), bufferVertex, ConsumerEdge{br.Port, 0}, G);
        unsigned index = 0;
        for (const auto ce : make_iterator_range(out_edges(bufferVertex, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[ce];
            add_edge(bufferVertex, target(ce, mBufferGraph), ConsumerEdge{br.Port, ++index}, G);
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
            const Binding & input = kernel->getInputStreamSetBinding(cd.Port); // <-- fix!
            if (LLVM_UNLIKELY(input.hasAttribute(AttrId::Deferred))) {
                cd.Minimum = Rational{0};
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

#endif

// TODO: rework consumer logic for external I/O.

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeConsumerGraph
 *
 * Copy the buffer graph but amalgamate any multi-edges into a single one
 ** ------------------------------------------------------------------------------------------------------------- */
ConsumerGraph PipelineCompiler::makeConsumerGraph()  const {

    ConsumerGraph G(LastStreamSet + 1);

    for (auto streamSet = FirstStreamSet; streamSet <= LastStreamSet; ++streamSet) {
        // copy the producing edge
        const auto pe = in_edge(streamSet, mBufferGraph);
        const BufferRateData & br = mBufferGraph[pe];
        add_edge(source(pe, mBufferGraph), streamSet, ConsumerEdge{br.Port, 0}, G);
        unsigned index = 0;
        for (const auto ce : make_iterator_range(out_edges(streamSet, mBufferGraph))) {
            const BufferRateData & br = mBufferGraph[ce];
            add_edge(streamSet, target(ce, mBufferGraph), ConsumerEdge{br.Port, ++index}, G);
        }
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addConsumerKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::addConsumerKernelProperties(BuilderRef b, const unsigned producer) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(out_edges(producer, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        // If the out-degree for this buffer is zero, then we've proven that its consumption rate
        // is identical to its production rate.
        const auto numOfIndependentConsumers = out_degree(streamSet, mConsumerGraph);
        if (LLVM_UNLIKELY(numOfIndependentConsumers == 0 && producer != PipelineInput)) {
            continue;
        }
        const BufferNode & bn = mBufferGraph[streamSet];
        const BufferRateData & rd = mBufferGraph[e];
        assert (rd.Port.Type == PortType::Output);
        const auto name = makeBufferName(producer, rd.Port) + CONSUMED_ITEM_COUNT_SUFFIX;

        // If we're tracing the consumer item counts, we need to store one for each
        // (non-nested) consumer. Any nested consumers will have their own trace.
        Type * countTy = sizeTy;
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            countTy = ArrayType::get(sizeTy, numOfIndependentConsumers + 1);
        }
        if (LLVM_LIKELY(bn.isOwned() || bn.isInternal())) {
            mTarget->addInternalScalar(countTy, name);
        } else {
            mTarget->addNonPersistentScalar(countTy, name);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::initializeConsumedItemCount(BuilderRef b, const StreamSetPort outputPort, Value * const produced) {
    Value * initiallyConsumed = produced;
    const Binding & binding = getOutputBinding(outputPort);
    if (LLVM_UNLIKELY(binding.hasAttribute(AttrId::LookBehind))) {
        const Attribute & attr = binding.findAttribute(AttrId::LookBehind);
        Constant * const lookBehind = b->getSize(attr.amount());
        Value * consumed = b->CreateSub(initiallyConsumed, lookBehind);
        Value * const satisfies = b->CreateICmpUGT(initiallyConsumed, lookBehind);
        initiallyConsumed = b->CreateSelect(satisfies, consumed, b->getSize(0));
    }
    const auto streamSet = getOutputBufferVertex(outputPort);
    const ConsumerNode & cn = mConsumerGraph[streamSet];
    cn.Consumed = initiallyConsumed;
    cn.Encountered = 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::readConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(mKernelId, mConsumerGraph))) {
        const ConsumerEdge & c = mConsumerGraph[e];
        const auto streamSet = target(e, mConsumerGraph);
        Value * consumed = nullptr;
        const StreamSetPort port(PortType::Output, c.Port);
        if (LLVM_UNLIKELY(out_degree(streamSet, mConsumerGraph) == 0)) {
            // This stream either has no consumers or we've proven that its consumption rate
            // is identical to its production rate.
            consumed = mInitiallyProducedItemCount(port);
        } else {
            const auto prefix = makeBufferName(mKernelId, port);
            Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
            if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
                Constant * const ZERO = b->getInt32(0);
                ptr = b->CreateInBoundsGEP(ptr, { ZERO, ZERO } );
            }
            consumed = b->CreateLoad(ptr);
        }
        mConsumedItemCount(port) = consumed; assert (consumed);
        #ifdef PRINT_DEBUG_MESSAGES
        const auto prefix = makeBufferName(mKernelIndex, StreamSetPort{PortType::Output, c.Port});
        debugPrint(b, prefix + "_consumed = %" PRIu64, consumed);
        #endif
    }
}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief createConsumedPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::createConsumedPhiNodes(BuilderRef b) {
    IntegerType * const sizeTy = b->getSizeTy();
    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const auto buffer = source(e, mConsumerGraph);
        const ConsumerNode & cn = mConsumerGraph[buffer];
        if (LLVM_LIKELY(cn.PhiNode == nullptr)) {
            const ConsumerEdge & c = mConsumerGraph[e];
            const StreamSetPort port(PortType::Input, c.Port);
            const auto prefix = makeBufferName(mKernelId, port);
            PHINode * const consumedPhi = b->CreatePHI(sizeTy, 2, prefix + "_consumed");
            assert (cn.Consumed);
            consumedPhi->addIncoming(cn.Consumed, mKernelInitiallyTerminatedPhiCatch);
            cn.PhiNode = consumedPhi;
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief computeMinimumConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::computeMinimumConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        //if (LLVM_UNLIKELY(mConsumerGraph[e] == FAKE_CONSUMER)) continue;
        const ConsumerEdge & c = mConsumerGraph[e];
        const StreamSetPort port(PortType::Input, c.Port);
        Value * processed = mFullyProcessedItemCount(port);
        // To support the lookbehind attribute, we need to withhold the items from
        // our consumed count and rely on the initial buffer underflow to access any
        // items before the start of the physical buffer.
        const Binding & input = getBinding(port);
        if (LLVM_UNLIKELY(input.hasAttribute(AttrId::LookBehind))) {
            const auto & lookBehind = input.findAttribute(AttrId::LookBehind);
            ConstantInt * const amount = b->getSize(lookBehind.amount());
            processed = b->CreateSaturatingSub(processed, amount);
        }
        const auto buffer = source(e, mConsumerGraph);
        assert (buffer >= FirstStreamSet && buffer <= LastStreamSet);
        const ConsumerNode & cn = mConsumerGraph[buffer]; assert (cn.Consumed);
        if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
            const ConsumerEdge & c = mConsumerGraph[e]; assert (c.Index > 0);
            setConsumedItemCount(b, buffer, processed, c.Index);
        }
        assert (cn.Consumed);
        cn.Consumed = b->CreateUMin(cn.Consumed, processed);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeFinalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeFinalConsumedItemCounts(BuilderRef b) {

    for (const auto e : make_iterator_range(in_edges(mKernelId, mConsumerGraph))) {
        const auto buffer = source(e, mConsumerGraph);
        //if (LLVM_UNLIKELY(mConsumerGraph[e] != FAKE_CONSUMER)) {
        const ConsumerNode & cn = mConsumerGraph[buffer];
        if (LLVM_LIKELY(cn.PhiNode != nullptr)) {
            cn.PhiNode->addIncoming(cn.Consumed, mKernelLoopExitPhiCatch);
            cn.Consumed = cn.PhiNode;
            cn.PhiNode = nullptr;
        }
        // check to see if we've fully finished processing any stream
        cn.Encountered++;
        assert (cn.Encountered <= out_degree(buffer, mConsumerGraph));
        if (out_degree(buffer, mConsumerGraph) == cn.Encountered) {
            #ifdef PRINT_DEBUG_MESSAGES
            const auto prod = in_edge(buffer, mBufferGraph);
            const BufferRateData & br = mBufferGraph[prod];
            const auto producer = source(prod, mBufferGraph);
            const auto prefix = makeBufferName(producer, br.Port);
            debugPrint(b, " * writing " + prefix + "_consumed = %" PRIu64, cn.Consumed);
            #endif
            setConsumedItemCount(b, buffer, cn.Consumed, 0);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setConsumedItemCount
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::setConsumedItemCount(BuilderRef b, const unsigned streamSet, not_null<Value *> consumed, const unsigned slot) const {
    const auto pe = in_edge(streamSet, mBufferGraph);
    const auto producer = source(pe, mBufferGraph);
    const BufferRateData & rd = mBufferGraph[pe];
    const auto prefix = makeBufferName(producer, rd.Port);
    Value * ptr = b->getScalarFieldPtr(prefix + CONSUMED_ITEM_COUNT_SUFFIX);
    if (LLVM_UNLIKELY(mTraceIndividualConsumedItemCounts)) {
        ptr = b->CreateInBoundsGEP(ptr, { b->getInt32(0), b->getInt32(slot) });
    }
    if (LLVM_UNLIKELY(mCheckAssertions)) {
        Value * const prior = b->CreateLoad(ptr);
        const Binding & output = rd.Binding;
        // TODO: cross reference which slot the traced count is for?
        b->CreateAssert(b->CreateICmpULE(prior, consumed),
                        "%s.%s: consumed item count is not monotonically nondecreasing "
                        "(prior %" PRIu64 " > current %" PRIu64 ")",
                        mKernelAssertionName,
                        b->GetString(output.getName()),
                        prior, consumed);
    }
    b->CreateStore(consumed, ptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief readExternalConsumerItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::readExternalConsumerItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(in_edges(PipelineOutput, mBufferGraph))) {
        const auto buffer = source(e, mBufferGraph);
        const BufferNode & bn = mBufferGraph[buffer];
        if (LLVM_LIKELY(bn.isOwned())) {
            const auto p = in_edge(buffer, mBufferGraph);
            const BufferRateData & rd = mBufferGraph[p];
            Value * const consumed = getConsumedOutputItems(rd.Port.Number); assert (consumed);
            const auto producer = source(p, mBufferGraph);
            const auto name = makeBufferName(producer, rd.Port);
            b->setScalarField(name + CONSUMED_ITEM_COUNT_SUFFIX, consumed);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief reportExternalConsumedItemCounts
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::writeExternalConsumedItemCounts(BuilderRef b) {
    for (const auto e : make_iterator_range(out_edges(PipelineInput, mBufferGraph))) {
        const auto streamSet = target(e, mBufferGraph);
        const BufferRateData & rd = mBufferGraph[e];
        Value * const ptr = getProcessedInputItemsPtr(rd.Port.Number);
        const ConsumerNode & cn = mConsumerGraph[streamSet];
        b->CreateStore(cn.Consumed, ptr);
    }
}

}

#endif // CONSUMER_LOGIC_HPP
