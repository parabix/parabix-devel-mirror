#include "pipeline_compiler.hpp"
#include <boost/graph/topological_sort.hpp>
#include <util/extended_boost_graph_containers.h>

namespace kernel {

#warning TODO: support call bindings that produce output that are inputs of other call bindings or become scalar outputs of the pipeline

namespace {

    using ScalarDependencyMap = RelationshipMap<ScalarDependencyGraph::vertex_descriptor>;

    void enumerateScalarProducerBindings(const unsigned producerVertex, const Bindings & bindings, ScalarDependencyGraph & G, ScalarDependencyMap & M) {
        const auto n = bindings.size();
        for (unsigned i = 0; i < n; ++i) {
            const Relationship * const rel = getRelationship(bindings[i]);
            assert (M.count(rel) == 0);
            Constant * const value = isa<ScalarConstant>(rel) ? cast<ScalarConstant>(rel)->value() : nullptr;
            const auto bufferVertex = add_vertex(value, G);
            add_edge(producerVertex, bufferVertex, i, G);
            M.emplace(rel, bufferVertex);
        }
    }

    ScalarDependencyGraph::vertex_descriptor makeIfConstant(const Relationship * const rel, ScalarDependencyGraph & G, ScalarDependencyMap & M) {
        const auto f = M.find(rel);
        if (LLVM_LIKELY(f != M.end())) {
            return f->second;
        } else if (LLVM_LIKELY(isa<ScalarConstant>(rel))) {
            const auto bufferVertex = add_vertex(cast<ScalarConstant>(rel)->value(), G);
            M.emplace(rel, bufferVertex);
            return bufferVertex;
        } else {
            report_fatal_error("unknown scalar value");
        }
    }

    template <typename Array>
    void enumerateScalarConsumerBindings(const unsigned consumerVertex, const Array & array, ScalarDependencyGraph & G, ScalarDependencyMap & M) {
        const auto n = array.size();
        for (unsigned i = 0; i < n; ++i) {
            const auto bufferVertex = makeIfConstant(getRelationship(array[i]), G, M);
            assert (bufferVertex < num_vertices(G));
            add_edge(bufferVertex, consumerVertex, i, G);
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeScalarDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
ScalarDependencyGraph PipelineCompiler::makeScalarDependencyGraph() const {

    const auto numOfKernels = mPipeline.size();
    const auto & callBindings = mPipelineKernel->getCallBindings();
    const auto numOfCallBindings = callBindings.size();
    const auto initialSize = numOfKernels + numOfCallBindings + 1;

    ScalarDependencyGraph G(initialSize);
    ScalarDependencyMap M;

    enumerateScalarProducerBindings(numOfKernels, mPipelineKernel->getInputScalarBindings(), G, M);
    // verify each scalar input of the kernel is an input to the pipeline
    for (unsigned i = 0; i < numOfKernels; ++i) {
        enumerateScalarConsumerBindings(i, mPipeline[i]->getInputScalarBindings(), G, M);
    }
    // enumerate the output scalars
    for (unsigned i = 0; i < numOfKernels; ++i) {
        enumerateScalarProducerBindings(i, mPipeline[i]->getOutputScalarBindings(), G, M);
    }
    // enumerate the call bindings
    for (unsigned k = 0; k < numOfCallBindings; ++k) {
        const CallBinding & call = callBindings[k];
        enumerateScalarConsumerBindings(numOfKernels + 1 + k, call.Args, G, M);
    }
    // enumerate the pipeline outputs
    enumerateScalarConsumerBindings(numOfKernels, mPipelineKernel->getOutputScalarBindings(), G, M);
    return G;
}



/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makePortDependencyGraph
 ** ------------------------------------------------------------------------------------------------------------- */
PortDependencyGraph PipelineCompiler::makePortDependencyGraph() const {

    const auto n = mKernel->getNumOfStreamInputs();
    const auto m = mKernel->getNumOfStreamOutputs();
    const auto l = n + m;

    PortDependencyGraph G(l);

    // enumerate the input relations
    for (unsigned i = 0; i < n; ++i) {
        const Binding & input = mKernel->getInputStreamSetBinding(i);
        const ProcessingRate & rate = input.getRate();
        if (rate.hasReference()) {
            Port port; unsigned j;
            std::tie(port, j) = mKernel->getStreamPort(rate.getReference());
            assert ("input stream cannot refer to an output stream" && port == Port::Input);
            add_edge(i, j, rate.getKind(), G);
        }
    }
    // and then enumerate the output relations
    for (unsigned i = 0; i < m; ++i) {
        const Binding & output = mKernel->getOutputStreamSetBinding(i);
        const ProcessingRate & rate = output.getRate();
        if (rate.hasReference()) {
            Port port; unsigned j;
            std::tie(port, j) = mKernel->getStreamPort(rate.getReference());
            add_edge((i + n), (j + (port == Port::Output) ? n : 0), rate.getKind(), G);
        }
    }
    return G;
}


namespace {

    using TerminationMap = RelationshipMap<TerminationGraph::vertex_descriptor>;

    void enumerateTerminationProducerBindings(const unsigned producerVertex, const Bindings & bindings, TerminationGraph & G, TerminationMap & M) {
        const auto n = bindings.size();
        for (unsigned i = 0; i < n; ++i) {
            const Relationship * const rel = getRelationship(bindings[i]);
            if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
            assert (M.count(rel) == 0);
            const auto bufferVertex = add_vertex(G);
            add_edge(producerVertex, bufferVertex, G); // producer -> buffer ordering
            M.emplace(rel, bufferVertex);
        }
    }

    template <typename Array>
    void enumerateTerminationConsumerBindings(const unsigned consumerVertex, const Array & array, TerminationGraph & G, TerminationMap & M) {
        const auto n = array.size();
        for (unsigned i = 0; i < n; ++i) {
            const Relationship * const rel = getRelationship(array[i]);
            if (LLVM_UNLIKELY(isa<ScalarConstant>(rel))) continue;
            const auto f = M.find(rel);
            const auto bufferVertex = f->second;
            add_edge(bufferVertex, consumerVertex, G); // buffer -> consumer ordering
        }
    }

    void printTerminationGraph(const TerminationGraph & G, const std::vector<Kernel *> & pipeline, const unsigned index) {

        auto & out = errs();

        const auto numOfKernels = pipeline.size();

        out << "digraph G" << index << " {\n";
        for (auto u : make_iterator_range(vertices(G))) {
            out << "v" << u << " [label=\"" << u << ": ";
            if (u < numOfKernels) {
                out << pipeline[u]->getName();
            } else if (u == numOfKernels) {
                out << "Pipeline";
            } else {
                out << "B" << (u - numOfKernels);
            }
            out << "\"];\n";
        }

        for (auto e : make_iterator_range(edges(G))) {
            const auto s = source(e, G);
            const auto t = target(e, G);
            out << "v" << s << " -> v" << t;
           // out << " label=\"" << G[e] << "\"";
            out << ";\n";
        }

        out << "}\n\n";
        out.flush();

    }


}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeTerminationGraph
 *
 * The termination graph is a transitive reduction of I/O dependency graph. It models which kernels to be checked
 * to determine whether it is safe to terminate once it has finished processing its current input.
 ** ------------------------------------------------------------------------------------------------------------- */
TerminationGraph PipelineCompiler::makeTerminationGraph() const {
    using VertexVector = std::vector<TerminationGraph::vertex_descriptor>;

    const auto numOfKernels = mPipeline.size();
    TerminationGraph G(numOfKernels + 1);
    TerminationMap M;

    // make an edge from the pipeline input to a buffer vertex
    enumerateTerminationProducerBindings(numOfKernels, mPipelineKernel->getInputScalarBindings(), G, M);
    enumerateTerminationProducerBindings(numOfKernels, mPipelineKernel->getInputStreamSetBindings(), G, M);
    G[numOfKernels] = nullptr;

    // make an edge from each producing kernel to a buffer vertex
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & producer = mPipeline[i];
        enumerateTerminationProducerBindings(i, producer->getOutputStreamSetBindings(), G, M);
        enumerateTerminationProducerBindings(i, producer->getOutputScalarBindings(), G, M);
        G[i] = nullptr;
    }

    // make an edge from each buffer to its consuming kernel(s)
    for (unsigned i = 0; i < numOfKernels; ++i) {
        const auto & consumer = mPipeline[i];
        enumerateTerminationConsumerBindings(i, consumer->getInputScalarBindings(), G, M);
        enumerateTerminationConsumerBindings(i, consumer->getInputStreamSetBindings(), G, M);
        if (LLVM_UNLIKELY(consumer->hasAttribute(AttrId::SideEffecting))) {
            add_edge(i, numOfKernels, G);
        }
    }

    // make an edge from a buffer vertex to each pipeline output
    for (const CallBinding & call : mPipelineKernel->getCallBindings()) {
        enumerateTerminationConsumerBindings(numOfKernels, call.Args, G, M);
    }

    clear_out_edges(numOfKernels, G);
    enumerateTerminationConsumerBindings(numOfKernels, mPipelineKernel->getOutputStreamSetBindings(), G, M);
    enumerateTerminationConsumerBindings(numOfKernels, mPipelineKernel->getOutputScalarBindings(), G, M);

    VertexVector ordering;
    ordering.reserve(num_vertices(G));
    topological_sort(G, std::back_inserter(ordering));

    // generate a transitive closure
    for (unsigned u : ordering) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            const auto s = source(e, G);
            for (auto f : make_iterator_range(out_edges(u, G))) {
                add_edge(s, target(f, G), G);
            }
        }
    }

    // delete all buffer edges
    const auto firstBuffer = numOfKernels + 1;
    const auto lastBuffer = num_vertices(G);
    for (auto i = firstBuffer; i < lastBuffer; ++i) {
        clear_vertex(i, G);
    }

    // then take the transitive reduction
    VertexVector sources;
    for (unsigned u = firstBuffer; u--; ) {
        for (auto e : make_iterator_range(in_edges(u, G))) {
            sources.push_back(source(e, G));
        }
        std::sort(sources.begin(), sources.end());
        for (auto e : make_iterator_range(out_edges(u, G))) {
            remove_in_edge_if(target(e, G), [&G, &sources](const TerminationGraph::edge_descriptor f) {
                return std::binary_search(sources.begin(), sources.end(), source(f, G));
            }, G);
        }
        sources.clear();
    }

    return G;
}

} // end of namespace
