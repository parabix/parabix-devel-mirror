#ifndef SCALAR_LOGIC_HPP
#define SCALAR_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeScalarDependencyGraph
 *
 * producer -> buffer/scalar -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
RelationshipGraph PipelineCompiler::makeScalarDependencyGraph() const {
    RelationshipGraph G(LastScalar + 1);
    for (unsigned i = FirstScalar; i <= LastScalar; ++i) {
        G[i].Relationship = mPipelineGraph[i].Relationship;
        for (const auto & e : make_iterator_range(in_edges(i, mPipelineGraph))) {
            add_edge(source(e, mPipelineGraph), i, mPipelineGraph[e], G);
        }
        for (const auto & e : make_iterator_range(out_edges(i, mPipelineGraph))) {
            add_edge(i, target(e, mPipelineGraph), mPipelineGraph[e], G);
        }
    }
    return G;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineCompiler::getFinalOutputScalars(BuilderRef b) {
    std::vector<Value *> args;
    b->setKernel(mPipelineKernel);
    for (unsigned k = FirstCall; k <= LastCall; ++k) {
        writeOutputScalars(b, k, args);
        Function * const f = mPipelineGraph[k].Callee; assert (f);
        auto i = f->arg_begin();
        for (auto j = args.begin(); j != args.end(); ++i, ++j) {
            assert (i != f->arg_end());
            *j = b->CreateZExtOrTrunc(*j, i->getType());
        }
        assert (i == f->arg_end());
        Value * const result = b->CreateCall(f, args);
        mScalarCache.emplace(k, result);
    }
    writeOutputScalars(b, PipelineOutput, args);
    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeOutputScalars(BuilderRef b, const unsigned index, std::vector<Value *> & args) {
    const auto n = in_degree(index, mScalarDependencyGraph);
    args.resize(n);
    for (const auto e : make_iterator_range(in_edges(index, mScalarDependencyGraph))) {
        const auto scalar = source(e, mScalarDependencyGraph);
        const auto k = mScalarDependencyGraph[e];
        args[k] = getScalar(b, scalar);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getScalar(BuilderRef b, const RelationshipGraph::vertex_descriptor scalar) {
    const auto f = mScalarCache.find(scalar);
    if (LLVM_UNLIKELY(f != mScalarCache.end())) {
        return f->second;
    }
    const auto producer = in_edge(scalar, mScalarDependencyGraph);
    const auto i = source(producer, mScalarDependencyGraph);
    const auto j = mScalarDependencyGraph[producer];
    Value * value = nullptr;
    if (i == PipelineInput) {
        if (LLVM_UNLIKELY(j == SCALAR_CONSTANT)) {
            const Relationship * const rel = mScalarDependencyGraph[scalar].Relationship; assert (rel);
            value = cast<ScalarConstant>(rel)->value();
        } else {
            const Binding & input = mPipelineKernel->getInputScalarBinding(j);
            value = b->getScalarField(input.getName());
        }
    } else { // output scalar of some kernel
        Value * const outputScalars = getScalar(b, i);
        if (LLVM_UNLIKELY(outputScalars == nullptr)) {
            report_fatal_error("Internal error: pipeline is unable to locate valid output scalar");
        }
        if (outputScalars->getType()->isAggregateType()) {
            value = b->CreateExtractValue(outputScalars, {j});
        } else { assert (j == 0 && "scalar type is not an aggregate");
            value = outputScalars;
        }
    }
    assert (value);
    mScalarCache.emplace(scalar, value);
    return value;
}

}

#endif // SCALAR_LOGIC_HPP
