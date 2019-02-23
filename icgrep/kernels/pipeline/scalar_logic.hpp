#ifndef SCALAR_LOGIC_HPP
#define SCALAR_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeScalarDependencyGraph
 *
 * producer -> buffer/scalar -> consumer
 ** ------------------------------------------------------------------------------------------------------------- */
ScalarGraph PipelineCompiler::makeScalarDependencyGraph() const {
    ScalarGraph G(LastScalar + 1);
    for (unsigned i = FirstScalar; i <= LastScalar; ++i) {
        for (const auto & e : make_iterator_range(in_edges(i, mPipelineGraph))) {
            add_edge(source(e, mPipelineGraph), i, mPipelineGraph[e].Number, G);
        }
        for (const auto & e : make_iterator_range(out_edges(i, mPipelineGraph))) {
            add_edge(i, target(e, mPipelineGraph), mPipelineGraph[e].Number, G);
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
    for (unsigned call = FirstCall; call <= LastCall; ++call) {
        writeOutputScalars(b, call, args);
        Function * const f = mPipelineGraph[call].Callee; assert (f);
        auto i = f->arg_begin();
        for (auto j = args.begin(); j != args.end(); ++i, ++j) {
            assert (i != f->arg_end());
            *j = b->CreateZExtOrTrunc(*j, i->getType());
        }
        assert (i == f->arg_end());
        mScalarValue[call] = b->CreateCall(f, args);
    }
    writeOutputScalars(b, PipelineOutput, args);
    return args;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writeOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::writeOutputScalars(BuilderRef b, const unsigned index, std::vector<Value *> & args) {
    const auto n = in_degree(index, mScalarGraph);
    args.resize(n);
    for (const auto e : make_iterator_range(in_edges(index, mScalarGraph))) {
        const auto scalar = source(e, mScalarGraph);
        const auto k = mScalarGraph[e];
        args[k] = getScalar(b, scalar);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getScalar
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getScalar(BuilderRef b, const unsigned index) {
    Value * value = mScalarValue[index];
    if (value) {
        return value;
    }
    if (LLVM_UNLIKELY(in_degree(index, mScalarGraph) == 0)) {
        assert (index >= FirstScalar && index <= LastScalar);
        const Relationship * const rel = mPipelineGraph[index].Relationship; assert (rel);
        value = cast<ScalarConstant>(rel)->value();
    } else {
        const auto producer = in_edge(index, mScalarGraph);
        const auto i = source(producer, mScalarGraph);
        const auto j = mScalarGraph[producer];
        if (i == PipelineInput) {
            const Binding & input = mPipelineKernel->getInputScalarBinding(j);
            value = b->getScalarField(input.getName());
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
    }
    assert (value);
    mScalarValue[index] = value;
    return value;
}

}

#endif // SCALAR_LOGIC_HPP
