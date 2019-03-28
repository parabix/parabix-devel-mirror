#ifndef SCALAR_LOGIC_HPP
#define SCALAR_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineCompiler::getFinalOutputScalars(BuilderRef b) {
    std::vector<Value *> args;
    b->setKernel(mPipelineKernel);
    for (unsigned call = FirstCall; call <= LastCall; ++call) {
        writeOutputScalars(b, call, args);
        Function * const f = cast<Function>(mStreamGraph[call].Callee); assert (f);
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
        const RelationshipType & rt = mScalarGraph[e];
        args[rt.Number] = getScalar(b, scalar);
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
        const RelationshipNode & rn = mScalarGraph[index];
        assert (rn.Type == RelationshipNode::IsRelationship);
        const Relationship * const rel = rn.Relationship; assert (rel);
        value = cast<ScalarConstant>(rel)->value();
    } else {
        const auto producer = in_edge(index, mScalarGraph);
        const auto i = source(producer, mScalarGraph);
        const RelationshipType & rt = mScalarGraph[producer];
        if (i == PipelineInput) {
            const Binding & input = mPipelineKernel->getInputScalarBinding(rt.Number);
            value = b->getScalarField(input.getName());
        } else { // output scalar of some kernel
            Value * const outputScalars = getScalar(b, i);
            if (LLVM_UNLIKELY(outputScalars == nullptr)) {
                report_fatal_error("Internal error: pipeline is unable to locate valid output scalar");
            }
            if (outputScalars->getType()->isAggregateType()) {
                value = b->CreateExtractValue(outputScalars, {rt.Number});
            } else { assert (rt.Number == 0 && "scalar type is not an aggregate");
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
