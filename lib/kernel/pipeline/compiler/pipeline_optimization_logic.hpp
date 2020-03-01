#ifndef PIPELINE_OPTIMIZATION_LOGIC_HPP
#define PIPELINE_OPTIMIZATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#ifndef NDEBUG
#include <llvm/IR/Verifier.h>
#endif
#include <llvm/Transforms/Scalar.h>
#include <llvm/Transforms/Utils/Local.h>
#include <llvm/IR/LegacyPassManager.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief runOptimizationPasses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::runOptimizationPasses(BuilderRef b) {

    // To make sure the optimizations aren't hiding an error, first run the verifier
    // detect any possible errors prior to optimizing it.

    Module * const m = b->getModule();

    #ifndef NDEBUG
    if (LLVM_UNLIKELY(verifyModule(*m, &errs(), nullptr))) {
        m->print(errs(), nullptr);
        report_fatal_error("Error during pipeline code generation!");
    }
    #endif

    simplifyPhiNodes(m);

    auto pm = make_unique<legacy::PassManager>();
    pm->add(createDeadCodeEliminationPass());        // Eliminate any trivially dead code
    pm->add(createCFGSimplificationPass());          // Remove dead basic blocks and unnecessary branch statements / phi nodes
    pm->run(*m);

    simplifyPhiNodes(m);

    // m->print(errs(), nullptr);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::simplifyPhiNodes(Module * const m) const {

    // LLVM is not aggressive enough with how it deals with phi nodes. To ensure that
    // we collapse every phi node in which all incoming values are identical into the
    // incoming value, we execute the following mini optimization pass.

    // TODO: check the newer versions of LLVM to see if any can do this now.

    SmallVector<BasicBlock *, 16> preds;
    SmallVector<Value *, 16> value;

    for (Function & f : m->getFunctionList()) {
        bool anyPhis = false;

        for (BasicBlock & bb : f.getBasicBlockList()) {

            preds.assign(pred_begin(&bb), pred_end(&bb));
            const auto n = preds.size();
            value.resize(n);

            Instruction * inst = &bb.front();
            while (isa<PHINode>(inst)) {
                PHINode * const phi = cast<PHINode>(inst);
                assert (phi->getNumIncomingValues() == n);
                inst = inst->getNextNode();
                if (LLVM_LIKELY(phi->hasNUsesOrMore(1))) {
                    Value * const value = phi->getIncomingValue(0);
                    const auto n = phi->getNumIncomingValues();
                    for (unsigned i = 1; i != n; ++i) {
                        if (LLVM_LIKELY(phi->getIncomingValue(i) != value)) {
                            goto keep_phi_node;
                        }
                    }
                    phi->replaceAllUsesWith(value);
                }
                RecursivelyDeleteDeadPHINode(phi);
                continue;
                // ----------------------------------------------------------------------------------
                //  canonicalize the phi node ordering for the eliminate duplicate phi node function
                // ----------------------------------------------------------------------------------
keep_phi_node:  bool canonicalize = false;
                for (unsigned i = 0; i != n; ++i) {
                    const auto f = std::find(preds.begin(), preds.end(), phi->getIncomingBlock(i));
                    assert ("phi-node has invalid incoming block?" && f != preds.end());
                    const auto j = std::distance(preds.begin(), f);
                    canonicalize |= (j != i);
                    value[j] = phi->getIncomingValue(i);
                }
                if (canonicalize) {
                    for (unsigned i = 0; i != n; ++i) {
                        phi->setIncomingBlock(i, preds[i]);
                        phi->setIncomingValue(i, value[i]);
                    }
                }
                anyPhis = true;
            }
            if (LLVM_LIKELY(anyPhis)) {
                EliminateDuplicatePHINodes(&bb);
            }
        }
    }
}

}

#endif // PIPELINE_OPTIMIZATION_LOGIC_HPP
