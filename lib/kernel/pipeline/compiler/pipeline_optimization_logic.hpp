#ifndef PIPELINE_OPTIMIZATION_LOGIC_HPP
#define PIPELINE_OPTIMIZATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#ifndef NDEBUG
#include <llvm/IR/Verifier.h>
#endif

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief runOptimizationPasses
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::runOptimizationPasses(BuilderRef b) {

    // To make sure the optimizations aren't hiding an error, first run the verifier
    // detect any possible errors prior to optimizing it.

    // assert (!verifyModule(*b->getModule()));

    simplifyPhiNodes(b->getModule());

    b->getModule()->print(errs(), nullptr);

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::simplifyPhiNodes(Module * const m) const {

    // LLVM is not aggressive enough with how it deals with phi nodes. To ensure that
    // we collapse every phi node in which all incoming values are identical into the
    // incoming value, we execute the following mini optimization pass.

    // TODO: check the newer versions of LLVM to see if any can do this now.




    for (Function & f : m->getFunctionList()) {
        for (BasicBlock & bb : f.getBasicBlockList()) {
            Instruction * inst = &bb.getInstList().front();
            while (isa<PHINode>(inst)) {
                PHINode * const phi = cast<PHINode>(inst);
                assert (phi->getNumIncomingValues() > 0);
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
keep_phi_node:  continue;
            }
        }
    }

}

}

#endif // PIPELINE_OPTIMIZATION_LOGIC_HPP
