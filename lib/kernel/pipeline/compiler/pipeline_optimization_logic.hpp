#ifndef PIPELINE_OPTIMIZATION_LOGIC_HPP
#define PIPELINE_OPTIMIZATION_LOGIC_HPP

#include "pipeline_compiler.hpp"
#ifndef NDEBUG
#include <llvm/IR/Verifier.h>
#endif

namespace kernel {


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief simplifyPhiNodes
 ** ------------------------------------------------------------------------------------------------------------- */
inline void PipelineCompiler::simplifyPhiNodes(BuilderRef b) const {

    // LLVM is not aggressive enough with how it deals with phi nodes. To ensure that
    // we collapse every phi node in which all incoming values are identical into the
    // incoming value, we execute the following mini optimization pass.

    // TODO: check the newer versions of LLVM to see if any can do this now.



    // To make sure this pass isn't hiding an error by removing a phi node, first run
    // the verifier on this to detect any possible errors prior to optimizing it.
    // assert (!verifyModule(*b->getModule()));

    Function * const f = b->GetInsertBlock()->getParent();
    for (BasicBlock & bb : f->getBasicBlockList()) {
        Instruction * inst = &bb.getInstList().front();
        for (;;) {
            if (LLVM_LIKELY(isa<PHINode>(inst))) {
                PHINode * const phi = cast<PHINode>(inst);
                inst = inst->getNextNode();
                if (LLVM_LIKELY(phi->hasNUsesOrMore(1))) {
                    assert (phi && phi->getNumIncomingValues() > 0);
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
            break;
        }
    }

}

}

#endif // PIPELINE_OPTIMIZATION_LOGIC_HPP
