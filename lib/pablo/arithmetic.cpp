#include <pablo/arithmetic.h>

namespace pablo {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief replaceUsesOfWith
 ** ------------------------------------------------------------------------------------------------------------- */
void Operator::replaceUsesOfWith(PabloAST * const from, PabloAST * const to, const bool recursive) {
    if (LLVM_LIKELY(from != to)) {
        for (unsigned i = 0; i != 2; ++i) {
           PabloAST * op = mOperand[i];
           if (op == from) {
               from->removeUser(this);
               to->addUser(this);
               mOperand[i] = to;
           } else if (LLVM_UNLIKELY(recursive)) {
               op->replaceUsesOfWith(from, to, true);
           }
        }
    }
}


}
