#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

#include <pablo/codegenstate.h>

namespace pablo {

struct ExpressionTable;
class PabloFunction;

class Simplifier {
public:
    static bool optimize(PabloFunction & function);
protected:
    Simplifier() = default;
private:
    static void eliminateRedundantCode(PabloBlock * const block, ExpressionTable * predecessor = nullptr);
    static void deadCodeElimination(PabloBlock * const block);
    static void eliminateRedundantEquations(PabloBlock * const block);
    static bool isSuperfluous(const Assign * const assign);
};

}
#endif // PABLO_SIMPLIFIER_HPP
