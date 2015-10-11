#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

#include <pablo/codegenstate.h>

namespace pablo {

struct ExpressionTable;
class PabloFunction;

class Simplifier {
public:
    static bool optimize(PabloFunction & function);
    static void deadCodeElimination(PabloBlock & block);
protected:
    Simplifier() = default;
private:
    static void eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor = nullptr);
    static void eliminateRedundantEquations(PabloBlock & block);
    static bool isSuperfluous(const Assign * const assign);
};

}
#endif // PABLO_SIMPLIFIER_HPP
