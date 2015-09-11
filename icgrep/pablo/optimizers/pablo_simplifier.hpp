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
    Simplifier();
private:
    static void eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor = nullptr);
    static void eliminateRedundantComplexStatements(PabloBlock & block);
    static bool isSuperfluous(const Assign * const assign);
private:

};

}
#endif // PABLO_SIMPLIFIER_HPP
