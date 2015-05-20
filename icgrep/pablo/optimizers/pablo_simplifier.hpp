#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

#include <pablo/codegenstate.h>

namespace pablo {

struct ExpressionTable;

class Simplifier {
public:
    static bool optimize(PabloBlock & block);
protected:
    Simplifier();
private:
    static void eliminateRedundantCode(PabloBlock & block, ExpressionTable * predecessor = nullptr);
    static void deadCodeElimination(PabloBlock & block);
    static void eliminateRedundantComplexStatements(PabloBlock & block);
private:

};

}
#endif // PABLO_SIMPLIFIER_HPP
