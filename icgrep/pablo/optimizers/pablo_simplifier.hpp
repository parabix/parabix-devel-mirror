#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

#include <pablo/codegenstate.h>

namespace pablo {

struct ExpressionTable;
class PabloFunction;

class Simplifier {
    friend class DistributivePass;
    friend class FactorizeDFG;
public:
    static bool optimize(PabloFunction & function);
protected:
    Simplifier() = default;
private:
    static void redundancyElimination(PabloBlock * const block, ExpressionTable * predecessor = nullptr);
    static PabloAST * fold(Variadic * var, PabloBlock * const block);
    static PabloAST * fold(Statement * const stmt, PabloBlock * const block);
    static void deadCodeElimination(PabloBlock * const block);
    static void strengthReduction(PabloBlock * const block);
    static bool isSuperfluous(const Assign * const assign);
};

}
#endif // PABLO_SIMPLIFIER_HPP
