#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

#include <pablo/codegenstate.h>

namespace pablo {

class PabloFunction;
struct ExpressionTable;

class Simplifier {
    friend class BooleanReassociationPass;
    struct VariableTable;
public:
    static bool optimize(PabloFunction & function);
protected:
    Simplifier() = default;
private:
    static PabloAST * fold(Variadic * var, PabloBlock * const block);
    static PabloAST * fold(Statement * const stmt, PabloBlock * const block);
    static void redundancyElimination(PabloBlock * const block, ExpressionTable * et = nullptr, VariableTable * const vt = nullptr);
    static void deadCodeElimination(PabloFunction & f);
    static void deadCodeElimination(PabloBlock * const block);
    static void strengthReduction(PabloBlock * const block);
};

}
#endif // PABLO_SIMPLIFIER_HPP
