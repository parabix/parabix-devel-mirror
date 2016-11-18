#ifndef PABLO_SIMPLIFIER_HPP
#define PABLO_SIMPLIFIER_HPP

namespace pablo {

class PabloKernel;
class PabloAST;
class Variadic;
class Statement;
class PabloBlock;

struct ExpressionTable;

class Simplifier {
    friend class BooleanReassociationPass;
    struct VariableTable;
public:
    static bool optimize(PabloKernel * kernel);
protected:
    Simplifier() = default;
private:
    static PabloAST * fold(Variadic * var, PabloBlock * const block);
    static PabloAST * fold(Statement * const stmt, PabloBlock * const block);
    static void redundancyElimination(PabloBlock * const block, ExpressionTable * et = nullptr, VariableTable * const vt = nullptr);
    static void deadCodeElimination(PabloKernel * const kernel);
    static void deadCodeElimination(PabloBlock * const block);
    static void strengthReduction(PabloBlock * const block);
};

}
#endif // PABLO_SIMPLIFIER_HPP
