#ifndef BUILDER_HPP
#define BUILDER_HPP

#include "codegenstate.h"
#include "expression_map.hpp"

namespace pablo {

class PabloBuilder {
public:

    inline static PabloBlock & Create() {
        return *(new PabloBlock());
    }

    inline static PabloBlock & Create(PabloBlock & predecessor) {
        return *(new PabloBlock(&predecessor));
    }


    inline Zeroes * createZeroes() const {
        return mPb->createZeroes();
    }

    inline Ones * createOnes() const {
        return mPb->createOnes();
    }

    Var * createVar(const std::string name) {
        return createVar(mPb->getName(name));
    }

    Var * createVar(String * name);

    PabloAST * createVar(const PabloAST * const) {
        throw std::runtime_error("Var objects should only refer to external Vars (i.e., input basis bit streams). Use Assign objects directly.");
    }

    inline Call * createCall(const std::string name) {
        return createCall(mPb->getName(name));
    }

    Call * createCall(String * name);

    Assign * createAssign(const std::string prefix, PabloAST * expr, const int outputIndex = -1);

    inline PabloAST * createAdvance(PabloAST * expr, const int shiftAmount) {
        if (shiftAmount == 0) {
            return expr;
        }
        return createAdvance(expr, mPb->getInteger(shiftAmount));
    }

    PabloAST * createAdvance(PabloAST * expr, PabloAST * shiftAmount);

    Next * createNext(Assign * assign, PabloAST * expr);

    PabloAST * createAnd(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createNot(PabloAST * expr);

    PabloAST * createOr(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createXor(PabloAST * expr1, PabloAST * expr2);

    PabloAST * createMatchStar(PabloAST * marker, PabloAST * charclass);

    PabloAST * createScanThru(PabloAST * from, PabloAST * thru);

    PabloAST * createSel(PabloAST * condition, PabloAST * trueExpr, PabloAST * falseExpr);

    inline If * createIf(PabloAST * condition, std::vector<Assign *> && definedVars, PabloBlock & body) {
        mPb->createIf(condition, std::move(definedVars), body);
    }

    inline While * createWhile(PabloAST * condition, PabloBlock & body) {
        mPb->createWhile(condition, body);
    }

private:

    PabloBlock *        mPb;
    ExpressionTable     mExprTable;
};


}


#endif // BUILDER_HPP
