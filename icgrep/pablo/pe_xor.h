/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include <pablo/pabloAST.h>
#include <array>

namespace pablo {

class PabloBlock;

class Xor : public PabloAST {
    friend struct OptimizeXor;
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Xor() {
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index < 2);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 2;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 2);
        mExprs[index] = value;
    }
    PabloAST * getExpr1() const {
        return mExprs[0];
    }
    PabloAST * getExpr2() const {
        return mExprs[1];
    }
protected:
    Xor(PabloAST * expr1, PabloAST * expr2)
    : PabloAST(ClassTypeId::Xor)
    , mExprs({{expr1, expr2}})
    {

    }
private:
    std::array<PabloAST*, 2> mExprs;
};

struct OptimizeXor {
    inline OptimizeXor(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2);
private:
    PabloBlock & cg;
};

}

#endif // XOR_H



