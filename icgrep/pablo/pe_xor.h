/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef XOR_H
#define XOR_H

#include <pablo/pabloAST.h>

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
    inline PabloAST * getExpr1() const {
        return mExpr1;
    }
    inline PabloAST * getExpr2() const {
        return mExpr2;
    }
protected:
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Xor(PabloAST * expr1, PabloAST * expr2)
    : PabloAST(ClassTypeId::Xor)
    , mExpr1(expr1)
    , mExpr2(expr2)
    {

    }
private:
    PabloAST * const mExpr1;
    PabloAST * const mExpr2;
};

struct OptimizeXor {
    inline OptimizeXor(PabloBlock & cg) : cg(cg) {}
    PabloAST * operator()(PabloAST * expr1, PabloAST * expr2);
private:
    PabloBlock & cg;
};

}

#endif // XOR_H



