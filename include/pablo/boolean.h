#ifndef BOOLEAN_H
#define BOOLEAN_H

#include <pablo/pabloAST.h>
#include <pablo/pe_integer.h>

namespace pablo {

class And final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::And;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~And() { }
protected:
    And(llvm::Type * const type, PabloAST * expr1, PabloAST * expr2, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::And, type, {expr1, expr2}, name, allocator)
    {

    }
};

class Or final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Or;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Or() { }
protected:
    Or(llvm::Type * const type, PabloAST * expr1, PabloAST * expr2, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Or, type, {expr1, expr2}, name, allocator)
    {

    }
};

class Xor final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Xor;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    Xor(llvm::Type * const type, PabloAST * expr1, PabloAST * expr2, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Xor, type, {expr1, expr2}, name, allocator)
    {

    }
};

class Not final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Not;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Not() {
    }
    PabloAST * getExpr() const {
        return getOperand(0);
    }
protected:
    Not(PabloAST * expr, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Not, expr->getType(), {expr}, name, allocator)
    {

    }
};

class Sel final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Sel;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Sel() {
    }
    inline PabloAST * getCondition() const {
        return getOperand(0);
    }
    inline PabloAST * getTrueExpr() const {
        return getOperand(1);
    }
    inline PabloAST * getFalseExpr() const {
        return getOperand(2);
    }
protected:
    Sel(PabloAST * condExpr, PabloAST * trueExpr, PabloAST * falseExpr, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Sel, trueExpr->getType(), {condExpr, trueExpr, falseExpr}, name, allocator) {

    }
};

class Ternary final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Ternary;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Ternary() {
    }
    inline Integer * getMask() const {
        return llvm::cast<Integer>(getOperand(0));
    }
    inline PabloAST * getA() const {
        return getOperand(1);
    }
    inline PabloAST * getB() const {
        return getOperand(2);
    }
    inline PabloAST * getC() const {
        return getOperand(3);
    }
protected:
    Ternary(PabloAST * mask, PabloAST * a, PabloAST * b, PabloAST * c, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::Ternary, a->getType(), {mask, a, b, c}, name, allocator) {
        assert(llvm::isa<Integer>(mask));
    }
};

}

#endif // BOOLEAN_H
