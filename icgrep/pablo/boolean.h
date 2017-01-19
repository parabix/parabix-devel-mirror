#ifndef BOOLEAN_H
#define BOOLEAN_H

#include <pablo/pabloAST.h>

namespace pablo {

class And : public Variadic {
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
    : Variadic(ClassTypeId::And, type, {expr1, expr2}, name, allocator)
    {

    }
    And(llvm::Type * const type, const unsigned reserved, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::And, type, reserved, name, allocator)
    {

    }
    template<typename iterator>
    And(llvm::Type * const type, iterator begin, iterator end, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::And, type, begin, end, name, allocator) {

    }
};

class Or : public Variadic {
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
    : Variadic(ClassTypeId::Or, type, {expr1, expr2}, name, allocator)
    {

    }
    Or(llvm::Type * const type, const unsigned reserved, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::Or, type, reserved, name, allocator)
    {

    }
    template<typename iterator>
    Or(llvm::Type * const type, iterator begin, iterator end, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::Or, type, begin, end, name, allocator) {

    }
};

class Xor : public Variadic {
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
    : Variadic(ClassTypeId::Xor, type, {expr1, expr2}, name, allocator)
    {

    }
    Xor(llvm::Type * const type, const unsigned reserved, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::Xor, type, reserved, name, allocator)
    {

    }
    template<typename iterator>
    Xor(llvm::Type * const type, iterator begin, iterator end, const String * name, Allocator & allocator)
    : Variadic(ClassTypeId::Xor, type, begin, end, name, allocator) {

    }
};

class Not : public Statement {
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

class Sel : public Statement {
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

}

#endif // BOOLEAN_H
