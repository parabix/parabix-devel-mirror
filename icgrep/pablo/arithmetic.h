#ifndef ARITHMETIC_H
#define ARITHMETIC_H

#include <pablo/pabloAST.h>

namespace pablo {

class Add : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Add;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Add() { }
protected:
    Add(Type * const type, PabloAST * expr1, PabloAST * expr2, const String * name)
    : Statement(ClassTypeId::And, type, {expr1, expr2}, name)
    {

    }
};

class Subtract : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Subtract;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Subtract() { }
protected:
    Subtract(Type * const type, PabloAST * expr1, PabloAST * expr2, const String * name)
    : Statement(ClassTypeId::And, type, {expr1, expr2}, name)
    {

    }
};


}

#endif // ARITHMETIC_H
