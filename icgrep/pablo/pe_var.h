/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>
#include <pablo/pe_integer.h>

namespace pablo {

class Assign;

// A Var is unique in that it is not a statement but a "lives" within a
// scope and cannot be accessed outside of it. A Var is mutable (via an
// Assign instruction.

class Var : public PabloAST {
    friend class PabloBlock;
    friend class PabloAST;
    friend class PabloKernel;
    friend class Statement;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
protected:
    Var(const PabloAST * name, Type * const type)
    : PabloAST(ClassTypeId::Var, type, cast<String>(name)) {

    }
};

class Extract : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Extract;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Extract(){
    }
    inline PabloAST * getArray() const {
        return getOperand(0);
    }
    inline PabloAST * getIndex() const {
        return getOperand(1);
    }
protected:
    Extract(PabloAST * array, PabloAST * const index, const String * const name, Type * type)
    : Statement(ClassTypeId::Extract, type, {array, index}, name) {

    }
};

}

#endif // PE_VAR_H


