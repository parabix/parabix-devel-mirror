/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_INFILE_H
#define PE_INFILE_H

#include <pablo/pabloAST.h>

namespace pablo {

class InFile : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::InFile;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~InFile(){
    }
    PabloAST * getExpr() const {
        return getOperand(0);
    }
protected:
    InFile(PabloAST * expr, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::InFile, expr->getType(), {expr}, name, allocator) {

    }
};

class AtEOF : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::AtEOF;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~AtEOF(){
    }
    PabloAST * getExpr() const {
        return getOperand(0);
    }
protected:
    AtEOF(PabloAST * expr, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::AtEOF, expr->getType(), {expr}, name, allocator) {

    }
};
    
}

#endif // PE_INFILE_H
