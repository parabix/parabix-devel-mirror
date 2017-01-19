/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_VAR_H
#define PE_VAR_H

#include <pablo/pabloAST.h>
#include <pablo/pe_string.h>

namespace pablo {

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
    bool isReadOnly() const {
        return mReadOnly;
    }
    void setReadOnly(const bool value = true) {
        mReadOnly = value;
    }
protected:
    Var(const PabloAST * name, llvm::Type * const type, Allocator & allocator, const bool readOnly = false)
    : PabloAST(ClassTypeId::Var, type, llvm::cast<String>(name), allocator)
    , mReadOnly(readOnly) {

    }
private:
    bool mReadOnly;
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
    Extract(PabloAST * array, PabloAST * const index, const String * const name, llvm::Type * type, Allocator & allocator)
    : Statement(ClassTypeId::Extract, type, {array, index}, name, allocator) {

    }
};

}

#endif // PE_VAR_H


