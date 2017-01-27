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

    enum Attribute {
        None = 0
        , ReadOnly = 1
        , ReadNone = 2
    };

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
    bool isReadOnly() const {
        return mAttribute & Attribute::ReadOnly;
    }
    void setReadOnly(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::ReadOnly;
        } else {
            mAttribute &= ~(Attribute::ReadOnly);
        }
    }
    bool isReadNone() const {
        return mAttribute & Attribute::ReadNone;
    }
    void setReadNone(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::ReadNone;
        } else {
            mAttribute &= ~(Attribute::ReadNone);
        }
    }

    const String & getName() const noexcept {
        return *mName;
    }

protected:
    Var(const String * name, llvm::Type * const type, Allocator & allocator, const Attribute attr = Attribute::None)
    : PabloAST(ClassTypeId::Var, type, allocator)
    , mAttribute(attr)
    , mName(name) {

    }
private:
    unsigned mAttribute;
    const String * const mName;
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


