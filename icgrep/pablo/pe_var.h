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

class Var : public NamedPabloAST {
    friend class PabloBlock;
    friend class PabloAST;
    friend class PabloKernel;
    friend class Statement;
public:

    enum Attribute {
        None = 0x00
        , ReadOnly = 0x01
        , ReadNone = 0x02
        , Scalar = 0x04
        , KernelParameter = 0x80
        // Composite attributes
        , KernelInputParameter = ReadOnly | KernelParameter
        , KernelOutputParameter = ReadNone | KernelParameter
    };

    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Var;
    }
    static inline bool classof(const void *) {
        return false;
    }
    bool isReadOnly() const {
        return (mAttribute & Attribute::ReadOnly) != 0;
    }
    void setReadOnly(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::ReadOnly;
        } else {
            mAttribute &= ~(Attribute::ReadOnly);
        }
    }
    bool isReadNone() const {
        return (mAttribute & Attribute::ReadNone) != 0;
    }
    void setReadNone(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::ReadNone;
        } else {
            mAttribute &= ~(Attribute::ReadNone);
        }
    }
    bool isKernelParameter() const {
        return (mAttribute & Attribute::KernelParameter) != 0;
    }
    void setKernelParameter(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::KernelParameter;
        } else {
            mAttribute &= ~(Attribute::KernelParameter);
        }
    }
    bool isScalar() const {
        return (mAttribute & Attribute::Scalar) != 0;
    }
    void setScalar(const bool value = true) {
        if (value) {
            mAttribute |= Attribute::Scalar;
        } else {
            mAttribute &= ~(Attribute::Scalar);
        }
    }

    const String & getName() const final {
        assert (mName);
        return *mName;
    }

protected:
    Var(const String * name, llvm::Type * const type, Allocator & allocator, const Attribute attr = Attribute::None)
    : NamedPabloAST(ClassTypeId::Var, type, name, allocator)
    , mAttribute(attr) {

    }
private:
    unsigned mAttribute;
};

class Extract : public PabloAST {
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
    inline Var * getArray() const {
        return mArray;
    }
    inline PabloAST * getIndex() const {
        return mIndex;
    }
protected:
    Extract(Var * array, PabloAST * const index, llvm::Type * type, Allocator & allocator)
    : PabloAST(ClassTypeId::Extract, type, allocator)
    , mArray(array)
    , mIndex(index) {

    }
private:
    Var * const mArray;
    PabloAST * const mIndex;
};

}

#endif // PE_VAR_H


