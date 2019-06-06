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
    friend class Extract;
    friend class PabloBlock;
    friend class PabloAST;
    friend class PabloKernel;
    friend class Statement;
public:

    enum Attribute : unsigned {
        None = 0x00
        , ReadOnly = 0x01
        , ReadNone = 0x02
        , Scalar = 0x04
        , KernelParameter = 0x80
        // Composite attributes
        , KernelInputStream = ReadOnly | KernelParameter
        , KernelInputScalar = KernelInputStream | Scalar
        , KernelOutputStream = ReadNone | KernelParameter
        , KernelOutputScalar = KernelOutputStream | Scalar
    };

    static inline bool classof(const PabloAST * e) {
        switch (e->getClassTypeId()) {
            case ClassTypeId::Var:
            case ClassTypeId::Extract:
                return true;
            default:
                return false;
        }
    }
    static inline bool classof(const void *) {
        return false;
    }
    bool isReadOnly() const {
        return (mAttribute & Attribute::ReadOnly) != 0;
    }
    bool isReadNone() const {
        return (mAttribute & Attribute::ReadNone) != 0;
    }

    bool isKernelParameter() const {
        return (mAttribute & Attribute::KernelParameter) != 0;
    }

    bool isScalar() const {
        return (mAttribute & Attribute::Scalar) != 0;
    }

    const String & getName() const override {
        assert (mName);
        return *mName;
    }

protected:
    Var(llvm::Type * const type, const String * name, Allocator & allocator, const Attribute attr = Attribute::None)
    : Var(ClassTypeId::Var, type, name, allocator, attr) {

    }

    explicit Var(const ClassTypeId typeId, llvm::Type * const type, const String * name, Allocator & allocator, const Attribute attr)
    : NamedPabloAST(typeId, type, name, allocator)
    , mAttribute(attr) {

    }

    Attribute getAttribute() const {
        return mAttribute;
    }

private:
    const Attribute mAttribute;
};

class Extract final : public Var {
    friend class PabloBlock;
    friend class PabloKernel;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Extract;
    }
    static inline bool classof(const void *) {
        return false;
    }
    inline Var * getArray() const {
        return mArray;
    }
    inline PabloAST * getIndex() const {
        return mIndex;
    }
    const String & getName() const final {
        llvm_unreachable("Extract::getName() not supported");
    }

protected:
    Extract(llvm::Type * type, Var * array, PabloAST * const index, Allocator & allocator)
    : Var(ClassTypeId::Extract, type, nullptr, allocator, array->getAttribute())
    , mArray(array)
    , mIndex(index) {

    }
private:
    Var * const mArray;
    PabloAST * const mIndex;
};

}

#endif // PE_VAR_H


