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
    friend class PabloFunction;
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
    inline Integer * getIndex() const {
        return cast<Integer>(getOperand(1));
    }
protected:
    Extract(PabloAST * array, PabloAST * const index, const String * const name)
    : Statement(ClassTypeId::Extract, cast<ArrayType>(array->getType())->getArrayElementType(), {array, cast<Integer>(index)}, name) {

    }
};

//class Extract : public PabloAST {
//    friend class PabloBlock;
//public:
//    static inline bool classof(const PabloAST * e) {
//        return e->getClassTypeId() == ClassTypeId::Extract;
//    }
//    static inline bool classof(const void *) {
//        return false;
//    }
//    virtual ~Extract(){
//    }
//    inline const String * getName() const {
//        if (isa<Parameter>(mArray)) {
//            return cast<Parameter>(mArray)->getName();
//        } else {
//            return cast<Statement>(mArray)->getName();
//        }
//    }
//    inline const PabloAST * getArray() const {
//        return mArray;
//    }
//    inline const Integer * getIndex() const {
//        return mIndex;
//    }
//protected:
//    Extract(PabloAST * array, PabloAST * const index)
//    : PabloAST(ClassTypeId::Extract, array->getType()->getIndexedType())
//    , mArray(array)
//    , mIndex(cast<Integer>(index)) {
//        assert (isa<Parameter>(array) || isa<Statement>(array));
//    }
//private:
//    PabloAST * mArray;
//    Integer * mIndex;
//};

//class ParameterBinding {
//    using Allocator = SlabAllocator<Parameter *>;

//public:

//    void add(Parameter * const param);

//private:
//    std::vector<Parameter *, Allocator> mParameters;
//};


}

#endif // PE_VAR_H


