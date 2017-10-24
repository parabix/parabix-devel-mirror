/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include <pablo/pabloAST.h>
#include <pablo/pe_integer.h>

namespace pablo {

class Advance final : public CarryProducingStatement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Advance;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Advance() {
    }
    inline PabloAST * getExpression() const {
        return getOperand(0);
    }
    inline int64_t getAmount() const {
        return llvm::cast<Integer>(getOperand(1))->value();
    }
protected:
    Advance(PabloAST * expr, PabloAST * shiftAmount, const String * name, Allocator & allocator)
    : CarryProducingStatement(ClassTypeId::Advance, expr->getType(), {expr, shiftAmount}, name, allocator) {
        assert(llvm::isa<Integer>(shiftAmount));
    }
};

class IndexedAdvance final : public CarryProducingStatement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::IndexedAdvance;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~IndexedAdvance() {
    }
    inline PabloAST * getExpression() const {
        return getOperand(0);
    }
    inline PabloAST * getIndex() const {
        return getOperand(1);
    }
    inline int64_t getAmount() const {
        return llvm::cast<Integer>(getOperand(2))->value();
    }
protected:
    IndexedAdvance(PabloAST * strm, PabloAST * index_strm, PabloAST * shiftAmount, const String * name, Allocator & allocator)
    : CarryProducingStatement(ClassTypeId::IndexedAdvance, strm->getType(), {strm, index_strm, shiftAmount}, name, allocator) {
        assert(llvm::isa<Integer>(shiftAmount));
    }
};


}

#endif // PE_ADVANCE_H



