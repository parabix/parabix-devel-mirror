/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ADVANCE_H
#define PE_ADVANCE_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <pablo/pe_integer.h>

namespace pablo {

class Advance : public Statement {
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
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    inline Integer::Type getAmount() const {
        return cast<Integer>(getOperand(1))->value();
    }
    inline void setLocalIndex(const unsigned idx) {
        localAdvanceIndex = idx;
    }
    inline unsigned getLocalIndex() const {
        return localAdvanceIndex;
    }
protected:
    Advance(PabloAST * expr, PabloAST * shiftAmount, String * name)
    : Statement(ClassTypeId::Advance, {expr, shiftAmount}, name)
    {
        assert(isa<Integer>(shiftAmount));
    }
private:
    unsigned localAdvanceIndex;
};

}

#endif // PE_ADVANCE_H



