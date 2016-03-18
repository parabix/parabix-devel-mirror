/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_LOOKAHEAD_H
#define PE_LOOKAHEAD_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <pablo/pe_integer.h>

namespace pablo {

class Lookahead : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Lookahead;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Lookahead() {
    }
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    inline Integer::Type getAmount() const {
        return cast<Integer>(getOperand(1))->value();
    }
    inline void setLocalIndex(const unsigned idx) {
        localLookaheadIndex = idx;
    }
    inline unsigned getLocalIndex() const {
        return localLookaheadIndex;
    }
protected:
    Lookahead(PabloAST * expr, PabloAST * shiftAmount, String * name)
    : Statement(ClassTypeId::Lookahead, {expr, shiftAmount}, name)
    {
        assert(isa<Integer>(shiftAmount));
    }
private:
    unsigned localLookaheadIndex;
};

}

#endif // PE_LOOKAHEAD_H



