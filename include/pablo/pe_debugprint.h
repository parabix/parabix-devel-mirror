/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_DEBUGPRINT_H
#define PE_DEBUGPRINT_H

#include <pablo/pabloAST.h>

namespace pablo {

class DebugPrint final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::DebugPrint;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~DebugPrint(){
    }
    PabloAST * getExpr() const {
        return getOperand(0);
    }
protected:
    DebugPrint(PabloAST * expr, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::DebugPrint, expr->getType(), {expr}, name, allocator) {
        setSideEffecting(true);
    }
};

}

#endif // PE_DEBUGPRINT_H
