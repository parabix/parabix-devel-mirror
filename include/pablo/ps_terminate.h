/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_TERMINATE_H
#define PS_TERMINATE_H

#include <pablo/pabloAST.h>

namespace pablo {

class TerminateAt final : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::TerminateAt;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~TerminateAt() {
    }
    inline PabloAST * getExpr() const {
        return getOperand(0);
    }
    inline int32_t getSignalCode() const {
        return llvm::cast<Integer>(getOperand(1))->value();
    }
protected:
    explicit TerminateAt(PabloAST * strm, PabloAST * code, const String * name, Allocator & allocator)
    : Statement(ClassTypeId::TerminateAt, strm->getType(), {strm, code}, name, allocator) {
        setSideEffecting();
        assert(llvm::isa<Integer>(code));
    }
};

}

#endif
