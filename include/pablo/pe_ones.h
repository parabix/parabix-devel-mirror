/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ONES_H
#define PE_ONES_H

#include <pablo/pabloAST.h>

namespace pablo {

class Ones final : public PabloAST {
    friend class PabloBlock;
    friend class PabloKernel;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::Ones;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Ones() {
    }
    inline bool operator==(const Ones &) const {
        return true;
    }
    virtual bool operator==(const PabloAST & other) const {
        return llvm::isa<Ones>(other);
    }
protected:
    Ones(llvm::Type * const type, Allocator & allocator)
    : PabloAST(ClassTypeId::Ones, type, allocator) {
    }
};

}

#endif // PE_ONES_H


