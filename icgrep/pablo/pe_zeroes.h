/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_ZEROES_H
#define PE_ZEROES_H

#include <pablo/pabloAST.h>
#include <pablo/pe_var.h>

namespace pablo {

class Zeroes : public PabloAST {
    friend class PabloBlock;
    friend class PabloFunction;
public:
    static inline bool classof(const PabloAST * expr) {
        assert (expr);
        return expr->getClassTypeId() == ClassTypeId::Zeroes;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Zeroes() {
    }
    inline bool operator==(const Zeroes &) const {
        return true;
    }
    virtual bool operator==(const PabloAST & other) const {
        return isa<Zeroes>(other);
    }
protected:
    Zeroes(const PabloType * const type) : PabloAST(ClassTypeId::Zeroes, type) { }
};

}

#endif // PE_ZEROES_H


