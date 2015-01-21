/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SCANTHRU_H
#define PS_SCANTHRU_H

#include <pablo/pabloAST.h>
#include <pablo/symbol_generator.h>
#include <array>

namespace pablo {

class ScanThru : public Statement {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::ScanThru;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~ScanThru() {
    }
    PabloAST * getScanFrom() const {
        return getOperand(0);
    }
    PabloAST * getScanThru() const {
        return getOperand(1);
    }
protected:
    ScanThru(PabloAST * from, PabloAST * thru, SymbolGenerator * sg, PabloBlock * parent)
    : Statement(ClassTypeId::ScanThru, {from, thru}, sg->make("scanthru"), parent)
    {

    }
};

}

#endif // PS_SCANTHRU_H



