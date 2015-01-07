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
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index < 2);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 2;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 2);
        mExprs[index] = value;
    }
    PabloAST * getScanFrom() const {
        return mExprs[0];
    }
    PabloAST * getScanThru() const {
        return mExprs[1];
    }
    ScanThru(PabloAST * from, PabloAST * thru, SymbolGenerator * sg, PabloBlock * parent)
    : Statement(ClassTypeId::ScanThru, sg->make("scanthru"), parent)
    , mExprs({{from, thru}})
    {
        from->addUser(this);
        thru->addUser(this);
    }
private:
    std::array<PabloAST*, 2> mExprs;
};

}

#endif // PS_SCANTHRU_H



