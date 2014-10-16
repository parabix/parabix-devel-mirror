/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SCANTHRU_H
#define PS_SCANTHRU_H

#include <pablo/pabloAST.h>

namespace pablo {

class ScanThru : public  PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::ScanThru;
    }
    static inline bool classof(const void *) {
        return false;
    }
    ScanThru(PabloAST * from, PabloAST * thru)
    : PabloAST(ClassTypeId::ScanThru)
    , mScanFrom(from)
    , mScanThru(thru)
    {

    }
    virtual ~ScanThru() {
    }
    PabloAST * getScanFrom() const {
        return mScanFrom;
    }
    PabloAST * getScanThru() const {
        return mScanThru;
    }
private:
    PabloAST * const mScanFrom;
    PabloAST * const mScanThru;
};

}

#endif // PS_SCANTHRU_H



