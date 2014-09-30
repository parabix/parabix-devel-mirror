/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SCANTHRU_H
#define PS_SCANTHRU_H

#include "ps_pablos.h"

namespace pablo {

class ScanThru : public  PabloE {
public:
    ScanThru(PabloE* from, PabloE* thru)
    : PabloE(ClassTypeId::ScanThru)
    , mScanFrom(from)
    , mScanThru(thru)
    {

    }

    virtual ~ScanThru() {
        delete mScanFrom;
        delete mScanThru;
    }

    PabloE * getScanFrom() const {
        return mScanFrom;
    }

    PabloE * getScanThru() const {
        return mScanThru;
    }
private:
    PabloE* mScanFrom;
    PabloE* mScanThru;
};

}

#endif // PS_SCANTHRU_H



