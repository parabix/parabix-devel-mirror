/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pe_scanthru.h"

ScanThru::ScanThru(PabloE* from, PabloE* thru)
{
    mScanFrom = from;
    mScanThru = thru;
}

ScanThru::~ScanThru()
{
    delete mScanFrom;
    delete mScanThru;
}

PabloE* ScanThru::getScanFrom()
{
    return mScanFrom;
}

PabloE* ScanThru::getScanThru()
{
    return mScanThru;
}


