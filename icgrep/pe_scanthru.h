#ifndef PS_SCANTHRU_H
#define PS_SCANTHRU_H

#include "ps_pablos.h"

class ScanThru :public  PabloE
{
public:
    ScanThru(PabloE* from , PabloE* thru);
    ~ScanThru();
    PabloE* getScanFrom();
    PabloE* getScanThru();
private:
    PabloE* mScanFrom;
    PabloE* mScanThru;
};

#endif // PS_SCANTHRU_H



