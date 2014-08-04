/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "llvm_gen_helper.h"

LLVM_Generator_Helper::LLVM_Generator_Helper(){}

int LLVM_Generator_Helper::CarryCount_PabloStatements(std::list<PabloS*> stmts)
{
    int retVal = 0;

    std::list<PabloS*>::iterator it;
    for (it = stmts.begin(); it != stmts.end(); ++it)
    {
        retVal += CarryCount_PabloS(*it);
    }

    return retVal;
}

int LLVM_Generator_Helper::CarryCount_PabloS(PabloS* stmt)
{
    int retVal = 0;

    if (Assign* sm = dynamic_cast<Assign*>(stmt))
    {
        retVal = CarryCount_PabloE(sm->getExpr());
    }
    else if (While* whl = dynamic_cast<While*>(stmt))
    {
        retVal = CarryCount_PabloE(whl->getExpr());
        retVal += CarryCount_PabloStatements(whl->getPSList());
    }

    return retVal;
}

int LLVM_Generator_Helper::CarryCount_PabloE(PabloE* expr)
{
    int retVal = 0;

    if (And* pablo_and = dynamic_cast<And*>(expr))
    {
        retVal =  CarryCount_PabloE(pablo_and->getExpr1()) + CarryCount_PabloE(pablo_and->getExpr2());
    }
    else if (Or* pablo_or = dynamic_cast<Or*>(expr))
    {
        retVal = CarryCount_PabloE(pablo_or->getExpr1()) + CarryCount_PabloE(pablo_or->getExpr2());
    }
    else if (Not* pablo_not = dynamic_cast<Not*>(expr))
    {
        retVal = CarryCount_PabloE(pablo_not->getExpr());
    }
    else if (Advance* adv = dynamic_cast<Advance*>(expr))
    {
        //Carry queues are needed for advances.
        retVal = 1 + CarryCount_PabloE(adv->getExpr());
    }
    else if(MatchStar* mstar = dynamic_cast<MatchStar*>(expr))
    {
        //Carry queues are also needed for MatchStar.
        retVal = 1 + CarryCount_PabloE(mstar->getExpr1()) + CarryCount_PabloE(mstar->getExpr2());
    }
    else if (ScanThru* sthru = dynamic_cast<ScanThru*>(expr))
    {
        retVal = 1 + CarryCount_PabloE(sthru->getScanFrom()) + CarryCount_PabloE(sthru->getScanThru());
    }

    return retVal;
}


