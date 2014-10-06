/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "llvm_gen_helper.h"

//Pablo Expressions
#include <pablo/codegenstate.h>
#include <pablo/pe_pabloe.h>
#include <pablo/pe_advance.h>
#include <pablo/pe_and.h>
#include <pablo/pe_charclass.h>
#include <pablo/pe_not.h>
#include <pablo/pe_or.h>
#include <pablo/pe_matchstar.h>
#include <pablo/pe_scanthru.h>

//Pablo Statements
#include <pablo/ps_assign.h>
#include <pablo/ps_if.h>
#include <pablo/ps_while.h>

using namespace pablo;

LLVM_Generator_Helper::LLVM_Generator_Helper(){}

int LLVM_Generator_Helper::CarryCount_PabloStatements(const ExpressionList & stmts)
{
    int retVal = 0;
    for (auto it = stmts.begin(); it != stmts.end(); ++it)
    {
        retVal += CarryCount_PabloS(*it);
    }

    return retVal;
}

int LLVM_Generator_Helper::CarryCount_PabloS(PabloE *stmt)
{
    int retVal = 0;

    if (Assign* sm = dyn_cast<Assign>(stmt))
    {
        retVal = CarryCount_PabloE(sm->getExpr());
    }
    else if (If* ifstmt = dyn_cast<If>(stmt)) {
        retVal = CarryCount_PabloE(ifstmt->getExpr());
        retVal += CarryCount_PabloStatements(ifstmt->getPSList());
        // If there is more than one internal carry, we create a stored
        // accumulator value that Ors together all the carries, so add 1
        // to the carry count.
        if (retVal > 1) retVal++;
    }
    else if (While* whl = dyn_cast<While>(stmt))
    {
        retVal = CarryCount_PabloE(whl->getCondition());
        retVal += CarryCount_PabloStatements(whl->getPSList());
    }

    return retVal;
}

int LLVM_Generator_Helper::CarryCount_PabloE(PabloE* expr)
{
    int retVal = 0;

    if (And* pablo_and = dyn_cast<And>(expr))
    {
        retVal =  CarryCount_PabloE(pablo_and->getExpr1()) + CarryCount_PabloE(pablo_and->getExpr2());
    }
    else if (Or* pablo_or = dyn_cast<Or>(expr))
    {
        retVal = CarryCount_PabloE(pablo_or->getExpr1()) + CarryCount_PabloE(pablo_or->getExpr2());
    }
    else if (Not* pablo_not = dyn_cast<Not>(expr))
    {
        retVal = CarryCount_PabloE(pablo_not->getExpr());
    }
    else if (Advance* adv = dyn_cast<Advance>(expr))
    {
        //Carry queues are needed for advances.
        retVal = 1 + CarryCount_PabloE(adv->getExpr());
    }
    else if(MatchStar* mstar = dyn_cast<MatchStar>(expr))
    {
        //Carry queues are also needed for MatchStar.
        retVal = 1 + CarryCount_PabloE(mstar->getExpr1()) + CarryCount_PabloE(mstar->getExpr2());
    }
    else if (ScanThru* sthru = dyn_cast<ScanThru>(expr))
    {
        retVal = 1 + CarryCount_PabloE(sthru->getScanFrom()) + CarryCount_PabloE(sthru->getScanThru());
    }

    return retVal;
}


