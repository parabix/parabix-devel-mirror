/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

/*
 *  Copyright © 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "cc_codegenobject.h"

CC_CodeGenObject::CC_CodeGenObject(std::string gensym_pattern)
{
    mGenSym_Template = gensym_pattern;
    mGenSymCounter = 0;
}

void CC_CodeGenObject::add_predefined(std::string key_value, Expression* mapped_value)
{
    mCommon_Expression_Map.insert(make_pair(key_value, mapped_value));
}

Expression* CC_CodeGenObject::add_assignment(std::string varname, Expression* expr)
{    
    //Add the new mapping to the list of pablo statements:
    mStmtsl.push_back(new Assign(varname, expr->pablo_expr));

    //Add the new mapping to the common expression map:
    std::string key_value = expr->expr_string;
    Expression* mapped_value = new Expression();
    mapped_value->expr_string = varname;
    mapped_value->pablo_expr = new Var(varname);

    std::pair<std::map<std::string, Expression*>::iterator, bool> ret = mCommon_Expression_Map.insert(make_pair(key_value, mapped_value));

    return ret.first->second;
}

Expression* CC_CodeGenObject::expr_to_variable(Expression* expr)
{
    if (mCommon_Expression_Map.count(expr->expr_string) > 0)
    {
        std::map<std::string, Expression*>::iterator itGet = mCommon_Expression_Map.find(expr->expr_string);
        return itGet->second;
    }
    else
    {
        mGenSymCounter++;
        std::string sym = mGenSym_Template + std::to_string(mGenSymCounter);
        return add_assignment(sym, expr);
    }
}

std::list<PabloS*> CC_CodeGenObject::get_stmtsl()
{
    return mStmtsl;
}


