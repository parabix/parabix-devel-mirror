/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CC_CODEGENOBJECT_H
#define CC_CODEGENOBJECT_H

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"

#include <iostream>
#include <sstream>
#include <string>
#include <list>
#include <map>

#define INT2STRING(i) static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str()

struct Expression{
    std::string expr_string;
    PabloE* pablo_expr;
};

class CC_CodeGenObject
{
public:
    CC_CodeGenObject(std::string gensym_pattern);
    void add_predefined(std::string key_value, Expression *mapped_value);
    Expression* add_assignment(std::string value, Expression* expr);
    Expression* expr_to_variable(Expression* cgo);
    std::list<PabloS*> get_stmtsl();
private:
    std::string mGenSym_Template;
    int mGenSymCounter;
    std::list<PabloS*> mStmtsl;
    std::map<std::string, Expression*> mCommon_Expression_Map;
};

#endif // CC_CODEGENOBJECT_H
