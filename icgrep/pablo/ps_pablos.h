/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_PABLOS_H
#define PS_PABLOS_H

#include <list>
#include <string>

namespace pablo {

class PabloE;

struct CodeGenState{
    std::list<PabloE *>  stmtsl;
    std::string          newsym;
};

}

#endif // PS_PABLOS_H
