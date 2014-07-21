/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PRINTER_RE_H
#define PRINTER_RE_H

//Regular Expressions
#include "re_re.h"
#include "re_alt.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_end.h"
#include "re_rep.h"
#include "re_seq.h"
#include "re_start.h"

#include <iostream>
#include <string>
#include <sstream>
#include <list>

#define INT2STRING(i) static_cast<std::ostringstream*>(&(std::ostringstream() << i))->str()

class Printer_RE
{
public:
    static std::string PrintRE(RE* re);
};

#endif // PRINTER_RE_H
