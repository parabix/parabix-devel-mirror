/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPILER_HELPER_H
#define COMPILER_HELPER_H

namespace pablo {

class PabloE;

PabloE * make_not(PabloE * expr);
PabloE * make_and(PabloE * expr1, PabloE * expr2);
PabloE * make_or(PabloE * expr1, PabloE * expr2);
PabloE * make_sel(PabloE * if_expr, PabloE * t_expr, PabloE * f_expr);
PabloE * make_xor(PabloE * expr1, PabloE * expr2);
bool equals(const PabloE * expr1, const PabloE *expr2);

}

#endif // COMPILER_HELPER_H
