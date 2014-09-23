/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include "pe_pabloe.h"

class Sel : public PabloE
{
public:
    Sel(PabloE* if_expr, PabloE* t_expr, PabloE* f_expr);
    ~Sel();
    PabloE* getIf_expr() const;
    PabloE* getT_expr() const;
    PabloE* getF_expr() const;
private:
    PabloE* mIf_expr;
    PabloE* mT_expr;
    PabloE* mF_expr;
};

#endif // PE_SEL_H

