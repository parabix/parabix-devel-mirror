/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_SEL_H
#define PE_SEL_H

#include "pe_pabloe.h"

namespace pablo {

class Sel : public PabloE
{
public:
    Sel(PabloE* if_expr, PabloE* t_expr, PabloE* f_expr)
    : PabloE(ClassTypeId::Sel)
    , mIf_expr(if_expr)
    , mT_expr(t_expr)
    , mF_expr(f_expr)
    {

    }

    virtual ~Sel() {
        delete mIf_expr;
        delete mT_expr;
        delete mF_expr;
    }

    inline PabloE * getIf_expr() const {
        return mIf_expr;
    }

    inline PabloE * getT_expr() const {
        return mT_expr;
    }

    inline PabloE * getF_expr() const {
        return mF_expr;
    }

private:
    PabloE* mIf_expr;
    PabloE* mT_expr;
    PabloE* mF_expr;
};

}

#endif // PE_SEL_H

