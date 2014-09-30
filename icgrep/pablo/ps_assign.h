/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include "pe_pabloe.h"
#include <string>

namespace pablo {

class Assign : public PabloE {
public:
    Assign(std::string m, PabloE* expr)
    : PabloE(ClassTypeId::Assign)
    , mM(m)
    , mExpr(expr)
    {

    }

    virtual ~Assign() {
        delete mExpr;
    }

    inline std::string getM() const {
        return mM;
    }

    inline PabloE* getExpr() const {
        return mExpr;
    }
private:
    std::string mM;
    PabloE* mExpr;
};

}

#endif // PS_SETMARKER_H

