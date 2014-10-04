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
    friend Assign * makeAssign(const std::string, PabloE *);
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
        delete mExpr;
    }

    inline const std::string & getM() const {
        return mM;
    }

    inline PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Assign(const std::string m, PabloE * expr)
    : PabloE(ClassTypeId::Assign)
    , mM(m)
    , mExpr(expr)
    {

    }
private:
    const std::string   mM;
    PabloE * const      mExpr;
};

inline Assign * makeAssign(const std::string marker, PabloE * expr) {
    return new Assign(marker, expr);
}

}

#endif // PS_SETMARKER_H

