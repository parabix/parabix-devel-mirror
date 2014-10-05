/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_SETMARKER_H
#define PS_SETMARKER_H

#include <pablo/pe_pabloe.h>
#include <pablo/pe_string.h>

namespace pablo {

class Var;

class Assign : public PabloE {
    friend Assign * makeAssign(const String *, PabloE *);
    friend Var * makeVar(const Assign *);
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

    inline const std::string & getName() const {
        return *mName;
    }

    inline PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Assign(const String * name, PabloE * expr)
    : PabloE(ClassTypeId::Assign)
    , mName(name)
    , mExpr(expr)
    {

    }
private:
    const String * const    mName;
    PabloE * const          mExpr;
};

inline Assign * makeAssign(const String * marker, PabloE * expr) {
    return new Assign(marker, expr);
}

}

#endif // PS_SETMARKER_H

