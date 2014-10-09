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

class Assign : public PabloE {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::Assign;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~Assign() {
    }
    inline const std::string & getName() const {
        return *mName;
    }
    inline PabloE * getExpr() const {
        return mExpr;
    }
protected:
    Assign(PabloE * name, PabloE * expr)
    : PabloE(ClassTypeId::Assign)
    , mName(cast<String>(name))
    , mExpr(expr)
    {

    }
private:
    const String * const    mName;
    PabloE * const          mExpr;
};

}

#endif // PS_SETMARKER_H

