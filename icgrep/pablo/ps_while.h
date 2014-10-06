/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PS_WHILE_H
#define PS_WHILE_H

#include <pablo/pe_pabloe.h>
#include <list>

namespace pablo {

class While : public PabloE {
    friend struct CodeGenState;
public:
    typedef std::list<PabloE*> List;
    friend While * makeWhile(PabloE * expr, List psl);

    static inline bool classof(const PabloE * e) {
        return e->getClassTypeId() == ClassTypeId::While;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~While() {
    }
    inline PabloE * getExpr() const {
        return mExpr;
    }
    inline const List & getPSList() const {
        return mPSList;
    }
protected:
    While(PabloE * expr, List psl)
    : PabloE(ClassTypeId::While)
    , mExpr(expr)
    , mPSList(psl)
    {

    }
private:
    PabloE * const  mExpr;
    List            mPSList;
};

inline While * makeWhile(PabloE * cond, While::List statements) {
    return new While(cond, statements);
}

}

#endif // PS_WHILE_H


