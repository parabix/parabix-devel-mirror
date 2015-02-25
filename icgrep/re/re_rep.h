/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REP_H
#define RE_REP_H

#include "re_re.h"

namespace re {

class Rep : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Rep;
    }
    static inline bool classof(const void *) {
        return false;
    }
    enum { UNBOUNDED_REP = -1 };
    RE * getRE() const;
    void setRE(RE * re = nullptr);
    int getLB() const;
    void setLB(const int lb);
    int getUB() const;
    void setUB(const int ub);
    virtual ~Rep();
protected:
    friend RE * makeRep(RE *, const int, const int);
    Rep(RE * re, const int lb, const int ub);
private:
    RE* mRE;
    int mLB;
    int mUB;
};

inline Rep::Rep(RE * re, const int lb, const int ub)
: RE(ClassTypeId::Rep)
, mRE(re)
, mLB(lb)
, mUB(ub)
{

}

inline Rep::~Rep() {
    delete mRE;
}

inline RE * Rep::getRE() const {
    return mRE;
}

inline void Rep::setRE(RE * re) {
    mRE = re;
}

inline int Rep::getLB() const {
    return mLB;
}

inline void Rep::setLB(const int lb) {
    mLB = lb;
}

inline int Rep::getUB() const {
    return mUB;
}

inline void Rep::setUB(const int ub) {
    mUB = ub;
}

RE * makeRep(RE * re, const int lower_bound, const int upper_bound);

}

#endif
