/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_REP_H
#define RE_REP_H

#include "re_re.h"

class Rep : public RE {
public:

    enum { UNBOUNDED_REP = -1 };

    Rep(RE* re, int lb, int ub);
    ~Rep();
    RE * getRE() const;
    void setRE(RE * re = nullptr);
    int getLB() const;
    void setLB(int lb);
    int getUB() const;
    void setUB(int ub);
private:
    RE* mRE;
    int mLB;
    int mUB;
};

inline Rep::Rep(RE * re, int lb, int ub)
: mRE(re)
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

inline void Rep::setLB(int lb) {
    mLB = lb;
}

inline int Rep::getUB() const {
    return mUB;
}

inline void Rep::setUB(int ub) {
    mUB = ub;
}

#endif
