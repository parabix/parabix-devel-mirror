/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pabloAST.h"
#include <array>

namespace pablo {

class MatchStar : public PabloAST {
    friend class PabloBlock;
public:
    static inline bool classof(const PabloAST * e) {
        return e->getClassTypeId() == ClassTypeId::MatchStar;
    }
    static inline bool classof(const void *) {
        return false;
    }
    virtual ~MatchStar() {
    }
    virtual PabloAST * getOperand(const unsigned index) const {
        assert (index < 2);
        return mExprs[index];
    }
    virtual unsigned getNumOperands() const {
        return 2;
    }
    virtual void setOperand(const unsigned index, PabloAST * value) {
        assert (index < 2);
        mExprs[index] = value;
    }
    inline PabloAST * getMarker() const {
        return mExprs[0];
    }
    inline PabloAST * getCharClass() const  {
        return mExprs[1];
    }
protected:
    MatchStar(PabloAST * marker, PabloAST * cc)
    : PabloAST(ClassTypeId::MatchStar)
    , mExprs({{marker, cc}})
    {
        marker->addUser(this);
        cc->addUser(this);
    }
private:
    std::array<PabloAST*, 2> mExprs;
};

}

#endif // PE_MATCHSTAR_H



