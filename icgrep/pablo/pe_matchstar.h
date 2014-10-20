/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PE_MATCHSTAR_H
#define PE_MATCHSTAR_H

#include "pabloAST.h"

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
    inline PabloAST * getMarker() const {
        return mMarker;
    }
    inline PabloAST * getCharClass() const  {
        return mCC;
    }
protected:
    MatchStar(PabloAST * marker, PabloAST * cc)
    : PabloAST(ClassTypeId::MatchStar)
    , mMarker(marker)
    , mCC(cc)
    {

    }
private:
    PabloAST * const mMarker;
    PabloAST * const mCC;
};

}

#endif // PE_MATCHSTAR_H



