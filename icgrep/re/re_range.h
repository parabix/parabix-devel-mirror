#ifndef RE_RANGE_H
#define RE_RANGE_H

#include <re/re_re.h>

//
// Ranges generally represent CCs, but might not be able to be
// converted to CCs during parsing, if one or both endpoints are
// symbolic.   [\N{some name}-\N{some other name}],  [\q(ch)-k]
namespace re {

class Range : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Range;
    }
    static inline bool classof(const void *) {
        return false;
    }
    RE * getLo() const {
        return mLo;
    }
    void setLo(RE * lh) {
        mLo = lh;
    }
    RE * getHi() const {
        return mHi;
    }
    void setHi(RE * rh) {
        mHi = rh;
    }
protected:
    friend RE * makeRange(RE*, RE*);
    Range(RE * lh, RE * rh)
    : RE(ClassTypeId::Range)
    , mLo(lh)
    , mHi(rh)
    {

    }
    virtual ~Range() {}
private:
    RE * mLo;
    RE * mHi;
};

RE * makeRange(RE * lh, RE * rh);
}

#endif // RE_RANGE_H
