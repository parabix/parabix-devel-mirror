#ifndef RE_INTERSECT_H
#define RE_INTERSECT_H

#include <re/re_re.h>

namespace re {

class Intersect : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Intersect;
    }
    static inline bool classof(const void *) {
        return false;
    }
    RE * getLH() const {
        return mLh;
    }
    RE * getRH() const {
        return mRh;
    }
    friend RE * makeIntersect(RE*, RE*);
protected:
    Intersect(RE * lh, RE * rh)
    : RE(ClassTypeId::Intersect)
    , mLh(lh)
    , mRh(rh)
    {

    }
    virtual ~Intersect() {}
private:
    RE * const mLh;
    RE * const mRh;
};

RE * makeIntersect(RE * lh, RE * rh);

}

#endif // RE_INTERSECT_H
