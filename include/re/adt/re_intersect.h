#ifndef RE_INTERSECT_H
#define RE_INTERSECT_H

#include <re/adt/re_re.h>

namespace re {

class Intersect : public RE {
public:
    RE * getLH() const {return mLh;}
    RE * getRH() const {return mRh;}
    static Intersect * Create(RE * lh, RE * rh) {return new Intersect(lh, rh);}
    RE_SUBTYPE(Intersect)
private:
    Intersect(RE * lh, RE * rh): RE(ClassTypeId::Intersect), mLh(lh), mRh(rh) {}
    RE * const mLh;
    RE * const mRh;
};

RE * makeIntersect(RE * lh, RE * rh);

}

#endif // RE_INTERSECT_H
