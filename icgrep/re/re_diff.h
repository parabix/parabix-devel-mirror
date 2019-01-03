#ifndef RE_DIFF_H
#define RE_DIFF_H

#include <re/re_re.h>

namespace re {

class Diff : public RE {
public:
    RE * getLH() const {return mLh;}
    RE * getRH() const {return mRh;}
    static Diff * Create(RE * lh, RE * rh) {return new Diff(lh, rh);}
    RE_SUBTYPE(Diff)
private:
    Diff(RE * lh, RE * rh): RE(ClassTypeId::Diff), mLh(lh), mRh(rh) {}
    RE * const mLh;
    RE * const mRh;
};

RE * makeDiff(RE * lh, RE * rh);
    
RE * resolveDiffs(RE * r);
}

#endif // RE_DIFF_H
