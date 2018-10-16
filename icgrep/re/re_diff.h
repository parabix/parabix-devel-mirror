#ifndef RE_DIFF_H
#define RE_DIFF_H

#include <re/re_re.h>

namespace re {

class Diff : public RE {
public:
    static inline bool classof(const RE * re) {
        return re->getClassTypeId() == ClassTypeId::Diff;
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
protected:
    friend RE * makeDiff(RE*, RE*);
    Diff(RE * lh, RE * rh)
    : RE(ClassTypeId::Diff)
    , mLh(lh)
    , mRh(rh)
    {

    }
    virtual ~Diff() {}
private:
    RE * const mLh;
    RE * const mRh;
};

RE * makeDiff(RE * lh, RE * rh);
}

#endif // RE_DIFF_H
