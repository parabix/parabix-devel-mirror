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
    void setLH(RE * lh) {
        mLh = lh;
    }
    RE * getRH() const {
        return mRh;
    }
    void setRH(RE * rh) {
        mRh = rh;
    }
protected:
    friend Diff * makeDiff(RE*, RE*);
    Diff(RE * lh, RE * rh)
    : RE(ClassTypeId::Diff)
    , mLh(lh)
    , mRh(rh)
    {

    }
    virtual ~Diff() {}
private:
    RE * mLh;
    RE * mRh;
};

inline Diff * makeDiff(RE * lh, RE * rh) {
    return new Diff(lh, rh);
}

}

#endif // RE_DIFF_H
