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
    friend Intersect * makeIntersect(RE*, RE*);
    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }
    Intersect(RE * lh, RE * rh)
    : RE(ClassTypeId::Intersect)
    , mLh(lh)
    , mRh(rh)
    {

    }
    virtual ~Intersect() {}
private:
    RE * mLh;
    RE * mRh;
};

inline Intersect * makeIntersect(RE * lh, RE * rh) {
    return new Intersect(lh, rh);
}

}

#endif // RE_INTERSECT_H
