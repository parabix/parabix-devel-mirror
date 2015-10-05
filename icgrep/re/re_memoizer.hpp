#ifndef RE_NAMEDICTIONARY_H
#define RE_NAMEDICTIONARY_H

#include <re/re_name.h>
#include <set>

namespace re {

namespace {

struct MemoizerComparator {
    inline bool operator() (const RE * lh, const RE * rh) const{
        if (LLVM_LIKELY(isa<Name>(lh) && isa<Name>(rh))) {
            return *cast<Name>(lh) < *cast<Name>(rh);
        } else if (isa<Name>(lh)) {
            return *cast<Name>(lh) < *cast<CC>(rh);
        }
        return *cast<Name>(rh) > *cast<CC>(lh);
    }
};

}

struct Memoizer : public std::set<RE *, MemoizerComparator> {

    inline Name * memoize(CC * cc) {
        auto f = find(cc);
        if (f != end()) {
            return cast<Name>(*f);
        } else {
            Name * name = makeName(cc);
            insert(name);
            return name;
        }
    }

    inline Name * memoize(Name * name) {
        return cast<Name>(*insert(name).first);
    }
};

}

#endif // RE_NAMEDICTIONARY_H
