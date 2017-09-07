#ifndef RE_MEMOIZER_H
#define RE_MEMOIZER_H

#include <re/re_name.h>
#include <set>

namespace re {

struct MemoizerComparator {
    bool operator() (const RE * lh, const RE * rh) const;
};

struct Memoizer : public std::set<RE *, MemoizerComparator> {

    RE * memoize(RE * const re) {
        return *(insert(re).first);
    }

    Name * memoize(Name * const name) {
        return llvm::cast<Name>(memoize(llvm::cast<RE>(name)));
    }

    Name * memoize(CC * const cc) {
        auto f = find(cc);
        if (f != end()) {
            return llvm::cast<Name>(*f);
        } else {
            return memoize(makeName(cc));
        }
    }
};

}

#endif // RE_MEMOIZER_H
