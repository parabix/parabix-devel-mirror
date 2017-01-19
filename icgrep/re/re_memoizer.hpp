#ifndef RE_NAMEDICTIONARY_H
#define RE_NAMEDICTIONARY_H

#include <re/re_name.h>
#include <set>

namespace re {

struct MemoizerComparator {
    inline bool operator() (const RE * lh, const RE * rh) const{
        if (LLVM_LIKELY(llvm::isa<Name>(lh) && llvm::isa<Name>(rh))) {
            return *llvm::cast<Name>(lh) < *llvm::cast<Name>(rh);
        } else if (llvm::isa<Name>(lh)) {
            return *llvm::cast<Name>(lh) < *llvm::cast<CC>(rh);
        }
        return *llvm::cast<Name>(rh) > *llvm::cast<CC>(lh);
    }
};

struct Memoizer : public std::set<RE *, MemoizerComparator> {

    inline Name * memoize(CC * cc) {
        auto f = find(cc);
        if (f != end()) {
            return llvm::cast<Name>(*f);
        } else {
            Name * name = makeName(cc);
            insert(name);
            return name;
        }
    }

    inline Name * memoize(Name * name) {
        return llvm::cast<Name>(*insert(name).first);
    }
};

}

#endif // RE_NAMEDICTIONARY_H
