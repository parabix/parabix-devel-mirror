#include <re/transforms/re_memoizing_transformer.h>
#include <re/adt/adt.h>

namespace re {

RE * RE_MemoizingTransformer::transform(RE * const from) {
    // Do we have a memoized version of the original RE?
    const auto f = mMap.find(from);
    if (LLVM_UNLIKELY(f != mMap.end())) {
        return f->second;
    }
    RE * to = RE_Transformer::transform(from);
    if (LLVM_UNLIKELY(from != to)) {
        // Do we already have a memoized version of the transformed RE?
        const auto f = mMap.find(to);
        if (LLVM_UNLIKELY(f != mMap.end())) {
            to = f->second;
        } else {
            mMap.insert(std::make_pair(to, to));
        }
    }
    mMap.insert(std::make_pair(from, to));
    return to;
}

}
