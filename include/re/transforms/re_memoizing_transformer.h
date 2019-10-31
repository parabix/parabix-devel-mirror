#ifndef MEMOIZER_MAP_HPP
#define MEMOIZER_MAP_HPP

#include <re/transforms/re_transformer.h>
#include <re/adt/memoization.h>
#include <map>

namespace re {

// NOTE: this class is a utility class to provide memoization functionality
// to other RE transformers and is not intended to be used on its own.
class RE_MemoizingTransformer : public RE_Transformer {
protected:
    RE_MemoizingTransformer(std::string transformationName, NameTransformationMode m = NameTransformationMode::None)
    : RE_Transformer(std::move(transformationName), m) {

    }

    RE * transform(RE * const from) override;
private:
    std::map<RE *, RE *, MemoizerComparator> mMap;
};

}

#endif // MEMOIZER_MAP_HPP
