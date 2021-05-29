#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <set>
#include <memory>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>

namespace cc { class MultiplexedAlphabet; }

namespace re {
    class RE;

RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * r,
                  re::NameTransformationMode mode = re::NameTransformationMode::None);

inline RE * transformCCs(const std::shared_ptr<cc::MultiplexedAlphabet> & mpx, RE * r,
                         re::NameTransformationMode mode =  re::NameTransformationMode::None) {
    return transformCCs(mpx.get(), r, mode);
}
}
#endif
