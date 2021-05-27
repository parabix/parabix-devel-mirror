#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <set>
#include <memory>

namespace cc { class MultiplexedAlphabet; }

namespace re {
    class RE; class Name;

RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * r, std::set<Name *> externalNames = {});

inline RE * transformCCs(const std::shared_ptr<cc::MultiplexedAlphabet> & mpx,
                         RE * r, std::set<Name *> externalNames = {}) {
    return transformCCs(mpx.get(), r, externalNames);
}
}
#endif
