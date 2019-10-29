#ifndef RE_MULTIPLEX_H
#define RE_MULTIPLEX_H

#include <memory>

namespace cc { class MultiplexedAlphabet; }

namespace re {
    class RE;
    RE * transformCCs(const cc::MultiplexedAlphabet * const mpx, RE * r);

    inline RE * transformCCs(const std::shared_ptr<cc::MultiplexedAlphabet> & mpx, RE * r) {
        return transformCCs(mpx.get(), r);
    }
}
#endif
