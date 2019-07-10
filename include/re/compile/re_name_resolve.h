#ifndef RE_NAME_RESOLVE_H
#define RE_NAME_RESOLVE_H

namespace re {

    class RE;
    class Name;

    RE * resolveUnicodeNames(RE * re);
    RE * resolveAnchors(RE * r, RE * breakRE);

}
#endif
