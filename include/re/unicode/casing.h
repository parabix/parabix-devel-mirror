#ifndef RE_CASING_H
#define RE_CASING_H

namespace re {

class RE;

RE * resolveCaseInsensitiveMode(RE * re, const bool globallyCaseInsensitive);

}

#endif
