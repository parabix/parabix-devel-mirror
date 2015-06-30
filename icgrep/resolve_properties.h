#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <UCD/unicode_set.h>

namespace re {
    class RE;
    class Name;
}

void resolveProperties(re::RE * re);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

#endif // RESOLVE_PROPERTIES_H
