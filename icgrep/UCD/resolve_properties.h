#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <UCD/unicode_set.h>

namespace re {
    class Name;
}

void resolveProperty(re::Name * const name);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

#endif // RESOLVE_PROPERTIES_H
