#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <UCD/unicode_set.h>

namespace re {
    class Name;
    class RE_Parser;
}

namespace UCD {

re::Name * resolveProperty(const std::string value, re::RE_Parser * parser);
re::Name * resolveProperty(const std::string prop, const std::string value, re::RE_Parser * parser);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

}

#endif // RESOLVE_PROPERTIES_H
