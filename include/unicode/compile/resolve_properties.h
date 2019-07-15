#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <string>
#include <unicode/core/unicode_set.h>
#include <unicode/data/PropertyObjects.h>

namespace re {
    class RE;
    class Name;    
}

namespace UCD {

LLVM_ATTRIBUTE_NORETURN void UnicodePropertyExpressionError(std::string errmsg);

bool resolvePropertyDefinition(re::Name * const property);
std::string resolvePropertyFunction(re::Name * const property);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

}

#endif // RESOLVE_PROPERTIES_H
