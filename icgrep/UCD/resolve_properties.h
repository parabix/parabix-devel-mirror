#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <string>
#include <UCD/unicode_set.h>
#include <UCD/PropertyObjects.h>

namespace re {
    class RE;
    class Name;    
}

namespace UCD {

LLVM_ATTRIBUTE_NORETURN void UnicodePropertyExpressionError(std::string errmsg);

void generateGraphemeClusterBoundaryRule(re::Name * const &property);
bool resolvePropertyDefinition(re::Name * const property);
std::string resolvePropertyFunction(re::Name * const property);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);
const std::string & getPropertyValueGrepString(const std::string & prop);

}

#endif // RESOLVE_PROPERTIES_H
