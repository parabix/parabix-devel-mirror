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

/**
 * Resolves a re::Name into a unicode set where the value of name is NOT a 
 * regular expression.
 *
 * For cases where name's value may be a regular expression, use
 * grep::resolveUnicodeSet(re::Name * const) instead. Note that the use of the
 * grep variant will require linking with the grep module which may not be
 * desirable depending on the use case.
 */
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

}

#endif // RESOLVE_PROPERTIES_H
