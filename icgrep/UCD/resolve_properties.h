#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <string>
#include <UCD/unicode_set.h>

namespace re {
    class RE;
    class Name;    
}

namespace UCD {

class UnicodePropertyExpressionError : public std::exception {
public:
    UnicodePropertyExpressionError(const std::string && msg) noexcept : _msg(msg) {}
    const char* what() const noexcept { return _msg.c_str();}
private:
    inline UnicodePropertyExpressionError() noexcept {}
    const std::string _msg;
};

void generateGraphemeClusterBoundaryRule(re::Name * const &property);
bool resolvePropertyDefinition(re::Name * const property);
std::string resolvePropertyFunction(re::Name * const property);
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);
const std::string& getPropertyValueGrepString(const std::string & prop);

}

#endif // RESOLVE_PROPERTIES_H
