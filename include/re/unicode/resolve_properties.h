#ifndef RESOLVE_PROPERTIES_H
#define RESOLVE_PROPERTIES_H

#include <string>
#include <llvm/Support/Compiler.h>
#include <unicode/data/PropertyObjects.h>

namespace re {
    class RE;
    class Name;
    class PropertyExpression;
}

namespace UCD {

LLVM_ATTRIBUTE_NORETURN void UnicodePropertyExpressionError(std::string errmsg);

/*  Link all property expression nodes to their property_enum code, and
    standardize the property name.   */
re::RE * linkProperties(re::RE * r);

/*  Convert all property expression to standardized form, using the
    full name of any enumerated properties. */
re::RE * standardizeProperties(re::RE * r);

/*  Resolve and store the equivalent regexp for all property expressions.
    Whenever property values are expressed by regular expression, use the
    passed in grep function to perform the resolution.  */
re::RE * resolveProperties(re::RE * r, GrepLinesFunctionType grep = nullptr);

/*  Link, standardize and resolve properties.  */
re::RE * linkAndResolve(re::RE * r, GrepLinesFunctionType grep = nullptr);

/*  Replace very simple codepoint properties (e.g. ASCII) with the equivalent CC. */
re::RE * inlineSimpleProperties(re::RE * r);

/*  Create named externals for all property expressions.  */
re::RE * externalizeProperties(re::RE * r);

}

#endif // RESOLVE_PROPERTIES_H
