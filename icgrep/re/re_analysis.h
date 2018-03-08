#ifndef RE_ANALYSIS_H
#define RE_ANALYSIS_H

#include <utility>
namespace re { class RE; class Name; class CC;}

namespace re {

// Does the RE match the empty string, considering that ^ and $ each
// do match an empty string.
bool matchesEmptyString(const RE * re);

//  Determine the set of all codepoints cp such that the given RE
//  matches a string consisting of a single Unicode character whose
//  codepoint value is cp.
const CC * matchableCodepoints(const RE * re);

bool isByteLength(const RE * re);
    
bool isUnicodeUnitLength(const RE * re);

std::pair<int, int> getUnicodeUnitLengthRange(const RE * re);

int minMatchLength(RE * re);

bool unitBoundedRep(const RE * re);

bool isTypeForLocal(const RE * re);
    
bool hasAssertion(const RE * re);
    
unsigned byteTestComplexity(RE * re);
    
void UndefinedNameError (const Name * n);
}

#endif // RE_ANALYSIS_H
