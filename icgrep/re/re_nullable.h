#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

#include <llvm/Support/Compiler.h>

namespace re { class RE; }
namespace re { class Vector; }

namespace re {

class RE_Nullable {
public:
    static RE * removeNullablePrefix(RE * re) LLVM_ATTRIBUTE_UNUSED_RESULT;
    static RE * removeNullableSuffix(RE * re) LLVM_ATTRIBUTE_UNUSED_RESULT;
    static RE * removeNullableAssertion(RE * re) LLVM_ATTRIBUTE_UNUSED_RESULT;
    static RE * removeNullableAfterAssertion(RE * re) LLVM_ATTRIBUTE_UNUSED_RESULT;
    static bool isNullable(const RE * re);
    static bool hasNullablePrefix(const RE * re);
    static bool hasNullableSuffix(const RE * re);
private:
    static bool isNullableAfterAssertion(const RE * re);
    static RE * removeNullableAfterAssertion_helper(RE * re);
    static bool isNullable(const Vector * vec);
};

}

#endif // RE_NULLABLE_H
