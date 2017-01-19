#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

namespace re { class RE; }
namespace re { class Vector; }

namespace re {

class RE_Nullable {
public:
    static RE * removeNullablePrefix(RE * re);
    static RE * removeNullableSuffix(RE * re);
    static RE * removeNullableAssertion(RE * re);
    static RE * removeNullableAfterAssertion(RE * re); 
private:
    static bool isNullableAfterAssertion(const RE * re);
    static RE * removeNullableAfterAssertion_helper(RE * re);
    static bool isNullable(const RE * re);
    static bool isNullable(const Vector * vec);
    static bool hasNullablePrefix(const RE * re);
    static bool hasNullableSuffix(const RE * re);
};

}

#endif // RE_NULLABLE_H
