#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

//Regular Expressions
#include "re_re.h"
#include "re_seq.h"
#include <vector>

namespace re {

class RE_Nullable {
public:
    static RE * removeNullablePrefix(RE * re);
    static RE * removeNullableSuffix(RE * re);
private:
    static bool isNullable(const RE * re);
    static bool isNullable(const Vector * vec);
    static bool hasNullablePrefix(const RE * re);
    static bool hasNullableSuffix(const RE * re);
};

}

#endif // RE_NULLABLE_H
