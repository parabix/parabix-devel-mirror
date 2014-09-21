#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

//Regular Expressions
#include "re_re.h"
#include "re_seq.h"
#include <vector>

class RE_Nullable {
    typedef RE::Vector Vector;
public:
    static RE* removeNullablePrefix(RE* re);
    static RE* removeNullableSuffix(RE* re);
private:
    static bool isNullable(const RE * re);
    static bool isNullableVector(const Vector * vec);
    static bool hasNullablePrefix(const RE *re);
    static bool hasNullableSuffix(const RE * re);
    static Seq * removeNullableSeqPrefix(const Seq * seq);
    static Seq * removeNullableSeqSuffix(const Seq *seq);
};

#endif // RE_NULLABLE_H
