#ifndef RE_NULLABLE_H
#define RE_NULLABLE_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"
#include "re_simplifier.h"

#include <list>

class RE_Nullable
{
public:
    static RE* removeNullablePrefix(RE* re);
    static RE* removeNullableSuffix(RE* re);
private:
    static bool isNullable(RE* re);
    static bool isNullableSeq(std::list<RE*>* re_list);
    static bool isNullableSeq_helper(std::list<RE*>* re_list, std::list<RE*>::iterator it);
    static bool isNullableAlt(std::list<RE*>* re_list);
    static bool isNullableAlt_helper(std::list<RE*>* re_list, std::list<RE*>::iterator it);
    static bool hasNullablePrefix(RE* re);
    static bool hasNullableSuffix(RE* re);
    static std::list<RE*>* removeNullableSeqPrefix(std::list<RE*>* re_list);
    static std::list<RE*>* removeNullableSeqSuffix(std::list<RE*>* re_list);
};

#endif // RE_NULLABLE_H
