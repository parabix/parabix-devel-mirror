#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"

#include <algorithm>
#include <list>

class RE_Simplifier
{
public:
    static RE* mkSeq(std::list<RE*>* re_list);
    static RE* mkRep(RE* re, int lb2, int ub2);
    static RE* mkAlt(std::list<RE*>* re_list);
    static RE* simplify(RE* re);
private:
    static std::list<RE*>* mkSeqList(std::list<RE*>* re_list);
    static std::list<RE*>* mkSeqList_helper(std::list<RE*>* ret_list, std::list<RE*>* re_list);
    static std::list<RE*>* mkAltList(std::list<RE*>* re_list);
    static std::list<RE*>* mkAltList_helper(std::list<RE*>* ret_list, std::list<RE*>* re_list);
    static int ubCombine(int h1, int h2);
};

#endif // RE_SIMPLIFIER_H
