#ifndef RE_SIMPLIFIER_H
#define RE_SIMPLIFIER_H

//Regular Expressions
#include "re_re.h"
#include "re_seq.h"
#include <list>

class RE_Simplifier {
    typedef RE::Vector Vector;
public:
    static RE * makeAlt(Vector & list);
    static RE * makeSeq(const Seq::Type type, Vector & list);
    static RE * makeRep(RE * re, const int lb2, const int ub2);
    static RE * simplify(RE* re);
private:
    static int ubCombine(const int h1, const int h2);
};

#endif // RE_SIMPLIFIER_H
