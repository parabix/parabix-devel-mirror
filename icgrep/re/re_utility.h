/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_UTILITY_H
#define RE_UTILITY_H

namespace re { class RE; }
namespace re { class Name; class Start; class End; class CC; class Seq; class Alt;
               class Rep; class Intersect; class Diff; class Range; class Group;
               class Assertion;
}

namespace re {

RE * makeComplement(RE * s);
RE * makeWordBoundary();
RE * makeWordNonBoundary();
RE * makeWordBegin();
RE * makeWordEnd();
Name * makeDigitSet();
Name * makeAlphaNumeric();
Name * makeWhitespaceSet();
Name * makeWordSet();
RE * makeUnicodeBreak();

void UndefinedNameError (const Name * n);

enum class NameTransformationMode {None, TransformDefinition};

class RE_Transformer {
public:
    RE_Transformer(NameTransformationMode m = NameTransformationMode::None) : mNameTransform(m) {}
    RE * transform(RE * r);
    virtual RE * transformName(Name * n);
    virtual RE * transformStart(Start * s);
    virtual RE * transformEnd(End * e);
    virtual RE * transformCC(CC * cc);
    virtual RE * transformSeq(Seq * s);
    virtual RE * transformAlt(Alt * a);
    virtual RE * transformRep(Rep * rep);
    virtual RE * transformIntersect(Intersect * e);
    virtual RE * transformDiff(Diff * d);
    virtual RE * transformRange(Range * rg);
    virtual RE * transformGroup(Group * g);
    virtual RE * transformAssertion(Assertion * a);
protected:
    NameTransformationMode mNameTransform;
};

}
#endif // RE_UTILITY_H
