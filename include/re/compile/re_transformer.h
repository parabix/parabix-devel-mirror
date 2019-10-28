/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <map>
#include <string>
#include <re/adt/adt_forward_decl.h>
#include <re/compile/memoization.h>

namespace re {

class RE;
class Name;

[[ noreturn ]]
void UndefinedNameError(const Name * n);

enum class NameTransformationMode {None, TransformDefinition};

class RE_Transformer {
public:
    RE * transformRE(RE * r);
protected:
    RE_Transformer(std::string transformationName, NameTransformationMode m = NameTransformationMode::None)
    : mTransformationName(std::move(transformationName)), mNameTransform(m) {}

    virtual ~RE_Transformer() {}
    RE * transform(RE * r);
    virtual RE * transformName(Name * n);
    virtual RE * transformCapture(Capture * c);
    virtual RE * transformReference(Reference * r);
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
private:
    const std::string mTransformationName;
    const NameTransformationMode mNameTransform;
    std::map<RE *, RE *, MemoizerComparator> mMap;
};

}
