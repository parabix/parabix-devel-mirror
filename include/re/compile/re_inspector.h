/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <set>
#include <re/adt/adt_forward_decl.h>
#include <re/compile/memoization.h>

namespace re {

enum class InspectionMode {TraverseNonUnique, IgnoreNonUnique};

class RE_Inspector {
public:
    void inspectRE(RE * r);
protected:
    RE_Inspector(const InspectionMode ignoreNonUnique = InspectionMode::IgnoreNonUnique) : mIgnoreNonUnique(ignoreNonUnique) {}
    virtual ~RE_Inspector() {}
    void inspect(RE * r);
    virtual void inspectName(Name * n);
    virtual void inspectCapture(Capture * c);
    virtual void inspectReference(Reference * r);
    virtual void inspectStart(Start * s);
    virtual void inspectEnd(End * e);
    virtual void inspectCC(CC * cc);
    virtual void inspectSeq(Seq * s);
    virtual void inspectAlt(Alt * a);
    virtual void inspectRep(Rep * rep);
    virtual void inspectIntersect(Intersect * e);
    virtual void inspectDiff(Diff * d);
    virtual void inspectRange(Range * rg);
    virtual void inspectGroup(Group * g);
    virtual void inspectAssertion(Assertion * a);
private:
    const InspectionMode mIgnoreNonUnique;
    std::set<RE *, MemoizerComparator> mMap;
};

}
