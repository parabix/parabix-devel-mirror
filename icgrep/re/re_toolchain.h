/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef RE_TOOLCHAIN_H
#define RE_TOOLCHAIN_H

#include <map>
#include <set>
#include <string>
#include <llvm/Support/Compiler.h>

namespace llvm { namespace cl { class OptionCategory; } }
namespace pablo { class PabloKernel; class PabloAST; }
namespace re { class RE; class CC;}
namespace re { class Name; class Start; class End; class CC; class Seq; class Alt;
    class Rep; class Intersect; class Diff; class Range; class Group;
    class Assertion;
}

namespace re {

enum RE_PrintFlags {
    ShowREs, ShowAllREs, ShowStrippedREs, ShowSimplifiedREs
};
    
enum RE_AlgorithmFlags {
    DisableLog2BoundedRepetition, DisableIfHierarchy, DisableMatchStar, DisableUnicodeMatchStar, 
    DisableUnicodeLineBreak, UsePregeneratedUnicode
};
    
bool LLVM_READONLY AlgorithmOptionIsSet(RE_AlgorithmFlags flag);
    
extern int IfInsertionGap;

const llvm::cl::OptionCategory * LLVM_READONLY re_toolchain_flags();
    
    
void UndefinedNameError (const Name * n);

enum class NameTransformationMode {None, TransformDefinition};

class RE_Transformer {
    struct MemoizerComparator {
        bool operator() (const RE * lh, const RE * rh) const;
    };
public:
    RE * transformRE(RE * r);
protected:
    RE_Transformer(std::string transformationName, NameTransformationMode m = NameTransformationMode::None)
    : mTransformationName(std::move(transformationName)), mNameTransform(m) {}

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
private:
    const std::string mTransformationName;
    const NameTransformationMode mNameTransform;
    std::map<RE *, RE *, MemoizerComparator> mMap;
};

enum class InspectionMode {TraverseNonUnique, IgnoreNonUnique};

class RE_Inspector {
    struct MemoizerComparator {
        bool operator() (const RE * lh, const RE * rh) const;
    };
public:
    void inspectRE(RE * r);
protected:
    RE_Inspector(const InspectionMode ignoreNonUnique = InspectionMode::IgnoreNonUnique) : mIgnoreNonUnique(ignoreNonUnique) {}
    void inspect(RE * r);
    virtual void inspectName(Name * n);
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

RE * resolveModesAndExternalSymbols(RE * r, bool globallyCaseInsensitive = false);

RE * excludeUnicodeLineBreak(RE * r);

RE * regular_expression_passes(RE * r);
    
}
#endif
