#include <re/unicode/boundaries.h>

#include <re/adt/adt.h>
#include <re/adt/re_name.h>
#include <re/adt/printer_re.h>
#include <re/analysis/validation.h>
#include <re/transforms/re_transformer.h>
#include <re/unicode/re_name_resolve.h>
#include <unicode/data/PropertyObjects.h>
#include <unicode/data/PropertyObjectTable.h>
#include <re/compile/re_compiler.h>

#include <vector>                  // for vector, allocator
#include <llvm/Support/Casting.h>  // for dyn_cast, isa
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>


/*
 Unicode Technical Standard #18 defines grapheme cluster mode, signified by the (?g) switch.
 The mode is defined in terms of the assertion of grapheme cluster boundary assertions \b{g}
 after every atomic literal.
 
 resolveGraphemeMode transforms a regular expression to perform the required insertion of
 grapheme cluster boundaries, and the elimination of grapheme cluster mode groups.

*/

using namespace llvm;

namespace re {
bool hasGraphemeClusterBoundary(const RE * re) {
    if (isa<CC>(re) || isa<Range>(re)) {
        return false;
    } else if (const Name * n = dyn_cast<Name>(re)) {
        if (n->getType() == Name::Type::ZeroWidth) {
            const std::string nameString = n->getName();
            return nameString == "\\b{g}";
        }
        return false;
    } else if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasGraphemeClusterBoundary(re)) return true;
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * re : *seq) {
            if (hasGraphemeClusterBoundary(re)) return true;
        }
        return false;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return hasGraphemeClusterBoundary(rep->getRE());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return hasGraphemeClusterBoundary(diff->getLH()) || hasGraphemeClusterBoundary(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return hasGraphemeClusterBoundary(e->getLH()) || hasGraphemeClusterBoundary(e->getRH());
    } else if (isa<Start>(re) || isa<End>(re)) {
        return false;
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return hasGraphemeClusterBoundary(a->getAsserted());
    } else if (const Group * g = dyn_cast<Group>(re)) {
        if ((g->getMode() == Group::Mode::GraphemeMode) && (g->getSense() == Group::Sense::On)) return true;
        else return hasGraphemeClusterBoundary(g->getRE());
    }
    else llvm_unreachable("Unknown RE type");
}
    
struct WordBoundaryAbsentValidator final : public RE_Validator {
    
    WordBoundaryAbsentValidator()
    : RE_Validator() {}
    
    bool validateName(const Name * n) override {
        return n->getName() != "\\b";
    }
};

bool hasWordBoundary(const RE * re) {
    WordBoundaryAbsentValidator v;
    return !(v.validateRE(re));
}


class GraphemeModeTransformer : public RE_Transformer {
public:
    GraphemeModeTransformer(bool inGraphemeMode = true) : RE_Transformer("ResolveGraphemeMode"), mGraphemeMode(inGraphemeMode) {}
    
    RE * transformName(Name * n) override {
        if (mGraphemeMode && (n->getName() == ".")) {
            RE * GCB = makeZeroWidth("\\b{g}");
            RE * nonGCB = makeDiff(makeSeq({}), GCB);
            return makeSeq({makeAny(), makeRep(makeSeq({nonGCB, makeAny()}), 0, Rep::UNBOUNDED_REP), GCB});
        }
        return n;
    }
    
    RE * transformCC(CC * cc) override {
        if (mGraphemeMode) return makeSeq({cc, makeZeroWidth("\\b{g}")});
        return cc;
    }
    
    RE * transformRange(Range * rg) override {
        if (mGraphemeMode) return makeSeq({rg, makeZeroWidth("\\b{g}")});
        return rg;
    }
    
    RE * transformGroup(Group * g) override {
        if (g->getMode() == Group::Mode::GraphemeMode) {
            RE * r = g->getRE();
            bool modeSave = mGraphemeMode;
            mGraphemeMode = g->getSense() == Group::Sense::On;
            RE * t = transform(r);
            mGraphemeMode = modeSave;
            return t;
        } else {
            return RE_Transformer::transformGroup(g);
        }
    }
    
    RE * transformSeq(Seq * seq) override {
        std::vector<RE*> list;
        bool afterSingleChar = false;
        bool changed = false;
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            bool atSingleChar = isa<CC>(*i) && (cast<CC>(*i)->count() == 1);
            if (afterSingleChar && mGraphemeMode && !atSingleChar) {
                list.push_back(makeZeroWidth("\\b{g}"));
                changed = true;
            }
            if (isa<CC>(*i)) {
                list.push_back(*i);
            } else {
                RE * t = transform(*i);
                if (*i != t) changed = true;
                list.push_back(t);
            }
            afterSingleChar = atSingleChar;
        }
        if (afterSingleChar && mGraphemeMode) {
            list.push_back(makeZeroWidth("\\b{g}"));
            changed = true;
        }
        if (!changed) return seq;
        return makeSeq(list.begin(), list.end());
    }

private:
    bool mGraphemeMode;
};

RE * resolveGraphemeMode(RE * re, bool inGraphemeMode) {
    return GraphemeModeTransformer(inGraphemeMode).transformRE(re);
}

#define Behind(x) makeLookBehindAssertion(x)
#define notBehind(x) makeNegativeLookBehindAssertion(x)
#define Ahead(x) makeLookAheadAssertion(x)
#define notAhead(x) makeNegativeLookAheadAssertion(x)

RE * generateGraphemeClusterBoundaryRule(bool extendedGraphemeClusters) {
    // 3.1.1 Grapheme Cluster Boundary Rules
    // Grapheme cluster boundary rules define a number of contexts where
    // breaks are not permitted.  In the following definitions, we identify
    // the points at which breaks are not permitted are identified by the
    // definitions marked GCX.
    
    // Rules GB1, GB2, GB4 and GB5 define rules where breaks occur overriding
    // later rules (specifically GB9, GB9a, GB9b).
    // Rules GB9 and GB9a are overridden by GB1 and GB4, to allow breaks
    // at start of text or after any control|CR|LF.  This is equivalent
    // to stating that the lookbehind context for GB9 and GB9b is any
    // non-control character (any actual character not in control|CR|LF).
    // Similarly, the overriding of GB9b simplifies to a lookahead assertion
    // on a noncontrol.
    //
    RE * GCB_CR = makeName("gcb", "cr", Name::Type::UnicodeProperty);
    RE * GCB_LF = makeName("gcb", "lf", Name::Type::UnicodeProperty);
    RE * GCB_Control = makeName("gcb", "control", Name::Type::UnicodeProperty);
    RE * GCB_Control_CR_LF = makeAlt({GCB_Control, GCB_CR, GCB_LF});
    
    // Break at the start and end of text.
    RE * GCB_1 = makeSOT();
    RE * GCB_2 = makeEOT();
    // Do not break between a CR and LF.
    RE * GCB_3 = makeSeq({Behind(GCB_CR), Ahead(GCB_LF)});
    // Otherwise, break before and after controls.
    RE * GCB_4 = Behind(GCB_Control_CR_LF);
    RE * GCB_5 = Ahead(GCB_Control_CR_LF);
    RE * GCB_1_5 = makeAlt({GCB_1, GCB_2, makeDiff(makeAlt({GCB_4, GCB_5}), GCB_3)});
    
    
    // Do not break Hangul syllable sequences.
    RE * GCB_L = makeName("gcb", "l", Name::Type::UnicodeProperty);
    RE * GCB_V = makeName("gcb", "v", Name::Type::UnicodeProperty);
    RE * GCB_LV = makeName("gcb", "lv", Name::Type::UnicodeProperty);
    RE * GCB_LVT = makeName("gcb", "lvt", Name::Type::UnicodeProperty);
    RE * GCB_T = makeName("gcb", "t", Name::Type::UnicodeProperty);
    RE * GCX_6 = makeSeq({Behind(GCB_L), Ahead(makeAlt({GCB_L, GCB_V, GCB_LV, GCB_LVT}))});
    RE * GCX_7 = makeSeq({Behind(makeAlt({GCB_LV, GCB_V})), Ahead(makeAlt({GCB_V, GCB_T}))});
    RE * GCX_8 = makeSeq({Behind(makeAlt({GCB_LVT, GCB_T})), Ahead(GCB_T)});
    
    // Do not break before extendiers or zero-width joiners.
    RE * GCB_EX = makeName("gcb", "ex", Name::Type::UnicodeProperty);
    RE * GCB_ZWJ = makeName("gcb", "zwj", Name::Type::UnicodeProperty);
    RE * GCX_9 = makeSeq({notBehind(GCB_Control_CR_LF), Ahead(makeAlt({GCB_EX, GCB_ZWJ}))});

    if (extendedGraphemeClusters) {
        RE * GCB_SpacingMark = makeName("gcb", "sm", Name::Type::UnicodeProperty);
        RE * GCB_Prepend = makeName("gcb", "pp", Name::Type::UnicodeProperty);
        RE * GCX_9a = makeSeq({notBehind(GCB_Control_CR_LF), Ahead(GCB_SpacingMark)});
        RE * GCX_9b = makeSeq({Behind(GCB_Prepend), notAhead(GCB_Control_CR_LF)});
        GCX_9 = makeAlt({GCX_9, GCX_9a, GCX_9b});
    }

    RE * ExtendedPictographic = makeName("Extended_Pictographic", Name::Type::UnicodeProperty);
    RE * EmojiSeq = makeSeq({ExtendedPictographic, makeRep(GCB_EX, 0, Rep::UNBOUNDED_REP), GCB_ZWJ});
    RE * GCX_11 = makeSeq({Behind(EmojiSeq), Ahead(ExtendedPictographic)});
    
    RE * GCB_RI = makeName("gcb", "ri", Name::Type::UnicodeProperty);
    // Note: notBehind(RI) == sot | [^RI]
    RE * odd_RI_seq = makeSeq({notBehind(GCB_RI), makeRep(makeSeq({GCB_RI, GCB_RI}), 0, Rep::UNBOUNDED_REP), GCB_RI});
    RE * GCX_12_13 = makeSeq({Behind(odd_RI_seq), Ahead(GCB_RI)});
    
    //Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    RE * GCX = makeAlt({GCX_6, GCX_7, GCX_8, GCX_9, GCX_11, GCX_12_13});
    
    // Otherwise, break everywhere.
    RE * GCB_999 = makeSeq({Behind(makeAny()), Ahead(makeAny())});
    
    //Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    RE * gcb = makeAlt({GCB_1_5, makeDiff(GCB_999, GCX)});
    return gcb;
}

RE * EnumeratedPropertyBoundary(UCD::EnumeratedPropertyObject * enumObj) {
    unsigned enum_count = enumObj->GetEnumCount();
    unsigned basis_size = std::log2(enum_count - 1) + 1;
    std::vector<RE *> assertions;
    auto prop = enumObj->getPropertyCode();
    PropertyExpression::Kind kind = PropertyExpression::Kind::Codepoint;
    PropertyExpression::Operator op = PropertyExpression::Operator::Eq;
    for (unsigned i = 0; i < basis_size; i++) {
        std::vector<RE *> props;
        for (unsigned j = 0; j < enum_count; j++) {
            if (((j >> i) & 1) == 1) {
                std::string enumVal = enumObj->GetValueEnumName(j);
                RE * expr = makePropertyExpression(kind, UCD::property_full_name[prop], op, enumVal);
                props.push_back(expr);
            }
        }
        RE * propRE = makeAlt(props.begin(), props.end());
        assertions.push_back(makeBoundaryAssertion(propRE));
    }
    return makeSeq(assertions.begin(), assertions.end());
}

class BoundaryPropertyResolver : public RE_Transformer {
public:
    BoundaryPropertyResolver() : RE_Transformer("ResolveBoundaryProperties") {}
    
    RE * transformPropertyExpression(PropertyExpression * propExpr) {
        if (propExpr->getKind() == PropertyExpression::Kind::Codepoint) {
            return propExpr;
        }
        int prop_code = propExpr->getPropertyCode();
        if (prop_code >= 0) {
            auto obj = UCD::property_object_table[prop_code];
            if ((propExpr->getValueString() == "") && isa<UCD::EnumeratedPropertyObject>(obj)) {
                return EnumeratedPropertyBoundary(cast<UCD::EnumeratedPropertyObject>(obj));
            }
            auto pe = makePropertyExpression(PropertyExpression::Kind::Codepoint,
                                                                propExpr->getPropertyIdentifier(),
                                                                propExpr->getOperator(),
                                                                propExpr->getValueString());
            cast<PropertyExpression>(pe)->setPropertyCode(prop_code);
            return pe;
        }
        if (propExpr->getPropertyIdentifier() == "g") {
            Name * gcb_name = makeZeroWidth("\\b{g}");
            gcb_name->setDefinition(generateGraphemeClusterBoundaryRule());
            return gcb_name;
        }
        if (propExpr->getPropertyIdentifier() == "w") {
            Name * wb_name = makeZeroWidth("\\b{w}");
            wb_name->setDefinition(nullptr);
            RE_Compiler::UnsupportedRE("\\b{w} not yet supported.");
            return wb_name;
        }
        RE_Compiler::UnsupportedRE(Printer_RE::PrintRE(propExpr));
    }

};

RE * resolveBoundaryProperties(RE * r) {
    return BoundaryPropertyResolver().transformRE(r);
}

}
