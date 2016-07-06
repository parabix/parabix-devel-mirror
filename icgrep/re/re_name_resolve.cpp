#include <re/re_name.h>
#include <re/re_any.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_grapheme_boundary.hpp>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <unordered_set>
#include <sstream>

namespace re {
  
static inline CC * getDefinitionIfCC(RE * re) {
    if (LLVM_LIKELY(isa<Name>(re))) {
        Name * name = cast<Name>(re);
        if (name->getDefinition() && isa<CC>(name->getDefinition())) {
            return cast<CC>(name->getDefinition());
        }
    }
    return nullptr;
}

Name * generateGraphemeClusterBoundaryRule() {
    // 3.1.1 Grapheme Cluster Boundary Rules
    #define Behind(x) makeLookBehindAssertion(x)
    #define Ahead(x) makeLookAheadAssertion(x)

    RE * GCB_Control = makeName("gcb", "cn", Name::Type::UnicodeProperty);
    RE * GCB_CR = makeName("gcb", "cr", Name::Type::UnicodeProperty);
    RE * GCB_LF = makeName("gcb", "lf", Name::Type::UnicodeProperty);
    RE * GCB_Control_CR_LF = makeAlt({GCB_CR, GCB_LF});

    // Break at the start and end of text.
    RE * GCB_1 = makeStart();
    RE * GCB_2 = makeEnd();
    // Do not break between a CR and LF.
    RE * GCB_3 = makeSeq({Behind(GCB_CR), Ahead(GCB_LF)});
    // Otherwise, break before and after controls.
    RE * GCB_4 = Behind(GCB_Control_CR_LF);
    RE * GCB_5 = Ahead(GCB_Control_CR_LF);
    RE * GCB_1_5 = makeAlt({GCB_1, GCB_2, makeDiff(makeAlt({GCB_4, GCB_5}), GCB_3)});

    RE * GCB_L = makeName("gcb", "l", Name::Type::UnicodeProperty);
    RE * GCB_V = makeName("gcb", "v", Name::Type::UnicodeProperty);
    RE * GCB_LV = makeName("gcb", "lv", Name::Type::UnicodeProperty);
    RE * GCB_LVT = makeName("gcb", "lvt", Name::Type::UnicodeProperty);
    RE * GCB_T = makeName("gcb", "t", Name::Type::UnicodeProperty);
    RE * GCB_RI = makeName("gcb", "ri", Name::Type::UnicodeProperty);
    // Do not break Hangul syllable sequences.
    RE * GCB_6 = makeSeq({Behind(GCB_L), Ahead(makeAlt({GCB_L, GCB_V, GCB_LV, GCB_LVT}))});
    RE * GCB_7 = makeSeq({Behind(makeAlt({GCB_LV, GCB_V})), Ahead(makeAlt({GCB_V, GCB_T}))});
    RE * GCB_8 = makeSeq({Behind(makeAlt({GCB_LVT, GCB_T})), Ahead(GCB_T)});
    // Do not break between regional indicator symbols.
    RE * GCB_8a = makeSeq({Behind(GCB_RI), Ahead(GCB_RI)});
    // Do not break before extending characters.
    RE * GCB_9 = Ahead(makeName("gcb", "ex", Name::Type::UnicodeProperty));
    // Do not break before SpacingMarks, or after Prepend characters.
    RE * GCB_9a = Ahead(makeName("gcb", "sm", Name::Type::UnicodeProperty));
    RE * GCB_9b = Behind(makeName("gcb", "pp", Name::Type::UnicodeProperty));
    RE * GCB_6_9b = makeAlt({GCB_6, GCB_7, GCB_8, GCB_8a, GCB_9, GCB_9a, GCB_9b});
    // Otherwise, break everywhere.
    RE * GCB_10 = makeSeq({Behind(makeAny()), Ahead(makeAny())});

    Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    gcb->setDefinition(makeAlt({GCB_1_5, makeDiff(GCB_10, GCB_6_9b)}));
    return gcb;
}

Name * graphemeClusterRule = nullptr;

RE * resolve(RE * re) {
    Memoizer memoizer;
    if (Name * name = dyn_cast<Name>(re)) {
	auto f = memoizer.find(name);
	if (f == memoizer.end()) {
	    if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
		name->setDefinition(resolve(name->getDefinition()));
	    } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty)) {
		if (UCD::resolvePropertyDefinition(name)) {
		    resolve(name->getDefinition());
		} else {
		    #ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
		    if (AlgorithmOptionIsSet(UsePregeneratedUnicode)) {
			const std::string functionName = UCD::resolvePropertyFunction(name);
			const UCD::ExternalProperty & ep = UCD::resolveExternalProperty(functionName);
			Call * call = mPB.createCall(Prototype::Create(functionName, std::get<1>(ep), std::get<2>(ep), std::get<0>(ep)), mCCCompiler.getBasisBits());
			name->setCompiled(call);
		    } else {
		    #endif
			name->setDefinition(makeCC(UCD::resolveUnicodeSet(name)));
		    #ifndef DISABLE_PREGENERATED_UCD_FUNCTIONS
		    }
		    #endif
		}
	    } else {
		throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
	    }
	} else {
	    return *f;
	}
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
	for (auto si = seq->begin(); si != seq->end(); ++si) {
	    *si = resolve(*si);
	}
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
	CC * unionCC = nullptr;
	std::stringstream name;
	for (auto ai = alt->begin(); ai != alt->end(); ) {
	    RE * re = resolve(*ai);
	    if (CC * cc = getDefinitionIfCC(re)) {
		if (unionCC == nullptr) {
		    unionCC = cc;
		} else {
		    unionCC = makeCC(unionCC, cc);
		    name << '+';
		}
		Name * n = cast<Name>(re);
		if (n->hasNamespace()) {
		    name << n->getNamespace() << ':';
		}
		name << n->getName();
		ai = alt->erase(ai);
	    } else {
		*ai++ = re;
	    }
	}
	if (unionCC) {
	    alt->push_back(makeName(name.str(), unionCC));
	}
	if (alt->size() == 1) {
	    return alt->front();
	}
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
	rep->setRE(resolve(rep->getRE()));
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
	a->setAsserted(resolve(a->getAsserted()));
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
	diff->setLH(resolve(diff->getLH()));
	diff->setRH(resolve(diff->getRH()));
	CC * lh = getDefinitionIfCC(diff->getLH());
	CC * rh = getDefinitionIfCC(diff->getRH());
	if (lh && rh) {
	    return resolve(makeName("diff", subtractCC(lh, rh)));
	}
    } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
	ix->setLH(resolve(ix->getLH()));
	ix->setRH(resolve(ix->getRH()));
	CC * lh = getDefinitionIfCC(ix->getLH());
	CC * rh = getDefinitionIfCC(ix->getRH());
	if (lh && rh) {
	    return resolve(makeName("intersect", intersectCC(lh, rh)));
	}
    } else if (GraphemeBoundary * gb = dyn_cast<GraphemeBoundary>(re)) {
	if (LLVM_LIKELY(gb->getBoundaryRule() == nullptr)) {
	    switch (gb->getType()) {
		case GraphemeBoundary::Type::ClusterBoundary:
		    if (graphemeClusterRule == nullptr) {
			graphemeClusterRule = cast<Name>(resolve(generateGraphemeClusterBoundaryRule()));
		    }
		    gb->setBoundaryRule(graphemeClusterRule);
		    break;
		default:
		    throw std::runtime_error("Only grapheme cluster boundary rules are supported in icGrep 1.0");
	    }
	}
	if (gb->getExpression()) {
	    resolve(gb->getExpression());
	}
    }
    return re;
}

UCD::UCDCompiler::NameMap nameMap;
std::unordered_set<Name *> visited;
    
void gather(RE * re) {
    assert ("RE object cannot be null!" && re);
    if (isa<Name>(re)) {
	if (visited.insert(cast<Name>(re)).second) {
	    if (isa<CC>(cast<Name>(re)->getDefinition())) {
		nameMap.emplace(cast<Name>(re), nullptr);
	    } else {
		gather(cast<Name>(re)->getDefinition());
	    }
	}
    } else if (isa<Seq>(re)) {
	for (RE * item : *cast<Seq>(re)) {
	    gather(item);
	}
    } else if (isa<Alt>(re)) {
	for (RE * item : *cast<Alt>(re)) {
	    gather(item);
	}
    } else if (isa<Rep>(re)) {
	gather(cast<Rep>(re)->getRE());
    } else if (isa<Assertion>(re)) {
	gather(cast<Assertion>(re)->getAsserted());
    } else if (isa<Diff>(re)) {
	gather(cast<Diff>(re)->getLH());
	gather(cast<Diff>(re)->getRH());
    } else if (isa<Intersect>(re)) {
	gather(cast<Intersect>(re)->getLH());
	gather(cast<Intersect>(re)->getRH());
    } else if (isa<GraphemeBoundary>(re)) {
	if (cast<GraphemeBoundary>(re)->getExpression()) {
	    gather(cast<GraphemeBoundary>(re)->getExpression());
	}
	gather(cast<GraphemeBoundary>(re)->getBoundaryRule());
    }
}
    
UCD::UCDCompiler::NameMap resolveNames(RE * re, Name * &Rule) {

    graphemeClusterRule = nullptr;
    re = resolve(re);
    gather(re);
    Rule = graphemeClusterRule;
    
    return nameMap;
    
}

}
