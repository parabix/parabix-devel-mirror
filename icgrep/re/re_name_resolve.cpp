#include <re/re_re.h>
#include "re_name_resolve.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_analysis.h>
#include <re/re_group.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_memoizer.hpp>
#include <UCD/resolve_properties.h>
#include <cc/alphabet.h>
#include <boost/container/flat_set.hpp>
#include <sstream>

using namespace boost::container;
using namespace llvm;

namespace re {
  
struct NameResolver {
    RE * resolveUnicodeProperties(RE * re) {
        if (Name * name = dyn_cast<Name>(re)) {
            auto f = mMemoizer.find(name);
            if (f == mMemoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    name->setDefinition(resolveUnicodeProperties(name->getDefinition()));
                } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty || name->getType() == Name::Type::ZeroWidth)) {
                    if (UCD::resolvePropertyDefinition(name)) {
                        name->setDefinition(resolveUnicodeProperties(name->getDefinition()));
                    } else {
                        name->setDefinition(makeCC(UCD::resolveUnicodeSet(name), &cc::Unicode));
                    }
                } else {
                    UndefinedNameError(name);
                }
                re = mMemoizer.memoize(name);
            } else {
                return *f;
            }
        } else if (Vector * vec = dyn_cast<Vector>(re)) {
            for (RE *& re : *vec) {
                re = resolveUnicodeProperties(re);
            }
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            rep->setRE(resolveUnicodeProperties(rep->getRE()));
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            a->setAsserted(resolveUnicodeProperties(a->getAsserted()));
        } else if (Range * rg = dyn_cast<Range>(re)) {
            return makeRange(resolveUnicodeProperties(rg->getLo()),
                             resolveUnicodeProperties(rg->getHi()));
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            diff->setLH(resolveUnicodeProperties(diff->getLH()));
            diff->setRH(resolveUnicodeProperties(diff->getRH()));
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(resolveUnicodeProperties(ix->getLH()));
            ix->setRH(resolveUnicodeProperties(ix->getRH()));
        } else if (Group * g = dyn_cast<Group>(re)) {
            g->setRE(resolveUnicodeProperties(g->getRE()));
        }
        return re;
    }
    
    RE * resolve(RE * re) {
        if (Name * name = dyn_cast<Name>(re)) {
            auto f = mMemoizer.find(name);
            if (f == mMemoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    name->setDefinition(resolve(name->getDefinition()));
                } else {
                    UndefinedNameError(name);
                }
                re = mMemoizer.memoize(name);
            } else {
                return *f;
            }
        } else if (Vector * vec = dyn_cast<Vector>(re)) {
            for (RE *& re : *vec) {
                re = resolve(re);
            }
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            rep->setRE(resolve(rep->getRE()));
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            a->setAsserted(resolve(a->getAsserted()));
        } else if (Range * rg = dyn_cast<Range>(re)) {
            return makeRange(resolve(rg->getLo()), resolve(rg->getHi()));
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            diff->setLH(resolve(diff->getLH()));
            diff->setRH(resolve(diff->getRH()));
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(resolve(ix->getLH()));
            ix->setRH(resolve(ix->getRH()));
        } else if (Group * g = dyn_cast<Group>(re)) {
            g->setRE(resolve(g->getRE()));
        }
        return re;
    }
    
private:
    Memoizer                mMemoizer;
};
    
    RE * resolveUnicodeProperties(RE * re) {
        NameResolver nameResolver;
        return nameResolver.resolveUnicodeProperties(re);
    }
    
    RE * resolveNames(RE * re) {
        NameResolver nameResolver;
        return nameResolver.resolve(re);
    }
    
    
    
bool hasAnchor(const RE * re) {
    if (const Alt * alt = dyn_cast<Alt>(re)) {
        for (const RE * re : *alt) {
            if (hasAnchor(re)) {
                return true;
            }
        }
        return false;
    } else if (const Seq * seq = dyn_cast<Seq>(re)) {
        for (const RE * re : *seq) {
            if (hasAnchor(re)) {
                return true;
            }
        }
        return false;
    } else if (const Rep * rep = dyn_cast<Rep>(re)) {
        return hasAnchor(rep->getRE());
    } else if (isa<Start>(re)) {
        return true;
    } else if (isa<End>(re)) {
        return true;
    } else if (const Assertion * a = dyn_cast<Assertion>(re)) {
        return hasAnchor(a->getAsserted());
    } else if (const Diff * diff = dyn_cast<Diff>(re)) {
        return hasAnchor(diff->getLH()) || hasAnchor(diff->getRH());
    } else if (const Intersect * e = dyn_cast<Intersect>(re)) {
        return hasAnchor(e->getLH()) || hasAnchor(e->getRH());
    } else if (isa<Any>(re)) {
        return false;
    } else if (isa<CC>(re)) {
        return false;
    } else if (const Group * g = dyn_cast<Group>(re)) {
        return hasAnchor(g->getRE());
    } else if (const Name * n = dyn_cast<Name>(re)) {
        return hasAnchor(n->getDefinition());
    }
    return false; // otherwise
}

RE * resolveAnchors(RE * r, RE * breakRE) {
    if (!hasAnchor(r)) return r;
    if (const Alt * alt = dyn_cast<Alt>(r)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * item : *alt) {
            item = resolveAnchors(item, breakRE);
            list.push_back(item);
        }
        return makeAlt(list.begin(), list.end());
    } else if (const Seq * seq = dyn_cast<Seq>(r)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * item : *seq) {
            item = resolveAnchors(item, breakRE);
            list.push_back(item);
        }
        return makeSeq(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(r)) {
        return makeAssertion(resolveAnchors(a->getAsserted(), breakRE), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(r)) {
        return makeRep(resolveAnchors(rep->getRE(), breakRE), rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(r)) {
        return makeDiff(resolveAnchors(diff->getLH(), breakRE), resolveAnchors(diff->getRH(), breakRE));
    } else if (Intersect * e = dyn_cast<Intersect>(r)) {
        return makeIntersect(resolveAnchors(e->getLH(), breakRE), resolveAnchors(e->getRH(), breakRE));
    } else if (isa<Start>(r)) {
        return makeAlt({r, makeLookBehindAssertion(breakRE)});
    } else if (isa<End>(r)) {
        return makeAlt({r, makeLookAheadAssertion(breakRE)});
    }
}
}
