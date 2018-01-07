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
#include <re/re_memoizer.hpp>
#include <UCD/resolve_properties.h>
#include <cc/alphabet.h>
#include <boost/container/flat_set.hpp>
#include <sstream>

using namespace boost::container;
using namespace llvm;

namespace re {
  
static inline CC * extractCC(RE * re) {
    if (isa<CC>(re)) {
        return cast<CC>(re);
    } else if (isa<Name>(re)) {
        return extractCC(cast<Name>(re)->getDefinition());
    }
    return nullptr;
}

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
    
}
