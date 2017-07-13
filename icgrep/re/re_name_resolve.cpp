#include "re_name_resolve.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
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
    RE * resolve(RE * re) {
        if (Name * name = dyn_cast<Name>(re)) {
            auto f = mMemoizer.find(name);
            if (f == mMemoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    name->setDefinition(resolve(name->getDefinition()));
                } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty || name->getType() == Name::Type::ZeroWidth)) {
                    if (UCD::resolvePropertyDefinition(name)) {
                        name->setDefinition(resolve(name->getDefinition()));
                    } else {
                        name->setDefinition(makeCC(std::move(UCD::resolveUnicodeSet(name))));
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
                if (CC * cc = extractCC(re)) {
                    if (unionCC == nullptr) {
                        unionCC = cc;
                    } else {
                        unionCC = makeCC(unionCC, cc);
                        name << '+';
                    }
                    if (LLVM_LIKELY(isa<Name>(re))) {
                        Name * n = cast<Name>(re);
                        if (n->hasNamespace()) {
                            name << n->getNamespace() << ':';
                        }
                        name << n->getName();
                    } else if (isa<CC>(re)) {
                        name << cast<CC>(re)->canonicalName(UnicodeClass);
                    }
                    ai = alt->erase(ai);
                } else {
                    *ai++ = re;
                }
            }
            if (unionCC) {
                alt->push_back(resolve(makeName(name.str(), unionCC)));
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
            CC * lh = extractCC(diff->getLH());
            CC * rh = extractCC(diff->getRH());
            if (lh && rh) {
                return resolve(makeName("diff", subtractCC(lh, rh)));
            }
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(resolve(ix->getLH()));
            ix->setRH(resolve(ix->getRH()));
            CC * lh = extractCC(ix->getLH());
            CC * rh = extractCC(ix->getRH());
            if (lh && rh) {
                return resolve(makeName("intersect", intersectCC(lh, rh)));
            }
        }
        return re;
    }

    NameResolver() {

    }

private:
    Memoizer                mMemoizer;
};
    
RE * resolveNames(RE *& re) {
    NameResolver nameResolver;
    return nameResolver.resolve(re);    
}

}
