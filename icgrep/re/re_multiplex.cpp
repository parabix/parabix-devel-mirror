#include "re_multiplex.h"
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
#include <cc/multiplex_CCs.h>
#include <sstream>
#include <iostream>
#include <functional>

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

RE * multiplex(RE * const re,
               const std::vector<const CC *> & UnicodeSets,
               const std::vector<std::vector<unsigned>> & exclusiveSetIDs) {

    Memoizer memoizer;

    std::function<RE *(RE *)> multiplex = [&](RE * const re) -> RE * {
        if (Name * name = dyn_cast<Name>(re)) {
            auto f = memoizer.find(name);
            if (f == memoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    if (CC * cc = dyn_cast<CC>(name->getDefinition())) {
                        const auto index = find(UnicodeSets.begin(), UnicodeSets.end(), cc) - UnicodeSets.begin();
                        const auto exclusive_IDs = exclusiveSetIDs[index];
                        CC * CC_union = makeCC();
                        for (auto i : exclusive_IDs) {
                            CC_union = makeCC(CC_union, makeCC(i));
                        }
                        name->setDefinition(CC_union);
                    } else {
                        multiplex(name->getDefinition());
                    }
                } else {
                    throw std::runtime_error("All non-unicode-property Name objects should have been defined prior to Unicode property resolution.");
                }
                return memoizer.memoize(name);
            } else {
                return *f;
            }
        } else if (Seq * seq = dyn_cast<Seq>(re)) {
            for (auto si = seq->begin(); si != seq->end(); ++si) {
                *si = multiplex(*si);
            }
        } else if (Alt * alt = dyn_cast<Alt>(re)) {
            CC * unionCC = nullptr;
            std::stringstream name;
            for (auto ai = alt->begin(); ai != alt->end(); ) {
                RE * re = multiplex(*ai);
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
                alt->push_back(multiplex(makeName(name.str(), unionCC)));
            }
            if (alt->size() == 1) {
                return alt->front();
            }
        } else if (Rep * rep = dyn_cast<Rep>(re)) {
            rep->setRE(multiplex(rep->getRE()));
        } else if (Assertion * a = dyn_cast<Assertion>(re)) {
            a->setAsserted(multiplex(a->getAsserted()));
        } else if (Diff * diff = dyn_cast<Diff>(re)) {
            diff->setLH(multiplex(diff->getLH()));
            diff->setRH(multiplex(diff->getRH()));
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(multiplex(ix->getLH()));
            ix->setRH(multiplex(ix->getRH()));
        }
        return re;
    };

    return multiplex(re);
}    

}
