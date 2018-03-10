#include "re_multiplex.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_group.h>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <re/printer_re.h>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <cc/alphabet.h>
#include <cc/multiplex_CCs.h>
#include <sstream>
#include <iostream>
#include <functional>
#include <llvm/Support/raw_ostream.h>

using namespace boost::container;
using namespace llvm;

namespace re {
  
RE * multiplex(RE * const re,
               const std::vector<const CC *> & UnicodeSets,
               const std::vector<std::vector<unsigned>> & exclusiveSetIDs) {

    Memoizer memoizer;

    std::function<RE *(RE *)> multiplex = [&](RE * const re) -> RE * {
        if (CC * cc = dyn_cast<CC>(re)) {
            if (cc->getAlphabet() != &cc::Unicode) return cc;
            const auto index = find(UnicodeSets.begin(), UnicodeSets.end(), cc) - UnicodeSets.begin();
            const auto exclusive_IDs = exclusiveSetIDs[index];
            CC * CC_union = makeCC();
            for (auto i : exclusive_IDs) {
                CC_union = makeCC(CC_union, makeCC(i));
            }
            return CC_union;
        } else if (Name * name = dyn_cast<Name>(re)) {
            auto f = memoizer.find(name);
            if (f == memoizer.end()) {
                if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
                    if (CC * cc = dyn_cast<CC>(name->getDefinition())) {
                        if (cc->getAlphabet() != &cc::Unicode) return cc;
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
                    UndefinedNameError(name);
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
            for (auto ai = alt->begin(); ai != alt->end(); ++ai) {
                *ai = multiplex(*ai);
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
        } else if (Group * g = dyn_cast<Group>(re)) {
            g->setRE(multiplex(g->getRE()));
        }
        return re;
    };

    return multiplex(re);
}    


RE * transformCCs(cc::MultiplexedAlphabet * mpx, RE * re) {
    if (CC * cc = dyn_cast<CC>(re)) {
        if (cc->getAlphabet() == mpx->getSourceAlphabet()) {
            re = mpx->transformCC(cc);
        }
    } else if (Name * name = dyn_cast<Name>(re)) {
        if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
            RE * xfrm = transformCCs(mpx, name->getDefinition());
            if (name->getType() == Name::Type::ZeroWidth)
                re = makeZeroWidth(name->getName(), xfrm);
            else if (name->getType() == Name::Type::Capture)
                re = makeCapture(name->getName(), xfrm);
            else
                re = makeName(name->getName(), xfrm);
        } else {
            UndefinedNameError(name);
        }
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE *> list;
        list.reserve(seq->size());
        for (RE * item : *seq) {
            item = transformCCs(mpx, item);
            list.push_back(item);
        }
        re = makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE *> list;
        list.reserve(alt->size());
        for (RE * item : *alt) {
            item = transformCCs(mpx, item);
            list.push_back(item);
        }
        re = makeAlt(list.begin(), list.end());
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        re = makeAssertion(transformCCs(mpx, a->getAsserted()), a->getKind(), a->getSense());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        RE * expr = transformCCs(mpx, rep->getRE());
        re = makeRep(expr, rep->getLB(), rep->getUB());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        re = makeDiff(transformCCs(mpx, diff->getLH()), transformCCs(mpx, diff->getRH()));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        re = makeIntersect(transformCCs(mpx, e->getLH()), transformCCs(mpx, e->getRH()));
    } else if (Group * g = dyn_cast<Group>(re)) {
        re = makeGroup(g->getMode(), transformCCs(mpx, g->getRE()), g->getSense());
    }
    return re;
};


}
