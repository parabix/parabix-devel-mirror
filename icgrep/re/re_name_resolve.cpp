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
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <sstream>

#include <re/printer_re.h>
#include <iostream>

using NameMap = UCD::UCDCompiler::NameMap;

using namespace boost::container;

namespace re {
  
static inline CC * getDefinition(RE * re) {
    if (isa<CC>(re)) {
        return cast<CC>(re);
    } else if (isa<Name>(re)) {
        return getDefinition(cast<Name>(re)->getDefinition());
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
                        if (name->getType() == Name::Type::ZeroWidth) {
                            mZeroWidth = name;
                        }
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
                if (CC * cc = getDefinition(re)) {
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
            CC * lh = getDefinition(diff->getLH());
            CC * rh = getDefinition(diff->getRH());
            if (lh && rh) {
                return resolve(makeName("diff", subtractCC(lh, rh)));
            }
        } else if (Intersect * ix = dyn_cast<Intersect>(re)) {
            ix->setLH(resolve(ix->getLH()));
            ix->setRH(resolve(ix->getRH()));
            CC * lh = getDefinition(ix->getLH());
            CC * rh = getDefinition(ix->getRH());
            if (lh && rh) {
                return resolve(makeName("intersect", intersectCC(lh, rh)));
            }
        }
        return re;
    }

    void gather(RE * re) {
        assert ("RE object cannot be null!" && re);
        if (isa<Name>(re)) {
            if (mVisited.insert(cast<Name>(re)).second) {
                if (isa<CC>(cast<Name>(re)->getDefinition())) {
                    mNameMap.emplace(cast<Name>(re), nullptr);
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
        }
    }

    NameResolver(NameMap & nameMap, Name *& zeroWidth)
    : mZeroWidth(zeroWidth)
    , mNameMap(nameMap) {

    }

private:

    Name *&                 mZeroWidth;
    NameMap &               mNameMap;
    Memoizer                mMemoizer;
    flat_set<Name *>        mVisited;

};
    
NameMap resolveNames(RE *& re, Name *& zeroWidth) {
    NameMap nameMap;
    NameResolver nameResolver(nameMap, zeroWidth);
    re = nameResolver.resolve(re);
    nameResolver.gather(re);
    return nameMap;
    
}

}
