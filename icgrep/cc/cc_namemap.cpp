#include <cc/cc_namemap.hpp>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>

#include <re/printer_re.h>
#include <iostream>

using namespace re;

namespace cc {

void CC_NameMap::addPredefined(const std::string friendlyName, re::CC * cc) {
    assert (cc);
    std::string classname = cc->getName();
    Name * name = makeName(classname, cc);
    assert (name->getCC() == cc);
    mNameMap.insert(std::make_pair(friendlyName, name));    
    insert(std::move(classname), name);
    assert (name->getCC() == cc);
}

void CC_NameMap::clear() {
    mNameMap.clear();
    mNameVector.clear();
}

RE * CC_NameMap::process(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = process(*i);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = process(*i);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(process(rep->getRE()));
    }
    else if (Name * name = dyn_cast<Name>(re)) {
        RE * cc = name->getCC();
        if (cc && !isa<CC>(cc)) {
            name->setCC(process(cc));
        }
        std::string classname = name->getName();
        auto f = mNameMap.find(classname);
        if (f == mNameMap.end()) {
            return insert(std::move(classname), name);
        }
        return f->second;
    }
    else if (CC * cc = dyn_cast<CC>(re)) {
        std::string classname = cc->getName();
        auto f = mNameMap.find(classname);
        if (f == mNameMap.end()) {
            return insert(std::move(classname), makeName(classname, cc));
        }
        return f->second;
    }
    return re;
}

}
