#include <cc/cc_namemap.hpp>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/printer_re.h>
#include <iostream>

using namespace re;

namespace cc {

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
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        diff->setRH(process(diff->getRH()));
        diff->setLH(process(diff->getLH()));
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        e->setRH(process(e->getRH()));
        e->setLH(process(e->getLH()));
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
        std::string classname = cc->canonicalName();
        auto f = mNameMap.find(classname);
        if (f == mNameMap.end()) {
            return insert(std::move(classname), makeName(classname, cc));
        }
        return f->second;
    }
    return re;
}

std::string CC_NameMap::printMap() {
    std::string retval = "";
    for (Name * name : mNameVector) {
        retval.append("mNameMap[" +  name->getName() + "] = " + Printer_RE::PrintRE(name->getCC()) + "]\n");
    }
    return retval;
}

}
