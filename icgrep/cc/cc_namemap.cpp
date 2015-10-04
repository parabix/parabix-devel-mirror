#include <cc/cc_namemap.hpp>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <UCD/resolve_properties.h>
#include <re/printer_re.h>

using namespace re;

namespace cc {

RE * CC_NameMap::process(RE * re, const CC_type type) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = process(*i, type);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = process(*i, type);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(process(rep->getRE(), type));
    }
    else if (Assertion * a = dyn_cast<Assertion>(re)) {
        a->setAsserted(process(a->getAsserted(), type));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        diff->setRH(process(diff->getRH(), type));
        diff->setLH(process(diff->getLH(), type));
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        e->setRH(process(e->getRH(), type));
        e->setLH(process(e->getLH(), type));
    }
    else if (Name * name = dyn_cast<Name>(re)) {
        RE * def = name->getDefinition();
        if (def) {
            if (!isa<CC>(def)) {
                name->setDefinition(process(def, type));
            }
        }
        else {
            std::string classname = name->getName();
            auto f = mNameMap.find(classname);
            if (f != mNameMap.end()) {
                return f->second;
            }            
            return insert(std::move(classname), name);
        }
    }
    else if (CC * cc = dyn_cast<CC>(re)) {
        std::string classname = cc->canonicalName(type);
        auto f = mNameMap.find(classname);
        if (f != mNameMap.end()) {
            return f->second;
        }
        return insert(std::move(classname), makeName(classname, cc));
    }
    return re;
}

std::string CC_NameMap::printMap() {
    std::string retval = "";
    for (Name * name : mNameVector) {
        retval.append("mNameMap[" +  name->getName() + "] = " + Printer_RE::PrintRE(name->getDefinition()) + "\n");
    }
    return retval;
}

}
