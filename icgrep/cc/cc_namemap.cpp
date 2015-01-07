#include <cc/cc_namemap.hpp>
#include <re/re_name.h>
#include <re/re_cc.h>
#include <re/re_alt.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/printer_re.h>
#include <iostream>

using namespace re;

namespace cc {

RE * CC_NameMap::process(RE * re, const CC_type t) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            *i = process(*i, t);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            *i = process(*i, t);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        rep->setRE(process(rep->getRE(), t));
    }
    else if (Assertion * a = dyn_cast<Assertion>(re)) {
        a->setAsserted(process(a->getAsserted(), t));
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        diff->setRH(process(diff->getRH(), t));
        diff->setLH(process(diff->getLH(), t));
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        e->setRH(process(e->getRH(), t));
        e->setLH(process(e->getLH(), t));
    }
    else if (Name * nameNode = dyn_cast<Name>(re)) {
        RE * def = nameNode->getDefinition();
        if (def && !isa<CC>(def)) {
            nameNode->setDefinition(process(def, t));
        }
        std::string classname = nameNode->getName();
        auto f = mNameMap.find(classname);
        if (f == mNameMap.end()) {
            // Insert into the name map.
            return insert(std::move(classname), nameNode);
        }
        return f->second;
    }
    else if (CC * cc = dyn_cast<CC>(re)) {
        std::string classname = cc->canonicalName(t);
        auto f = mNameMap.find(classname);
        if (f == mNameMap.end()) {
	    Name * n;
	    if (t == ByteClass) {
	      n = makeByteName(classname, cc);
	    }
	    else {
	      n = makeName(classname, cc);
	    }
            return insert(std::move(classname), n);
        }
        return f->second;
    }
    return re;
}

std::string CC_NameMap::printMap() {
    std::string retval = "";
    for (Name * name : mNameVector) {
        retval.append("mNameMap[" +  name->getName() + "] = " + Printer_RE::PrintRE(name->getDefinition()) + "]\n");
    }
    return retval;
}

}
