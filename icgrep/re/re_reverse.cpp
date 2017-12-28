/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_reverse.h"
#include <re/re_cc.h>
#include <re/re_name.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <re/re_any.h>
#include <re/re_seq.h>
#include <re/re_alt.h>
#include <re/re_rep.h>
#include <re/re_group.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <llvm/Support/ErrorHandling.h>
#include <map>


using namespace llvm;

namespace re {

using CaptureMap = std::map<std::string, re::Name *>;

RE * reverse(RE * re, CaptureMap & captureMap) {
    if (isa<CC>(re)) {
        return re;
    } else if (Range * rg = dyn_cast<Range>(re)) {
        return makeRange(rg->getLo(), rg->getHi());
    } else if (Seq * seq = dyn_cast<Seq>(re)) {
        std::vector<RE*> list;
        for (auto i = seq->rbegin(); i != seq->rend(); ++i) {
            list.push_back(reverse(*i, captureMap));
        }
        return makeSeq(list.begin(), list.end());
    } else if (Alt * alt = dyn_cast<Alt>(re)) {
        std::vector<RE*> list;
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            list.push_back(reverse(*i, captureMap));
        }
        return makeAlt(list.begin(), list.end());
    } else if (Rep * rep = dyn_cast<Rep>(re)) {
        return makeRep(reverse(rep->getRE(), captureMap), rep->getLB(), rep->getUB());
    } else if (Group * g = dyn_cast<Group>(re)) {
        return makeGroup(g->getMode(), reverse(g->getRE(), captureMap), g->getSense());
    } else if (Diff * diff = dyn_cast<Diff>(re)) {
        return makeDiff(reverse(diff->getLH(), captureMap), reverse(diff->getRH(), captureMap));
    } else if (Intersect * e = dyn_cast<Intersect>(re)) {
        return makeIntersect(reverse(e->getLH(), captureMap), reverse(e->getRH(), captureMap));
    } else if (Assertion * a = dyn_cast<Assertion>(re)) {
        return makeAssertion(reverse(a->getAsserted(), captureMap), Assertion::reverseKind(a->getKind()), a->getSense());
    } else if (isa<Start>(re)) {
        return makeEnd();
    } else if (isa<End>(re)) {
        return makeStart();
    } else if (Name * n = dyn_cast<Name>(re)) {
        switch (n->getType()) {
            case Name::Type::Unicode:
                return makeName(cast<CC>(n->getDefinition()));
            case Name::Type::UnicodeProperty:
                return makeName(n->getNamespace(), n->getName(), Name::Type::UnicodeProperty);
            case Name::Type::ZeroWidth:
                return makeZeroWidth(n->getName(), reverse(n->getDefinition(), captureMap));
            case Name::Type::Capture:
                {
                    std::string cname = n->getName();
                    auto f = captureMap.find(cname);
                    if (f != captureMap.end()) {
                        return makeReference(f->second->getName(), f->second);
                    }
                    else {
                        std::string newName = "\\" + std::to_string(captureMap.size() + 1);
                        Name * capture = makeCapture(newName, reverse(n->getDefinition(), captureMap));
                        captureMap.insert(std::make_pair(cname, capture));
                        return capture;
                    }
                }
            case Name::Type::Reference:
                {
                    Name * referent = cast<Name>(n->getDefinition());
                    std::string cname = referent->getName();
                    auto f = captureMap.find(cname);
                    if (f != captureMap.end()) {
                       return makeReference(f->second->getName(), f->second);
                    }
                    else {
                        std::string newName = "\\" + std::to_string(captureMap.size() + 1);
                        Name * capture = makeCapture(newName, reverse(referent->getDefinition(), captureMap));
                        captureMap.insert(std::make_pair(cname, capture));
                        return capture;
                    }
                }
            case Name::Type::Unknown:
                return makeName(n->getName(), n->getDefinition());
            default:
                llvm::report_fatal_error("re::reverse: unhandled Name type");
        }
    } else if (isa<Any>(re)) {
        return makeAny();
    } else {
        llvm::report_fatal_error("re::reverse: unhandled regexp type");
    }

}

RE * reverse(RE * re) {
    CaptureMap captureMap;
    return reverse(re, captureMap);
}

}
