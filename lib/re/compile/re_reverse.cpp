/*
 *  Copyright (c) 2018 International Characters.
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
#include <re/re_toolchain.h>
#include <llvm/Support/ErrorHandling.h>
#include <map>


using namespace llvm;

namespace re {
    
    using CaptureMap = std::map<std::string, Name *>;

class ReverseTransformer : public RE_Transformer {
public:
    ReverseTransformer() : RE_Transformer("Reverse") {}
    RE * transformName(Name * n) override {
        Name::Type nType = n ->getType();
        if (nType == Name::Type::Capture) {
            std::string cname = n->getName();
            auto f = mCaptureMap.find(cname);
            if (f != mCaptureMap.end()) {
                return makeReference(f->second->getName(), f->second);
            }
            else {
                std::string newName = "\\" + std::to_string(mCaptureMap.size() + 1);
                Name * capture = makeCapture(newName, transform(n->getDefinition()));
                mCaptureMap.emplace(cname, capture);
                return capture;
            }
        }
        if (nType == Name::Type::Reference) {
            Name * referent = cast<Name>(n->getDefinition());
            std::string cname = referent->getName();
            auto f = mCaptureMap.find(cname);
            if (f != mCaptureMap.end()) {
                return makeReference(f->second->getName(), f->second);
            }
            else {
                std::string newName = "\\" + std::to_string(mCaptureMap.size() + 1);
                Name * capture = makeCapture(newName, transform(referent->getDefinition()));
                mCaptureMap.emplace(cname, capture);
                return capture;
            }
        }
        return n;
    }
    RE * transformSeq (Seq * seq) override {
        std::vector<RE*> list;
        for (auto i = seq->rbegin(); i != seq->rend(); ++i) {
            list.push_back(transform(*i));
        }
        return makeSeq(list.begin(), list.end());
    }
    RE * transformStart (Start *) override { return makeEnd();}
    RE * transformEnd (End *) override { return makeStart();}
    RE * transformAssertion (Assertion * a) override {
        RE * asserted = a->getAsserted();
        RE * reversed = transform(asserted);
        if ((a->getKind() == Assertion::Kind::Boundary) && (reversed == asserted)) {
            return a;
        }
        return makeAssertion(reversed, Assertion::reverseKind(a->getKind()), a->getSense());
    }
    
private:
    CaptureMap mCaptureMap;
};

RE * reverse(RE * re) {
    return ReverseTransformer().transformRE(re);
}

}
