#include "re_name_gather.h"
#include <re/re_name.h>
#include <re/re_alt.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_range.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_group.h>
#include <re/re_analysis.h>
#include <re/re_memoizer.hpp>
#include <UCD/ucd_compiler.hpp>
#include <UCD/resolve_properties.h>
#include <boost/container/flat_set.hpp>
#include <sstream>

using NameMap = UCD::UCDCompiler::NameMap;

using namespace boost::container;
using namespace llvm;

namespace re {

struct NameGather {

    void gather(RE * re) {
        assert ("RE object cannot be null!" && re);
        if (isa<Name>(re)) {
            if (mVisited.insert(cast<Name>(re)).second) {
                if (cast<Name>(re)->getType() == Name::Type::ZeroWidth) {
                    mZeroWidth = cast<Name>(re);
                }
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
        } else if (Range * rg = dyn_cast<Range>(re)) {
            gather(rg->getLo());
            gather(rg->getHi());
        } else if (isa<Diff>(re)) {
            gather(cast<Diff>(re)->getLH());
            gather(cast<Diff>(re)->getRH());
        } else if (isa<Intersect>(re)) {
            gather(cast<Intersect>(re)->getLH());
            gather(cast<Intersect>(re)->getRH());
        } else if (isa<Group>(re)) {
            gather(cast<Group>(re)->getRE());
        }
    }
    NameGather(NameMap & nameMap, Name *& zeroWidth)
    : mZeroWidth(zeroWidth)
    , mNameMap(nameMap) {

    }

private:

    Name *&                 mZeroWidth;
    NameMap &               mNameMap;
    flat_set<Name *>        mVisited;

};
    
NameMap gatherNames(RE *& re, Name *& zeroWidth) {
    NameMap nameMap;
    NameGather nameGather(nameMap, zeroWidth);
    nameGather.gather(re);
    return nameMap;
    
}

}
