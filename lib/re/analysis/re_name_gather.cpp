#include <re/analysis/re_name_gather.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/analysis/re_analysis.h>

using namespace llvm;
namespace re {
    
    
void gatherUnicodeProperties (RE * re, std::set<Name *> & nameSet) {
    if (Name * name = dyn_cast<Name>(re)) {
        RE * defn = name->getDefinition();
        if (defn && isa<CC>(defn) && (cast<CC>(defn)->getAlphabet() == &cc::Unicode)) {
            nameSet.emplace(cast<Name>(re));
        }
    } else if (isa<Seq>(re)) {
        for (RE * item : *cast<Seq>(re)) {
            gatherUnicodeProperties(item, nameSet);
        }
    } else if (isa<Alt>(re)) {
        for (RE * item : *cast<Alt>(re)) {
            gatherUnicodeProperties(item, nameSet);
        }
    } else if (isa<Rep>(re)) {
        gatherUnicodeProperties(cast<Rep>(re)->getRE(), nameSet);
    } else if (isa<Assertion>(re)) {
        gatherUnicodeProperties(cast<Assertion>(re)->getAsserted(), nameSet);
    } else if (Range * rg = dyn_cast<Range>(re)) {
        gatherUnicodeProperties(rg->getLo(), nameSet);
        gatherUnicodeProperties(rg->getHi(), nameSet);
    } else if (isa<Diff>(re)) {
        gatherUnicodeProperties(cast<Diff>(re)->getLH(), nameSet);
        gatherUnicodeProperties(cast<Diff>(re)->getRH(), nameSet);
    } else if (isa<Intersect>(re)) {
        gatherUnicodeProperties(cast<Intersect>(re)->getLH(), nameSet);
        gatherUnicodeProperties(cast<Intersect>(re)->getRH(), nameSet);
    } else if (isa<Group>(re)) {
        gatherUnicodeProperties(cast<Group>(re)->getRE(), nameSet);
    }
}

}
