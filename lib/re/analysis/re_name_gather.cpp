#include <re/analysis/re_name_gather.h>

#include <llvm/Support/Casting.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/analysis/re_analysis.h>
#include <re/analysis/re_inspector.h>

using namespace llvm;
namespace re {
    
struct NameCollector final : public RE_Inspector {

    NameCollector(std::set<Name *> & nameSet)
    : RE_Inspector(InspectionMode::IgnoreNonUnique)
    , mNameSet(nameSet) {

    }

    void inspectName(Name * n) final {
        RE * defn = n->getDefinition();
        if (defn) inspect(defn);
        mNameSet.insert(n);
    }

private:
    std::set<Name *> & mNameSet;
};

void gatherNames(RE * const re, std::set<Name *> & nameSet) {
    NameCollector collector(nameSet);
    collector.inspectRE(re);
}
}
