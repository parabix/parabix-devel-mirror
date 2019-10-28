/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <grep/grep_name_resolve.h>

#include <grep/resolve_properties.h>
#include <re/adt/adt.h>
#include <re/transforms/re_transformer.h>
#include <re/unicode/resolve_properties.h>
#include <llvm/Support/ErrorHandling.h>

using namespace re;

namespace grep {

// Same UnicodeNameResolver that is used in re/unicode/re_name_resolver.cpp 
// but it uses grep::resolveUnicodeSet instead of UCD::resolveUnicodeSet
class UnicodeNameResolver final : public RE_Transformer {
public:
    UnicodeNameResolver() : RE_Transformer("UnicodeNames") {}
    RE * transformName(Name * name) override;
    RE * transformRange(Range * rg) override;
};
    
RE * UnicodeNameResolver::transformName(Name * name) {
    if (LLVM_LIKELY(name->getDefinition() != nullptr)) {
        name->setDefinition(transform(name->getDefinition()));
    } else if (LLVM_LIKELY(name->getType() == Name::Type::UnicodeProperty || name->getType() == Name::Type::ZeroWidth)) {
        if (UCD::resolvePropertyDefinition(name)) {
            name->setDefinition(transform(name->getDefinition()));
        } else {
            name->setDefinition(makeCC(grep::resolveUnicodeSet(name), &cc::Unicode));
        }        
    } else {
        UndefinedNameError(name);
    }
    return name;
}

RE * getDefinitionOf(RE * const re) {
    if (Name * const name = llvm::dyn_cast<Name>(re)) {
        RE * const def = name->getDefinition();
        if (LLVM_UNLIKELY(def == nullptr)) {
            llvm::report_fatal_error(name->getFullName() + " could not be resolved.");
        }
        return def;
    }
    return re;
}

RE * UnicodeNameResolver::transformRange(Range * rg) {
    RE * const x = getDefinitionOf(transform(rg->getLo()));
    RE * const y = getDefinitionOf(transform(rg->getHi()));
    return makeRange(x, y);
}

RE * resolveUnicodeNames(RE * re) {
    return UnicodeNameResolver().transformRE(re);
}

}
