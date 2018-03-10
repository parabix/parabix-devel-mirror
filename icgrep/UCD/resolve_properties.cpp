/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <re/re_re.h>
#include "resolve_properties.h"
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_diff.h>
#include <re/re_start.h> 
#include <re/re_end.h> 
#include <re/re_cc.h> 
#include <re/re_seq.h> 
#include <re/re_assertion.h>
#include <re/re_parser.h>
#include <re/re_name_resolve.h>
#include <re/grapheme_clusters.h>
#include <re/re_compiler.h>
#include "UCD/PropertyAliases.h"
#include "UCD/PropertyObjects.h"
#include "UCD/PropertyObjectTable.h"
#include "UCD/PropertyValueAliases.h"
#include <llvm/Support/ErrorHandling.h>

using namespace UCD;
using namespace re;
using namespace llvm;

namespace UCD {
    
void UnicodePropertyExpressionError(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}
    
    
RE * UnicodeBreakRE() {
    return makeAlt({makeCC(0x0A, 0x0C), makeCC(0x85), makeCC(0x2028,0x2029), makeSeq({makeCC(0x0D), makeNegativeLookAheadAssertion(makeCC(0x0A))})});
}

bool resolvePropertyDefinition(Name * const property) {
    if (property->hasNamespace()) {
        auto propit = alias_map.find(property->getNamespace());
        if (propit == alias_map.end()) {
            UnicodePropertyExpressionError("Expected a property name but '" + property->getNamespace() + "' was found instead");
        }
        auto theprop = propit->second;
        if (isa<BinaryPropertyObject>(property_object_table[theprop])){
            auto valit = Binary_ns::aliases_only_map.find(property->getName());
            if (valit != Binary_ns::aliases_only_map.end()) {
                if (valit->second == Binary_ns::N) {
                    Name * binprop = makeName(property_enum_name[theprop], Name::Type::UnicodeProperty);
                    property->setDefinition(makeDiff(makeAny(), binprop));
                    return true;
                }
            }
        }
    } else {
        const std::string value = property->getName();
        // Try special cases of Unicode TR #18
        if ((value == "any") || (value == ".")) {
            property->setDefinition(makeCC(0, 0x10FFFF));
            return true;
        } else if (value == "ascii") {
            property->setDefinition(makeName("blk", "ascii", Name::Type::UnicodeProperty));
            return true;
        } else if (value == "assigned") {
            Name * unassigned = makeName("cn", Name::Type::UnicodeProperty);
            property->setDefinition(makeDiff(makeAny(), unassigned));
            return true;
        } else if (value == "\\b{g}") {
            RE * gcb = generateGraphemeClusterBoundaryRule();
            property->setDefinition(resolveUnicodeProperties(gcb));
            return true;
        } else if (value == "^s") {  // "start anchor (^) in single-line mode"
            property->setDefinition(makeNegativeLookBehindAssertion(makeCC(0, 0x10FFFF)));
            return true;
        } else if (value == "$s") { // "end anchor ($) in single-line mode"
            property->setDefinition(makeNegativeLookAheadAssertion(makeCC(0, 0x10FFFF)));
            return true;
        }
    }
    return false;
}

const std::string & getPropertyValueGrepString(const std::string & prop) {
    auto propit = alias_map.find(canonicalize_value_name(prop));
    if (propit == alias_map.end()) {
        UnicodePropertyExpressionError("Expected a property name, but '" + prop + "' found instead");
    }
    auto theprop = propit->second;
    if (EnumeratedPropertyObject * p = dyn_cast<EnumeratedPropertyObject>(property_object_table[theprop])){
        return p->GetPropertyValueGrepString();
    } else if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(property_object_table[theprop])) {
        return p->GetPropertyValueGrepString();
    }

    UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized but not supported in icgrep 1.0");
}

UnicodeSet resolveUnicodeSet(Name * const name) {
    if (name->getType() == Name::Type::UnicodeProperty) {
        std::string prop = name->getNamespace();
        std::string value = name->getName();
        if (prop.length() > 0) {
            prop = canonicalize_value_name(prop);
            auto propit = alias_map.find(prop);
            if (propit == alias_map.end()) {
                UnicodePropertyExpressionError("Expected a property name, but '" + name->getNamespace() + "' found instead");
            }
            auto propObj = property_object_table[propit->second];
            if ((value.length() > 0) && (value[0] == '/')) {
                // resolve a regular expression
                re::RE * propValueRe = RE_Parser::parse(value.substr(1), re::DEFAULT_MODE, re::PCRE, false);
                propValueRe = re::resolveNames(propValueRe);  // Recursive name resolution may be required.
                return propObj->GetCodepointSetMatchingPattern(propValueRe);
            }
            if ((value.length() > 0) && (value[0] == '@')) {
                // resolve a @property@ or @identity@ expression.
                std::string otherProp = canonicalize_value_name(value.substr(1));
                if (otherProp == "identity") {
                    return propObj->GetReflexiveSet();
                }
                auto propit = alias_map.find(prop);
                if (propit == alias_map.end()) {
                    UnicodePropertyExpressionError("Expected a property name, but '" + value.substr(1) + "' found instead");
                }
                auto propObj2 = property_object_table[propit->second];
                if (isa<BinaryPropertyObject>(propObj) && isa<BinaryPropertyObject>(propObj2)) {
                    return ~(cast<BinaryPropertyObject>(propObj)->GetCodepointSet(UCD::Binary_ns::Y) ^
                             cast<BinaryPropertyObject>(propObj2)->GetCodepointSet(UCD::Binary_ns::Y));
                }
                else {
                    UnicodePropertyExpressionError("unsupported");
                }
            }
            else {
                return propObj->GetCodepointSet(value);
            }
        }
        else {
            // No namespace (property) name.   Try as a general category.
            const auto & gcobj = cast<EnumeratedPropertyObject>(property_object_table[gc]);
            int valcode = gcobj->GetPropertyValueEnumCode(value);
            if (valcode >= 0) {
                return gcobj->GetCodepointSet(valcode);
            }
            const auto & scObj = cast<EnumeratedPropertyObject>(property_object_table[sc]);
            valcode = scObj->GetPropertyValueEnumCode(value);
            if (valcode >= 0) {
                return scObj->GetCodepointSet(valcode);
            }
            // Try as a binary property.
            auto propit = alias_map.find(value);
            if (propit != alias_map.end()) {
                auto theprop = propit->second;
                if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(property_object_table[theprop])) {
                    return p->GetCodepointSet(Binary_ns::Y);
                }
                else {
                    UnicodePropertyExpressionError("Error: property " + property_full_name[theprop] + " specified without a value");
                }
            }
            // Try special cases of Unicode TR #18
            // Now compatibility properties of UTR #18 Annex C
                    
            else if (value == ".") return UnicodeSet(0, 0x10FFFF);
            else if (value == "alnum") {
                Name * digit = makeName("nd", Name::Type::UnicodeProperty);
                Name * alpha = makeName("alphabetic", Name::Type::UnicodeProperty);
                return resolveUnicodeSet(digit) + resolveUnicodeSet(alpha);
            } else if (value == "xdigit") {
                Name * digit = makeName("nd", Name::Type::UnicodeProperty);
                Name * hexdigit = makeName("hexdigit", Name::Type::UnicodeProperty);
                return resolveUnicodeSet(digit) + resolveUnicodeSet(hexdigit);
            } else if (value == "blank") {
                Name * space_sep = makeName("space_separator", Name::Type::UnicodeProperty);
                return resolveUnicodeSet(space_sep) + UnicodeSet(0x09) /* tab */;
            } else if (value == "print") {
                Name * graph = makeName("graph", Name::Type::UnicodeProperty);
                Name * space_sep = makeName("space_separator", Name::Type::UnicodeProperty);
                return resolveUnicodeSet(graph) + resolveUnicodeSet(space_sep);
            } else if (value == "word") {
                Name * alnum = makeName("alnum", Name::Type::UnicodeProperty);
                Name * mark = makeName("mark", Name::Type::UnicodeProperty);
                Name * conn = makeName("connectorpunctuation", Name::Type::UnicodeProperty);
                Name * join = makeName("joincontrol", Name::Type::UnicodeProperty);
                return resolveUnicodeSet(alnum) + resolveUnicodeSet(mark) + resolveUnicodeSet(conn) + resolveUnicodeSet(join);
            } else if (value == "graph") {
                Name * space = makeName("space", Name::Type::UnicodeProperty);
                Name * ctrl = makeName("control", Name::Type::UnicodeProperty);
                Name * surr = makeName("surrogate", Name::Type::UnicodeProperty);
                Name * unassigned = makeName("cn", Name::Type::UnicodeProperty);
                return ~(resolveUnicodeSet(space) + resolveUnicodeSet(ctrl) + resolveUnicodeSet(surr) + resolveUnicodeSet(unassigned));
            }


        }
    }
    UnicodePropertyExpressionError("Expected a general category, script or binary property name, but '" + name->getName() + "' found instead");
}

}
