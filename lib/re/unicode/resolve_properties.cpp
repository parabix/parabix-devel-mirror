/*
 *  Copyright (c) 2020 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <re/unicode/resolve_properties.h>

#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>
#include <re/adt/adt.h>
#include <re/adt/re_name.h>
#include <re/compile/re_compiler.h>
#include <re/unicode/re_name_resolve.h>
#include <re/unicode/boundaries.h>
#include <unicode/data/PropertyAliases.h>
#include <unicode/data/PropertyObjects.h>
#include <unicode/data/PropertyValueAliases.h>

using namespace UCD;
using namespace re;
using namespace llvm;

namespace UCD {
    
void UnicodePropertyExpressionError(std::string errmsg) {
    llvm::report_fatal_error(errmsg);
}

bool resolvePropertyDefinition(Name * const property) {
    if (property->hasNamespace()) {
        auto propit = alias_map.find(property->getNamespace());
        if (propit == alias_map.end()) {
            UnicodePropertyExpressionError("Expected a property name but '" + property->getNamespace() + "' was found instead");
        }
        auto theprop = static_cast<UCD::property_t>(propit->second);
        if (isa<BinaryPropertyObject>(getPropertyObject(theprop))){
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
            property->setDefinition(resolveUnicodeNames(gcb));
            return true;
        } else if (value == "\\b") {
            RE * wb = makeBoundaryAssertion(makeName("word", Name::Type::UnicodeProperty));
            property->setDefinition(resolveUnicodeNames(wb));
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
            auto theprop = static_cast<UCD::property_t>(propit->second);
            auto propObj = getPropertyObject(theprop);
            if ((value.length() > 0) && (value[0] == '/')) {
                assert(false && "UCD::resolveUnicodeSet(re::Name *) does not support regex name resolution, use grep::resolveUnicodeSet(re::Name *) instead");
                llvm::report_fatal_error("");
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
                auto prop2 = static_cast<UCD::property_t>(propit->second);
                auto propObj2 = getPropertyObject(prop2);
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
            std::string canon = canonicalize_value_name(value);
            const auto & gcobj = cast<EnumeratedPropertyObject>(getPropertyObject(gc));
            int valcode = gcobj->GetPropertyValueEnumCode(canon);
            if (valcode >= 0) {
                return gcobj->GetCodepointSet(valcode);
            }
            const auto & scObj = cast<EnumeratedPropertyObject>(getPropertyObject(sc));
            valcode = scObj->GetPropertyValueEnumCode(canon);
            if (valcode >= 0) {
                return scObj->GetCodepointSet(valcode);
            }
            // Try as a binary property.
            
            auto propit = alias_map.find(canon);
            if (propit != alias_map.end()) {
                auto theprop = static_cast<UCD::property_t>(propit->second);
                auto propObj = getPropertyObject(theprop);
                if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(propObj)) {
                    return p->GetCodepointSet(Binary_ns::Y);
                }
                else {
                    UnicodePropertyExpressionError("Error: property " + property_full_name[theprop] + " specified without a value");
                }
            }
        }
    }
    UnicodePropertyExpressionError("Expected a general category, script or binary property name, but '" + name->getName() + "' found instead");
}

struct PropertyLinker : public RE_Transformer {
    PropertyLinker() : RE_Transformer("PropertyLinker") {}
    RE * transformPropertyExpression (PropertyExpression * exp) override {
        std::string id = exp->getPropertyIdentifier();
        std::string canon = UCD::canonicalize_value_name(id);
        auto propit = UCD::alias_map.find(canon);
        if (propit !=  UCD::alias_map.end()) {
            property_t prop = static_cast<UCD::property_t>(propit->second);
            exp->setPropertyCode(prop);
            return exp;
        }
        // If the property identifier is not a standard property name, it
        // could be the value of a script or a general category property,
        // or one of a few other special cases, provided that there is no "value".
        if (exp->getValue() != nullptr) return exp;
        const auto & gcObj = cast<EnumeratedPropertyObject>(getPropertyObject(gc));
        int valcode = gcObj->GetPropertyValueEnumCode(canon);
        if (valcode >= 0) {
            // Found a general category.
            exp->setValue(makeName(gcObj->GetValueFullName(valcode), Name::Type::PropertyValue));
            exp->setPropertyIdentifier(property_enum_name[gc]);
            exp->setPropertyCode(gc);
            return exp;
        }
        const auto & scObj = cast<EnumeratedPropertyObject>(getPropertyObject(sc));
        valcode = scObj->GetPropertyValueEnumCode(canon);
        if (valcode >= 0) {
            // Found a script.
            exp->setValue(makeName(scObj->GetValueFullName(valcode), Name::Type::PropertyValue));
            exp->setPropertyIdentifier(property_enum_name[sc]);
            exp->setPropertyCode(sc);
            return exp;
        }
        if (canon == "ascii") {  // block:ascii special case
            exp->setValue(makeName("ascii", Name::Type::PropertyValue));
            exp->setPropertyIdentifier(property_enum_name[blk]);
            exp->setPropertyCode(blk);
            return exp;
        }
        if (canon == "assigned") {  // cn:n special case
            // general category != unassigned
            exp->setValue(makeName("unassigned", Name::Type::PropertyValue));
            exp->setPropertyIdentifier(property_enum_name[gc]);
            exp->setOperator(PropertyExpression::Operator::NEq);
            exp->setPropertyCode(gc);
            return exp;
        }
        return exp;
    }
};

void linkProperties(RE * r) {
    PropertyLinker().transformRE(r);
}

}
