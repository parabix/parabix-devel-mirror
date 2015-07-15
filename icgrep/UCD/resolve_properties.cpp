/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include "resolve_properties.h"
#include <re/re_re.h>
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_cc.h>
#include <re/re_seq.h>
#include <re/re_rep.h>
#include <re/re_name.h>
#include <re/re_diff.h>
#include <re/re_intersect.h>
#include <re/re_assertion.h>
#include <re/re_start.h>
#include <re/re_end.h>
#include <cc/cc_namemap.hpp>
#include "UCD/PropertyAliases.h"
#include "UCD/PropertyObjects.h"
#include "UCD/PropertyObjectTable.h"
#include "UCD/PropertyValueAliases.h"
#include <boost/algorithm/string/case_conv.hpp>
#include <string>
#include <iostream>

using namespace UCD;
using namespace re;

class UnicodePropertyExpressionError : public std::exception {
public:
    UnicodePropertyExpressionError(const std::string && msg) noexcept : _msg(msg) {}
    const char* what() const noexcept { return _msg.c_str();}
private:
    inline UnicodePropertyExpressionError() noexcept {}
    const std::string _msg;
};

inline std::string lowercase(const std::string & name) {
    std::locale loc;
    return boost::algorithm::to_lower_copy(name, loc);
}

inline int GetPropertyValueEnumCode(const UCD::property_t type, const std::string & value) {
    return property_object_table[type]->GetPropertyValueEnumCode(value);
}

void resolveProperty(Name * name) {
    const std::string prop = canonicalize_value_name(name->getNamespace());
    const std::string value = canonicalize_value_name(name->getName());
    if (prop.length() != 0) {
        auto propit = alias_map.find(prop);
        if (propit == alias_map.end()) {
            throw UnicodePropertyExpressionError("Expected a property name, but '" + name->getNamespace() + "' found instead");
        }
        auto theprop = propit->second;
        if (theprop == gc) {
            // General Category
            int valcode = GetPropertyValueEnumCode(gc, value);
            if (valcode < 0) {
                throw UnicodePropertyExpressionError("Erroneous property value for general_category property");
            }
            name->setFunctionName("__get_gc_" + GC_ns::enum_names[valcode]);
        }
        else if (theprop == sc) {
            // Script property identified
            int valcode = GetPropertyValueEnumCode(sc, value);
            if (valcode < 0) {
                throw UnicodePropertyExpressionError("Erroneous property value for script property");
            }
            name->setFunctionName("__get_sc_" + SC_ns::enum_names[valcode]);
        }
        else if (theprop == scx) {
            // Script extension property identified
            int valcode = GetPropertyValueEnumCode(sc, value);
            if (valcode < 0) {
                throw UnicodePropertyExpressionError("Erroneous property value for script_extension property");
            }
            name->setFunctionName("__get_scx_" + SC_ns::enum_names[valcode]);
        }
        else if (theprop == blk) {
            // Block property identified
            int valcode = GetPropertyValueEnumCode(blk, value);
            if (valcode < 0) {
                 throw UnicodePropertyExpressionError("Erroneous property value for block property");
            }
            name->setFunctionName("__get_blk_" + BLK_ns::enum_names[valcode]);
        }
        else if (isa<BinaryPropertyObject>(property_object_table[theprop])){
            auto valit = Binary_ns::aliases_only_map.find(value);
            if (valit == Binary_ns::aliases_only_map.end()) {
                throw UnicodePropertyExpressionError("Erroneous property value for binary property " + property_full_name[theprop]);
            }
            if (valit->second == Binary_ns::Y) {
                name->setFunctionName("__get_" + lowercase(property_enum_name[theprop]) + "_Y");
            }
            else {
                Name * binprop = makeName("__get_" + lowercase(property_enum_name[theprop]) + "_Y", Name::Type::UnicodeProperty);
                name->setDefinition(makeDiff(makeAny(), binprop));
            }
        }
        else {
            throw UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized, but not supported in icgrep 1.0");
        }
    }
    else {

        // No namespace (property) name.   Try as a general category.

        int valcode;

        if ((valcode = GetPropertyValueEnumCode(gc, value)) >= 0) {
            name->setFunctionName("__get_gc_" + GC_ns::enum_names[valcode]);
            return;
        }

        if ((valcode = GetPropertyValueEnumCode(sc, value)) >= 0) {
            name->setFunctionName("__get_sc_" + SC_ns::enum_names[valcode]);
            return;
        }

        // Try as a binary property.
        auto propit = alias_map.find(value);
        if (propit != alias_map.end()) {
            auto theprop = propit->second;
            if (isa<BinaryPropertyObject>(property_object_table[theprop])) {
                name->setFunctionName("__get_" + lowercase(property_enum_name[theprop]) + "_Y");
            }
            else {
                throw UnicodePropertyExpressionError("Error: property " + property_full_name[theprop] + " specified without a value");
            }
        }
        else {
            throw UnicodePropertyExpressionError("Expected a general category, script or binary property name, but '" + name->getName() + "' found instead");
        }
    }
}

UnicodeSet resolveUnicodeSet(Name * const name) {
    if (name->getType() == Name::Type::UnicodeProperty) {
        std::string prop = name->getNamespace();
        std::string value = canonicalize_value_name(name->getName());
        if (prop.length() > 0) {
            prop = canonicalize_value_name(prop);
            auto propit = alias_map.find(prop);
            if (propit == alias_map.end()) {
                throw UnicodePropertyExpressionError("Expected a property name, but '" + name->getNamespace() + "' found instead");
            }
            auto theprop = propit->second;
            if (theprop == gc) {
                // General Category
                return cast<EnumeratedPropertyObject>(property_object_table[gc])->GetCodepointSet(value);
            }
            else if (theprop == sc) {
                // Script property identified
                return cast<EnumeratedPropertyObject>(property_object_table[sc])->GetCodepointSet(value);
            }
            else if (theprop == scx) {
                // Script extension property identified
                return cast<ExtensionPropertyObject>(property_object_table[scx])->GetCodepointSet(value);
            }
            else if (theprop == blk) {
                // Block property identified
                return cast<EnumeratedPropertyObject>(property_object_table[blk])->GetCodepointSet(value);
            }
            else if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(property_object_table[theprop])){
                auto valit = Binary_ns::aliases_only_map.find(value);
                if (valit == Binary_ns::aliases_only_map.end()) {
                    throw UnicodePropertyExpressionError("Erroneous property value for binary property " + property_full_name[theprop]);
                }
                return p->GetCodepointSet(value);
            }           
            throw UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized but not supported in icgrep 1.0");
        }
        else {
            // No namespace (property) name.   Try as a general category.
            int valcode = GetPropertyValueEnumCode(gc, value);
            if (valcode >= 0) {
                return cast<EnumeratedPropertyObject>(property_object_table[gc])->GetCodepointSet(valcode);
            }
            valcode = GetPropertyValueEnumCode(sc, value);
            if (valcode >= 0) {
                return cast<EnumeratedPropertyObject>(property_object_table[sc])->GetCodepointSet(valcode);
            }
            // Try as a binary property.
            auto propit = alias_map.find(value);
            if (propit != alias_map.end()) {
                auto theprop = propit->second;
                if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(property_object_table[theprop])) {
                    return p->GetCodepointSet(Binary_ns::Y);
                }
                else {
                    throw UnicodePropertyExpressionError("Error: property " + property_full_name[theprop] + " specified without a value");
                }
            }
        }
    }
    throw UnicodePropertyExpressionError("Expected a general category, script or binary property name, but '" + name->getName() + "' found instead");
}
