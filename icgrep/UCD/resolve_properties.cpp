/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include "resolve_properties.h"
#include <re/re_alt.h>
#include <re/re_any.h>
#include <re/re_name.h>
#include <re/re_diff.h>
#include <re/re_parser.h>
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

namespace UCD {

Name * resolveProperty(const std::string prop, const std::string value, re::RE_Parser * parser) {
    auto propit = alias_map.find(prop);
    if (propit == alias_map.end()) {
        throw UnicodePropertyExpressionError("Expected a property name but '" + prop + "' was found instead");
    }

    Name * property = makeName(prop, value, Name::Type::UnicodeProperty);

    auto theprop = propit->second;
    if (theprop == gc) {
        // General Category
        int valcode = GetPropertyValueEnumCode(gc, value);
        if (valcode < 0) {
            throw UnicodePropertyExpressionError("Erroneous property value for general_category property");
        }
        property->setFunctionName("__get_gc_" + GC_ns::enum_names[valcode]);
    }
    else if (theprop == sc) {
        // Script property identified
        int valcode = GetPropertyValueEnumCode(sc, value);
        if (valcode < 0) {
            throw UnicodePropertyExpressionError("Erroneous property value for script property");
        }
        property->setFunctionName("__get_sc_" + SC_ns::enum_names[valcode]);
    }
    else if (theprop == scx) {
        // Script extension property identified
        int valcode = GetPropertyValueEnumCode(sc, value);
        if (valcode < 0) {
            throw UnicodePropertyExpressionError("Erroneous property value for script_extension property");
        }
        property->setFunctionName("__get_scx_" + SC_ns::enum_names[valcode]);
    }
    else if (theprop == blk) {
        // Block property identified
        int valcode = GetPropertyValueEnumCode(blk, value);
        if (valcode < 0) {
             throw UnicodePropertyExpressionError("Erroneous property value for block property");
        }
        property->setFunctionName("__get_blk_" + BLK_ns::enum_names[valcode]);
    }
    else if (isa<BinaryPropertyObject>(property_object_table[theprop])){
        auto valit = Binary_ns::aliases_only_map.find(value);
        if (valit == Binary_ns::aliases_only_map.end()) {
            throw UnicodePropertyExpressionError("Erroneous property value for binary property " + property_full_name[theprop]);
        }
        if (valit->second == Binary_ns::Y) {
            property->setFunctionName("__get_" + lowercase(property_enum_name[theprop]) + "_Y");
        }
        else {
            Name * binprop = parser->createName("__get_" + lowercase(property_enum_name[theprop]) + "_Y");
            property->setDefinition(makeDiff(makeAny(), binprop));
        }
    }
    else {
        throw UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized but not supported in icgrep 1.0");
    }

    return property;
}

Name * resolveProperty(const std::string value, re::RE_Parser * parser) {

    // No namespace (property) name.

    Name * property = makeName(value, Name::Type::UnicodeProperty);

    // Try special cases of Unicode TR #18
    if (value == "any") {
        property->setDefinition(makeAny());
    }
    else if (value == "ascii") {
        property->setDefinition(parser->createName("blk", "ascii"));
    }
    else if (value == "assigned") {
        Name * unassigned = parser->createName("cn");
        property->setDefinition(makeDiff(makeAny(), unassigned));
    }
    // Now compatibility properties of UTR #18 Annex C
    else if (value == "xdigit") {
        Name * digit = parser->createName("nd");
        Name * hexdigit = parser->createName("hexdigit");
        property->setDefinition(makeAlt({digit, hexdigit}));
    }
    else if (value == "alnum") {
        Name * digit = parser->createName("nd");
        Name * alpha = parser->createName("alphabetic");
        property->setDefinition(makeAlt({digit, alpha}));
    }
    else if (value == "blank") {
        Name * space_sep = parser->createName("space_separator");
        CC * tab = makeCC(0x09);
        property->setDefinition(makeAlt({space_sep, tab}));
    }
    else if (value == "graph") {
        Name * space = parser->createName("space");
        Name * ctrl = parser->createName("control");
        Name * surr = parser->createName("surrogate");
        Name * unassigned = parser->createName("cn");
        property->setDefinition(makeDiff(makeAny(), makeAlt({space, ctrl, surr, unassigned})));
    }
    else if (value == "print") {
        Name * graph = parser->createName("graph");
        Name * space_sep = parser->createName("space_separator");
        property->setDefinition(makeAlt({graph, space_sep}));
    }
    else if (value == "word") {
        Name * alnum = parser->createName("alnum");
        Name * mark = parser->createName("mark");
        Name * conn = parser->createName("connectorpunctuation");
        Name * join = parser->createName("joincontrol");
        property->setDefinition(makeAlt({alnum, mark, conn, join}));
    }
    else { // Try as a general category, script or binary property.
        int valcode;
        if ((valcode = GetPropertyValueEnumCode(gc, value)) >= 0) {
            property->setFunctionName("__get_gc_" + GC_ns::enum_names[valcode]);
        }
        else if ((valcode = GetPropertyValueEnumCode(sc, value)) >= 0) {
            property->setFunctionName("__get_sc_" + SC_ns::enum_names[valcode]);
        }
        else { // Try as a binary property.
            auto propit = alias_map.find(value);
            if (propit != alias_map.end()) {
                auto theprop = propit->second;
                if (isa<BinaryPropertyObject>(property_object_table[theprop])) {
                    property->setFunctionName("__get_" + lowercase(property_enum_name[theprop]) + "_Y");
                }
                else {
                    throw UnicodePropertyExpressionError("Error: property " + property_full_name[theprop] + " specified without a value");
                }
            }
            else {
                throw UnicodePropertyExpressionError("Expected a general category, script or binary property name but '" + value + "' was found instead");
            }
        }
    }
    return property;
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

}
