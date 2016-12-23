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
#include <re/re_any.h>
#include <re/re_start.h> 
#include <re/re_end.h> 
#include <re/re_cc.h> 
#include <re/re_seq.h> 
#include <re/re_rep.h> 
#include <re/re_intersect.h> 
#include <re/re_assertion.h>
#include "UCD/PropertyAliases.h"
#include "UCD/PropertyObjects.h"
#include "UCD/PropertyObjectTable.h"
#include "UCD/PropertyValueAliases.h"
#include <string>
#include <iostream>

using namespace UCD;
using namespace re;


inline int GetPropertyValueEnumCode(const UCD::property_t type, const std::string & value) {
    return property_object_table[type]->GetPropertyValueEnumCode(value);
}

namespace UCD {

void generateGraphemeClusterBoundaryRule(Name * const &property) {
    // 3.1.1 Grapheme Cluster Boundary Rules
#define Behind(x) makeLookBehindAssertion(x)
#define Ahead(x) makeLookAheadAssertion(x)

//    RE * GCB_Control = makeName("gcb", "cn", Name::Type::UnicodeProperty);
    RE * GCB_CR = makeName("gcb", "cr", Name::Type::UnicodeProperty);
    RE * GCB_LF = makeName("gcb", "lf", Name::Type::UnicodeProperty);
    RE * GCB_Control_CR_LF = makeAlt({GCB_CR, GCB_LF});

    // Break at the start and end of text.
    RE * GCB_1 = makeStart();
    RE * GCB_2 = makeEnd();
    // Do not break between a CR and LF.
    RE * GCB_3 = makeSeq({Behind(GCB_CR), Ahead(GCB_LF)});
    // Otherwise, break before and after controls.
    RE * GCB_4 = Behind(GCB_Control_CR_LF);
    RE * GCB_5 = Ahead(GCB_Control_CR_LF);
    RE * GCB_1_5 = makeAlt({GCB_1, GCB_2, makeDiff(makeAlt({GCB_4, GCB_5}), GCB_3)});

    RE * GCB_L = makeName("gcb", "l", Name::Type::UnicodeProperty);
    RE * GCB_V = makeName("gcb", "v", Name::Type::UnicodeProperty);
    RE * GCB_LV = makeName("gcb", "lv", Name::Type::UnicodeProperty);
    RE * GCB_LVT = makeName("gcb", "lvt", Name::Type::UnicodeProperty);
    RE * GCB_T = makeName("gcb", "t", Name::Type::UnicodeProperty);
    RE * GCB_RI = makeName("gcb", "ri", Name::Type::UnicodeProperty);
    // Do not break Hangul syllable sequences.
    RE * GCB_6 = makeSeq({Behind(GCB_L), Ahead(makeAlt({GCB_L, GCB_V, GCB_LV, GCB_LVT}))});
    RE * GCB_7 = makeSeq({Behind(makeAlt({GCB_LV, GCB_V})), Ahead(makeAlt({GCB_V, GCB_T}))});
    RE * GCB_8 = makeSeq({Behind(makeAlt({GCB_LVT, GCB_T})), Ahead(GCB_T)});
    // Do not break between regional indicator symbols.
    RE * GCB_8a = makeSeq({Behind(GCB_RI), Ahead(GCB_RI)});
    // Do not break before extending characters.
    RE * GCB_9 = Ahead(makeName("gcb", "ex", Name::Type::UnicodeProperty));
    // Do not break before SpacingMarks, or after Prepend characters.
    RE * GCB_9a = Ahead(makeName("gcb", "sm", Name::Type::UnicodeProperty));
    RE * GCB_9b = Behind(makeName("gcb", "pp", Name::Type::UnicodeProperty));
    RE * GCB_6_9b = makeAlt({GCB_6, GCB_7, GCB_8, GCB_8a, GCB_9, GCB_9a, GCB_9b});
    // Otherwise, break everywhere.
    RE * GCB_10 = makeSeq({Behind(makeAny()), Ahead(makeAny())});

    //Name * gcb = makeName("gcb", Name::Type::UnicodeProperty);
    property->setDefinition(makeAlt({GCB_1_5, makeDiff(GCB_10, GCB_6_9b)}));
}

bool resolvePropertyDefinition(Name * const property) {
    if (property->hasNamespace()) {
        auto propit = alias_map.find(property->getNamespace());
        if (propit == alias_map.end()) {
            throw UnicodePropertyExpressionError("Expected a property name but '" + property->getNamespace() + "' was found instead");
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
        if (value == "any") {
            property->setDefinition(makeAny());
            return true;
        } else if (value == "ascii") {
            property->setDefinition(makeName("blk", "ascii", Name::Type::UnicodeProperty));
            return true;
        } else if (value == "assigned") {
            Name * unassigned = makeName("cn", Name::Type::UnicodeProperty);
            property->setDefinition(makeDiff(makeAny(), unassigned));
            return true;
        }
        // Now compatibility properties of UTR #18 Annex C
        else if (value == "xdigit") {
            Name * digit = makeName("nd", Name::Type::UnicodeProperty);
            Name * hexdigit = makeName("hexdigit", Name::Type::UnicodeProperty);
            property->setDefinition(makeAlt({digit, hexdigit}));
            return true;
        } else if (value == "alnum") {
            Name * digit = makeName("nd", Name::Type::UnicodeProperty);
            Name * alpha = makeName("alphabetic", Name::Type::UnicodeProperty);
            property->setDefinition(makeAlt({digit, alpha}));
            return true;
        } else if (value == "blank") {
            Name * space_sep = makeName("space_separator", Name::Type::UnicodeProperty);
            CC * tab = makeCC(0x09);
            property->setDefinition(makeAlt({space_sep, tab}));
            return true;
        } else if (value == "graph") {
            Name * space = makeName("space", Name::Type::UnicodeProperty);
            Name * ctrl = makeName("control", Name::Type::UnicodeProperty);
            Name * surr = makeName("surrogate", Name::Type::UnicodeProperty);
            Name * unassigned = makeName("cn", Name::Type::UnicodeProperty);
            property->setDefinition(makeDiff(makeAny(), makeAlt({space, ctrl, surr, unassigned})));
            return true;
        } else if (value == "print") {
            Name * graph = makeName("graph", Name::Type::UnicodeProperty);
            Name * space_sep = makeName("space_separator", Name::Type::UnicodeProperty);
            property->setDefinition(makeAlt({graph, space_sep}));
            return true;
        } else if (value == "word") {
            Name * alnum = makeName("alnum", Name::Type::UnicodeProperty);
            Name * mark = makeName("mark", Name::Type::UnicodeProperty);
            Name * conn = makeName("connectorpunctuation", Name::Type::UnicodeProperty);
            Name * join = makeName("joincontrol", Name::Type::UnicodeProperty);
            property->setDefinition(makeAlt({alnum, mark, conn, join}));
            return true;
        } else if (value == "GCB" || value == "NonGCB"){
            generateGraphemeClusterBoundaryRule(property);
            return true;
        }
    }
    return false;
}

std::string resolvePropertyFunction(Name * const property) {
    const std::string value = property->getName();
    std::string functionName;
    if (property->hasNamespace()) {
        auto propit = alias_map.find(property->getNamespace());
        if (propit == alias_map.end()) {
            throw UnicodePropertyExpressionError("Expected a property name but '" + property->getNamespace() + "' was found instead");
        }
        auto theprop = propit->second;
        if (EnumeratedPropertyObject * p = dyn_cast<EnumeratedPropertyObject>(property_object_table[theprop])){
            int valcode = p->GetPropertyValueEnumCode(value);
            if (valcode < 0) {
                throw UnicodePropertyExpressionError("Erroneous property value '" + value + "' for " + property_full_name[theprop] + " property");
            }
            functionName = "__get_" + property_enum_name[theprop] + "_" + p->GetValueEnumName(valcode);
        }
        else if (theprop == scx) {
            // Script extension property identified
            int valcode = GetPropertyValueEnumCode(sc, value);
            if (valcode < 0) {
                throw UnicodePropertyExpressionError("Erroneous property value for script_extension property");
            }
            functionName = "__get_scx_" + SC_ns::enum_names[valcode];
        }
        else if (isa<BinaryPropertyObject>(property_object_table[theprop])){
            auto valit = Binary_ns::aliases_only_map.find(value);
            if (valit == Binary_ns::aliases_only_map.end()) {
                throw UnicodePropertyExpressionError("Erroneous property value for binary property " + property_full_name[theprop]);
            }
            if (valit->second == Binary_ns::Y) {
                functionName = "__get_" + property_enum_name[theprop] + "_Y";
            } else {
                throw UnicodePropertyExpressionError("Unexpected property value for binary property " + property_full_name[theprop]);
            }
        }
        else {
            throw UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized but not supported in icgrep 1.0");
        }
    } else { // No namespace (property) name.
        // Try as a general category, script or binary property.
        int valcode;
        if ((valcode = GetPropertyValueEnumCode(gc, value)) >= 0) {
            functionName = "__get_gc_" + GC_ns::enum_names[valcode];
        }
        else if ((valcode = GetPropertyValueEnumCode(sc, value)) >= 0) {
            functionName = "__get_sc_" + SC_ns::enum_names[valcode];
        }
        else { // Try as a binary property.
            auto propit = alias_map.find(value);
            if (propit != alias_map.end()) {
                auto theprop = propit->second;
                if (isa<BinaryPropertyObject>(property_object_table[theprop])) {
                    functionName = "__get_" + property_enum_name[theprop] + "_Y";
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
    assert (functionName.length() > 0);
    return functionName;
}

const std::string & getPropertyValueGrepString(const std::string & prop) {
    auto propit = alias_map.find(canonicalize_value_name(prop));
    if (propit == alias_map.end()) {
        throw UnicodePropertyExpressionError("Expected a property name, but '" + prop + "' found instead");
    }
    auto theprop = propit->second;
    if (EnumeratedPropertyObject * p = dyn_cast<EnumeratedPropertyObject>(property_object_table[theprop])){
        return p->GetPropertyValueGrepString();
    } else if (BinaryPropertyObject * p = dyn_cast<BinaryPropertyObject>(property_object_table[theprop])) {
        return p->GetPropertyValueGrepString();
    }

    throw UnicodePropertyExpressionError("Property " + property_full_name[theprop] + " recognized but not supported in icgrep 1.0");
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
            if (EnumeratedPropertyObject * p = dyn_cast<EnumeratedPropertyObject>(property_object_table[theprop])){
                return p->GetCodepointSet(value);
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
