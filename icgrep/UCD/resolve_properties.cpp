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
#include <re/re_start.h> 
#include <re/re_end.h> 
#include <re/re_cc.h> 
#include <re/re_seq.h> 
#include <re/re_assertion.h>
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
        } else if (value == "GCB" || value == "NonGCB"){
            generateGraphemeClusterBoundaryRule(property);
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
            auto theprop = propit->second;
            return property_object_table[theprop]->GetCodepointSet(value);
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
