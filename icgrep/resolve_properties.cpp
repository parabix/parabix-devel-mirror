/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <re/re_re.h>
#include <re/re_alt.h>
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


class UnicodePropertyExpressionError : public std::exception {
public:
    UnicodePropertyExpressionError(const std::string && msg) noexcept : _msg(msg) {};
    const char* what() const noexcept { return _msg.c_str();};
private:
    inline UnicodePropertyExpressionError() noexcept {}
    const std::string _msg;
};

std::string canonicalize(std::string prop_or_val) {
    std::locale loc;
    std::string s = "";
    for (unsigned int i = 0; i < prop_or_val.length(); ++i) {
        char c = prop_or_val.at(i);
        if ((c != '_') && (c != ' ') && (c != '-')) {
            s += std::tolower(c, loc);
        }
    }
    return s;
}

std::string lowercase(std::string prop_or_val) {
    std::locale loc;
    std::string s = "";
    for (unsigned int i = 0; i < prop_or_val.length(); ++i) {
        char c = prop_or_val.at(i);
        s += std::tolower(c, loc);
    }
    return s;
}

using namespace re;

void resolveProperties(RE * re) {
    if (Alt * alt = dyn_cast<Alt>(re)) {
        for (auto i = alt->begin(); i != alt->end(); ++i) {
            resolveProperties(*i);
        }
    }
    else if (Seq * seq = dyn_cast<Seq>(re)) {
        for (auto i = seq->begin(); i != seq->end(); ++i) {
            resolveProperties(*i);
        }
    }
    else if (Rep * rep = dyn_cast<Rep>(re)) {
        resolveProperties(rep->getRE());
    }
    else if (Assertion * a = dyn_cast<Assertion>(re)) {
        resolveProperties(a->getAsserted());
    }
    else if (Diff * diff = dyn_cast<Diff>(re)) {
        resolveProperties(diff->getRH());
        resolveProperties(diff->getLH());
    }
    else if (Intersect * e = dyn_cast<Intersect>(re)) {
        resolveProperties(e->getRH());
        resolveProperties(e->getLH());
    }
    else if (Name * name = dyn_cast<Name>(re)) {
        if (name->getType() == Name::Type::UnicodeProperty) {
            std::string prop = name->getNamespace();
            std::string v = canonicalize_value_name(name->getName());
            UCD::property_t theprop;
            if (prop != "") {
                prop = canonicalize_value_name(prop);
                auto propit = UCD::alias_map.find(prop);
                if (propit == UCD::alias_map.end()) {
                    throw UnicodePropertyExpressionError("Expected a property name, but '" + name->getNamespace() + "' found instead");
                }
                theprop = propit->second;
                if (theprop == UCD::gc) {
                    // General Category
                    int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::gc])->GetPropertyValueEnumCode(v);                    
                    if (valcode > 0) {
                        name->setName("__get_gc_" + UCD::GC_ns::enum_names[valcode]);
                    }
                    else throw UnicodePropertyExpressionError("Erroneous property value for general_category property");
                }
                else if (theprop == UCD::sc) {
                    // Script property identified
                    int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::sc])->GetPropertyValueEnumCode(v);                    
                    if (valcode > 0) {
                        name->setName("__get_sc_" + UCD::SC_ns::enum_names[valcode]);
                    }
                    else throw UnicodePropertyExpressionError("Erroneous property value for script property");
                }
                else if (theprop == UCD::scx) {
                    // Script extension property identified
                    int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::sc])->GetPropertyValueEnumCode(v);                    
                    if (valcode > 0) {
                        name->setName("__get_scx_" + UCD::SC_ns::enum_names[valcode]);
                    }
                    else throw UnicodePropertyExpressionError("Erroneous property value for script_extension property");
                }
                else if (theprop == UCD::blk) {
                    // Block property identified
                    int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::blk])->GetPropertyValueEnumCode(v);                    
                    if (valcode > 0) {
                        name->setName("__get_blk_" + UCD::BLK_ns::enum_names[valcode]);
                    }
                    else throw UnicodePropertyExpressionError("Erroneous property value for block property");
                }
                else if (UCD::property_object_table[theprop]->the_kind == UCD::BinaryProperty){
                    auto valit = UCD::Binary_ns::aliases_only_map.find(v);
                    if (valit == UCD::Binary_ns::aliases_only_map.end()) {
                        throw UnicodePropertyExpressionError("Erroneous property value for binary property " + UCD::property_full_name[theprop]);
                    }
                    if (valit->second == UCD::Binary_ns::Y) {
                        name->setName("__get_" + lowercase(UCD::property_enum_name[theprop]) + "_Y");
                        return;
                    }
                    else {
                        re::Name * binprop = re::makeName("__get_" + lowercase(UCD::property_enum_name[theprop]) + "_Y", Name::Type::UnicodeProperty);
                        name->setDefinition(re::makeDiff(re::makeAny(), binprop));
                        return;
                    }
                }
                else {
                    throw UnicodePropertyExpressionError("Property " + UCD::property_full_name[theprop] + " recognized, but not supported in icgrep 1.0");
                }
            }
            else {
                // No namespace (property) name.   Try as a general category.
                int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::gc])->GetPropertyValueEnumCode(v);
                if (valcode > 0) {
                    theprop = UCD::gc;
                    name->setName("__get_gc_" + UCD::GC_ns::enum_names[valcode]);
                    return;
                }
                valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::sc])->GetPropertyValueEnumCode(v);
                if (valcode > 0) {
                    theprop = UCD::sc;
                    name->setName("__get_sc_" + UCD::SC_ns::enum_names[valcode]);
                    return;
                }
                // Try as a binary property.
                auto propit = UCD::alias_map.find(v);
                if (propit != UCD::alias_map.end()) {
                    theprop = propit->second;
                    if (UCD::property_object_table[theprop]->the_kind == UCD::BinaryProperty) {
                        name->setName("__get_" + lowercase(UCD::property_enum_name[theprop]) + "_Y");
                        return;
                    }
                    else {
                        throw UnicodePropertyExpressionError("Error: property " + UCD::property_full_name[theprop] + " specified without a value");
                    }
                }
                // Now try special cases of Unicode TR #18
                else if (v == "any") {
                    name->setDefinition(re::makeAny());
                    return;
                }
                else if (v == "assigned") {
                    re::Name * Cn = re::makeName("Cn", Name::Type::UnicodeProperty);
                    resolveProperties(Cn);
                    name->setDefinition(re::makeDiff(re::makeAny(), Cn));
                    return;
                }
                else if (v == "ascii") {
                    name->setName("__get_blk_ASCII");
                    return;
                }
                // Now compatibility properties of UTR #18 Annex C
                else if (v == "xdigit") {
                    re::Name * Nd = re::makeName("Nd", Name::Type::UnicodeProperty);
                    resolveProperties(Nd);
                    re::Name * hexdigit = re::makeName("Hex_digit", Name::Type::UnicodeProperty);
                    resolveProperties(hexdigit);
                    std::vector<RE *> alts = {Nd, hexdigit};
                    name->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    return;
                }
                else if (v == "alnum") {
                    re::Name * digit = re::makeName("Nd", Name::Type::UnicodeProperty);
                    resolveProperties(digit);
                    re::Name * alpha = re::makeName("alphabetic", Name::Type::UnicodeProperty);
                    resolveProperties(alpha);
                    std::vector<RE *> alts = {digit, alpha};
                    name->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    return;
                }
                else if (v == "blank") {
                    re::Name * space_sep = re::makeName("space_separator", Name::Type::UnicodeProperty);
                    resolveProperties(space_sep);
                    re::CC * tab = re::makeCC(0x09);
                    std::vector<RE *> alts = {space_sep, tab};
                    name->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    return;
                }
                else if (v == "graph") {
                    re::Name * space = re::makeName("space", Name::Type::UnicodeProperty);
                    resolveProperties(space);
                    re::Name * ctrl = re::makeName("control", Name::Type::UnicodeProperty);
                    resolveProperties(ctrl);
                    re::Name * surr = re::makeName("surrogate", Name::Type::UnicodeProperty);
                    resolveProperties(surr);
                    re::Name * unassigned = re::makeName("Cn", Name::Type::UnicodeProperty);
                    resolveProperties(unassigned);
                    std::vector<RE *> alts = {space, ctrl, surr, unassigned};
                    re::Name * nongraph = re::makeName("[^graph]", Name::Type::UnicodeProperty);
                    nongraph->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    name->setDefinition(re::makeDiff(re::makeAny(), nongraph));
                    return;
                }
                else if (v == "print") {
                    re::Name * graph = re::makeName("graph", Name::Type::UnicodeProperty);
                    resolveProperties(graph);
                    re::Name * space_sep = re::makeName("space_separator", Name::Type::UnicodeProperty);
                    resolveProperties(space_sep);
                    std::vector<RE *> alts = {graph, space_sep};
                    name->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    return;
                }
                else if (v == "word") {
                    re::Name * alnum = re::makeName("alnum", Name::Type::UnicodeProperty);
                    resolveProperties(alnum);
                    re::Name * mark = re::makeName("mark", Name::Type::UnicodeProperty);
                    resolveProperties(mark);
                    re::Name * conn = re::makeName("Connector_Punctuation", Name::Type::UnicodeProperty);
                    resolveProperties(conn);
                    re::Name * join = re::makeName("Join_Control", Name::Type::UnicodeProperty);
                    resolveProperties(join);
                    std::vector<RE *> alts = {alnum,mark,conn,join};
                    name->setDefinition(re::makeAlt(alts.begin(), alts.end()));
                    return;
                }
                else {
                    throw UnicodePropertyExpressionError("Expected a general category, script or binary property name, but '" + name->getName() + "' found instead");
                }
            }
            
	        //name->setCompiled(compileCC(cast<CC>(d), mCG));
        }
    }
    else if (!isa<CC>(re) && !isa<Start>(re) && !isa<End>(re) && !isa<Any>(re)) {
        throw UnicodePropertyExpressionError("Unknown RE type in resolveProperties.");
    }
}

