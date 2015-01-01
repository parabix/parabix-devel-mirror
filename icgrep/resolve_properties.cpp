/*
 *  Copyright (c) 2014 International Characters.
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
#include <re/re_start.h>
#include <re/re_end.h>

#include <cc/cc_namemap.hpp>
#include "UCD/PropertyAliases.h"
#include "UCD/PropertyObjects.h"
#include "UCD/PropertyObjectTable.h"
#include "UCD/PropertyValueAliases.h"



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
                    throw std::runtime_error("Unknown property value: " + prop);
                }
                theprop = propit->second;
                if (theprop == UCD::gc) {
                    // General Category
                    
                    int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::gc])->GetPropertyValueEnumCode(v);
                    
                    if (valcode > 0) {
                        name->setName("__get_gc_" + UCD::GC_ns::enum_names[valcode]);
                    }
                }
                else if (theprop == UCD::sc) {
                    // Script property identified
                    throw std::runtime_error("Script property identified, aborting\n");
                }
                else if (theprop == UCD::scx) {
                    // Script extension property identified
                    throw std::runtime_error("Script extensions property identified, aborting\n");
                }
                else {
                    throw std::runtime_error("other property identified, aborting\n");
                }
            }
            else {
                // No namespace (property) name.   Try as a general category.
                int valcode = dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::gc])->GetPropertyValueEnumCode(v);
                if (valcode > 0) {
                    theprop = UCD::gc;
                    name->setName("__get_gc_" + UCD::GC_ns::enum_names[valcode]);
                }
                else if (dynamic_cast<UCD::EnumeratedPropertyObject *> (UCD::property_object_table[UCD::sc])->GetPropertyValueEnumCode(v) > 0) {
                    theprop = UCD::sc;
                    throw std::runtime_error("Script property identified by value, aborting\n");
                }

                else {
                    throw std::runtime_error("Unknown property, aborting\n");
                }
            }
            
	        //name->setCompiled(compileCC(cast<CC>(d), mCG));
        }
    }
    else if (!isa<CC>(re) && !isa<Start>(re) && !isa<End>(re) && !isa<Any>(re)) {
        throw std::runtime_error("Unknown RE type in resolveProperties.");
    }
}

