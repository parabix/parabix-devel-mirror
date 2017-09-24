/*
 *  Copyright (c) 2014 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */

#include "PropertyObjects.h"
#include "PropertyObjectTable.h"
#include <llvm/Support/Casting.h>
#include <algorithm>
#include <assert.h>
#include <sstream>

using namespace llvm;

namespace UCD {

std::string canonicalize_value_name(const std::string & prop_or_val) {
    std::locale loc;
    std::stringstream s;

    for (char c : prop_or_val) {
        if ((c != '_') && (c != ' ') && (c != '-')) {
            s << std::tolower(c, loc);
        }
    }
    return s.str();
}

int PropertyObject::GetPropertyValueEnumCode(const std::string & value_spec) {
    throw std::runtime_error("Property " + value_spec + " unsupported.");
}
const std::string & PropertyObject::GetPropertyValueGrepString() {
    throw std::runtime_error("Property Value Grep String unsupported.");
}

UnicodeSet UnsupportedPropertyObject::GetCodepointSet(const std::string &) {
    throw std::runtime_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
}

UnicodeSet UnsupportedPropertyObject::GetCodepointSet(const int) {
    throw std::runtime_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
}

const UnicodeSet & EnumeratedPropertyObject::GetCodepointSet(const std::string & value_spec) {
    const int property_enum_val = GetPropertyValueEnumCode(value_spec);
    if (property_enum_val < 0) {
        throw std::runtime_error("Enumerated Property " + UCD::property_full_name[the_property] + ": unknown value: " + value_spec);
    }
    return GetCodepointSet(property_enum_val);
}

const UnicodeSet & EnumeratedPropertyObject::GetCodepointSet(const int property_enum_val) const {
    assert (property_enum_val >= 0);
    return *(property_value_sets[property_enum_val]);
}

std::vector<UnicodeSet> & EnumeratedPropertyObject::GetEnumerationBasisSets() {
    // Return the previously computed vector of basis sets, if it exists.
    if (LLVM_UNLIKELY(enumeration_basis_sets.empty())) {
        // Otherwise compute and return.
        // Basis set i is the set of all codepoints whose numerical enumeration code e
        // has bit i set, i.e., (e >> i) & 1 == 1.
        unsigned basis_count = 1;
        while ((1UL << basis_count) < independent_enum_count) {
            basis_count++;
        }
        for (unsigned i = 0; i < basis_count; i++) {
            enumeration_basis_sets.push_back(UnicodeSet());
            for (unsigned e = 0; e < independent_enum_count; e++) {
                if (((e >> i) & 1UL) == 0) {
                    enumeration_basis_sets[i] = enumeration_basis_sets[i] + *property_value_sets[e];
                }
            }
        }
    }
    return enumeration_basis_sets;
}

const std::string & EnumeratedPropertyObject::GetPropertyValueGrepString() {
    if (LLVM_LIKELY(mPropertyValueGrepString.empty())) {
        std::stringstream buffer;
        for (unsigned i = 0; i != property_value_full_names.size(); i++) {
            buffer << property_value_full_names[i] + "\n";
        }
        for (unsigned i = 0; i != property_value_enum_names.size(); i++) {
            if (property_value_enum_names[i] == property_value_full_names[i]) continue;
            buffer << property_value_enum_names[i] + "\n";
        }
        for (auto & a : property_value_aliases) {
            buffer << a.first + "\n";
        }
        mPropertyValueGrepString = buffer.str();
    }
    return mPropertyValueGrepString;
}

int EnumeratedPropertyObject::GetPropertyValueEnumCode(const std::string & value_spec) {
    // The canonical full names are not stored in the precomputed alias map,
    // to save space in the executable.   Add them if the property is used.
    if (uninitialized) {
        for (unsigned i = 0; i != property_value_full_names.size(); i++) {
            property_value_aliases.insert({canonicalize_value_name(property_value_full_names[i]), i});
        }
        for (unsigned i = 0; i != property_value_enum_names.size(); i++) {
            property_value_aliases.insert({canonicalize_value_name(property_value_enum_names[i]), i});
        }
        uninitialized = false;
    }
    const auto valit = property_value_aliases.find(value_spec);
    if (valit == property_value_aliases.end())
        return -1;
    return valit->second;
}

PropertyObject::iterator ExtensionPropertyObject::begin() const {
    if (const auto * obj = dyn_cast<EnumeratedPropertyObject>(property_object_table[base_property])) {
        return obj->begin();
    }
    throw std::runtime_error("Iterators unsupported for this type of PropertyObject.");
}

PropertyObject::iterator ExtensionPropertyObject::end() const {
    if (const auto * obj = dyn_cast<EnumeratedPropertyObject>(property_object_table[base_property])) {
        return obj->end();
    }
    throw std::runtime_error("Iterators unsupported for this type of PropertyObject.");
}

const UnicodeSet & ExtensionPropertyObject::GetCodepointSet(const std::string & value_spec) {
    int property_enum_val = GetPropertyValueEnumCode(value_spec);
    if (property_enum_val == -1) {
        throw std::runtime_error("Extension Property " + UCD::property_full_name[the_property] +  ": unknown value: " + value_spec);
    }
    return GetCodepointSet(property_enum_val);
}

const UnicodeSet & ExtensionPropertyObject::GetCodepointSet(const int property_enum_val) const {
    assert (property_enum_val >= 0);
    return *(property_value_sets[property_enum_val]);
}

int ExtensionPropertyObject::GetPropertyValueEnumCode(const std::string & value_spec) {
    return property_object_table[base_property]->GetPropertyValueEnumCode(value_spec);
}

const std::string & ExtensionPropertyObject::GetPropertyValueGrepString() {
    return property_object_table[base_property]->GetPropertyValueGrepString();
}

const UnicodeSet & BinaryPropertyObject::GetCodepointSet(const std::string & value_spec) {
    int property_enum_val = Binary_ns::Y;
    if (value_spec.length() != 0) {
        auto valit = Binary_ns::aliases_only_map.find(value_spec);
        if (valit == Binary_ns::aliases_only_map.end()) {
            throw std::runtime_error("Binary Property " + UCD::property_full_name[the_property] +  ": bad value: " + value_spec);
        }
        property_enum_val = valit->second;
    }
    return GetCodepointSet(property_enum_val);
}

const UnicodeSet & BinaryPropertyObject::GetCodepointSet(const int property_enum_val) {
    if (property_enum_val == Binary_ns::Y) {
        return mY;
    }
    if (mNoUninitialized) {
        mN = ~mY;
        mNoUninitialized = false;
    }
    return mN;
}

const std::string & BinaryPropertyObject::GetPropertyValueGrepString() {
    if (mPropertyValueGrepString.empty()) {
        std::stringstream buffer;
        for (const auto & prop : Binary_ns::aliases_only_map) {
            buffer << std::get<0>(prop) + "\n";
        }
        mPropertyValueGrepString = buffer.str();
    }
    return mPropertyValueGrepString;
}

}
