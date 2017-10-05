/*
 *  Copyright (c) 2017 International Characters, Inc.
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
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/ErrorHandling.h>
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

const std::string & PropertyObject::GetPropertyValueGrepString() {
    llvm::report_fatal_error("Property Value Grep String unsupported.");
}

const UnicodeSet PropertyObject::GetCodepointSet(const std::string &) {
    llvm::report_fatal_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
}

const UnicodeSet EnumeratedPropertyObject::GetCodepointSet(const std::string & value_spec) {
    const int property_enum_val = GetPropertyValueEnumCode(value_spec);
    if (property_enum_val < 0) {
        llvm::report_fatal_error("Enumerated Property " + UCD::property_full_name[the_property] + ": unknown value: " + value_spec);
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
    const auto valit = property_value_aliases.find(canonicalize_value_name(value_spec));
    if (valit == property_value_aliases.end())
        return -1;
    return valit->second;
}

PropertyObject::iterator ExtensionPropertyObject::begin() const {
    if (const auto * obj = dyn_cast<EnumeratedPropertyObject>(property_object_table[base_property])) {
        return obj->begin();
    }
    llvm::report_fatal_error("Iterators unsupported for this type of PropertyObject.");
}

PropertyObject::iterator ExtensionPropertyObject::end() const {
    if (const auto * obj = dyn_cast<EnumeratedPropertyObject>(property_object_table[base_property])) {
        return obj->end();
    }
    llvm::report_fatal_error("Iterators unsupported for this type of PropertyObject.");
}

const UnicodeSet ExtensionPropertyObject::GetCodepointSet(const std::string & value_spec) {
    int property_enum_val = GetPropertyValueEnumCode(value_spec);
    if (property_enum_val == -1) {
        llvm::report_fatal_error("Extension Property " + UCD::property_full_name[the_property] +  ": unknown value: " + value_spec);
    }
    return GetCodepointSet(property_enum_val);
}

const UnicodeSet & ExtensionPropertyObject::GetCodepointSet(const int property_enum_val) const {
    assert (property_enum_val >= 0);
    return *(property_value_sets[property_enum_val]);
}

int ExtensionPropertyObject::GetPropertyValueEnumCode(const std::string & value_spec) {
    return cast<EnumeratedPropertyObject>(property_object_table[base_property])->GetPropertyValueEnumCode(value_spec);
}

const std::string & ExtensionPropertyObject::GetPropertyValueGrepString() {
    return property_object_table[base_property]->GetPropertyValueGrepString();
}

const UnicodeSet BinaryPropertyObject::GetCodepointSet(const std::string & value_spec) {
    int property_enum_val = Binary_ns::Y;
    if (value_spec.length() != 0) {
        auto valit = Binary_ns::aliases_only_map.find(canonicalize_value_name(value_spec));
        if (valit == Binary_ns::aliases_only_map.end()) {
            llvm::report_fatal_error("Binary Property " + UCD::property_full_name[the_property] +  ": bad value: " + value_spec);
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
    
const unsigned firstCodepointLengthAndVal(const std::string & s, codepoint_t & cp) {
    size_t lgth = s.length();
    if (lgth == 0) return 0;
    unsigned char s0 = s[0];
    cp = static_cast<codepoint_t>(s0);
    if (s0 < 0x80) return 1;
    if (lgth == 1) return 0;  // invalid UTF-8
    cp = ((cp & 0x1F) << 6) | (s[1] & 0x3F);
    if ((s0 >= 0xC2) && (s0 <= 0xDF)) return 2;
    if (lgth == 2) return 0;  // invalid UTF-8
    cp = ((cp & 0x3FFF) << 6) | (s[2] & 0x3F);
    if ((s0 >= 0xE0) && (s0 <= 0xEF)) return 3;
    if (lgth == 3) return 0;  // invalid UTF-8
    cp = ((cp & 0x7FFF) << 6) | (s[3] & 0x3F);
    if ((s0 >= 0xF0) && (s0 <= 0xF4)) return 4;
    return 0;
}
    
const UnicodeSet NumericPropertyObject::GetCodepointSet(const std::string & value_spec) {
    if (value_spec == "NaN") return mNaNCodepointSet;
    else {
        UnicodeSet result_set;
        unsigned val_bytes = value_spec.length();
        const char * value_str = value_spec.c_str();
        const char * search_str = mStringBuffer;
        unsigned buffer_line = 0;
        while (buffer_line < mExplicitCps.size()) {
            const char * eol = strchr(search_str, '\n');
            unsigned len = eol - search_str;
            if ((len == val_bytes) && (memcmp(search_str, value_str, len) == 0)) {
                result_set.insert(mExplicitCps[buffer_line]);
            }
            buffer_line++;
            search_str = eol+1;
        }
        return result_set;
    }
}

const UnicodeSet StringPropertyObject::GetCodepointSet(const std::string & value_spec) {
    if (value_spec == "") return mNullCodepointSet;
    else {
        UnicodeSet result_set;
        unsigned val_bytes = value_spec.length();
        codepoint_t cp;
        if (val_bytes == firstCodepointLengthAndVal(value_spec, cp)) {
            if (mSelfCodepointSet.contains(cp)) {
                result_set.insert(cp);
            }
        }
        const char * value_str = value_spec.c_str();
        const char * search_str = mStringBuffer;
        unsigned buffer_line = 0;
        while (buffer_line < mExplicitCps.size()) {
            const char * eol = strchr(search_str, '\n');
            unsigned len = eol - search_str;
            if ((len == val_bytes) && (memcmp(search_str, value_str, len) == 0)) {
                result_set.insert(mExplicitCps[buffer_line]);
            }
            buffer_line++;
            search_str = eol+1;
        }
        return result_set;
    }
}

const std::string & ObsoletePropertyObject::GetPropertyValueGrepString() {
    llvm::report_fatal_error("Property " + UCD::property_full_name[the_property] + " is obsolete.");
}

const UnicodeSet ObsoletePropertyObject::GetCodepointSet(const std::string &) {
    llvm::report_fatal_error("Property " + UCD::property_full_name[the_property] + " is obsolete.");
}


}
