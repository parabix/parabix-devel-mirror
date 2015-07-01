/*
 *  Copyright (c) 2014 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */

#include "PropertyObjects.h"
#include <sstream>

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

UnicodeSet UnsupportedPropertyObject::GetCodepointSet(const std::string &) {
    throw std::runtime_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
}

UnicodeSet UnsupportedPropertyObject::GetCodepointSet(const int) {
    throw std::runtime_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
}

UnicodeSet EnumeratedPropertyObject::GetCodepointSet(const std::string & value_spec) {
    int property_enum_val = GetPropertyValueEnumCode(value_spec);
    if (property_enum_val == -1) {
        throw std::runtime_error("Enumerated Property " + UCD::property_full_name[the_property] +  ": unknown value: " + value_spec);
    }
    return GetCodepointSet(property_enum_val);
}

UnicodeSet EnumeratedPropertyObject::GetCodepointSet(const int property_enum_val) const {
    assert (property_enum_val >= 0);
    return property_value_sets[property_enum_val];
}

int EnumeratedPropertyObject::GetPropertyValueEnumCode(const std::string & value_spec) {
    // The canonical full names are not stored in the precomputed alias map,
    // to save space in the executable.   Add them if the property is used.
    if (!aliases_initialized) {
        for (unsigned i = 0; i != property_value_full_names.size(); i++) {
            property_value_aliases.insert({canonicalize_value_name(property_value_full_names[i]), i});
        }
        for (unsigned i = 0; i != property_value_enum_names.size(); i++) {
            property_value_aliases.insert({canonicalize_value_name(property_value_enum_names[i]), i});
        }
        aliases_initialized = true;
    }
    const auto valit = property_value_aliases.find(value_spec);
    if (valit == property_value_aliases.end())
        return -1;
    return valit->second;
}

UnicodeSet BinaryPropertyObject::GetCodepointSet(const std::string & value_spec) const {
    if (value_spec.length() != 0) {
        auto valit = Binary_ns::aliases_only_map.find(value_spec);
        if (valit == Binary_ns::aliases_only_map.end()) {
            throw std::runtime_error("Binary Property " + UCD::property_full_name[the_property] +  ": bad value: " + value_spec);
        }
        if (valit->second == Binary_ns::Y)
            return the_codepoint_set;
        return ~the_codepoint_set;
    }
    return the_codepoint_set;
}

UnicodeSet BinaryPropertyObject::GetCodepointSet(const int property_enum_val) const {
    if (property_enum_val == Binary_ns::Y)
        return the_codepoint_set;
    return ~the_codepoint_set;
}

}
