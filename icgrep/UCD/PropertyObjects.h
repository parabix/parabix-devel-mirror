#ifndef PROPERTYOBJECTS_H
#define PROPERTYOBJECTS_H
/*
 *  Copyright (c) 2014 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */

#include <string>
#include <vector>
#include <unordered_map>
#include "unicode_set.h"
#include "PropertyAliases.h"
#include "PropertyValueAliases.h"

std::string canonicalize_value_name(std::string prop_or_val) {
	std::locale loc;
	std::string s = "";
	for (unsigned int i = 0; i < prop_or_val.length(); ++i) {
		char c = prop_or_val.at(i);
		if ((c != '_') && (c != '.') && (c != '-')) {
			s += std::tolower(c, loc);
		}
	}
	return s;
}


namespace UCD {
	enum property_kind_t {
		NumericProperty, CodepointProperty, StringProperty, MiscellaneousProperty, EnumeratedProperty, CatalogProperty, BinaryProperty
	};
	
	class PropertyObject {
	public:
		PropertyObject(property_t p, property_kind_t k) : the_property(p), the_kind(k) {}
		
		property_t the_property;
		property_kind_t the_kind;
		
		virtual UnicodeSet GetCodepointSet(std::string value_spec) = 0;	
	};
	
	class UnsupportedPropertyObject : public PropertyObject {
	public:
		UnsupportedPropertyObject(property_t p, property_kind_t k) : PropertyObject(p, k) {}
		UnicodeSet GetCodepointSet(std::string value_spec);	
	};
	
	UnicodeSet UnsupportedPropertyObject::GetCodepointSet(std::string value_spec) {
		std::cerr << "Property " << UCD::property_full_name[the_property] << " unsupported.\n";
		exit(-1);
	}
	
	class EnumeratedPropertyObject : public PropertyObject {
	public:
		
		EnumeratedPropertyObject(UCD::property_t p, 
                                         const std::vector<std::string> names, 
                                         const std::unordered_map<std::string, int> aliases,
                                         const std::vector<UnicodeSet> sets) : 
		PropertyObject(p, EnumeratedProperty), property_value_full_names(names), property_value_aliases(aliases), aliases_initialized(false), property_value_sets(sets) {}
		int GetPropertyValueEnumCode(std::string s);
		UnicodeSet GetCodepointSet(std::string value_spec);	
		
	private:
		const std::vector<std::string> property_value_full_names;  // never changes
		std::unordered_map<std::string, int> property_value_aliases; 
		bool aliases_initialized; // full names must be added dynamically.
		std::vector<UnicodeSet> property_value_sets;                 
	};
	
	UnicodeSet EnumeratedPropertyObject::GetCodepointSet(std::string value_spec) {
		int property_enum_val = GetPropertyValueEnumCode(value_spec);
		if (property_enum_val == -1) {
			std::cerr << "Enumerated Property " << UCD::property_full_name[the_property] << ": unknown value: " << value_spec << ".\n";
			exit(-1);
		}
		else {
			std::cout << "Enumerated Property: " << UCD::property_full_name[the_property] << ", value: " << property_value_full_names[property_enum_val] << "(" << property_enum_val << ").\n";
			return property_value_sets[property_enum_val];
		}
	}
	
	int EnumeratedPropertyObject::GetPropertyValueEnumCode(std::string s) {
		// The canonical full names are not stored in the precomputed alias map,
		// to save space in the executable.   Add them if the property is used.
		if (!aliases_initialized) {
			for (int v = 0; v < property_value_full_names.size(); v++) {
				property_value_aliases.insert({canonicalize_value_name(property_value_full_names[v]), v});
			}
			aliases_initialized = true;
		}
		auto valit = property_value_aliases.find(s);
		if (valit == property_value_aliases.end()) return -1;
		else return valit->second;
	}
	
	class BinaryPropertyObject : public PropertyObject {
	public:
		UnicodeSet the_codepoint_set;
		BinaryPropertyObject(UCD::property_t p, UnicodeSet s) : PropertyObject(p, BinaryProperty), the_codepoint_set(s) {}
		UnicodeSet GetCodepointSet(std::string value_spec);	
	};
	
	UnicodeSet BinaryPropertyObject::GetCodepointSet(std::string value_spec) {
		int property_enum_val = Binary::Y; // default value
		if (value_spec != "") {
			auto valit = Binary::aliases_only_map.find(value_spec);
			if (valit == Binary::aliases_only_map.end()) {
				std::cerr << "Binary property " << property_full_name[the_property] << ": bad value: " << value_spec << ".\n";
				exit(-1);
			}
			else property_enum_val = valit->second;
			if (property_enum_val == Binary::Y) return the_codepoint_set;
			else return uset_complement(the_codepoint_set);
		}
	}
}
	
#endif
