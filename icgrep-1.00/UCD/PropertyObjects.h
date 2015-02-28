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
#include <iostream>
#include <unordered_map>
#include <stdexcept>
#include "unicode_set.h"
#include "PropertyAliases.h"
#include "PropertyValueAliases.h"

std::string canonicalize_value_name(std::string prop_or_val) {
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


namespace UCD {
    
	class PropertyObject {
	public:
        enum class ClassTypeId : unsigned {
            NumericProperty, CodepointProperty, StringProperty, MiscellaneousProperty, EnumeratedProperty, CatalogProperty, BinaryProperty,  UnsupportedProperty
        };
        inline ClassTypeId getClassTypeId() const {
            return the_kind;
        }
		PropertyObject(property_t p, ClassTypeId k) : the_property(p), the_kind(k) {}
        
		property_t the_property;
		ClassTypeId the_kind;
		
		virtual UnicodeSet GetCodepointSet(std::string value_spec) = 0;
	};
	
	class UnsupportedPropertyObject : public PropertyObject {
	public:
        static inline bool classof(const PropertyObject * p) {
            return p->getClassTypeId() == ClassTypeId::UnsupportedProperty;
        }
        static inline bool classof(const void *) {
            return false;
        }
        
		UnsupportedPropertyObject(property_t p, ClassTypeId k) : PropertyObject(p, k) {}
		UnicodeSet GetCodepointSet(std::string value_spec);	
	};
	
	UnicodeSet UnsupportedPropertyObject::GetCodepointSet(std::string value_spec) {
		throw std::runtime_error("Property " + UCD::property_full_name[the_property] + " unsupported.");
	}
	
	class EnumeratedPropertyObject : public PropertyObject {
	public:
        static inline bool classof(const PropertyObject * p) {
            return p->getClassTypeId() == ClassTypeId::EnumeratedProperty;
        }
        static inline bool classof(const void *) {
            return false;
        }
		
		EnumeratedPropertyObject(UCD::property_t p, 
                                 const std::vector<std::string> enum_names,
                                 const std::vector<std::string> names,
                                         const std::unordered_map<std::string, int> aliases,
                                         const std::vector<UnicodeSet> sets) : 
		PropertyObject(p, ClassTypeId::EnumeratedProperty), property_value_enum_names(enum_names), property_value_full_names(names), property_value_aliases(aliases), aliases_initialized(false), property_value_sets(sets) {}
		int GetPropertyValueEnumCode(std::string s);
		UnicodeSet GetCodepointSet(std::string value_spec);	
		
	private:
        const std::vector<std::string> property_value_enum_names;  // never changes
        const std::vector<std::string> property_value_full_names;  // never changes
		std::unordered_map<std::string, int> property_value_aliases;
		bool aliases_initialized; // full names must be added dynamically.
		std::vector<UnicodeSet> property_value_sets;                 
	};
	
	UnicodeSet EnumeratedPropertyObject::GetCodepointSet(std::string value_spec) {
		int property_enum_val = GetPropertyValueEnumCode(value_spec);
		if (property_enum_val == -1) {
		        throw std::runtime_error("Enumerated Property " + UCD::property_full_name[the_property] +  ": unknown value: " + value_spec);
		}
		else {
			//std::cout << "Enumerated Property: " << UCD::property_full_name[the_property] << ", value: " << property_value_full_names[property_enum_val] << "(" << property_enum_val << ").\n";
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
            for (int v = 0; v < property_value_enum_names.size(); v++) {
                property_value_aliases.insert({canonicalize_value_name(property_value_enum_names[v]), v});
            }
			aliases_initialized = true;
		}
		auto valit = property_value_aliases.find(s);
		if (valit == property_value_aliases.end()) return -1;
		else return valit->second;
	}
	
	class BinaryPropertyObject : public PropertyObject {
	public:
        static inline bool classof(const PropertyObject * p) {
            return p->getClassTypeId() == ClassTypeId::BinaryProperty;
        }
        static inline bool classof(const void *) {
            return false;
        }
		
		BinaryPropertyObject(UCD::property_t p, UnicodeSet s) : PropertyObject(p, ClassTypeId::BinaryProperty), the_codepoint_set(s) {}
		UnicodeSet GetCodepointSet(std::string value_spec);	
    private:
		UnicodeSet the_codepoint_set;        
	};
	
	UnicodeSet BinaryPropertyObject::GetCodepointSet(std::string value_spec) {
		int property_enum_val = Binary_ns::Y; // default value
		if (value_spec != "") {
			auto valit = Binary_ns::aliases_only_map.find(value_spec);
			if (valit == Binary_ns::aliases_only_map.end()) {
		                throw std::runtime_error("Binary Property " + UCD::property_full_name[the_property] +  ": bad value: " + value_spec);
			}
			else property_enum_val = valit->second;
			if (property_enum_val == Binary_ns::Y) return the_codepoint_set;
			else return uset_complement(the_codepoint_set);
		}
        else return the_codepoint_set;
	}
}
	
#endif
