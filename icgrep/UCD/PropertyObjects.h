#ifndef PROPERTYOBJECTS_H
#define PROPERTYOBJECTS_H
/*
 *  Copyright (c) 2014 International Characters, Inc.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters, Inc.
 *
 */

#include "PropertyAliases.h"
#include "PropertyValueAliases.h"
#include "unicode_set.h"
#include <string>
#include <vector>
#include <unordered_map>

namespace UCD {

    std::string canonicalize_value_name(const std::string prop_or_val);

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
		
        virtual UnicodeSet GetCodepointSet(const std::string value_spec) = 0;
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
        UnicodeSet GetCodepointSet(const std::string value_spec);
	};
	
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
        int GetPropertyValueEnumCode(const std::string s);
        UnicodeSet GetCodepointSet(const std::string value_spec);
		
	private:
        const std::vector<std::string> property_value_enum_names;  // never changes
        const std::vector<std::string> property_value_full_names;  // never changes
		std::unordered_map<std::string, int> property_value_aliases;
		bool aliases_initialized; // full names must be added dynamically.
		std::vector<UnicodeSet> property_value_sets;                 
	};
	
	class BinaryPropertyObject : public PropertyObject {
	public:
        static inline bool classof(const PropertyObject * p) {
            return p->getClassTypeId() == ClassTypeId::BinaryProperty;
        }
        static inline bool classof(const void *) {
            return false;
        }
		
		BinaryPropertyObject(UCD::property_t p, UnicodeSet s) : PropertyObject(p, ClassTypeId::BinaryProperty), the_codepoint_set(s) {}
        UnicodeSet GetCodepointSet(const std::string value_spec);
    private:
		UnicodeSet the_codepoint_set;        
	};
	
}
	
#endif
