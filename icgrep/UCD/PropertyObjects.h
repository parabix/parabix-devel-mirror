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

std::string canonicalize_value_name(const std::string & prop_or_val);

class PropertyObject {
public:
    enum class ClassTypeId : unsigned {
        NumericProperty
        , CodepointProperty
        , StringProperty
        , MiscellaneousProperty
        , EnumeratedProperty
        , ExtensionProperty
        , CatalogProperty
        , BinaryProperty
        , UnsupportedProperty
    };
    using iterator = const std::vector<std::string>::const_iterator;
    inline ClassTypeId getClassTypeId() const {
        return the_kind;
    }
    inline property_t getPropertyCode() const {
        return the_property;
    }
    PropertyObject(property_t p, ClassTypeId k) : the_property(p), the_kind(k) {}
    virtual int GetPropertyValueEnumCode(const std::string & value_spec);
    property_t the_property;
    ClassTypeId the_kind;
};

class UnsupportedPropertyObject : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::UnsupportedProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    UnsupportedPropertyObject(property_t p, ClassTypeId)
    : PropertyObject(p, ClassTypeId::UnsupportedProperty) {

    }
    UnicodeSet GetCodepointSet(const std::string &);
    UnicodeSet GetCodepointSet(const int);
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
                             const std::vector<std::string> & enum_names,
                             const std::vector<std::string> & names,
                             std::unordered_map<std::string, int> & aliases,
                             std::vector<const UnicodeSet *> && sets)
    : PropertyObject(p, ClassTypeId::EnumeratedProperty)
    , property_value_enum_names(enum_names)
    , property_value_full_names(names)
    , property_value_aliases(aliases)
    , uninitialized(true)
    , property_value_sets(sets) {

    }

    virtual int GetPropertyValueEnumCode(const std::string & value_spec);
    const UnicodeSet & GetCodepointSet(const std::string & value_spec);
    const UnicodeSet & GetCodepointSet(const int property_enum_val) const;
    const std::string & GetValueEnumName(const int property_enum_val) const {return property_value_enum_names[property_enum_val]; }
    const std::string & GetValueFullName(const int property_enum_val) const {return property_value_full_names[property_enum_val]; }

    iterator begin() const {
        return property_value_enum_names.cbegin();
    }

     iterator end() const {
        return property_value_enum_names.cend();
    }

private:
    const std::vector<std::string> & property_value_enum_names;  // never changes
    const std::vector<std::string> & property_value_full_names;  // never changes
    std::unordered_map<std::string, int> & property_value_aliases;
    bool uninitialized; // full names must be added dynamically.
    const std::vector<const UnicodeSet *> property_value_sets;
};

class ExtensionPropertyObject : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::ExtensionProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    ExtensionPropertyObject(UCD::property_t p,
                            UCD::property_t base,
                            std::vector<const UnicodeSet *> && sets)
    : PropertyObject(p, ClassTypeId::ExtensionProperty)
    , base_property(base)
    , property_value_sets(sets) {


    }

    iterator begin() const;

    iterator end() const;

    virtual int GetPropertyValueEnumCode(const std::string & value_spec);
    const UnicodeSet & GetCodepointSet(const std::string & value_spec);
    const UnicodeSet & GetCodepointSet(const int property_enum_val) const;

private:
    const property_t base_property;
    const std::vector<const UnicodeSet *> property_value_sets;
};

class BinaryPropertyObject : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::BinaryProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    BinaryPropertyObject(UCD::property_t p, UnicodeSet s)
    : PropertyObject(p, ClassTypeId::BinaryProperty)
    , mNoUninitialized(true)
    , mY(s) {

    }
    const UnicodeSet & GetCodepointSet(const std::string & value_spec);
    const UnicodeSet & GetCodepointSet(const int property_enum_val);
private:
    bool mNoUninitialized;
    UnicodeSet mY;
    UnicodeSet mN;
};

}

#endif
