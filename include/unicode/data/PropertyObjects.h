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
#include <unicode/core/unicode_set.h>
#include <string>
#include <vector>
#include <unordered_map>

namespace re { class RE; }

namespace UCD {
    
std::string canonicalize_value_name(const std::string & prop_or_val);

// Recursive property resolution is implemented using a grep lines function
// that takes a text buffer and returns a vector of matched line numbers.
typedef std::vector<uint64_t> (*GrepLinesFunctionType)(re::RE *, const char * buf, size_t bufSize);

class PropertyObject {
public:
    enum class ClassTypeId : unsigned {
        BinaryProperty,
        EnumeratedProperty,
        ExtensionProperty,
        NumericProperty,
        StringProperty,
        StringOverrideProperty,
        BoundaryProperty,
        ObsoleteProperty,
        UnsupportedProperty
    };

    using iterator = const std::vector<std::string>::const_iterator;
    inline ClassTypeId getClassTypeId() const {
        return the_kind;
    }
    inline property_t getPropertyCode() const {
        return the_property;
    }
    PropertyObject(property_t p, ClassTypeId k) : the_property(p), the_kind(k) {}
    virtual const UnicodeSet GetCodepointSet(const std::string & prop_value_string);
    virtual const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern, GrepLinesFunctionType);
    virtual const UnicodeSet GetReflexiveSet() const;
    virtual const std::string GetStringValue(UCD::codepoint_t cp) const;

    virtual const std::string & GetPropertyValueGrepString();
    const property_t the_property;
    const ClassTypeId the_kind;
    virtual ~PropertyObject() {}
};

class BinaryPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::BinaryProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    BinaryPropertyObject(UCD::property_t p, const UnicodeSet && set)
    : PropertyObject(p, ClassTypeId::BinaryProperty)
    , mY(std::move(set))
    , mN() {

    }
    int GetPropertyValueEnumCode(const std::string & value_spec);
    const UnicodeSet GetCodepointSet(const std::string & prop_value_string) override;
    // const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern) override;
    const UnicodeSet & GetCodepointSet(const int property_enum_val);
    const std::string & GetPropertyValueGrepString() override;
private:
    const UnicodeSet mY;
    std::unique_ptr<UnicodeSet> mN;
    std::string mPropertyValueGrepString;
};

class EnumeratedPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::EnumeratedProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    EnumeratedPropertyObject(UCD::property_t p,
                             const unsigned independent_enums,
                             const std::vector<std::string> && enum_names,
                             const std::vector<std::string> && names,
                             std::unordered_map<std::string, int> && aliases,
                             std::vector<const UnicodeSet *> && sets)
    : PropertyObject(p, ClassTypeId::EnumeratedProperty)
    , independent_enum_count(independent_enums)
    , property_value_enum_names(std::move(enum_names))
    , property_value_full_names(std::move(names))
    , property_value_aliases(std::move(aliases))
    , uninitialized(true)
    , property_value_sets(std::move(sets)) {

    }

    virtual int GetPropertyValueEnumCode(const std::string & value_spec);
    int GetEnumCount() {return independent_enum_count;}
    const std::string & GetPropertyValueGrepString() override;
    const UnicodeSet GetCodepointSet(const std::string & value_spec) override;
    const UnicodeSet GetCodepointSetMatchingPattern(re::RE *, GrepLinesFunctionType) override;
    const UnicodeSet & GetCodepointSet(const int property_enum_val) const;
    std::vector<UnicodeSet> & GetEnumerationBasisSets();
    unsigned GetEnumerationValue(codepoint_t cp);
    const std::string & GetValueEnumName(const int property_enum_val) const {return property_value_enum_names[property_enum_val]; }
    const std::string & GetValueFullName(const int property_enum_val) const {return property_value_full_names[property_enum_val]; }

    iterator begin() const {
        return property_value_enum_names.cbegin();
    }

    iterator end() const {
        return property_value_enum_names.cend();
    }

private:
    const unsigned independent_enum_count;
    const std::vector<std::string> property_value_enum_names;
    const std::vector<std::string> property_value_full_names;
    std::unordered_map<std::string, int> property_value_aliases;
    std::string mPropertyValueGrepString;
    bool uninitialized; // full names must be added dynamically.
    const std::vector<const UnicodeSet *> property_value_sets;
    std::vector<UnicodeSet> enumeration_basis_sets;
};

class ExtensionPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::ExtensionProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    ExtensionPropertyObject(UCD::property_t p,
                            UCD::property_t base,
                            const std::vector<const UnicodeSet *> && sets)
    : PropertyObject(p, ClassTypeId::ExtensionProperty)
    , base_property(base)
    , property_value_sets(sets) {


    }

    iterator begin() const;

    iterator end() const;

    int GetPropertyValueEnumCode(const std::string & value_spec);
    const std::string & GetPropertyValueGrepString() override;
    const UnicodeSet GetCodepointSet(const std::string & value_spec) override;
    const UnicodeSet & GetCodepointSet(const int property_enum_val) const;
    const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern, GrepLinesFunctionType) override;

private:
    const property_t base_property;
    const std::vector<const UnicodeSet *> property_value_sets;
};

class NumericPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::NumericProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    NumericPropertyObject(UCD::property_t p, const UnicodeSet && NaN_Set, const char * string_buffer, unsigned bufsize, const std::vector<UCD::codepoint_t> && cps)
    : PropertyObject(p, ClassTypeId::NumericProperty)
    , mNaNCodepointSet(std::move(NaN_Set))
    , mStringBuffer(string_buffer)
    , mBufSize(bufsize)
    , mExplicitCps(std::move(cps))
    {

    }
    const UnicodeSet GetCodepointSet(const std::string & numeric_spec) override;
    const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern, GrepLinesFunctionType) override;

private:
    const UnicodeSet mNaNCodepointSet;  // codepoints for which the property value is NaN (not a number).
    const char * mStringBuffer;  // buffer holding all string values for other codepoints, in sorted order.
    unsigned mBufSize;
    const std::vector<UCD::codepoint_t> mExplicitCps;
};

class StringPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::StringProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }
    StringPropertyObject(UCD::property_t p, const UnicodeSet && nullSet, const UnicodeSet && mapsToSelf, const char * string_buffer,
                         const std::vector<unsigned> && offsets, const std::vector<UCD::codepoint_t> && cps)
    : PropertyObject(p, ClassTypeId::StringProperty)
    , mNullCodepointSet(std::move(nullSet))
    , mSelfCodepointSet(std::move(mapsToSelf))
    , mStringBuffer(string_buffer)
    , mStringOffsets(offsets)
    , mExplicitCps(cps)
    {

    }
    const UnicodeSet GetCodepointSet(const std::string & value_spec) override;
    const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern, GrepLinesFunctionType) override;
    const UnicodeSet GetReflexiveSet() const override;
    const std::string GetStringValue(UCD::codepoint_t cp) const override;

private:
    const UnicodeSet mNullCodepointSet;  // codepoints for which the property value is the null string.
    const UnicodeSet mSelfCodepointSet;  // codepoints for which the property value is the codepoint itself.
    // Codepoints other than those in these two sets are explicitly represented.
    const char * mStringBuffer;  // buffer holding all string values for explicit codepoints, in sorted order.
    const std::vector<unsigned> mStringOffsets;        // the offsets of each string within the buffer.
    //unsigned mBufSize;                               // mStringOffsets has one extra element for buffer size.
    const std::vector<UCD::codepoint_t> mExplicitCps;  // the codepoints having explicit strings
};

class StringOverridePropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::StringOverrideProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }
    StringOverridePropertyObject(UCD::property_t p, UCD::property_t baseProp, const UnicodeSet && overriddenSet, const char * string_buffer,
                                 const std::vector<unsigned> && offsets, const std::vector<UCD::codepoint_t> && cps)
    : PropertyObject(p, ClassTypeId::StringOverrideProperty)
    , mBaseProperty(baseProp)
    , mOverriddenSet(std::move(overriddenSet))
    , mStringBuffer(string_buffer)
    , mStringOffsets(offsets)
    , mExplicitCps(cps)
    {

    }
    const UnicodeSet GetCodepointSet(const std::string & value_spec) override;
    const UnicodeSet GetCodepointSetMatchingPattern(re::RE * pattern, GrepLinesFunctionType) override;
    const UnicodeSet GetReflexiveSet() const override;
    const UCD::property_t GetBaseProperty() {return mBaseProperty;}
    const UnicodeSet & GetOverriddenSet() const {return mOverriddenSet;}
    const std::string GetStringValue(UCD::codepoint_t cp) const override;

private:
    UCD::property_t mBaseProperty;  // the base object that provides default values for this property unless overridden.
    const UnicodeSet mOverriddenSet;   // codepoints for which the baseObject value is overridden.
    const char * mStringBuffer;  // buffer holding all string values for overridden codepoints, in sorted order.
    const std::vector<unsigned> mStringOffsets;        // the offsets of each string within the buffer.
    //unsigned mBufSize;                               // mStringOffsets has one extra element for buffer size.
    const std::vector<codepoint_t> mExplicitCps;
};

class BoundaryPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::BoundaryProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    BoundaryPropertyObject(UCD::property_t p)
    : PropertyObject(p, ClassTypeId::BoundaryProperty)
    {

    }
};

class ObsoletePropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::ObsoleteProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    ObsoletePropertyObject(property_t p)
    : PropertyObject(p, ClassTypeId::ObsoleteProperty) {}

    const std::string & GetPropertyValueGrepString() override;
    const UnicodeSet GetCodepointSet(const std::string & value_spec) override;

};

class UnsupportedPropertyObject final : public PropertyObject {
public:
    static inline bool classof(const PropertyObject * p) {
        return p->getClassTypeId() == ClassTypeId::UnsupportedProperty;
    }
    static inline bool classof(const void *) {
        return false;
    }

    UnsupportedPropertyObject(property_t p, ClassTypeId)
    : PropertyObject(p, ClassTypeId::UnsupportedProperty) {}
};

}

#endif
