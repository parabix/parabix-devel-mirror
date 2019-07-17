/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <unicode/core/unicode_set.h>
#include <unicode/data/PropertyObjects.h>

namespace re { class RE; class Name; }

namespace grep {

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::PropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::BinaryPropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::EnumeratedPropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::ExtensionPropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::NumericPropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::StringPropertyObject * propObj, re::RE * pattern);

const UCD::UnicodeSet GetCodepointSetMatchingPattern(UCD::StringOverridePropertyObject * propObj, re::RE * pattern);



/**
 * Resolves an re::Name into a unicode set where the name's value may be a 
 * regular expression.
 *
 * If it is known that the value of name is not a regular expression, use of
 * UCD::resolveUnicodeSet(re::Name * const) is preferred as its use does not 
 * require linking with the grep module.
 */
UCD::UnicodeSet resolveUnicodeSet(re::Name * const name);

}
