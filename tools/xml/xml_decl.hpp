/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#pragma once

#include "multiliteral.hpp"

#include <cassert>
#include <llvm/Support/Compiler.h>
#include <util/lookup_table.hpp>

constexpr bool is_ascii_letter(char c) {
    using lc_alpha = lut::char_range<'a', 'z'>;
    using uc_alpha = lut::char_range<'A', 'Z'>;
    return lc_alpha::contains(c) || uc_alpha::contains(c);
}

constexpr bool is_ascii_name_char(char c) {
    using digit = lut::char_range<'0', '9'>;
    using lc_alpha = lut::char_range<'a', 'z'>;
    using uc_alpha = lut::char_range<'A', 'Z'>;
    return digit::contains(c)
        || lc_alpha::contains(c)
        || uc_alpha::contains(c)
        || c == '-' || c == '_' || c == '.';
}

constexpr auto ascii_letter_char_table = lut::char_table(is_ascii_letter);
constexpr auto ascii_name_char_table = lut::char_table(is_ascii_name_char);
constexpr auto whitespace_char_table = lut::char_table(lut::is_whitespace);

/// Alias type for parser positions.
using PositionType = const uint8_t *;

/// A pair of a position and a value.
///
/// Some parser functions need to return both a next position and the value
/// which was parsed. Those functions return this type.
template<typename T>
struct ParseResult {
    PositionType position;
    T            value;
};

/// Stack-based optional type.
template<typename T>
class Result {
public:

    static Result<T> err(PositionType errPos) {
        Result<T> r{};
        r.value.err_ptr = errPos;
        return r;
    };

    static Result<T> ok(T const & t) {
        return Result<T>{t};
    }

public:

    bool is_err() const noexcept { return !is_valid; }
    bool is_some() const noexcept { return is_valid; }

    T const & unwrap() const noexcept {
        assert(is_some());
        return value.value;
    }

    PositionType unwrap_err() const noexcept {
        assert(is_err());
        return value.err_ptr;
    }

private:
    union optional_t {
        PositionType err_ptr;
        T            value;

        optional_t() { this->err_ptr = nullptr; }
        explicit optional_t(T const & t) { this->value = t; }
    };

    Result()
    : is_valid(false), value()
    {}

    explicit Result(T const & t)
    : is_valid(true), value(t)
    {}

    bool       is_valid;
    optional_t value;
};

/// XML Declaration Container
class XmlDeclaration {
public:
    enum class Version : uint8_t { V_1_0, V_1_1 };
    enum class Standalone : uint8_t { YES, NO, UNKNOWN };

    static Result<XmlDeclaration> parse(const uint8_t * ptr);

public:

    XmlDeclaration()
    : mValid(false)
    , mVersion(Version::V_1_0)
    , mEncoding(nullptr)
    , mStandalone(Standalone::UNKNOWN)
    {}

    XmlDeclaration(Version version, const uint8_t * encoding, Standalone standalone)
    : mValid(true)
    , mVersion(version)
    , mEncoding(encoding)
    , mStandalone(standalone)
    {}

    bool isValid() const noexcept { return mValid; }

    Version getVersion() const noexcept { return mVersion; }

    Standalone getStandalone() const noexcept { return mStandalone; }

    bool hasEncoding() const noexcept { return mEncoding != nullptr; }

    bool isUTF8() const {
        assert(mValid);
        assert(mEncoding);
        using utf8_ml = multiliteral<'u', 't', 'f', '-', '8'>;
        return caseless_matches<utf8_ml>(mEncoding);
    }

private:
    bool            mValid;
    Version         mVersion;
    const uint8_t * mEncoding;
    Standalone      mStandalone;
};

/// Advances `ptr` thorugh 0 or more whitespace characters.
inline PositionType scanThruWhitespace(PositionType ptr) {
    while (whitespace_char_table[*ptr]) {
        ptr++;
    }
    return ptr;
}

/// Advances `ptr` through 1 or more whitespace characters.
///
/// Returns `err` if there is not at least one whitespace character.
inline Result<PositionType> scanThruOneOrMoreWhitespace(PositionType ptr) {
    if (!whitespace_char_table[*ptr])
        return Result<PositionType>::err(ptr);
    return Result<PositionType>::ok(scanThruWhitespace(ptr + 1));
}

/// Advances over the '=' character and optional whitespace separating attributes
/// from their values.
///
/// Returns `err` if no such character exists.
inline Result<PositionType> skipAttrValSeparator(PositionType ptr) {
    ptr = scanThruWhitespace(ptr);
    if (*ptr != '=') {
        return Result<PositionType>::err(ptr);
    }
    ptr += 1;
    return Result<PositionType>::ok(scanThruWhitespace(ptr));
}

/// Assuming `ptr` is pointing at the start of an XML declaration (i.e., the
/// opening '?' in "<?xml ...?>"), this function parses the opening "xml" text
/// and returns a pointer to the begining of the next point of interest.
inline Result<PositionType> parseXML(PositionType ptr) {
    using xml_ml = multiliteral<'?', 'x', 'm', 'l'>;
    if (LLVM_UNLIKELY(!matches<xml_ml>(ptr))) {
        return Result<PositionType>::err(ptr);
    }
    ptr += 4; // skip past "?xml"
    return scanThruOneOrMoreWhitespace(ptr);
}

/// Assuming `ptr` is pointing at the start of a "version" attribute, this
/// function parses the whole attribute, returns the version number and
/// advances to the next point of interest.
inline Result<ParseResult<XmlDeclaration::Version>> parseVersion(PositionType ptr) {
    using Version = XmlDeclaration::Version;
    using ResultType = Result<ParseResult<Version>>;

    using version_ml = multiliteral<'v', 'e', 'r', 's', 'i', 'o', 'n'>;
    if (LLVM_UNLIKELY(!matches<version_ml>(ptr))) {
        return ResultType::err(ptr);
    }
    ptr += 7; // skip past "version"

    auto opt_ptr = skipAttrValSeparator(ptr);
    if (LLVM_UNLIKELY(opt_ptr.is_err())) {
        return ResultType::err(ptr);
    }
    ptr = opt_ptr.unwrap();

    using dq_v10_ml = multiliteral<'"', '1', '.', '0', '"'>;
    using sq_v10_ml = multiliteral<'\'', '1', '.', '0', '\''>;
    using dq_v11_ml = multiliteral<'"', '1', '.', '1', '"'>;
    using sq_v11_ml = multiliteral<'\'', '1', '.', '1', '\''>;
    Version version;
    if (LLVM_LIKELY(matches<dq_v10_ml>(ptr) || matches<sq_v10_ml>(ptr))) {
        version = Version::V_1_0;
    } else if (matches<dq_v11_ml>(ptr) || matches<sq_v11_ml>(ptr)) {
        version = Version::V_1_1;
    } else {
        return ResultType::err(ptr);
    }
    ptr += 5; // skip past version attribute value

    if (*ptr != '?') {
        opt_ptr = scanThruOneOrMoreWhitespace(ptr);
        if (LLVM_UNLIKELY(opt_ptr.is_err())) {
            return ResultType::err(ptr);
        }
        ptr = opt_ptr.unwrap();
    }
    return ResultType::ok({ptr, version});
}

/// Assuming `ptr` is pointing at the start of an "encoding" attribute, this
/// function parses the whole attribute, returns a pointer to the encoding value
/// and advances to the next point of interest.
inline Result<ParseResult<const uint8_t *>> parseEncoding(PositionType ptr) {
    using ResultType = Result<ParseResult<const uint8_t *>>;

    using encoding_ml = multiliteral<'e', 'n', 'c', 'o', 'd', 'i', 'n', 'g'>;
    if (LLVM_UNLIKELY(!matches<encoding_ml>(ptr))) {
        return ResultType::err(ptr);
    }
    ptr += 8; // skip past "encoding"

    auto opt_ptr = skipAttrValSeparator(ptr);
    if (LLVM_UNLIKELY(opt_ptr.is_err())) {
        return ResultType::err(ptr);
    }
    ptr = opt_ptr.unwrap();

    if (LLVM_UNLIKELY(*ptr != '"' && *ptr != '\'')) {
        return ResultType::err(ptr);
    }
    char const quoteStyle = *ptr;
    ptr++;
    auto encoding = ptr; // note the location of the encoding value

    // scan through the encoding value
    if (LLVM_UNLIKELY(!ascii_letter_char_table[*ptr])) {
        return ResultType::err(ptr);
    }
    ptr++; // skip past first ascii letter
    while (ascii_name_char_table[*ptr]) {
        ptr++;
    }
    if (LLVM_UNLIKELY(*ptr != quoteStyle)) {
        return ResultType::err(ptr);
    }
    ptr++;

    // advance to the next point of interest and return
    if (*ptr != '?') {
        opt_ptr = scanThruOneOrMoreWhitespace(ptr);
        if (LLVM_UNLIKELY(opt_ptr.is_err())) {
            return ResultType::err(ptr);
        }
        ptr = opt_ptr.unwrap();
    }
    return ResultType::ok({ptr, encoding});
}

/// Assuming `ptr` is pointing at the start of an "standalone" attribute, this
/// function parses the whole attribute, returns a pointer to the encoding value
/// and advances to the next point of interest.
inline Result<ParseResult<XmlDeclaration::Standalone>> parseStandalone(PositionType ptr) {
    using Standalone = XmlDeclaration::Standalone;
    using ResultType = Result<ParseResult<Standalone>>;

    using standalone_ml_1 = multiliteral<'s', 't', 'a', 'n', 'd', 'a', 'l', 'o'>;
    using standalone_ml_2 = multiliteral<'n', 'e'>;
    if (LLVM_UNLIKELY(!matches<standalone_ml_1>(ptr) || !matches<standalone_ml_2>(ptr + 8))) {
        return ResultType::err(ptr);
    }
    ptr += 10; // skip past "standalone"

    auto opt_ptr = skipAttrValSeparator(ptr);
    if (LLVM_UNLIKELY(opt_ptr.is_err())) {
        return ResultType::err(ptr);
    }
    ptr = opt_ptr.unwrap();

    using dq_yes_ml = multiliteral<'"', 'y', 'e', 's', '"'>;
    using sq_yes_ml = multiliteral<'\'', 'y', 'e', 's', '\''>;
    using dq_no_ml = multiliteral<'"', 'n', 'o', '"'>;
    using sq_no_ml = multiliteral<'\'', 'n', 'o', '\''>;
    Standalone standalone;
    if (matches<dq_yes_ml>(ptr) || matches<sq_yes_ml>(ptr)) {
        standalone = Standalone::YES;
        ptr += 5; // skip past value
    } else if (matches<dq_no_ml>(ptr) || matches<sq_no_ml>(ptr)) {
        standalone = Standalone::NO;
        ptr += 4; // skip past value
    } else {
        // invalid "standalone" value
        return ResultType::err(ptr);
    }

    ptr = scanThruWhitespace(ptr);
    return ResultType::ok({ptr, standalone});
}

/// Parses a whole XML declaration.
inline Result<XmlDeclaration> XmlDeclaration::parse(PositionType ptr) {
    // parse "?xml"
    auto const optXmlParseResult = parseXML(ptr);
    if (LLVM_UNLIKELY(optXmlParseResult.is_err())) {
        return Result<XmlDeclaration>::err(optXmlParseResult.unwrap_err());
    }
    ptr = optXmlParseResult.unwrap();

    // parse "version='...'"
    auto const optVersionResult = parseVersion(ptr);
    if (LLVM_UNLIKELY(optVersionResult.is_err())) {
        return Result<XmlDeclaration>::err(optVersionResult.unwrap_err());
    }
    ptr = optVersionResult.unwrap().position;
    Version const version = optVersionResult.unwrap().value;

    // check if we are at the end of the declaration
    if (*ptr == '?') {
        return Result<XmlDeclaration>::ok(XmlDeclaration{version, nullptr, Standalone::UNKNOWN});
    }

    const uint8_t * encoding = nullptr;
    // "encoding" is optional
    if (*ptr == 'e') {
        // parse "encoding='...'"
        auto const optEncodingResult = parseEncoding(ptr);
        if (LLVM_UNLIKELY(optEncodingResult.is_err())) {
            return Result<XmlDeclaration>::err(optEncodingResult.unwrap_err());
        }
        ptr = optEncodingResult.unwrap().position;
        encoding = optEncodingResult.unwrap().value;

        // check if we are at the end of the declaration
        if (*ptr == '?') {
            return Result<XmlDeclaration>::ok(XmlDeclaration{version, encoding, Standalone::UNKNOWN});
        }
    }

    // parse "standalone='...'"
    auto const optStandaloneResult = parseStandalone(ptr);
    if (LLVM_UNLIKELY(optStandaloneResult.is_err())) {
        return Result<XmlDeclaration>::err(optStandaloneResult.unwrap_err());
    }
    ptr = optStandaloneResult.unwrap().position;
    Standalone const standalone = optStandaloneResult.unwrap().value;

    // if we aren't at the end now, the declaration is malformed
    if (LLVM_UNLIKELY(*ptr != '?')) {
        return Result<XmlDeclaration>::err(ptr);
    }

    return Result<XmlDeclaration>::ok(XmlDeclaration{version, encoding, standalone});
}
