/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "post_process.h"

#include <cassert>
#include <cstdlib>
#include <stack>
#include <string>
#include <llvm/Support/Compiler.h>
#include <xml/namechars.h>
#include <xml/test_suite_error.h>

static uint8_t HexValTable[] = {
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,   0x08, 0x09, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,

    0xff, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
    0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,   0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

static bool asciiSeqEq(const uint8_t * ptr, std::string const & reference) {
    const char * cptr = reinterpret_cast<const char *>(ptr);
    for (size_t i = 0; i < reference.length(); ++i) {
        if (cptr[i] != reference[i]) {
            return false;
        }
    }
    return true;
}

#define ASCII_TO_LOWER(C) (C >= 'A' && C <= 'Z' ? (C + 0x20) : C)

static bool asciiCaselessSeqEq(const uint8_t * ptr, std::string const & reference) {
    const char * cptr = reinterpret_cast<const char *>(ptr);
    for (size_t i = 0; i < reference.length(); ++i) {
        if (ASCII_TO_LOWER(cptr[i]) != ASCII_TO_LOWER(reference[i])) {
            return false;
        }
    }
    return true;
}

static bool atHexDigit(const uint8_t * p) {
    return *p >= 128 ? false : HexValTable[*p] != 0xff;
}

static bool atDecDigit(const uint8_t * p) {
    return *p >= '0' && *p <= '9';
}

static uint32_t hexValue(char c) {
    assert ((int) c < 128);
    uint8_t v = HexValTable[(int) c];
    assert (v != 0xff);
    return (uint32_t) v;
}

static uint32_t decValue(char c) {
    // still use the hex table for convenience but we assert that c < 64 to
    // block out alpha hex numerals (i.e., a-f)
    assert ((int) c < 64);
    uint8_t v = HexValTable[(int) c];
    assert (v != 0xff);
    return (uint32_t) v;
}

void postproc_validateNameStart(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    if (XML_10_UTF8_NameStrt_bytes(ptr) == 0) {
        ReportError(XmlTestSuiteError::NAME_START, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validateName(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    if (XML_10_UTF8_NameChar_bytes(ptr) == 0) {
        ReportError(XmlTestSuiteError::NAME, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validatePIName(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    if (asciiCaselessSeqEq(ptr + 1, "xml") && (ptr[4] == '?' || ptr[4] <= ' ')) {
        if (lineNum != 1 || (ptr - lineBegin != 1) || !asciiSeqEq(ptr + 1, "xml")) {
            // ptr is pointing at the '?' character, but to get the correct
            // location for the error message it should be pointing at the
            // character just after (usually the 'x' in "xml").
            auto correctedPtr = ptr + 1;
            ReportError(XmlTestSuiteError::XML_PI_NAME, correctedPtr, lineBegin, lineEnd, lineNum);
        }
    }
}

void postproc_validateCDATA(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    if (!asciiSeqEq(ptr, "[CDATA[")) {
        ReportError(XmlTestSuiteError::CDATA, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validateGenRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    bool valid =    asciiSeqEq(ptr, "gt;")
                 || asciiSeqEq(ptr, "lt;")
                 || asciiSeqEq(ptr, "amp;")
                 || asciiSeqEq(ptr, "quote;")
                 || asciiSeqEq(ptr, "apos;");
    if (!valid) {
        ReportError(XmlTestSuiteError::UNDEFREF, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validateHexRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    int val = 0;
    const uint8_t * cursor = ptr;
    while (atHexDigit(cursor)) {
        val = hexValue(*cursor) + (val << 4);
        if (val > 0x10FFFF) {
            ReportError(XmlTestSuiteError::CHARREF, ptr, lineBegin, lineEnd, lineNum);
        }
        cursor++;
    }
    if ((val == 0x0) || ((val | 0x7FF) == 0xDFFF)|| ((val | 0x1) == 0xFFFF)){
        ReportError(XmlTestSuiteError::CHARREF, ptr, lineBegin, lineEnd, lineNum);
    }
    else if (((val < 0x20) && (val != 0x9) && (val != 0xD) && (val != 0xA))){
        ReportError(XmlTestSuiteError::XML10CHARREF, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validateDecRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    int val = 0;
    const uint8_t * cursor = ptr;
    while (atDecDigit(cursor)) {
        val = decValue(*cursor) + (val * 10);
        if (val > 0x10FFFF) {
            ReportError(XmlTestSuiteError::CHARREF, ptr, lineBegin, lineEnd, lineNum);
        }
        cursor++;
    }
    if ((val == 0x0) || ((val | 0x7FF) == 0xDFFF)|| ((val | 0x1) == 0xFFFF)){
        ReportError(XmlTestSuiteError::CHARREF, ptr, lineBegin, lineEnd, lineNum);
    }
    else if (((val < 0x20) && (val != 0x9) && (val != 0xD) && (val != 0xA))){
        ReportError(XmlTestSuiteError::XML10CHARREF, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_validateAttRef(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum) {
    const uint8_t * cursor = ptr;
    if (*cursor == '#') {
        cursor++;
        uint32_t val = 0;
        if (*cursor == 'x' || *cursor == 'X') {
            cursor++;
            while (atHexDigit(cursor)) {
                val = hexValue(*cursor) + (val << 4);
                cursor++;
            }
        } else {
            while (atDecDigit(cursor)) {
                val = decValue(*cursor) + (val * 10);
                cursor++;
            }
        }
        if (val == (uint32_t) '<') {
            ReportError(XmlTestSuiteError::ATTREF, ptr, lineBegin, lineEnd, lineNum);
        }
    } else if (asciiSeqEq(ptr, "lt;")) {
        ReportError(XmlTestSuiteError::ATTREF, ptr, lineBegin, lineEnd, lineNum);
    }
}

void postproc_errorStreamsCallback(const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNum, uint8_t code) {
    // Error streams are indexed as such:
    //  type Error = <i1>[9] {
    //      IllegalChar,
    //      UTF8_Error,
    //      PI_Error,
    //      CT_Error,
    //      CT_CD_PI_Error,
    //      Tag_Error,
    //      Ref_Error,
    //      Name_Error,
    //      CheckStreams_Error
    //  }
    // Indexing matches up with the order of enumeration values so we need not
    // use a switch statement to pick error codes.
    auto c = (int) XmlTestSuiteError::ILLEGAL_CHAR;
    if (code == 0) {
        // A little hack to get the number of error streams down to 8 so that
        // the existing P2S kernel can be used.
        ReportError(XmlTestSuiteError::CD_CLOSER, ptr, lineBegin, lineEnd, lineNum);
    }
    while (code != 0) {
        if (code & 0x1UL) {
            ReportError((XmlTestSuiteError) c, ptr, lineBegin, lineEnd, lineNum);
        }
        code = code >> 1;
        c++;
    }
}

static std::stack<std::string> TagStack{};

void postproc_tagMatcher(const uint8_t * begin, const uint8_t * end, uint64_t namePosition) {
    assert (begin <= end);
    if (begin == end) { // these errors are handled by the error streams
        return;
    }

    auto b = reinterpret_cast<const char *>(begin);
    auto e = reinterpret_cast<const char *>(end);
    auto pos = namePosition + 1; // to 1-indexed
    std::string name(b, e);
    
    if (name == "/>") {
        // end of empty tag; pop off top element
        assert (!TagStack.empty()); // should never have a '/' name without a tag in the stack
        TagStack.pop();
    } else if (name[0] == '/') {
        // closing tag; pop off and match
        if (TagStack.empty()) {
            ReportError(XmlTestSuiteError::TAG_MATCH_ERROR, pos);
            return;
        }

        std::string opener = TagStack.top();
        TagStack.pop();
        if (opener != name.substr(1)) {
            ReportError(XmlTestSuiteError::TAG_NAME_MISMATCH, pos);
            return;
        }
    } else {
        // opening tag
        TagStack.push(name);
    }
}
