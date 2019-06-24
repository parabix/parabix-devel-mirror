/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TEST_SUITE_ERROR_H
#define TEST_SUITE_ERROR_H

#include <cinttypes>

enum class XmlTestSuiteError {
    NAME_START = 1,
    NAME,
    XML_PI_NAME,
    CDATA,
    UNDEFREF,
    CHARREF,
    XML10CHARREF,
    ATTREF
};

const char * AsMessage(XmlTestSuiteError error);

void ReportError(XmlTestSuiteError code, const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNumber);

/**
 * Prints the first logged error to stderr.
 */
void ShowError();

#endif
