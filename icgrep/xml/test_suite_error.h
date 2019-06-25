/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef TEST_SUITE_ERROR_H
#define TEST_SUITE_ERROR_H

#include <cinttypes>

enum class XmlTestSuiteError {
    /* Post Processing Errors */
    NAME_START = 1,
    NAME,
    XML_PI_NAME,
    CDATA,
    UNDEFREF,
    CHARREF,
    XML10CHARREF,
    ATTREF,

    /* Error Stream Errors */
    ILLEGAL_CHAR,
    UTF8_ERROR,
    PI_SYNTAX,
    COMMENT,
    PI_CD_CT_ERROR,
    TAG,
    REF,
    NAME_SYNTAX,
    CD_CLOSER
};

const char * AsMessage(XmlTestSuiteError error);

void ReportError(XmlTestSuiteError code, const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * lineEnd, uint64_t lineNumber);

/**
 * Prints the first logged error to stderr.
 */
void ShowError();

#endif
