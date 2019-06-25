/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "test_suite_error.h"

#include <cstddef>
#include <cassert>
#include <mutex>
#include <string>
#include <queue>
#include <vector>

template<typename T, class Compare>
class atomic_priority_queue {
public:
    atomic_priority_queue() = default;
    void push(T const & t) {
        std::lock_guard<std::mutex> lock(m_mutex);
        m_queue.push(t);
    }
    T pop() {
        assert (!m_queue.empty());
        std::lock_guard<std::mutex> lock(m_mutex);
        T val = m_queue.top();
        m_queue.pop();
        return val;
    }
    bool empty() {
        std::lock_guard<std::mutex> lock(m_mutex);
        return m_queue.empty();
    }
private:
    std::mutex m_mutex;
    std::priority_queue<T, std::vector<T>, Compare> m_queue;
};

struct TestSuiteError {
    XmlTestSuiteError   code;
    uint64_t            line;
    uint64_t            column;

    struct Comparator {
        bool operator() (TestSuiteError const & lhs, TestSuiteError const & rhs) const {
            if (lhs.line == rhs.line) {
                return lhs.column > rhs.column;
            }
            return lhs.line > rhs.line;
        }
    };
};


static atomic_priority_queue<TestSuiteError, TestSuiteError::Comparator> ErrorQueue{};

const char * AsMessage(XmlTestSuiteError error) {
    switch (error) {
        case XmlTestSuiteError::NAME_START:
            return "name start error";
        case XmlTestSuiteError::NAME:
            return "name error";
        case XmlTestSuiteError::XML_PI_NAME:
            return "[Xx][Mm][Ll] illegal as PI name";
        case XmlTestSuiteError::CDATA:
            return "CDATA error";
        case XmlTestSuiteError::UNDEFREF:
            return "Undefined reference";
        case XmlTestSuiteError::CHARREF:
            return "Illegal character reference";
        case XmlTestSuiteError::XML10CHARREF:
            return "Illegal XML 1.0 character reference";
        case XmlTestSuiteError::ATTREF:
            return "Attribute values contain '<' characters after reference expansion";
        case XmlTestSuiteError::ILLEGAL_CHAR:
            return "Error: illegal character";
        case XmlTestSuiteError::UTF8_ERROR:
            return "UTF-8 error found";
        case XmlTestSuiteError::PI_SYNTAX:
            return "Error in PI syntax";
        case XmlTestSuiteError::COMMENT:
            return "Error in comment syntax";
        case XmlTestSuiteError::PI_CD_CT_ERROR:
            return "Error in comment, CDATA or processing instruction syntax";
        case XmlTestSuiteError::TAG:
            return "Tag parsing error found";
        case XmlTestSuiteError::REF:
            return "Reference error found";
        case XmlTestSuiteError::NAME_SYNTAX:
            return "name syntax error";
        case XmlTestSuiteError::CD_CLOSER:
            return "Error: ]]> in text";
        default:
            assert ("unexpected error xml test suite error" && false);
            return "Invalid XML Error Code";
    }
}

void ReportError(XmlTestSuiteError code, const uint8_t * ptr, const uint8_t * lineBegin, const uint8_t * /*lineEnd*/, uint64_t lineNumber) {
    ptrdiff_t column = ptr - lineBegin;
    assert (column >= 0);
    column += 1; // convert from 0-indexed to 1-indexed column
    TestSuiteError err{code, lineNumber, (uint64_t) column};
    ErrorQueue.push(err);
}

void ShowError() {
    if (ErrorQueue.empty()) {
        return;
    }
    auto err = ErrorQueue.pop();
    fprintf(stderr, "%s at line %" PRIu64 ", column %" PRIu64 "\n", AsMessage(err.code), err.line, err.column);
}
