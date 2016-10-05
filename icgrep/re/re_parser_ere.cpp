/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/re_parser_ere.h>
#include <re/re_parser_helper.h>

namespace re{

    // \d and \D removed
    const uint64_t setEscapeCharacters = bit3C('b') | bit3C('p') | bit3C('q') | bit3C('w') | bit3C('s') | bit3C('<') | bit3C('>') |
                                         bit3C('B') | bit3C('P') | bit3C('Q') | bit3C('W') | bit3C('S') | bit3C('N') | bit3C('X');

    bool RE_Parser_ERE::isSetEscapeChar(char c) {
        return c >= 0x3C && c <= 0x7B && ((setEscapeCharacters >> (c - 0x3C)) & 1) == 1;
    }

    inline bool RE_Parser_ERE::isUnsupportChartsetOperator(char c) {
        switch (c) {
            case '\\':
                return true;
            default:
                return false;
        }
    }
}