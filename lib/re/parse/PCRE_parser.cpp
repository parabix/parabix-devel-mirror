/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "PCRE_parser.h"
#include <re/parsers/parser_helper.h>

namespace re{

    // \< and \> removed
    const uint64_t setEscapeCharacters = bit3C('b') | bit3C('p') | bit3C('q') | bit3C('d') | bit3C('w') | bit3C('s') | bit3C('B') |
                                         bit3C('P') | bit3C('Q') | bit3C('D') | bit3C('W') | bit3C('S') | bit3C('N') | bit3C('X');

    bool PCRE_Parser::isSetEscapeChar(char c) {
        return c >= 0x3C && c <= 0x7B && ((setEscapeCharacters >> (c - 0x3C)) & 1) == 1;
    }

}
