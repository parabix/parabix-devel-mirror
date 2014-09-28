/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PRINTER_RE_H
#define PRINTER_RE_H

#include <string>

namespace re {
    class RE;
}

class Printer_RE
{
public:
    static const std::string PrintRE(const re::RE *re);
};

#endif // PRINTER_RE_H
