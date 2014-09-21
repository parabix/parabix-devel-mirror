/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "re_alt.h"

Alt::Alt()
{

}

Alt::Alt(iterator begin, iterator end)
: std::vector<RE*>(begin, end)
{

}

Alt::~Alt()
{
    for (RE * re : *this) {
        delete re;
    }
}
