/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "parsefailure.h"

ParseFailure::ParseFailure(const std::string && msg) noexcept
: _msg(msg)
{

}

ParseFailure::~ParseFailure() noexcept { }

const char* ParseFailure::what() const noexcept {
    return _msg.c_str();
}
