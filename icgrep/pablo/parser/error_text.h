/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <string>

namespace pablo {
namespace parse {

inline std::string errtxt_IllegalSymbol(char c) {
    return "illegal symbol '" + std::string{c} + "'";
}

inline std::string errtxt_IllegalSymbol(std::string const & s) {
    return "illegal symbol '" + s + "'";
}

} // namespace pablo::parse
} // namespace pablo
