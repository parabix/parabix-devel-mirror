/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablo_parser.h"
#include <fstream>

namespace pablo {

std::unique_ptr<PabloKernel> PabloParser::parse(std::string const & filename) {
    std::ifstream fin(filename);
    return parse(fin);
}

} // namespace pablo
