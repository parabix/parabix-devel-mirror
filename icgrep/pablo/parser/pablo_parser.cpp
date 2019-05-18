/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablo_parser.h"

#include <pablo/parser/source_file.h>

namespace pablo {
namespace parse {

boost::optional<std::vector<std::unique_ptr<PabloKernel>>> PabloParser::parse(std::string const & filename) {
    // TODO: handle boost::exception thrown by SourceFile constructor
    std::shared_ptr<SourceFile> source(new SourceFile{filename});
    return parse(source);
}

} // namespace pablo::parse
} // namespace pablo
