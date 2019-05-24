/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "pablo_parser.h"

#include <pablo/parser/kernel_signature.h>
#include <pablo/parser/source_file.h>

namespace pablo {
namespace parse {

bool PabloParser::parseKernel(std::string const & filename, PabloSourceKernel * kernel, std::string const & kernelName) {
    // TODO: handle boost::exception thrown by SourceFile constructor
    std::shared_ptr<SourceFile> source(new SourceFile{filename});
    return parseKernel(source, kernel, kernelName);
}

} // namespace pablo::parse
} // namespace pablo
