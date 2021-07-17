/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/parse/pablo_parser.h>

#include <pablo/parse/kernel_signature.h>
#include <pablo/parse/source_file.h>

namespace pablo {
namespace parse {

bool PabloParser::parseKernel(std::string const & filename, PabloSourceKernel * kernel, std::string const & kernelName) {
    // TODO: handle boost::exception thrown by SourceFile constructor
    std::shared_ptr<SourceFile> source(new SourceFile{filename});
    return parseKernel(source, kernel, kernelName);
}

} // namespace pablo::parse
} // namespace pablo
