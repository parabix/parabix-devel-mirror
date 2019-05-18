/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <ostream>
#include <memory>
#include <vector>
#include <boost/optional.hpp>
#include <pablo/pablo_kernel.h>

namespace pablo {
namespace parse {

class SourceFile;

/**
 * Abstract interface for all pablo parser implementations.
 * 
 * The two primary methods, parse and unparse, must be inverse operations in
 * all implementations (i.e., parse(unparse(X)) == X for all valid X).
 */
class PabloParser {
public:

    virtual ~PabloParser() = default;

    /**
     * Constructs a series of PabloKernels by parsing a given source file.
     * 
     * @param sourceFile A shared instance to the source file to parse.
     * @return A series of PabloKernels or boost::none if an error occurred.
     */
    virtual boost::optional<std::vector<std::unique_ptr<PabloKernel>>> parse(std::shared_ptr<SourceFile> sourceFile) = 0;

    /**
     * The reverse operation of parse. Writes pablo source code to an output
     * stream by reading a given set kernels.
     */
    virtual void unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) = 0;

    /**
     * Convenience parse method which accepts a filename to parse instead of
     * a source file instance.
     * 
     * @param filename The file to parse.
     * @return A series of PabloKernels or boost::none if an error occurred.
     */
    boost::optional<std::vector<std::unique_ptr<PabloKernel>>> parse(std::string const & filename);

};

} // namespace pablo::parse
} // namespace pablo
