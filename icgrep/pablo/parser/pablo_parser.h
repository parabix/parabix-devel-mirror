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
#include <pablo/builder.hpp>
#include <pablo/pablo_source_kernel.h>
#include <pablo/parser/kernel_signature.h>

namespace pablo {
namespace parse {

class ErrorManager;
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
     * Parses a specific kernel body from a specified source file. In the event
     * of a parse error, `false` will be returned and error information can be
     * retrivied through the parser's `ErrorManager` delegate.
     * 
     * This method only parses the definition of the requested kernel and not
     * the whole source file.
     * 
     * @param sourceFile    A pointer to the source file to parse.
     * @param kernel        An instance of the kernel to populate.
     * @param kernelName    The name of the kernel to parse in the source file.
     * @return `true` iff parse was successful, `false` otherwise.
     */
    virtual bool parseKernel(std::shared_ptr<SourceFile> sourceFile, PabloSourceKernel * kernel, std::string const & kernelName) = 0;

    /**
     * The reverse operation of parse. Writes pablo source code to an output
     * stream by reading a given set kernels.
     */
    virtual void unparse(std::ostream & out, std::vector<PabloKernel *> const & kernels) = 0;

    /**
     * Returns a pointer to this parser's error manager delegate.
     */
    virtual std::shared_ptr<ErrorManager> getErrorManager() const = 0;

    /**
     * Convenience parse method. Takes a filename instead of a `SourceFile`.
     * 
     * @param filename      The name of the file to open and parse.
     * @param kernel        An instance of the kernel to populate.
     * @param kernelName    The name of the kernel to parse.
     * @return `true` iff parse was successful, `false` otherwise.
     */
    virtual bool parseKernel(std::string const & filename, PabloSourceKernel * kernel, std::string const & kernelName);

};

} // namespace pablo::parse
} // namespace pablo
