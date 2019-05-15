/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <istream>
#include <ostream>
#include <pablo/pabloAST.h>
#include <pablo/pablo_kernel.h>

namespace pablo {

/**
 * Abstract interface for all pablo parser implementations.
 * 
 * The two primary methods, parse and unparse, must be inverse operations in
 * all implementations (i.e., parse(unparse(X)) == X for all valid X).
 */
class PabloParser {
public:

    /**
     * Constructs a PabloKernel instance by parsing an input stream.
     * 
     * @param in a input stream to read from
     * @return a PabloKernel instance or nullptr in the event of an error
     */
    virtual std::unique_ptr<PabloKernel> parse(std::istream & in) = 0;

    /**
     * The reverse operation of parse. Writes pablo source code to an output
     * stream by reading a given kernel.
     */
    virtual void unparse(std::ostream & out, PabloKernel * kernel) = 0;

    /**
     * Convenience parse method which accepts a filename to parse instead of
     * an input stream.
     * 
     * @param filename the file to parse
     * @return a PabloKernel instance or nullptr in the event of an error
     */
    std::unique_ptr<PabloKernel> parse(std::string const & filename);

};

} // namespace pablo
