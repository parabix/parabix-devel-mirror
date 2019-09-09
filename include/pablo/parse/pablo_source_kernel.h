/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <memory>
#include <vector>
#include <pablo/pablo_kernel.h>
#include <pablo/parse/error.h>
#include <pablo/parse/parser.h>
#include <pablo/parse/stateless_parser.h>

namespace pablo {

namespace parse {
    class SourceFile;
}

class PabloSourceKernel final : public PabloKernel {
public:
    // PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
    //                   std::shared_ptr<parse::PabloParser> parser,
    //                   std::shared_ptr<parse::SourceFile> sourceFile,
    //                   std::string const & kernelName,
    //                   kernel::Bindings inputStreamBindings = {},
    //                   kernel::Bindings outputStreamBindings = {},
    //                   kernel::Bindings inputScalarBindings = {},
    //                   kernel::Bindings outputScalarBindings = {});

    // PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
    //                   std::shared_ptr<parse::PabloParser> parser,
    //                   std::string const & sourceFile,
    //                   std::string const & kernelName,
    //                   kernel::Bindings inputStreamBindings,
    //                   kernel::Bindings outputStreamBindings,
    //                   kernel::Bindings inputScalarBindings,
    //                   kernel::Bindings outputScalarBindings);

    PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
                      std::shared_ptr<parse::SourceFile> sourceFile,
                      std::string const & kernelName,
                      std::vector<kernel::Relationship *> const & inputArguments,
                      std::vector<kernel::Relationship *> const & outputArguments);

    bool isCachable() const override {
        return true;
    }

    bool hasSignature() const override {
        return false;
    }
private:
    void generatePabloMethod() override;

    // std::shared_ptr<parse::PabloParser>  mParser;
    std::shared_ptr<parse::SourceFile>   mSource;
    std::string                          mKernelName;
    std::shared_ptr<parse::ErrorManager> mErrorManager;
    std::unique_ptr<parse::ParserState>  mParserState;
};

} // namespace pablo
