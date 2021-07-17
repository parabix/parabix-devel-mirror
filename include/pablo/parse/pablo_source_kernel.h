/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#pragma once

#include <memory>
#include <vector>
#include <pablo/pablo_kernel.h>

namespace pablo {

namespace parse {
    class PabloParser;
    class SourceFile;
}

class PabloSourceKernel final : public PabloKernel {
public:
    PabloSourceKernel(BuilderRef builder,
                      std::shared_ptr<parse::PabloParser> parser,
                      std::shared_ptr<parse::SourceFile> sourceFile,
                      std::string const & kernelName,
                      kernel::Bindings inputStreamBindings = {},
                      kernel::Bindings outputStreamBindings = {},
                      kernel::Bindings inputScalarBindings = {},
                      kernel::Bindings outputScalarBindings = {});

    PabloSourceKernel(BuilderRef builder,
                      std::shared_ptr<parse::PabloParser> parser,
                      std::string const & sourceFile,
                      std::string const & kernelName,
                      kernel::Bindings inputStreamBindings,
                      kernel::Bindings outputStreamBindings,
                      kernel::Bindings inputScalarBindings,
                      kernel::Bindings outputScalarBindings);

private:
    void generatePabloMethod() override;

    std::shared_ptr<parse::PabloParser>     mParser;
    std::shared_ptr<parse::SourceFile>      mSource;
    std::string                             mKernelName;
};

} // namespace pablo
