/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <pablo/parse/pablo_source_kernel.h>

#include <llvm/Support/raw_ostream.h>
#include <pablo/printer_pablos.h>
#include <pablo/parse/error.h>
#include <pablo/parse/parser.h>
#include <pablo/parse/source_file.h>
#include <pablo/parse/stateless_parser.h>

namespace pablo {

// void PabloSourceKernel::generatePabloMethod() {
//     bool parseSuccessful = mParser->parseKernel(mSource, this, mKernelName);
//     if (!parseSuccessful) {
//         auto em = mParser->getErrorManager();
//         em->dumpErrors();
//         llvm::report_fatal_error("parse error", false);
//     }
// }

// PabloSourceKernel::PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
//                                      std::shared_ptr<parse::PabloParser> parser,
//                                      std::shared_ptr<parse::SourceFile> sourceFile,
//                                      std::string const & kernelName,
//                                      kernel::Bindings inputStreamBindings,
//                                      kernel::Bindings outputStreamBindings,
//                                      kernel::Bindings inputScalarBindings,
//                                      kernel::Bindings outputScalarBindings)
// : PabloKernel(builder, std::string(kernelName), inputStreamBindings, outputStreamBindings, inputScalarBindings, outputScalarBindings)
// , mParser(parser)
// , mSource(sourceFile)
// , mKernelName(kernelName)
// {}

// PabloSourceKernel::PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
//                                      std::shared_ptr<parse::PabloParser> parser,
//                                      std::string const & sourceFile,
//                                      std::string const & kernelName,
//                                      kernel::Bindings inputStreamBindings,
//                                      kernel::Bindings outputStreamBindings,
//                                      kernel::Bindings inputScalarBindings,
//                                      kernel::Bindings outputScalarBindings)
// : PabloKernel(builder, std::string(kernelName), inputStreamBindings, outputStreamBindings, inputScalarBindings, outputScalarBindings)
// , mParser(parser)
// , mSource(new parse::SourceFile(sourceFile))
// , mKernelName(kernelName)
// {}

PabloSourceKernel::PabloSourceKernel(std::unique_ptr<kernel::KernelBuilder> const & builder,
                                     std::shared_ptr<parse::SourceFile> sourceFile,
                                     std::string const & kernelName,
                                     std::vector<kernel::Relationship *> const & inputArguments,
                                     std::vector<kernel::Relationship *> const & outputArguments)
: PabloKernel(builder, std::string(kernelName))
, mSource(std::move(sourceFile))
, mKernelName(kernelName)
, mErrorManager(parse::ErrorManager::Create())
, mParserState()
{
    auto parseState = parse::Parser::constructKernelSignature(this, mKernelName, mSource, mErrorManager, inputArguments, outputArguments);
    if (parseState == nullptr) {
        mErrorManager->dumpErrors();
        llvm::report_fatal_error("parse error");
    }
    mParserState.swap(parseState);
}

void PabloSourceKernel::generatePabloMethod() {
    PabloBuilder builder(getEntryScope());
    mParserState->setBuilder(&builder);
    bool parseSuccessful = parse::Parser::constructKernelBody(*mParserState);
    if (!parseSuccessful) {
        mErrorManager->dumpErrors();
        llvm::report_fatal_error("parse error");
    }
    // `mParserState` will outlive `builder` so we want to remove its pointer to it
    mParserState->setBuilder(nullptr);
}

} // namespace pablo
