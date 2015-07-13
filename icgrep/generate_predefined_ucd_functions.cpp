/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <cc/cc_compiler.h>
#include <UCD/unicode_set.h>
#include <UCD/PropertyObjectTable.h>
#include <UCD/ucd_compiler.hpp>
#include <pablo/pablo_compiler.h>
#include <pablo/builder.hpp>
#include <pablo/function.h>
#include <llvm/Support/CommandLine.h>
#include <utf_encoding.h>
#include <pablo/optimizers/pablo_simplifier.hpp>
#include <pablo/optimizers/pablo_codesinking.hpp>
#ifdef ENABLE_MULTIPLEXING
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#endif
#include <llvm/IR/Verifier.h>
#include <llvm/Support/Debug.h>
#include <llvm/Support/TargetRegistry.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Target/TargetLibraryInfo.h>
#include <llvm/Target/TargetMachine.h>
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>
#include <llvm/Support/ToolOutputFile.h>
#include <llvm/Pass.h>
#include <llvm/PassManager.h>
#include <llvm/ADT/STLExtras.h>
#include <llvm/Target/TargetSubtargetInfo.h>
#include <llvm/Support/FormattedStream.h>

#include "llvm/Support/FileSystem.h"

#include <boost/algorithm/string/case_conv.hpp>
#include <iostream>

using namespace pablo;
using namespace UCD;
using namespace cc;
using namespace llvm;

inline std::string lowercase(const std::string & name) {
    std::locale loc;
    return boost::algorithm::to_lower_copy(name, loc);
}

static cl::opt<std::string>
OutputFilename("o", cl::desc("Output filename"), cl::value_desc("filename"));

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUnicodeSet
 ** ------------------------------------------------------------------------------------------------------------- */
void compileUnicodeSet(std::string name, const UnicodeSet & set, PabloCompiler & pc, Module * module) {
    PabloFunction function = PabloFunction::Create(std::move(name));
    Encoding encoding(Encoding::Type::UTF_8, 8);
    CC_Compiler ccCompiler(function, encoding);
    UCDCompiler ucdCompiler(ccCompiler);
    PabloBuilder builder(function.getEntryBlock());
    // Build the unicode set function
    ucdCompiler.generateWithDefaultIfHierarchy(set, builder);
    // Optimize it at the pablo level
    Simplifier::optimize(function);
    CodeSinking::optimize(function);
    #ifdef ENABLE_MULTIPLEXING
    AutoMultiplexing::optimize(function);
    #endif
    // Now compile the function ...
    pc.compile(function, module);
    releaseSlabAllocatorMemory();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * generateUCDModule() {
    PabloCompiler pc;
    Module * module = new Module("ucd", getGlobalContext());
    for (PropertyObject * obj : property_object_table) {

        if (isa<UnsupportedPropertyObject>(obj)) continue;

        if (auto * enumObj = dyn_cast<EnumeratedPropertyObject>(obj)) {
            for (const std::string value : *enumObj) {
                const UnicodeSet & set = enumObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[enumObj->getPropertyCode()] + "_" + lowercase(value);
                compileUnicodeSet(name, set, pc, module);
            }
            break;
        }
        else if (auto * extObj = dyn_cast<ExtensionPropertyObject>(obj)) {
            for (const std::string value : *extObj) {
                const UnicodeSet & set = extObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[extObj->getPropertyCode()] + "_" + lowercase(value);
                compileUnicodeSet(name, set, pc, module);
            }
        }
        else if (auto * binObj = dyn_cast<BinaryPropertyObject>(obj)) {
            const UnicodeSet & set = binObj->GetCodepointSet(Binary_ns::Y);
            std::string name = "__get_" + property_enum_name[binObj->getPropertyCode()] + "_y";
            compileUnicodeSet(name, set, pc, module);
        }
    }

    // Print an error message if our module is malformed in any way.
    verifyModule(*module, &dbgs());

    return module;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
void compileUCDModule(Module * module) {
    Triple TheTriple;

    if (TheTriple.getTriple().empty()) {
        TheTriple.setTriple(sys::getDefaultTargetTriple());
    }

    // Get the target specific parser.
    std::string msg;
    const Target * TheTarget = TargetRegistry::lookupTarget(TheTriple.getTriple(), msg);

    if (TheTarget == nullptr) {
        throw std::runtime_error(msg);
    }

    auto MCPU = llvm::sys::getHostCPUName();

    TargetOptions Options;

    std::unique_ptr<TargetMachine> Target(
                TheTarget->createTargetMachine(TheTriple.getTriple(), MCPU, "", Options,
                                               Reloc::Default, CodeModel::Small, CodeGenOpt::Aggressive));

    if (Target == nullptr) {
        throw std::runtime_error("Could not allocate target machine!");
    }

    #ifdef USE_LLVM_3_5
    std::string error;
    std::unique_ptr<tool_output_file> Out = make_unique<tool_output_file>(OutputFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    std::error_code error;
    std::unique_ptr<tool_output_file> Out = make_unique<tool_output_file>(OutputFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    // Build up all of the passes that we want to do to the module.
    PassManager PM;

    // Add an appropriate TargetLibraryInfo pass for the module's triple.
    TargetLibraryInfo * TLI = new TargetLibraryInfo(TheTriple);

    PM.add(TLI);

    // Add the target data from the target machine, if it exists, or the module.
    #ifdef USE_LLVM_3_5
    const DataLayout * DL = Target->getDataLayout();
    #else
    const DataLayout * DL = Target->getSubtargetImpl()->getDataLayout();
    #endif
    if (DL) {
        module->setDataLayout(DL);
    }
    PM.add(new DataLayoutPass());

    formatted_raw_ostream FOS(Out->os());
    // Ask the target to add backend passes as necessary.
    if (Target->addPassesToEmitFile(PM, FOS, TargetMachine::CGFT_ObjectFile)) {
        throw std::runtime_error("Target does not support generation of this file type!\n");
    }

    PM.run(*module);

    Out->keep();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief main
 ** ------------------------------------------------------------------------------------------------------------- */
int main(int argc, char *argv[]) {
    cl::ParseCommandLineOptions(argc, argv, "UCD Compiler\n");
    Module * module = generateUCDModule();
    compileUCDModule(module);
    return 0;
}
