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
#include <llvm/Transforms/Scalar.h>
#include <iostream>

using namespace pablo;
using namespace UCD;
using namespace cc;
using namespace llvm;

static cl::opt<std::string>
ObjectFilename("o", cl::desc("Output object filename"), cl::value_desc("filename"), cl::Required);

static cl::opt<std::string>
UCDSourcePath("dir", cl::desc("UCD source code directory"), cl::value_desc("directory"), cl::Required);

#ifdef ENABLE_MULTIPLEXING
static cl::opt<bool> EnableMultiplexing("multiplexing", cl::init(false),
                                        cl::desc("combine Advances whose inputs are mutual exclusive into the fewest number of advances possible (expensive)."));
#endif

using property_list = std::vector<std::pair<std::string, size_t>>;

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUnicodeSet
 ** ------------------------------------------------------------------------------------------------------------- */
size_t compileUnicodeSet(std::string name, const UnicodeSet & set, PabloCompiler & pc, Module * module) {
    PabloFunction function = PabloFunction::Create(std::move(name), 8, 1);
    Encoding encoding(Encoding::Type::UTF_8, 8);
    CC_Compiler ccCompiler(function, encoding);
    UCDCompiler ucdCompiler(ccCompiler);
    PabloBuilder builder(function.getEntryBlock());
    // Build the unicode set function
    PabloAST * target = ucdCompiler.generateWithDefaultIfHierarchy(set, builder);
    function.setResult(0, builder.createAssign("matches", target));
    // Optimize it at the pablo level
    Simplifier::optimize(function);
    CodeSinking::optimize(function);
    #ifdef ENABLE_MULTIPLEXING
    if (EnableMultiplexing) {
        AutoMultiplexing::optimize(function);
        Simplifier::optimize(function);
    }
    #endif
    // Now compile the function ...
    auto func = pc.compile(function, module);
    releaseSlabAllocatorMemory();

    return func.second;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief writePropertyInstaller
 ** ------------------------------------------------------------------------------------------------------------- */

void writePrecompiledProperties(property_list && properties) {

    const std::string headerFilename = UCDSourcePath + "/precompiled_properties.h";
    #ifdef USE_LLVM_3_5
    std::string error;
    raw_fd_ostream header(headerFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    std::error_code error;
    raw_fd_ostream header(headerFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    header << "#ifndef PRECOMPILED_PROPERTIES\n";
    header << "#define PRECOMPILED_PROPERTIES\n\n";
    header << "#include <string>\n\n";
    header << "#include <tuple>\n";
    header << "namespace UCD {\n\n";
    header << "using ExternalProperty = std::tuple<void *, unsigned, unsigned, size_t>;\n\n";
    header << "const ExternalProperty & resolveExternalProperty(const std::string & name);\n\n";
    header << "}\n\n";
    header << "#endif\n";
    header.close();

    const std::string cppFilename = UCDSourcePath + "/precompiled_properties.cpp";
    #ifdef USE_LLVM_3_5
    raw_fd_ostream cpp(cppFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    raw_fd_ostream cpp(cppFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    cpp << "#include \"precompiled_properties.h\"\n";
    cpp << "#include <include/simd-lib/bitblock.hpp>\n";
    cpp << "#include <stdexcept>\n";
    cpp << "#include <unordered_map>\n\n";
    cpp << "namespace UCD {\n\n";
    cpp << "struct Input {\n    BitBlock bit[8];\n};\n\n";
    cpp << "struct Output {\n    BitBlock bit[1];\n};\n\n";
    for (auto prop : properties) {
        cpp << "extern \"C\" void " + prop.first + "(const Input &, BitBlock *, Output &);\n";
    }

    cpp << "\nconst static std::unordered_map<std::string, ExternalProperty> ExternalPropertyMap = {\n";
    for (auto itr = properties.begin(); itr != properties.end(); ) {
        cpp << "    {\"" + itr->first + "\", std::make_tuple(reinterpret_cast<void *>(&" + itr->first + "), 8, 1, " + std::to_string(itr->second) + ")}";
        if (++itr != properties.end()) {
            cpp << ",";
        }
        cpp << "\n";
    }
    cpp << "};\n\n";

    cpp << "const ExternalProperty & resolveExternalProperty(const std::string & name) {\n";
    cpp << "    auto f = ExternalPropertyMap.find(name);\n";
    cpp << "    if (f == ExternalPropertyMap.end())\n";
    cpp << "        throw std::runtime_error(\"No external property named \\\"\" + name + \"\\\" found!\");\n";
    cpp << "    return f->second;\n";
    cpp << "}\n\n}\n";

    cpp.close();

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
Module * generateUCDModule() {

    property_list properties;

    PabloCompiler pc;
    Module * module = new Module("ucd", getGlobalContext());
    for (PropertyObject * obj : property_object_table) {
        if (EnumeratedPropertyObject * enumObj = dyn_cast<EnumeratedPropertyObject>(obj)) {
            for (const std::string value : *enumObj) {
                const UnicodeSet & set = enumObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[enumObj->getPropertyCode()] + "_" + value;
                properties.emplace_back(name, compileUnicodeSet(name, set, pc, module));
            }
        }
        else if (ExtensionPropertyObject * extObj = dyn_cast<ExtensionPropertyObject>(obj)) {
            for (const std::string value : *extObj) {
                const UnicodeSet & set = extObj->GetCodepointSet(canonicalize_value_name(value));
                std::string name = "__get_" + property_enum_name[extObj->getPropertyCode()] + "_" + value;
                properties.emplace_back(name, compileUnicodeSet(name, set, pc, module));
            }
        }
        else if (BinaryPropertyObject * binObj = dyn_cast<BinaryPropertyObject>(obj)) {
            const UnicodeSet & set = binObj->GetCodepointSet(Binary_ns::Y);
            std::string name = "__get_" + property_enum_name[binObj->getPropertyCode()] + "_Y";
            properties.emplace_back(name, compileUnicodeSet(name, set, pc, module));
        }
    }

    // Print an error message if our module is malformed in any way.
    verifyModule(*module, &dbgs());

    writePrecompiledProperties(std::move(properties));

    return module;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief compileUCDModule
 ** ------------------------------------------------------------------------------------------------------------- */
void compileUCDModule(Module * module) {
    Triple TheTriple;

    // Initialize targets first, so that --version shows registered targets.
    InitializeAllTargets();
    InitializeAllTargetMCs();
    InitializeAllAsmPrinters();
    InitializeAllAsmParsers();

    TheTriple.setTriple(sys::getDefaultTargetTriple());

    // Get the target specific parser.
    std::string msg;
    const Target * TheTarget = TargetRegistry::lookupTarget(TheTriple.getTriple(), msg);
    if (TheTarget == nullptr) {
        throw std::runtime_error(msg);
    }

    TargetOptions Options;

    std::unique_ptr<TargetMachine> Target(
                TheTarget->createTargetMachine(TheTriple.getTriple(), sys::getHostCPUName(), "", Options,
                                               Reloc::Default, CodeModel::Small, CodeGenOpt::Aggressive));

    if (Target == nullptr) {
        throw std::runtime_error("Could not allocate target machine!");
    }

    #ifdef USE_LLVM_3_5
    std::string error;
    std::unique_ptr<tool_output_file> out = make_unique<tool_output_file>(ObjectFilename.c_str(), error, sys::fs::F_None);
    if (!error.empty()) {
        throw std::runtime_error(error);
    }
    #else
    std::error_code error;
    std::unique_ptr<tool_output_file> out = make_unique<tool_output_file>(ObjectFilename, error, sys::fs::F_None);
    if (error) {
        throw std::runtime_error(error.message());
    }
    #endif

    // Build up all of the passes that we want to do to the module.
    PassManager PM;

    // Add an appropriate TargetLibraryInfo pass for the module's triple.
    PM.add(new TargetLibraryInfo(TheTriple));

    // Add the target data from the target machine, if it exists, or the module.
    #ifdef USE_LLVM_3_5
    const DataLayout * DL = Target->getDataLayout();
    #else
    const DataLayout * DL = Target->getSubtargetImpl()->getDataLayout();
    #endif
    if (DL) {
        module->setDataLayout(DL);
    }
    #ifdef USE_LLVM_3_5
    PM.add(new DataLayoutPass(module));
    #else
    PM.add(new DataLayoutPass());
    #endif
    PM.add(createReassociatePass());
    PM.add(createInstructionCombiningPass());
    PM.add(createSinkingPass());

    formatted_raw_ostream FOS(out->os());
    // Ask the target to add backend passes as necessary.
    if (Target->addPassesToEmitFile(PM, FOS, TargetMachine::CGFT_ObjectFile)) {
        throw std::runtime_error("Target does not support generation of object file type!\n");
    }

    PM.run(*module);

    out->keep();
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
