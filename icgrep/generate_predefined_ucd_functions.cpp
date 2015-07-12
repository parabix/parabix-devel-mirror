/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <string>
#include <iostream>
#include <fstream>

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
#include <pablo/optimizers/pablo_automultiplexing.hpp>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <boost/algorithm/string/case_conv.hpp>

#include <iostream>

using namespace pablo;
using namespace UCD;
using namespace cc;

inline std::string lowercase(const std::string & name) {
    std::locale loc;
    return boost::algorithm::to_lower_copy(name, loc);
}

ExecutionEngine * compile(std::string name, const UnicodeSet & set, PabloCompiler & pc, ExecutionEngine * engine) {

    PabloFunction function = PabloFunction::Create(std::move(name));
    Encoding encoding(Encoding::Type::UTF_8, 8);
    CC_Compiler ccCompiler(function, encoding);
    UCDCompiler ucdCompiler(ccCompiler);
    PabloBuilder builder(function.getEntryBlock());

    std::cerr << "Compiling " << name << std::endl;

    // Build the unicode set function
    ucdCompiler.generateWithDefaultIfHierarchy(set, builder);
    // Optimize it at the pablo level
    Simplifier::optimize(function);
    CodeSinking::optimize(function);
    // AutoMultiplexing::optimize(function);


    if (engine) {
        engine->removeModule(pc.getModule());
    }

    // Now compile the function ...
    return pc.compile(function).getExecutionEngine();
}

int main(int argc, char *argv[]) {

    PabloCompiler pc;
    ExecutionEngine * engine = nullptr;

    for (PropertyObject * obj : property_object_table) {

        if (isa<UnsupportedPropertyObject>(obj)) continue;

        if (auto * enumObj = dyn_cast<EnumeratedPropertyObject>(obj)) {
            for (const std::string value : *enumObj) {
                const UnicodeSet & set = enumObj->GetCodepointSet(canonicalize_value_name(value));
                engine = compile("__get_" + property_enum_name[enumObj->getPropertyCode()] + "_" + value, set, pc, engine);
            }
            break;
        }

//        if (auto * extObj = dyn_cast<ExtensionPropertyObject>(obj)) {
//            for (const std::string value : *extObj) {
//                const UnicodeSet & set = extObj->GetCodepointSet(canonicalize_value_name(value));
//                engine = compile("__get_" + property_enum_name[extObj->getPropertyCode()] + "_" + value, set, pc, engine);
//            }
//        }

//        if (auto * binObj = dyn_cast<BinaryPropertyObject>(obj)) {
//            const UnicodeSet & set = binObj->GetCodepointSet(Binary_ns::Y);
//            compile("__get_" + property_enum_name[binObj->getPropertyCode()] + "_Y", set, pc);
//        }
    }

    pc.getModule()->dump();

    return 0;
}
