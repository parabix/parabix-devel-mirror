/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "UCD_property_kernel.h"
#include <re/re_toolchain.h>
#include <re/re_name.h>
#include <cc/cc_compiler.h>
#include <UCD/ucd_compiler.hpp>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>

using namespace kernel;
using namespace pablo;
using namespace boost::archive::iterators;

std::string base64(std::string to_encode) {
    typedef base64_from_binary< transform_width<std::string::const_iterator, 6, 8> > base64_t;
    return std::string(base64_t(to_encode.begin()), base64_t(to_encode.end()));
}


UnicodePropertyKernelBuilder::UnicodePropertyKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::Name * property_value_name, StreamSet * Source, StreamSet * property)
: PabloKernel(iBuilder,
"UCD:" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()) + base64(property_value_name->getFullName().c_str()),
{Binding{"source", Source}},
{Binding{"property_stream", property}}),
  mName(property_value_name) {

}

void UnicodePropertyKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = llvm::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = llvm::make_unique<cc::Parabix_CC_Compiler>(getEntryScope(), getInputStreamSet("source"));
    }
    UCD::UCDCompiler ucdCompiler(*ccc.get());
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(mName, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(mName);
    if (f == nameMap.end()) llvm::report_fatal_error("Unknown property");
    PabloAST * theStream = f-> second;
    Var * const property_stream = getOutputStreamVar("property_stream");
    pb.createAssign(pb.createExtract(property_stream, pb.getInteger(0)), theStream);
}

