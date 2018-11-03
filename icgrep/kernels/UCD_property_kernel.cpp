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

using namespace kernel;
using namespace pablo;


UnicodePropertyKernelBuilder::UnicodePropertyKernelBuilder(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, re::Name * property_value_name, StreamSet *BasisBits, StreamSet * property)
: PabloKernel(iBuilder,
"UCD:" + property_value_name->getFullName(),
{Binding{"basis", BasisBits}},
{Binding{"property_stream", property}}),
  mName(property_value_name) {

}

void UnicodePropertyKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    UCD::UCDCompiler ucdCompiler(ccc);
    UCD::UCDCompiler::NameMap nameMap;
    nameMap.emplace(mName, nullptr);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    auto f = nameMap.find(mName);
    if (f == nameMap.end()) llvm::report_fatal_error("Unknown property");
    PabloAST * theStream = f-> second;
    Var * const property_stream = getOutputStreamVar("property_stream");
    pb.createAssign(pb.createExtract(property_stream, pb.getInteger(0)), theStream);
}

