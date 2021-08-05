/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/unicode/UCD_property_kernel.h>

#include <kernel/core/kernel.h>
#include <re/adt/re_name.h>
#include <re/cc/cc_compiler.h>
#include <re/cc/cc_compiler_target.h>
#include <re/ucd/ucd_compiler.hpp>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <llvm/Support/ErrorHandling.h>

using namespace kernel;
using namespace pablo;
using namespace cc;

UnicodePropertyKernelBuilder::UnicodePropertyKernelBuilder(BuilderRef iBuilder, re::Name * property_value_name, StreamSet * Source, StreamSet * property)
: PabloKernel(iBuilder,
"UCD:" + std::to_string(Source->getNumElements()) + "x" + std::to_string(Source->getFieldWidth()) + getStringHash(property_value_name->getFullName()),
{Binding{"source", Source}},
{Binding{"property_stream", property}}),
  mName(property_value_name) {

}

void UnicodePropertyKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = std::make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = std::make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("source"));
    }
    UCD::UCDCompiler unicodeCompiler(*ccc.get(), pb);
    pablo::Var * propertyVar = pb.createVar(mName->getFullName(), pb.createZeroes());
    re::RE * property_defn = mName->getDefinition();
    if (re::CC * propertyCC = llvm::dyn_cast<re::CC>(property_defn)) {
        unicodeCompiler.addTarget(propertyVar, propertyCC);
    } else if (re::PropertyExpression * pe = llvm::dyn_cast<re::PropertyExpression>(property_defn)) {
        if (pe->getKind() == re::PropertyExpression::Kind::Codepoint) {
            re::CC * propertyCC = llvm::cast<re::CC>(pe->getResolvedRE());
            unicodeCompiler.addTarget(propertyVar, propertyCC);
        }
    }
    unicodeCompiler.compile();
    Var * const property_stream = getOutputStreamVar("property_stream");
    pb.createAssign(pb.createExtract(property_stream, pb.getInteger(0)), propertyVar);
}

