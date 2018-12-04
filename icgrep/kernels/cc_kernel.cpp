/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "cc_kernel.h"
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

// ccNameStr, std::vector<re::CC *>{cc}, ByteStream, ccStream

DirectCharacterClassKernelBuilder::DirectCharacterClassKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, std::vector<re::CC *> charClasses, StreamSet * byteStream, StreamSet * ccStream, Scalar * signalNullObject)
: PabloKernel(b, ccSetName + (signalNullObject ? "_direct_abort_on_null" : "_direct"),
// input
{Binding{"byteStream", byteStream}},
// output
{Binding{"ccStream", ccStream}},
makeInputScalarBindings(signalNullObject),
{}), mCharClasses(std::move(charClasses)), mAbortOnNull(signalNullObject != nullptr) {
    if (LLVM_UNLIKELY(ccStream->getNumElements() != mCharClasses.size())) {
        report_fatal_error("cc streamset must have " + std::to_string(mCharClasses.size()) + " streams");
    }
    if (mAbortOnNull) {
        addAttribute(CanTerminateEarly());
    }
}

Bindings DirectCharacterClassKernelBuilder::makeInputScalarBindings(Scalar * signalNullObject) {
    if (signalNullObject) {
        return {Binding{"handler_address", signalNullObject}};
    } else {
        return {};
    }
}

void DirectCharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Direct_CC_Compiler ccc(getEntryScope(), getInputStreamSet("byteStream")[0]);
    Var * outputVar = getOutputStreamVar("ccStream");
    PabloAST * nonNull = nullptr;
    if (mAbortOnNull) {
        PabloAST * nullCC = pb.createTerminateAt(ccc.compileCC(makeCC(0, &cc::Byte)), pb.getInteger(0));
        nonNull = pb.createNot(nullCC);
    }
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        PabloAST * cc = ccc.compileCC(mCharClasses[i]);
        if (mAbortOnNull) {
            cc = pb.createAnd(nonNull, cc);
        }
        pb.createAssign(pb.createExtract(outputVar, i), cc);
    }
}


ParabixCharacterClassKernelBuilder::ParabixCharacterClassKernelBuilder (const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, const std::vector<CC *> & charClasses, StreamSet * basisStream, StreamSet * outputStream)
: PabloKernel(b, ccSetName +"_kernel",
// stream inputs
{Binding{"basis", basisStream}}
// stream outputs
, {Binding("outputStream", outputStream)}
)
, mCharClasses(charClasses) {
    if (LLVM_UNLIKELY(outputStream->getNumElements() != mCharClasses.size())) {
        report_fatal_error("output streamset must have " + std::to_string(mCharClasses.size()) + " streams");
    }
}

void ParabixCharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    Var * outputVar = getOutputStreamVar("outputStream");
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        pb.createAssign(pb.createExtract(outputVar, i), ccc.compileCC(mCharClasses[i]));
    }
}
