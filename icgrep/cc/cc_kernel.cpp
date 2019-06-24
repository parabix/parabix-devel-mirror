/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "cc_kernel.h"
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <kernels/kernel_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;

// ccNameStr, std::vector<re::CC *>{cc}, ByteStream, ccStream

CharacterClassKernelBuilder::CharacterClassKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, std::vector<re::CC *> charClasses, StreamSet * sourceStream, StreamSet * ccStream, Scalar * signalNullObject)
: PabloKernel(b, ccSetName + std::to_string(sourceStream->getNumElements()) + "x" + std::to_string(sourceStream->getFieldWidth())
                           + (signalNullObject ? "_abort_on_null" : ""),
// input
{Binding{"sourceStream", sourceStream}},
// output
{Binding{"ccStream", ccStream}},
makeInputScalarBindings(signalNullObject),
{}), mCharClasses(std::move(charClasses)), mAbortOnNull(signalNullObject != nullptr) {
    if (LLVM_UNLIKELY(ccStream->getNumElements() != mCharClasses.size())) {
        report_fatal_error("cc streamset must have " + std::to_string(mCharClasses.size()) + " streams");
    }
    if (mAbortOnNull) {
        addAttribute(CanTerminateEarly());
        addAttribute(MayFatallyTerminate());
    }
}

Bindings CharacterClassKernelBuilder::makeInputScalarBindings(Scalar * signalNullObject) {
    if (signalNullObject) {
        return {Binding{"handler_address", signalNullObject}};
    } else {
        return {};
    }
}

void CharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    bool useDirectCC = getInput(0)->getType()->getArrayNumElements() == 1;
    if (useDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("sourceStream"));
    }
    Var * outputVar = getOutputStreamVar("ccStream");
    if (mAbortOnNull) {
        pb.createTerminateAt(ccc->compileCC(makeCC(0, &cc::Byte)), pb.getInteger(0));
    }
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        PabloAST * cc = ccc->compileCC(mCharClasses[i]);
        pb.createAssign(pb.createExtract(outputVar, i), cc);
    }
}
