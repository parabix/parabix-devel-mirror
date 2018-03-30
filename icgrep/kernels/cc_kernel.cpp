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

DirectCharacterClassKernelBuilder::DirectCharacterClassKernelBuilder(
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, std::vector<re::CC *> charClasses)
: PabloKernel(b, ccSetName +"_direct",
              {Binding{b->getStreamSetTy(1, 8), "byteStream"}},
              {Binding{b->getStreamSetTy(charClasses.size(), 1), "ccStream"}})
, mCharClasses(charClasses) {
}

void DirectCharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Direct_CC_Compiler ccc(getEntryScope(), getInputStreamSet("byteStream")[0]);
    Var * outputVar = getOutputStreamVar("ccStream");
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        pb.createAssign(pb.createExtract(outputVar, i), ccc.compileCC(mCharClasses[i]));
    }
}


ParabixCharacterClassKernelBuilder::ParabixCharacterClassKernelBuilder (
        const std::unique_ptr<kernel::KernelBuilder> & b, std::string ccSetName, const std::vector<CC *> & charClasses, unsigned codeUnitWidth)
: PabloKernel(b, ccSetName +"_kernel",
// stream inputs
{Binding{b->getStreamSetTy(codeUnitWidth), "basis"}}
// stream outputs
, {Binding(b->getStreamSetTy((unsigned int)charClasses.size()), "outputStream")}
)
, mCharClasses(charClasses) {

}

void ParabixCharacterClassKernelBuilder::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    Var * outputVar = getOutputStreamVar("outputStream");
    for (unsigned i = 0; i < mCharClasses.size(); ++i) {
        pb.createAssign(pb.createExtract(outputVar, i), ccc.compileCC(mCharClasses[i]));
    }
}
