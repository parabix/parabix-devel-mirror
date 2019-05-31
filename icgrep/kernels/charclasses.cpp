/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "charclasses.h"
#include <re/re_toolchain.h>
#include <kernels/kernel_builder.h>
#include <UCD/ucd_compiler.hpp>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <re/re_name.h>
#include <pablo/builder.hpp>
#include <llvm/Support/ErrorHandling.h>
#include <llvm/Support/raw_ostream.h>

using NameMap = UCD::UCDCompiler::NameMap;

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;
using namespace UCD;

inline std::string signature(const std::vector<re::CC *> & ccs) {
    if (LLVM_UNLIKELY(ccs.empty())) {
        return "[]";
    } else {
        std::string tmp;
        raw_string_ostream out(tmp);
        char joiner = '[';
        for (const auto & set : ccs) {
            out << joiner;
            set->print(out);
            joiner = ',';
        }
        out << ']';
        return out.str();
    }
}

CharClassesSignature::CharClassesSignature(const std::vector<CC *> &ccs, bool useDirectCC)
: mUseDirectCC(useDirectCC),
  mSignature((useDirectCC ? "d" : "p") + signature(ccs)) {
}


CharClassesKernel::CharClassesKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<CC *> && ccs, StreamSet * BasisBits, StreamSet * CharClasses)
: CharClassesSignature(ccs, BasisBits->getNumElements() == 1)
, PabloKernel(iBuilder, "cc" + getStringHash(mSignature), {Binding{"basis", BasisBits}}, {Binding{"charclasses", CharClasses}})
, mCCs(std::move(ccs)) {

}

std::string CharClassesKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) const {
    return mSignature;
}

void CharClassesKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    if (mUseDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    unsigned n = mCCs.size();

    NameMap nameMap;
    std::vector<Name *> names;
    for (unsigned i = 0; i < n; i++) {
        Name * name = re::makeName("mpx_basis" + std::to_string(i), mCCs[i]);
        nameMap.emplace(name, nullptr);
        names.push_back(name);
    }

    UCD::UCDCompiler ucdCompiler(*ccc.get());
    if (LLVM_UNLIKELY(AlgorithmOptionIsSet(DisableIfHierarchy))) {
        ucdCompiler.generateWithoutIfHierarchy(nameMap, pb);
    } else {
        ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    }
    for (unsigned i = 0; i < names.size(); i++) {
        auto t = nameMap.find(names[i]);
        if (t != nameMap.end()) {
            Extract * const r = pb.createExtract(getOutput(0), pb.getInteger(i));
            pb.createAssign(r, pb.createInFile(t->second));
        } else {
            llvm::report_fatal_error("Can't compile character classes.");
        }
    }
}


ByteClassesKernel::ByteClassesKernel(const std::unique_ptr<kernel::KernelBuilder> &iBuilder,
                                     std::vector<re::CC *> && ccs,
                                     StreamSet * inputStream,
                                     StreamSet * CharClasses):
CharClassesSignature(ccs, inputStream->getNumElements() == 1)
, PabloKernel(iBuilder, "ByteClassesKernel_" + getStringHash(mSignature), {Binding{"basis", inputStream}}, {Binding{"charclasses", CharClasses}})
, mCCs(std::move(ccs)) {

}

std::string ByteClassesKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) const {
    return mSignature;
}

void ByteClassesKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;
    if (mUseDirectCC) {
        ccc = make_unique<cc::Direct_CC_Compiler>(getEntryScope(), pb.createExtract(getInput(0), pb.getInteger(0)));
    } else {
        ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), getInputStreamSet("basis"));
    }
    unsigned n = mCCs.size();

    NameMap nameMap;
    std::vector<Name *> names;
    for (unsigned i = 0; i < n; i++) {
        Name * name = re::makeName("mpx_basis" + std::to_string(i), mCCs[i]);

        nameMap.emplace(name, ccc->compileCC(mCCs[i]));
        names.push_back(name);

    }
    for (unsigned i = 0; i < names.size(); i++) {
        auto t = nameMap.find(names[i]);
        if (t != nameMap.end()) {
            Extract * const r = pb.createExtract(getOutput(0), pb.getInteger(i));
            pb.createAssign(r, pb.createInFile(t->second));
        } else {
            llvm::report_fatal_error("Can't compile character classes.");
        }
    }
}
