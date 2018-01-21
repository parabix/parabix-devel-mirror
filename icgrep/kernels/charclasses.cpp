/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "charclasses.h"
#include <re/re_toolchain.h>
#include <kernels/kernel_builder.h>
#include <UCD/ucd_compiler.hpp>
#include <cc/cc_compiler.h>
#include <re/re_name.h>
#include <boost/uuid/sha1.hpp>
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

inline static std::string sha1sum(const std::string & str) {
    char buffer[41];    // 40 hex-digits and the terminating null
    uint32_t digest[5]; // 160 bits in total
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(str.c_str(), str.size());
    sha1.get_digest(digest);
    snprintf(buffer, sizeof(buffer), "%.8x%.8x%.8x%.8x%.8x",
             digest[0], digest[1], digest[2], digest[3], digest[4]);
    return std::string(buffer);
}

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

CharClassesSignature::CharClassesSignature(const std::vector<CC *> &ccs)
: mSignature(signature(ccs)) {

}


CharClassesKernel::CharClassesKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<CC *> && ccs)
: CharClassesSignature(ccs)
, PabloKernel(iBuilder,
              "cc" + sha1sum(mSignature),
              {Binding{iBuilder->getStreamSetTy(8), "basis"}},
              {Binding{iBuilder->getStreamSetTy(ccs.size(), 1), "charclasses"}})
, mCCs(std::move(ccs)) {

}

std::string CharClassesKernel::makeSignature(const std::unique_ptr<kernel::KernelBuilder> &) {
    return mSignature;
}

void CharClassesKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());

    CC_Compiler ccc(this, getInput(0));
    unsigned n = mCCs.size();

    NameMap nameMap;
    std::vector<Name *> names;
    for (unsigned i = 0; i < n; i++) {
        Name * name = re::makeName("mpx_basis" + std::to_string(i), mCCs[i]);
        nameMap.emplace(name, nullptr);
        names.push_back(name);
    }

    UCD::UCDCompiler ucdCompiler(ccc);
    if (LLVM_UNLIKELY(AlgorithmOptionIsSet(DisableIfHierarchy))) {
        ucdCompiler.generateWithoutIfHierarchy(nameMap, pb);
    } else {
        ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    }

    // The first UnicodeSet in the vector ccs represents the last bit of the character class basis bit streams.
    std::reverse(names.begin(), names.end());
    for (unsigned i = 0; i < names.size(); i++) {
        auto t = nameMap.find(names[i]); 
        if (t != nameMap.end()) {
            PabloAST * const r = pb.createExtract(getOutput(0), pb.getInteger(i));
            pb.createAssign(r, pb.createInFile(t->second));
        } else {
            llvm::report_fatal_error("Can't compile character classes.");
        }
    }
}


