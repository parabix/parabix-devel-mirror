/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "charclasses.h"
#if BOOST_VERSION < 106600
#include <boost/uuid/sha1.hpp>
#else
#include <boost/uuid/detail/sha1.hpp>
#endif
#include <re/re_toolchain.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_count.h>
#include <UCD/resolve_properties.h>
#include <UCD/ucd_compiler.hpp>
#include <re/re_cc.h>
#include <cc/cc_compiler.h>
#include <re/re_name.h>
#include <llvm/Support/raw_ostream.h>

using NameMap = UCD::UCDCompiler::NameMap;

using namespace cc;
using namespace kernel;
using namespace pablo;
using namespace re;
using namespace llvm;
using namespace UCD;

CharClassesKernel::CharClassesKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, std::vector<UCD::UnicodeSet> multiplexedCCs)
: PabloKernel(iBuilder,
              "cc",
              {Binding{iBuilder->getStreamSetTy(8), "basis"}},
              {Binding{iBuilder->getStreamSetTy(multiplexedCCs.size(), 1), "charclasses"}}) 
, mMultiplexedCCs(multiplexedCCs) {

}

void CharClassesKernel::generatePabloMethod() {
    CC_Compiler ccc(this, getInput(0));
    auto & pb = ccc.getBuilder();
    unsigned n = mMultiplexedCCs.size();

    NameMap nameMap;
    std::vector<Name *> names;
    for (unsigned i = 0; i < n; i++) {
        Name * name = re::makeName("cc" + std::to_string(i), makeCC(std::move(mMultiplexedCCs[i])));
        nameMap.emplace(name, nullptr);
        names.push_back(name);
    }

    UCD::UCDCompiler ucdCompiler(ccc);
    if (LLVM_UNLIKELY(AlgorithmOptionIsSet(DisableIfHierarchy))) {
        ucdCompiler.generateWithoutIfHierarchy(nameMap, pb);
    } else {
        ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    }

    // The first UnicodeSet in the vector multiplexedCCs represents the last bit of the character class basis bit streams.
    std::reverse(names.begin(), names.end());
    for (unsigned i = 0; i < names.size(); i++) {
        auto t = nameMap.find(names[i]); 
        if (t != nameMap.end()) {
            PabloAST * const r = pb.createExtract(getOutput(0), pb.getInteger(i));
            if (t->first->getType() == Name::Type::Byte) {
                pb.createAssign(r, ccc.compileCC(dyn_cast<CC>(t->first->getDefinition())));
            } else {
                pb.createAssign(r, t->second);
            }
        } else {
          throw std::runtime_error("Can't compile character classes.");
        }
    }
}


