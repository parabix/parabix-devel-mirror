/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "grapheme_kernel.h"
#include <re/re_toolchain.h>
#include <re/re_name.h>
#include <cc/cc_compiler.h>         // for CC_Compiler
#include <UCD/ucd_compiler.hpp>
#include <re/re_compiler.h>
#include <re/grapheme_clusters.h>
#include <re/re_name_gather.h>
#include <re/re_name_resolve.h>
#include <re/to_utf8.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <pablo/builder.hpp>

using namespace kernel;
using namespace pablo;


GraphemeClusterBreakKernel::GraphemeClusterBreakKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder)
: PabloKernel(iBuilder,
              "gcb",
              {Binding{iBuilder->getStreamSetTy(8), "basis"}, Binding{iBuilder->getStreamSetTy(1), "nonFinal"}},
              {Binding{iBuilder->getStreamSetTy(1, 1), "\\b{g}", FixedRate(), Add1()}}) {
}

void GraphemeClusterBreakKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler ccc(getEntryScope(), getInputStreamSet("basis"));
    UCD::UCDCompiler ucdCompiler(ccc);
    re::RE_Compiler re_compiler(getEntryScope(), ccc);
    re::RE * GCB = re::generateGraphemeClusterBoundaryRule();
    std::set<re::Name *> externals;
    re::gatherUnicodeProperties(GCB, externals);
    UCD::UCDCompiler::NameMap nameMap;
    for (auto & name : externals) {
        nameMap.emplace(name, nullptr);
    }
    GCB = resolveUnicodeNames(GCB);
    ucdCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    re_compiler.addPrecompiled("UTF8_nonfinal", pb.createExtract(getInputStreamVar("nonFinal"), pb.getInteger(0)));
    PabloAST * const gcb = re_compiler.compile(GCB);
    Var * const breaks = getOutputStreamVar("\\b{g}");
    pb.createAssign(pb.createExtract(breaks, pb.getInteger(0)), gcb);
}

