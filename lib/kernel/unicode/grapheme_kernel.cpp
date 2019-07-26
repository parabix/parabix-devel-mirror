/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/unicode/grapheme_kernel.h>

#include <re/toolchain/toolchain.h>
#include <re/adt/re_name.h>
#include <re/cc/cc_compiler.h>         // for CC_Compiler
#include <re/cc/cc_compiler_target.h>
#include <re/compile/re_compiler.h>
#include <re/compile/re_name_gather.h>
#include <re/compile/to_utf8.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/grapheme_clusters.h>
#include <re/unicode/re_name_resolve.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>

using namespace kernel;
using namespace pablo;


GraphemeClusterBreakKernel::GraphemeClusterBreakKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, StreamSet *BasisBits, StreamSet * RequiredStreams, StreamSet * GCB_stream)
: PabloKernel(iBuilder, re::AnnotateWithREflags("gcb"),
// inputs
{Binding{"basis", BasisBits},
 Binding{"nonFinal", RequiredStreams, FixedRate(), ZeroExtended()}},
// output
{Binding{"\\b{g}", GCB_stream, FixedRate(), Add1()}}) {

}

void GraphemeClusterBreakKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
    UCD::UCDCompiler unicodeCompiler(ccc);
    re::RE_Compiler re_compiler(getEntryScope(), ccc);
    re::RE * GCB = re::generateGraphemeClusterBoundaryRule();
    std::set<re::Name *> externals;
    re::gatherUnicodeProperties(GCB, externals);
    UCD::UCDCompiler::NameMap nameMap;
    for (auto & name : externals) {
        nameMap.emplace(name, nullptr);
    }
    // GCB rule shouldn't have any regex names so using re::resolveUnicodeNames is good enough.
    GCB = resolveUnicodeNames(GCB);
    unicodeCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    re_compiler.addPrecompiled("UTF8_nonfinal", pb.createExtract(getInputStreamVar("nonFinal"), pb.getInteger(0)));
    PabloAST * const gcb = re_compiler.compile(GCB);
    Var * const breaks = getOutputStreamVar("\\b{g}");
    pb.createAssign(pb.createExtract(breaks, pb.getInteger(0)), gcb);
}
