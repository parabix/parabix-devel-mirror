/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/unicode/grapheme_kernel.h>

#include <re/toolchain/toolchain.h>
#include <re/adt/adt.h>
#include <re/cc/cc_compiler.h>         // for CC_Compiler
#include <re/cc/cc_compiler_target.h>
#include <re/compile/re_compiler.h>
#include <re/analysis/re_name_gather.h>
#include <re/transforms/to_utf8.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/grapheme_clusters.h>
#include <re/unicode/re_name_resolve.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>

using namespace kernel;
using namespace pablo;


GraphemeClusterBreakKernel::GraphemeClusterBreakKernel(BuilderRef iBuilder, StreamSet *BasisBits, StreamSet * u8index, StreamSet * GCB_stream)
: PabloKernel(iBuilder, re::AnnotateWithREflags("gcb"),
// inputs
{Binding{"basis", BasisBits},
 Binding{"u8index", u8index, FixedRate(), ZeroExtended()}},
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
    re::gatherNames(GCB, externals);
    UCD::UCDCompiler::NameMap nameMap;
    for (auto & name : externals) {
        nameMap.emplace(name, nullptr);
    }
    // GCB rule shouldn't have any regex names so using re::resolveUnicodeNames is good enough.
    GCB = resolveUnicodeNames(GCB);
    unicodeCompiler.generateWithDefaultIfHierarchy(nameMap, pb);
    re_compiler.addPrecompiled("UTF8_index", pb.createExtract(getInputStreamVar("u8index"), pb.getInteger(0)));
    PabloAST * const gcb = re_compiler.compile(GCB);
    Var * const breaks = getOutputStreamVar("\\b{g}");
    pb.createAssign(pb.createExtract(breaks, pb.getInteger(0)), gcb);
}

