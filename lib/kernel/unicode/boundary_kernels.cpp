/*
 *  Copyright (c) 2020 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/unicode/boundary_kernels.h>

#include <re/toolchain/toolchain.h>
#include <re/adt/adt.h>
#include <re/cc/cc_compiler.h>         // for CC_Compiler
#include <re/cc/cc_compiler_target.h>
#include <re/compile/re_compiler.h>
#include <re/analysis/re_name_gather.h>
#include <re/transforms/to_utf8.h>
#include <re/ucd/ucd_compiler.hpp>
#include <re/unicode/boundaries.h>
#include <re/unicode/re_name_resolve.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>

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
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), basis);
    UCD::UCDCompiler unicodeCompiler(ccc);
    re::RE_Compiler re_compiler(getEntryScope(), &cc::UTF8);
    re_compiler.addAlphabet(&cc::UTF8, basis);
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
    PabloAST * u8index = pb.createExtract(getInputStreamVar("u8index"), pb.getInteger(0));
    re_compiler.addPrecompiled("UTF8_index", re::RE_Compiler::Marker(u8index));
    re::RE_Compiler::Marker gcb_marker = re_compiler.compileRE(GCB);
    Var * const breaks = getOutputStreamVar("\\b{g}");
    pb.createAssign(pb.createExtract(breaks, pb.getInteger(0)), gcb_marker.stream());
}

BoundaryKernel::BoundaryKernel(BuilderRef kb, StreamSet * PropertyBasis, StreamSet * IndexStream, StreamSet * BoundaryStream, bool invert)
: PabloKernel(kb, "boundary_" + std::to_string(PropertyBasis->getNumElements()) + (invert ? "x1_negated" : "x1"),
              {Binding{"basis", PropertyBasis}, Binding{"index", IndexStream, FixedRate(), ZeroExtended()}},
              {Binding{"boundary", BoundaryStream, FixedRate(), Add1()}}),
  mHasIndex(IndexStream != nullptr), mInvert(invert) {
}

void BoundaryKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * idx = nullptr;
    if (mHasIndex) {
        idx = getInputStreamSet("index")[0];
    }
    Var * boundaryVar = getOutputStreamVar("boundary");
    PabloAST * boundary = pb.createZeroes();
    for (unsigned i = 0; i < basis.size(); i++) {
        boundary = pb.createOrXor(boundary, basis[i], pb.createIndexedAdvance(basis[i], idx, 1));
    }
    if (mInvert) {
        boundary = pb.createNot(boundary);
        if (mHasIndex) boundary = pb.createAnd(boundary, idx);
    }
    pb.createAssign(pb.createExtract(boundaryVar, 0), boundary);
}
