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
#include <re/unicode/resolve_properties.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>

using namespace kernel;
using namespace pablo;

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
