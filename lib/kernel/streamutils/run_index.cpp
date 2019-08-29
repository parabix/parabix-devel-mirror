/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/streamutils/run_index.h>
#include <kernel/core/kernel_builder.h>
#include <pablo/builder.hpp>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_infile.h>

using namespace pablo;

namespace kernel {

RunIndex::RunIndex(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const runMarks, StreamSet * runIndex)
: PabloKernel(b, "RunIndex-" + std::to_string(runIndex->getNumElements()),
           // input
{Binding{"runMarks", runMarks}},
           // output
{Binding{"runIndex", runIndex}}),
mIndexCount(runIndex->getNumElements()) {
    assert(mIndexCount > 0);
    assert(mIndexCount <= 5);
}

void RunIndex::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    Var * runMarks = pb.createExtract(getInputStreamVar("runMarks"), pb.getInteger(0));
    PabloAST * runStart = pb.createAnd(runMarks, pb.createAdvance(pb.createNot(runMarks), 1), "runStart");
    PabloAST * selectZero = runMarks;
    PabloAST * outputEnable = runMarks;
    Var * runIndexVar = getOutputStreamVar("runIndex");
    std::vector<PabloAST *> runIndex(mIndexCount);
    PabloAST * even = nullptr;
    for (unsigned i = 0; i < mIndexCount; i++) {
        switch (i) {
            case 0: even = pb.createRepeat(1, pb.getInteger(0x55, 8)); break;
            case 1: even = pb.createRepeat(1, pb.getInteger(0x33, 8)); break;
            case 2: even = pb.createRepeat(1, pb.getInteger(0x0F, 8)); break;
            case 3: even = pb.createRepeat(1, pb.getInteger(0x00FF, 16)); break;
            case 4: even = pb.createRepeat(1, pb.getInteger(0x0000FFFF, 32)); break;
            case 5: even = pb.createRepeat(1, pb.getInteger(0x00000000FFFFFFFF, 64)); break;
        }
        PabloAST * odd = pb.createNot(even);
        PabloAST * evenStart = pb.createAnd(even, runStart);
        PabloAST * oddStart = pb.createAnd(odd, runStart);
        PabloAST * idx = pb.createOr(pb.createAnd(pb.createMatchStar(evenStart, runMarks), odd),
                                     pb.createAnd(pb.createMatchStar(oddStart, runMarks), even));
        idx = pb.createAnd(idx, selectZero);
        for (unsigned j = 0; j < i; j++) {
            idx = pb.createOr(idx, pb.createAdvance(idx, 1<<j));
        }
        runIndex[i] = pb.createAnd(idx, outputEnable, "runidx[" + std::to_string(i) + "]");
        pb.createAssign(pb.createExtract(runIndexVar, pb.getInteger(i)), runIndex[i]);
        selectZero = pb.createAnd(selectZero, pb.createNot(idx), "selectZero");
        outputEnable = pb.createAnd(outputEnable, pb.createAdvance(outputEnable, 1<<i), "outputEnable");
    }
}
}

