/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/bixhash.h>
#include <pablo/builder.hpp>

using namespace llvm;
using namespace pablo;

namespace kernel {

const unsigned hash_bits = 8;

// Values for selecting bits to mix together in the log2 hash
// calculation strategy.
const std::vector<std::vector<unsigned>> bitmix =
   {{3, 2, 7, 1, 6, 4, 5, 0}, {4, 6, 1, 2, 3, 0, 7, 5},
    {6, 2, 3, 7, 5, 1, 0, 4}, {5, 7, 0, 3, 1, 4, 2, 6},
    {2, 1, 0, 7, 6, 5, 4, 3}, {1, 5, 3, 6, 7, 2, 4, 0}};

void BixHash::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * run = getInputStreamSet("run")[0];
    std::vector<PabloAST *> hash(hash_bits);
    // For every byte we create an in-place hash, in which each bit
    // of the byte is xor'd with one other bit.
    for (unsigned i = 0; i < hash_bits; i++) {
        hash[i] = pb.createXor(basis[i], basis[bitmix[0][i]]);
    }
    // In each step, the select stream will mark positions that are
    // to receive bits from prior locations in the symbol.   The
    // select stream must ensure that no bits from outside the symbol
    // are included in the calculated hash value.
    PabloAST * select = run;
    for (unsigned j = 0; j < mHashSteps; j++) {
        unsigned shft = 1<<j;
        // Select bits from prior positions.
        unsigned mixIdx = (j + 1) % bitmix.size();
        for (unsigned i = 0; i < hash_bits; i++) {
            PabloAST * priorBits = pb.createAdvance(hash[bitmix[mixIdx][i]], shft);
            // Mix in bits from prior positions.
            hash[i] = pb.createXor(hash[i], pb.createAnd(select, priorBits));
        }
        select = pb.createAnd(select, pb.createAdvance(select, shft));
    }
    Var * hashVar = getOutputStreamVar("hashes");
    for (unsigned i = 0; i < hash_bits; i++) {
        pb.createAssign(pb.createExtract(hashVar, pb.getInteger(i)), hash[i]);
    }
}
}
