/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <kernel/util/bixhash.h>
#include <pablo/builder.hpp>
#include <vector>
#include <algorithm>
#include <random>

using namespace llvm;
using namespace pablo;

namespace kernel {

    
void BixHash::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> basis = getInputStreamSet("basis");
    PabloAST * run = getInputStreamSet("run")[0];
    std::vector<PabloAST *> hash(mHashBits);
    //unsigned mWords = mHashBits/8;
    // For every byte we create an in-place hash, in which each bit
    // of the byte is xor'd with one other bit.
    std::vector<int> bitmix(mHashBits);
    std::iota(bitmix.begin(), bitmix.end(), 0);
    std::mt19937 random_shuffle_engine(mSeed);
    shuffle (bitmix.begin(), bitmix.end(), random_shuffle_engine);
    // could be used to increase randomness in the subsequent hashes
    if (mWordNum == 0) {
        for (unsigned i = 0; i < mHashBits; i++) {
            hash[i] = pb.createXor(basis[i % basis.size()], basis[bitmix[i] % basis.size()]);
        }
    }
    else {
        for (unsigned i = 0; i < mHashBits; i++) {
            hash[i] = basis[i];
        }
    }
    // In each step, the select stream will mark positions that are
    // to receive bits from prior locations in the symbol. The
    // select stream must ensure that no bits from outside the symbol
    // are included in the calculated hash value.
    PabloAST * select = run;
    for (unsigned j = 0; j < mHashSteps; j++) {
        unsigned shft = 1<<j;
        // Select bits from prior positions.
        shuffle (bitmix.begin(), bitmix.end(), random_shuffle_engine);
        for (unsigned i = 0; i < mHashBits; i++) {
            PabloAST * priorBits = pb.createAdvance(hash[bitmix[i]], shft);
            // Mix in bits from prior positions.
            hash[i] = pb.createXor(hash[i], pb.createAnd(select, priorBits));
        }
        select = pb.createAnd(select, pb.createAdvance(select, shft));
    }
    PabloAST * prevWord = pb.createNot(run);
    for (unsigned i = 0; i < mHashBits; i++) {
        PabloAST * priorBits = pb.createAdvance(hash[i], 1);
        // Mix in bits from prior word's last byte.
        hash[i] = pb.createXor(hash[i], pb.createAnd(prevWord, priorBits));
    }
    Var * hashVar = getOutputStreamVar("hashes");
    for (unsigned i = 0; i < mHashBits; i++) {
        pb.createAssign(pb.createExtract(hashVar, pb.getInteger(i)), hash[i]);
    }
}

} //namespace kernel
