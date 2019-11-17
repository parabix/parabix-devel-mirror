/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef BIXHASH_H
#define BIXHASH_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel {

/*  The BixHash kernel computes hash values for symbols in a source stream.
    Inputs: basis: source stream represented as a set of 8 basis bits.
            run: bit stream marking the nonstart bytes of symbols.
            steps: the number of steps to apply, which determines the
                   max length (1<<steps) of fully hashed values.
    Outputs:   hashes: hash values represented as n-bit bixnums.
 
    Note:   hash values are computed at every position within a symbol,
            based on the prefix of the symbol up to and including the
            given position.   This may be useful if symbol prefixes as
            well as symbols are to be indexed.
 */

class BixHash final: public pablo::PabloKernel {
public:
    BixHash(BuilderRef b,
            StreamSet * basis, StreamSet * run, StreamSet * hashes, unsigned steps=4, unsigned seed = 179321)
    : PabloKernel(b, "BixHash" + std::to_string(hashes->getNumElements()) + "_" + std::to_string(steps) + "_" + std::to_string(seed),
                  {Binding{"basis", basis}, Binding{"run", run}},
                  {Binding{"hashes", hashes}}),
    mHashBits(hashes->getNumElements()), mHashSteps(steps), mSeed(seed) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    unsigned mHashBits;
    unsigned mHashSteps;
    unsigned mSeed;
};

}
#endif
