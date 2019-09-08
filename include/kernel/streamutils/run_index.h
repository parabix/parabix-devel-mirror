#ifndef RUN_INDEX_H
#define RUN_INDEX_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel {
    
/*  Given a stream with zero or more runs of 1 bits, this kernel
    generator enumerates the positions of each run (mod 2^K where
    K is the number of output streams).   The output is represented
    as a K-bit bixnum R, such that for each bit 0 <= i < K and each
    position p in a run from positions m to n,  R[i][p] = (p - m)/2^i mod 2.
 
    The runIndex parameter must be created and passed in as a stream
    set of K bitstreams.
 
    An overflow bit stream may also be added as an output parameter, in which
    case this stream will mark all those positions within runs that are
    indexed above 2^K - 1.
*/

class RunIndex : public pablo::PabloKernel {
public:
    RunIndex(const std::unique_ptr<KernelBuilder> & b,
               StreamSet * const runMarks, StreamSet * runIndex, StreamSet * overflow = nullptr);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;
private:
    unsigned mIndexCount;
    bool mOverflow;
};

}

#endif // RUN_INDEX_H
