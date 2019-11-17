#ifndef SENTINEL_H
#define SENTINEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel

namespace kernel {

// Extend an input stream by one position with adding a 1 bit.
class AddSentinel final : public pablo::PabloKernel {
public:
    AddSentinel(BuilderRef b,
               StreamSet * const input, StreamSet * const output);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    void generatePabloMethod() override;
};

}

#endif // SENTINEL_H
