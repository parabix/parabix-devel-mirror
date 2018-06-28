/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef LINEBREAK_KERNEL_H
#define LINEBREAK_KERNEL_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <cc/alphabet.h>

namespace kernel { class KernelBuilder; }

namespace kernel {
#define USE_DIRECT_LF_BUILDER

class LineFeedKernelBuilder final : public pablo::PabloKernel {
public:
    LineFeedKernelBuilder(const std::unique_ptr<KernelBuilder> & b, kernel::Binding && inputStreamSet, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mNumOfStreams;
    unsigned mStreamFieldWidth;
    cc::BitNumbering mBasisSetNumbering;
};


class LineBreakKernelBuilder final : public pablo::PabloKernel {
public:
    LineBreakKernelBuilder(const std::unique_ptr<KernelBuilder> & b, unsigned basisBitsCount);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

}
#endif
