/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SPREAD_BY_MASK_H
#define SPREAD_BY_MASK_H

#include <kernel/core/kernel.h>
#include <llvm/IR/Value.h>
#include <string>
#include <kernel/pipeline/driver/driver.h>

namespace kernel {

enum class StreamExpandOptimization {None, NullCheck};

/*  SpreadByMask - spread out the bits of input streams according to a mask.
    Outputs are 1-to-1 with mask positions.   For a mask bit of 0, the corresponding
    output bits are always 0.   Otherwise, the nth 1 bit in the mask selects
    an input bit from position n in the input stream.

    Input stream:    abcdef
    Mask stream:     ...1.1...111..1..     . represents 0
    Output stream:   ...a.b...cde..f..     . represents 0

    The input stream is processed at the Popcount rate of the mask stream.

    The number of streams to process is governed by the size of the output stream set.
    The input streams to process are selected sequentially from the stream set toSpread,
    starting from the position indicated by the streamOffset value.

    Depending on the expected density of the mask stream, different optimizations
    can be chosen.   The expansionFieldWidth parameter may also affect performance.

    SpreadByMask employs two kernels in sequence:  StreamExpandKernel followed by
    FieldDepositKernel.
 
 */

void SpreadByMask(const std::unique_ptr<ProgramBuilder> & P,
                  StreamSet * mask, StreamSet * toSpread, StreamSet * outputs,
                  unsigned streamOffset = 0,
                  StreamExpandOptimization opt = StreamExpandOptimization::None,
                  unsigned expansionFieldWidth = 64);

/* The following kernels are used by SpreadByMask internally. */

class StreamExpandKernel final : public MultiBlockKernel {
public:
    StreamExpandKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                       StreamSet * mask,
                       StreamSet * source,
                       StreamSet * expanded,
                       Scalar * base,
                       const StreamExpandOptimization = StreamExpandOptimization::None,
                       const unsigned FieldWidth = sizeof(size_t) * 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfBlocks) override;
private:
    const unsigned mFieldWidth;
    const unsigned mSelectedStreamCount;
    const StreamExpandOptimization mOptimization;
};

class FieldDepositKernel final : public MultiBlockKernel {
public:
    FieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> &, StreamSet * mask, StreamSet * input, StreamSet * output, const unsigned fieldWidth = sizeof(size_t) * 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const unsigned mFieldWidth;
    const unsigned mStreamCount;
};

class PDEPFieldDepositKernel final : public MultiBlockKernel {
public:
    PDEPFieldDepositKernel(const std::unique_ptr<kernel::KernelBuilder> &, StreamSet * mask, StreamSet * expanded, StreamSet * outputs, const unsigned fieldWidth = sizeof(size_t) * 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const unsigned mPDEPWidth;
    const unsigned mStreamCount;
};

/*
 Swizzled PDEP kernel - now deprecated.

 Conceptually, given an unbounded input stream set of k streams and a marker stream, this kernel uses the
 Parallel Bits Deposit (PDEP) instruction to copy the input items from the i-th input stream to the i-th
 output stream the positions indicated by the marker bits. All other output items are set to zero. E.g.,

 SOURCE >  abcdefgh i0000000 00000000 00000000
 MARKER >  ...1.1.1 .....11. ..1...1. ...1.1..
 OUTPUT >  ...a.b.c .....de. ..f...g. ...h.i..

 The complicating factor of this Kernel is that it assumes the input streams are *swizzled*. I.e., it
 "divides" each block of the marker stream into k elements, M_1 ... M_k, and applies the PDEP operation
 using M_i to the each of the k elements in the i-th input (swizzled) stream.

 CONCEPTUAL VIEW OF INPUT STREAM SET                    ACTUAL LAYOUT OF INPUT STREAM SET
 
 STREAM 0  abcde...  fg......  hijklm..  nopqrst.     SWIZZLE 0  abcde...  uvwxy...  OPQRS...  89abc...
 STREAM 1  uvwxy...  zA......  BCDEFG..  HIJKLMN.     SWIZZLE 1  fg......  zA......  TU......  de......
 STREAM 2  OPQRS...  TU......  VWXYZ0..  1234567.     SWIZZLE 2  hijklm..  BCDEFG..  VWXYZ0..  fghijk..
 STREAM 3  89abc...  de......  fghijk..  lmnopqr.     SWIZZLE 3  nopqrst.  HIJKLMN.  1234567.  lmnopqr.


 NOTE: this kernel does *NOT* unswizzle the output. This will eventually be the responsibility of the
 pipeline to ensure it is done when needed.

 */
class PDEPkernel final : public MultiBlockKernel {
public:
    PDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned swizzleFactor = 4, std::string name = "PDEP");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;
private:
    const unsigned mSwizzleFactor;
};


}

#endif
