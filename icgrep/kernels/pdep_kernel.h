/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef PDEP_KERNEL_H
#define PDEP_KERNEL_H

#include "kernel.h"
#include <llvm/IR/Value.h>
#include <string>

/*

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

namespace kernel {

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
