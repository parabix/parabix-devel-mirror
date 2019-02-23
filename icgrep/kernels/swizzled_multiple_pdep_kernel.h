/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef ICGREP_SWIZZLED_MULTIPLE_PDEP_KERNEL_H
#define ICGREP_SWIZZLED_MULTIPLE_PDEP_KERNEL_H

#include <kernels/core/kernel.h>
#include <llvm/IR/Value.h>
#include <string>

/**
 * For every input stream set, SwizzledMultiplePDEPkernel do exactly the same thing as PDEPkernel kernel.
 * However, instead of only handing one swizzled source stream, the SwizzledMultiplePDEPkernel handle
 * the PDEP logic of multiple source streams at the same time to improve the performance in single thread
 * environment.
 */

namespace kernel {

class SwizzledMultiplePDEPkernel final : public MultiBlockKernel {
public:
    SwizzledMultiplePDEPkernel(const std::unique_ptr<kernel::KernelBuilder> & b, const unsigned swizzleFactor = 4, const unsigned numberOfStreamSet = 1, std::string name = "SwizzledMultiplePDEP");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const numOfStrides) final;
private:
    const unsigned mSwizzleFactor;
    const unsigned mNumberOfStreamSet;
};

}

#endif //ICGREP_SWIZZLED_MULTIPLE_PDEP_KERNEL_H
