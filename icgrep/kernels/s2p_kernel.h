/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }
//#define S2P_MULTIBLOCK
namespace kernel {
#ifdef S2P_MULTIBLOCK
    class S2PKernel final : public MultiBlockKernel {
#else
    class S2PKernel final : public BlockOrientedKernel {
#endif
public:
    S2PKernel(const std::unique_ptr<kernel::KernelBuilder> & b, bool aligned = true);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
#ifdef S2P_MULTIBLOCK
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb) override;
#else
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, llvm::Value * remainingBytes) override;
#endif
private:
    bool mAligned;
};

}
#endif
