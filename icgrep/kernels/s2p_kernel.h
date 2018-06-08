/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "kernel.h"  // for KernelBuilder

#include <pablo/pablo_kernel.h>
#include <string>

namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class S2PKernel final : public MultiBlockKernel {
public:
    S2PKernel(const std::unique_ptr<kernel::KernelBuilder> & b, bool aligned = true, std::string prefix = "");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    bool mAligned;
};

class S2P_21Kernel final : public MultiBlockKernel {
public:
    S2P_21Kernel(const std::unique_ptr<kernel::KernelBuilder> & b);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
};

class S2P_PabloKernel final : public pablo::PabloKernel {
public:
    S2P_PabloKernel(const std::unique_ptr<KernelBuilder> & b, unsigned codeUnitWidth = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mCodeUnitWidth;
};


class S2PByPextKernel final : public BlockOrientedKernel {
public:
    S2PByPextKernel(const std::unique_ptr<kernel::KernelBuilder> & b, std::string prefix = "");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
};

}
#endif
