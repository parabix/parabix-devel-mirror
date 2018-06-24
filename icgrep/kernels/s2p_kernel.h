/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "kernel.h"  // for KernelBuilder
#include <cc/alphabet.h>
#include <pablo/pablo_kernel.h>
#include <string>

namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class S2PKernel final : public MultiBlockKernel {
public:
    S2PKernel(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian, bool aligned = true, std::string prefix = "");
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    cc::BitNumbering mBasisSetNumbering;
    bool mAligned;
};

class S2P_21Kernel final : public MultiBlockKernel {
public:
    S2P_21Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
    cc::BitNumbering mBasisSetNumbering;
};

class S2P_PabloKernel final : public pablo::PabloKernel {
public:
    S2P_PabloKernel(const std::unique_ptr<KernelBuilder> & b, unsigned codeUnitWidth = 8, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mCodeUnitWidth;
    cc::BitNumbering mBasisSetNumbering;
};


}
#endif
