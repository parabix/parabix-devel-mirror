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

    S2PKernel(const std::unique_ptr<kernel::KernelBuilder> &,
              StreamSet * const codeUnitStream,
              StreamSet * const BasisBits,
              const cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian,
              const bool aligned = true);


    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const cc::BitNumbering mBasisSetNumbering;
    const bool mAligned;
    unsigned mNumOfStreams;
};

class S2PMultipleStreamsKernel final : public MultiBlockKernel {
public:
    S2PMultipleStreamsKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                             StreamSet * codeUnitStream,
                             const StreamSets & outputStreams,
                             const cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian,
                             const bool aligned = true);
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const cc::BitNumbering mBasisSetNumbering;
    const bool mAligned;
};


class S2P_21Kernel final : public MultiBlockKernel {
public:
    S2P_21Kernel(const std::unique_ptr<kernel::KernelBuilder> &, StreamSet * const codeUnitStream, StreamSet * const BasisBits, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
    const cc::BitNumbering mBasisSetNumbering;
};

class S2P_PabloKernel final : public pablo::PabloKernel {
public:
    S2P_PabloKernel(const std::unique_ptr<KernelBuilder> & b, StreamSet * const codeUnitStream, StreamSet * const BasisBits, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    const cc::BitNumbering  mBasisSetNumbering;
    const unsigned          mCodeUnitWidth;
};


}
#endif
