/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include <cc/alphabet.h>
#include "kernel.h"  // for KernelBuilder

namespace IDISA { class IDISA_Builder; }

namespace kernel {

    class P2S4StreamByPDEP final : public BlockOrientedKernel{
    public:
        P2S4StreamByPDEP(const std::unique_ptr<kernel::KernelBuilder> & b);
    private:
        void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    };

class P2SKernel final : public BlockOrientedKernel {
public:
    P2SKernel(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian, std::string prefix = "", unsigned numOfStreams = 8);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    cc::BitNumbering mBasisSetNumbering;
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    unsigned mNumOfStreams;
};

class P2SKernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2SKernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    cc::BitNumbering mBasisSetNumbering;
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};

class P2S16Kernel final : public BlockOrientedKernel {
public:
    P2S16Kernel(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    cc::BitNumbering mBasisSetNumbering;
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};
    
class P2S16KernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & b, cc::BitNumbering basisNumbering = cc::BitNumbering::LittleEndian);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
    cc::BitNumbering mBasisSetNumbering;
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};
    
}

#endif
