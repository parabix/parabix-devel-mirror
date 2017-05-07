/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef P2S_KERNEL_H
#define P2S_KERNEL_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }

namespace kernel {

   
class P2SKernel final : public BlockOrientedKernel {
public:
    P2SKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
private:
    void generateDoBlockMethod() override;
};

class P2SKernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2SKernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
private:
    void generateDoBlockMethod() override;
};

class P2S16Kernel final : public BlockOrientedKernel {
public:
    P2S16Kernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
private:
    void generateDoBlockMethod() override;
};
    
class P2S16KernelWithCompressedOutput final : public BlockOrientedKernel {
public:
    P2S16KernelWithCompressedOutput(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
private:
    void generateDoBlockMethod() override;
};
    
}

#endif
