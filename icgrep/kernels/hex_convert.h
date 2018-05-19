/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef HEX_CONVERT_H
#define HEX_CONVERT_H

#include <kernels/kernel.h>
#include <kernels/kernel_builder.h>
#include <kernels/interface.h>
#include <kernels/streamset.h>
#include <llvm/IR/Value.h>

namespace kernel {
//
// Convert a byte stream of hexadecimal values into a bit stream. 
// Each hexadecimal input byte generates 4 bits to the output stream.
// 

class HexToBinary final : public kernel::BlockOrientedKernel {
public:
    HexToBinary(const std::unique_ptr<kernel::KernelBuilder> & b);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
};

//
// Produce a hexadecimal output stream with one hexadecimal byte
// for each 4 bits of an input bit stream.
//

class BinaryToHex final : public kernel::BlockOrientedKernel {
public:
    BinaryToHex(const std::unique_ptr<kernel::KernelBuilder> & b);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b) override;
    void generateFinalBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const remainingBits) override;
};
}
#endif
