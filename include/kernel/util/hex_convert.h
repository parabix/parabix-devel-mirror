/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef HEX_CONVERT_H
#define HEX_CONVERT_H

#include <kernel/core/kernel.h>

namespace kernel {
//
// Convert a byte stream of hexadecimal values into a bit stream.
// Each hexadecimal input byte generates 4 bits to the output stream.
//

class HexToBinary final : public kernel::BlockOrientedKernel {
public:
    HexToBinary(BuilderRef b, StreamSet * hexStream, StreamSet * binStream);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(BuilderRef b) override;
};

//
// Produce a hexadecimal output stream with one hexadecimal byte
// for each 4 bits of an input bit stream.
//

class BinaryToHex final : public kernel::BlockOrientedKernel {
public:
    BinaryToHex(BuilderRef b, StreamSet * binStream, StreamSet * hexStream);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateDoBlockMethod(BuilderRef b) override;
    void generateFinalBlockMethod(BuilderRef b, llvm::Value * const remainingBits) override;
};

}
#endif
