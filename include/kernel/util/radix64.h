/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef RADIX64_H
#define RADIX64_H

#include <kernel/core/kernel.h>

namespace llvm { class Module; }
namespace llvm { class Value; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

/*  expand3_4 transforms a byte sequence by duplicating every third byte.
    Each 3 bytes of the input abc produces a 4 byte output abcc.
    This is a useful preparatory transformation in various radix-64 encodings. */

class expand3_4Kernel final : public MultiBlockKernel {
public:
    expand3_4Kernel(BuilderRef b, StreamSet * input, StreamSet * expandedOutput);
private:
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
};

class radix64Kernel final : public BlockOrientedKernel {
public:
    radix64Kernel(BuilderRef, StreamSet * input, StreamSet * output);
private:
    virtual void generateDoBlockMethod(BuilderRef b) override;
    virtual void generateFinalBlockMethod(BuilderRef b, llvm::Value * remainingBytes) override;
    llvm::Value * processPackData(BuilderRef b, llvm::Value* packData) const;
};

class base64Kernel final : public BlockOrientedKernel {
public:
    base64Kernel(BuilderRef, StreamSet * input, StreamSet * output);
private:
    virtual void generateDoBlockMethod(BuilderRef b) override;
    virtual void generateFinalBlockMethod(BuilderRef b, llvm::Value * remainingBytes) override;
    llvm::Value* processPackData(BuilderRef b, llvm::Value* packData) const;
};

}
#endif
