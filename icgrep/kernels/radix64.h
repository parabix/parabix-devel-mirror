/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef RADIX64_H
#define RADIX64_H

#include "kernel.h"

namespace llvm { class Module; }
namespace llvm { class Value; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

/*  expand3_4 transforms a byte sequence by duplicating every third byte. 
    Each 3 bytes of the input abc produces a 4 byte output abcc.   
    This is a useful preparatory transformation in various radix-64 encodings. */
 
class expand3_4Kernel : public SegmentOrientedKernel {
public:   
    expand3_4Kernel(IDISA::IDISA_Builder * iBuilder);
private:
    void generateDoSegmentMethod() override final;
};

class radix64Kernel : public BlockOrientedKernel {
public:
    radix64Kernel(IDISA::IDISA_Builder * iBuilder);
private:
    virtual void generateDoBlockMethod() override final;
    virtual void generateFinalBlockMethod(llvm::Value * remainingBytes) override final;
    llvm::Value* processPackData(llvm::Value* packData) const;
};

class base64Kernel : public BlockOrientedKernel {
public:
    base64Kernel(IDISA::IDISA_Builder * iBuilder);
private:
    virtual void generateDoBlockMethod() override final;
    virtual void generateFinalBlockMethod(llvm::Value * remainingBytes) override final;
    llvm::Value* processPackData(llvm::Value* packData) const;
};

}
#endif
