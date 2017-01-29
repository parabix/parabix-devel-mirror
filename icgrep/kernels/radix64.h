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
    void generateDoSegmentMethod(llvm::Function * doSegmentFunction, llvm::Value *self, llvm::Value *doFinal, const std::vector<llvm::Value *> &producerPos) const override;
};

class radix64Kernel : public BlockOrientedKernel {
public:
    radix64Kernel(IDISA::IDISA_Builder * iBuilder);
private:
    virtual void generateDoBlockMethod(llvm::Function * function, llvm::Value * self, llvm::Value * blockNo) const override;
    virtual void generateFinalBlockMethod(llvm::Function * function, llvm::Value * self, llvm::Value * remainingBytes, llvm::Value * blockNo) const override;
};

class base64Kernel : public BlockOrientedKernel {
public:
    base64Kernel(IDISA::IDISA_Builder * iBuilder);
private:
    virtual void generateFinalBlockMethod(llvm::Function * function, llvm::Value * self, llvm::Value * remainingBytes, llvm::Value * blockNo) const override;
    virtual void generateDoBlockMethod(llvm::Function * function, llvm::Value * self, llvm::Value * blockNo) const override;
    
};

}
#endif
