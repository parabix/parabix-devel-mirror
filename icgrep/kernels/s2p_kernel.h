/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class S2PKernel final : public BlockOrientedKernel {
public:    
    S2PKernel(const std::unique_ptr<IDISA::IDISA_Builder> & b, bool aligned = true);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
protected:
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod(llvm::Value * remainingBytes) override;
private:
    bool mAligned;
};

}
#endif
