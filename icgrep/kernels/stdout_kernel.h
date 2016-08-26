/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

#include "streamset.h"
#include "kernel.h"
#include <llvm/IR/Type.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class stdOutKernel : public KernelBuilder {
public:
    stdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth) :
    KernelBuilder(iBuilder, "stdout",
                  {StreamSetBinding{parabix::StreamSetType(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {}) {
        mStreamType = PointerType::get(parabix::StreamSetType(1, codeUnitWidth).getStreamSetBlockType(iBuilder), 0);
        mScalarInputs = {ScalarBinding{mStreamType , "bufferPtr"}};
    }
    
private:
    void prepareKernel() override;
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod() override;
    void generateDoSegmentMethod() override;
    
    llvm::Type * mStreamType;
};
}

#endif
