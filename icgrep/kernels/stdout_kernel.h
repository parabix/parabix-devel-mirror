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

class StdOutKernel : public KernelBuilder {
public:
    StdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth) :
    KernelBuilder(iBuilder, "stdout",
                  {Binding{iBuilder->getStreamSetTy(1, codeUnitWidth), "codeUnitBuffer"}}, {}, {}, {}, {}),
    mCodeUnitWidth(codeUnitWidth) {
        setNoTerminateAttribute(true);
    }
    
private:
    unsigned mCodeUnitWidth;
  
    void generateDoSegmentMethod() const override;
    
};
}

#endif
