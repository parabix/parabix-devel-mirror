/*
 *  Copyright (c) 2017 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EVEN_ODD_H
#define EVEN_ODD_H

#include "kernel.h"  // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class EvenOddKernel : public KernelBuilder {
public:
    
    EvenOddKernel(IDISA::IDISA_Builder * builder);

    virtual ~EvenOddKernel() {}
        
private:
    void generateDoBlockLogic(llvm::Value * self, llvm::Value * blockNo) const override;    
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;

};

}
#endif
