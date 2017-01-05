/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include "streamset.h"
#include "interface.h"
#include "kernel.h"

namespace llvm { class Module; }

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class S2PKernel : public KernelBuilder {
public:
    
    S2PKernel(IDISA::IDISA_Builder * builder);

    virtual ~S2PKernel() {}
        
private:
    void generateDoBlockLogic(Value * self, Value * blockNo) const override;
    void generateDoBlockMethod() const override;
    void generateFinalBlockMethod() const override;
    
};

    

}
#endif
