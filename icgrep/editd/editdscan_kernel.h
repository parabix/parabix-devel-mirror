/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITDSCAN_KERNEL_H
#define EDITDSCAN_KERNEL_H

#include <kernels/kernel.h>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Function; }
namespace llvm { class Module; }

namespace kernel {
    
class editdScanKernel : public BlockOrientedKernel {
public:
    editdScanKernel(IDISA::IDISA_Builder * iBuilder, unsigned dist);
        
private:
    void generateDoBlockMethod() override;
    llvm::Function * generateScanWordRoutine(llvm::Module * m) const;
        
    unsigned mEditDistance;
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
