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
    
class CCScanKernel : public BlockOrientedKernel {
public:
    CCScanKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned streamNum);
        
private:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    llvm::Function * generateScanWordRoutine(const std::unique_ptr<KernelBuilder> & iBuilder) const;
        
    unsigned mStreamNum;
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
