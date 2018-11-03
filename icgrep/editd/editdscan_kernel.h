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
    editdScanKernel(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * matchResults);
        
private:
    void generateDoBlockMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
    llvm::Function * generateScanWordRoutine(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) const;
        
    unsigned mNumElements;
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
