/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITDSCAN_KERNEL_H
#define EDITDSCAN_KERNEL_H

#include <kernel/core/kernel.h>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Function; }
namespace llvm { class Module; }

namespace kernel {

class CCScanKernel : public BlockOrientedKernel {
public:
    CCScanKernel(BuilderRef b, unsigned streamNum);

private:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
    llvm::Function * generateScanWordRoutine(BuilderRef iBuilder) const;

    unsigned mStreamNum;
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
