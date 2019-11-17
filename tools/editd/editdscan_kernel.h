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

class editdScanKernel : public BlockOrientedKernel {
public:
    editdScanKernel(BuilderRef b, StreamSet * matchResults);

private:
    void generateDoBlockMethod(BuilderRef iBuilder) override;
    llvm::Function * generateScanWordRoutine(BuilderRef iBuilder) const;

    unsigned mNumElements;
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
