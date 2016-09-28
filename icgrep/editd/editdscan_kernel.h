/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef EDITDSCAN_KERNEL_H
#define EDITDSCAN_KERNEL_H

#include <kernels/streamset.h>
#include <kernels/kernel.h>
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>

namespace llvm { class Module; class Function;}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    
class editdScanKernel : public KernelBuilder {
public:
    editdScanKernel(IDISA::IDISA_Builder * iBuilder) :
    KernelBuilder(iBuilder, "scanMatch",
                  {StreamSetBinding{parabix::StreamSetType(3, parabix::i1), "matchResults"}},
                  {}, {}, {}, {}),
    mScanwordBitWidth(Triple(llvm::sys::getProcessTriple()).isArch32Bit() ? 32 : 64) {}
        
private:
    void generateDoBlockMethod() override;
    llvm::Function * generateScanWordRoutine(llvm::Module * m);
        
    unsigned mScanwordBitWidth;
};

}

#endif // EDITDSCAN_KERNEL_H
