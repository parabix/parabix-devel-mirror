/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

#include "streamset.h"
#include "kernel.h"
#include <llvm/Support/Host.h>
#include <llvm/ADT/Triple.h>

namespace llvm { class Module; class Function;}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    
class scanMatchKernel : public KernelBuilder {
public:
    scanMatchKernel(IDISA::IDISA_Builder * iBuilder, bool isNameExpression) :
    KernelBuilder(iBuilder, "scanMatch",
                  {Binding{parabix::StreamSetType(iBuilder,2, 1), "matchResults"}},
                    {}, 
                    {Binding{iBuilder->getInt8PtrTy(), "FileBuf"}, Binding{iBuilder->getSizeTy(), "FileSize"}, Binding{iBuilder->getSizeTy(), "FileIdx"}}, 
                    {}, 
                    {Binding{iBuilder->getSizeTy(), "BlockNo"}, Binding{iBuilder->getSizeTy(), "LineStart"}, Binding{iBuilder->getSizeTy(), "LineNum"}}),

    mScanwordBitWidth(Triple(llvm::sys::getProcessTriple()).isArch32Bit() ? 32 : 64),
    mIsNameExpression(isNameExpression) {}
        
private:
    void generateDoBlockMethod() override;
    llvm::Function * generateScanWordRoutine(llvm::Module * m);
        
    unsigned mScanwordBitWidth;
    bool mIsNameExpression;
};
}

#endif // SCANMATCHGEN_H
