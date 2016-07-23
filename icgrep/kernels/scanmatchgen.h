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
    scanMatchKernel(IDISA::IDISA_Builder * iBuilder, parabix::StreamSetBuffer& matchResults, bool isNameExpression) :
    KernelBuilder(iBuilder, "scanMatch",
                  {StreamSetBinding{matchResults, "matchResults"}}, 
                    {}, 
                    {ScalarBinding{iBuilder->getInt8PtrTy(), "FileBuf"}, ScalarBinding{iBuilder->getSizeTy(), "FileSize"}, ScalarBinding{iBuilder->getSizeTy(), "FileIdx"}}, 
                    {}, 
                    {ScalarBinding{iBuilder->getSizeTy(), "BlockNo"}, ScalarBinding{iBuilder->getSizeTy(), "LineStart"}, ScalarBinding{iBuilder->getSizeTy(), "LineNum"}}),

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
