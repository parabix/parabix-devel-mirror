/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

#include "streamset.h"
#include "kernel.h"

namespace llvm { class Module; class Function;}

namespace IDISA { class IDISA_Builder; }

namespace kernel {
    
class scanMatchKernel : public KernelBuilder {
public:
    scanMatchKernel(IDISA::IDISA_Builder * iBuilder, unsigned scanwordBitWidth, bool isNameExpression) :
    KernelBuilder(iBuilder, "scanMatch",
                    {StreamSetBinding{StreamSetType(2, 1), "matchResults"}}, 
                    {}, 
                    {ScalarBinding{iBuilder->getInt8PtrTy(), "FileBuf"}, ScalarBinding{iBuilder->getInt64Ty(), "FileSize"}, ScalarBinding{iBuilder->getInt64Ty(), "FileIdx"}}, 
                    {}, 
                    {ScalarBinding{iBuilder->getInt64Ty(), "BlockNo"}, ScalarBinding{iBuilder->getInt64Ty(), "LineStart"}, ScalarBinding{iBuilder->getInt64Ty(), "LineNum"}}),

    mScanwordBitWidth(scanwordBitWidth),
    mIsNameExpression(isNameExpression) {}
        
private:
    void generateDoBlockMethod() override;
    llvm::Function * generateScanWordRoutine(llvm::Module * m);
        
    unsigned mScanwordBitWidth;
    bool mIsNameExpression;
};
}

#endif // SCANMATCHGEN_H
