/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

#include "grep_type.h"  // for GrepType
#include "kernel.h"     // for KernelBuilder
namespace IDISA { class IDISA_Builder; }  // lines 16-16
namespace llvm { class Function; }  // lines 14-14
namespace llvm { class Module; }  // lines 14-14

namespace kernel {
    
class ScanMatchKernel : public KernelBuilder {
public:
    ScanMatchKernel(IDISA::IDISA_Builder * iBuilder, GrepType grepType) :
    KernelBuilder(iBuilder, "scanMatch",
                  {Binding{iBuilder->getStreamSetTy(2, 1), "matchResults"}},
                    {}, 
                    {Binding{iBuilder->getInt8PtrTy(), "FileBuf"}, Binding{iBuilder->getSizeTy(), "FileSize"}, Binding{iBuilder->getSizeTy(), "FileIdx"}}, 
                    {}, 
                    {Binding{iBuilder->getSizeTy(), "BlockNo"}, Binding{iBuilder->getSizeTy(), "LineStart"}, Binding{iBuilder->getSizeTy(), "LineNum"}}),

    mGrepType(grepType) {}
        
private:

    void generateDoBlockMethod() const override;

    llvm::Function * generateScanWordRoutine(llvm::Module * m) const;

private:
    GrepType mGrepType;
};
}

#endif // SCANMATCHGEN_H
