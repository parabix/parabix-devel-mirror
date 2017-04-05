/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef SCANMATCHGEN_H
#define SCANMATCHGEN_H

#include "grep_type.h"
#include "kernel.h"
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Function; }
namespace llvm { class Module; }

namespace kernel {
    
class ScanMatchKernel final : public BlockOrientedKernel {
public:
    ScanMatchKernel(IDISA::IDISA_Builder * iBuilder, GrepType grepType, unsigned codeUnitWidth);
    bool moduleIDisSignature() override {return true;}
protected:
    void generateInitMethod() override;
    void generateDoBlockMethod() override;
    void generateFinalBlockMethod(llvm::Value * remainingItems) override;
private:
    GrepType        mGrepType;
    const unsigned  mCodeUnitWidth;
};
}

#endif // SCANMATCHGEN_H
