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
    ScanMatchKernel(IDISA::IDISA_Builder * const iBuilder, const GrepType grepType, const unsigned codeUnitWidth);
    bool isCachable() const override { return true; }
    bool moduleIDisSignature() const override { return true; }
protected:
    void generateDoBlockMethod() override;
private:
    const GrepType      mGrepType;
};
}

#endif // SCANMATCHGEN_H
