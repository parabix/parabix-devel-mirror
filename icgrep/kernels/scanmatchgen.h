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
    

class MatchAccumulator {
public:
    MatchAccumulator() {};
    virtual void accumulate_match(const size_t lineNum, size_t line_start, size_t line_end) = 0;
};

    
class ScanMatchKernel : public MultiBlockKernel {
public:
	ScanMatchKernel(const std::unique_ptr<kernel::KernelBuilder> & b, const GrepType grepType, const unsigned codeUnitWidth);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
private:
	void generateMultiBlockLogic(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;
	const GrepType      mGrepType;
};

}
#endif // SCANMATCHGEN_H
