/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

#include "kernel.h"

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StdOutKernel : public SegmentOrientedKernel {
public:

    StdOutKernel(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth);
    
private:
  
    void generateDoSegmentMethod() const override;

private:
    const unsigned mCodeUnitWidth;
    
};


class FileSink : public SegmentOrientedKernel {
public:
    
    FileSink(IDISA::IDISA_Builder * iBuilder, unsigned codeUnitWidth);
    
private:
    
    void generateInitMethod() const override;
    void generateDoSegmentMethod() const override;
    
private:
    const unsigned mCodeUnitWidth;
    
};
}



#endif
