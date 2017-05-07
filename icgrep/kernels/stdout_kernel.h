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
    StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth);
private:
    void generateDoSegmentMethod() override final;
private:
    const unsigned mCodeUnitWidth;
    
};


class FileSink : public SegmentOrientedKernel {
public:  
    FileSink(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, unsigned codeUnitWidth);
protected:
    void generateInitializeMethod() override final;
    void generateDoSegmentMethod() override final;
private:
    const unsigned mCodeUnitWidth;
    
};
}



#endif
