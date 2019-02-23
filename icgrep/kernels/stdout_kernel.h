/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

#include <kernels/core/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StdOutKernel final : public SegmentOrientedKernel {
public:
    StdOutKernel(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, StreamSet * codeUnitBuffer);
private:
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    const unsigned mCodeUnitWidth;

};

class FileSink final : public SegmentOrientedKernel {
public:
    FileSink(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, Scalar * outputFileName, StreamSet * codeUnitBuffer);
protected:
    void generateInitializeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) override;
    void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & b) override;
    void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) override;
private:
    const unsigned mCodeUnitWidth;

};
}



#endif
