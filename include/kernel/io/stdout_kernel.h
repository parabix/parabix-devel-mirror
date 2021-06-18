/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STDOUT_KERNEL_H
#define STDOUT_KERNEL_H

#include <kernel/core/kernel.h>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class StdOutKernel final : public SegmentOrientedKernel {
public:
    StdOutKernel(BuilderRef iBuilder, StreamSet * codeUnitBuffer);
private:
    void generateDoSegmentMethod(BuilderRef b) override;
private:
    const unsigned mCodeUnitWidth;

};

class FileSink final : public SegmentOrientedKernel {
public:
    FileSink(BuilderRef iBuilder, Scalar * outputFileName, StreamSet * codeUnitBuffer);
protected:
    void generateInitializeMethod(BuilderRef iBuilder) override;
    void generateDoSegmentMethod(BuilderRef b) override;
    void generateFinalizeMethod(BuilderRef b) override;
private:
    const unsigned mCodeUnitWidth;

};
}



#endif
