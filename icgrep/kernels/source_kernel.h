#ifndef SOURCE_KERNEL_H
#define SOURCE_KERNEL_H

#include "kernel.h"
namespace IDISA { class IDISA_Builder; }

namespace kernel {

/* The MMapSourceKernel is a simple wrapper for an external MMap file buffer.
   The doSegment method of this kernel feeds one segment at a time to a
   pipeline. */

class MMapSourceKernel final : public SegmentOrientedKernel {
public:
    MMapSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
    bool moduleIDisSignature() final { return true; }
protected:
    void linkExternalMethods() override;
    void generateInitializeMethod() override;
    void generateDoSegmentMethod() override;
    void generateFinalizeMethod() override;
protected:
    const unsigned          mSegmentBlocks;
    const unsigned          mCodeUnitWidth;
    llvm::Function *        mFileSizeFunction;
};

class ReadSourceKernel final : public SegmentOrientedKernel {
public:
    ReadSourceKernel(IDISA::IDISA_Builder * iBuilder, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
    bool moduleIDisSignature() override { return true; }
protected:
    void generateInitializeMethod() override;
    void generateDoSegmentMethod() override;
    void generateFinalizeMethod() override;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
};

class MemorySourceKernel final : public SegmentOrientedKernel {
public:
    MemorySourceKernel(IDISA::IDISA_Builder * iBuilder, llvm::Type * type, unsigned blocksPerSegment = 1, unsigned codeUnitWidth = 8);
    bool moduleIDisSignature() override { return true; }
protected:
    void generateInitializeMethod() override;
    void generateDoSegmentMethod() override;
private:
    unsigned mSegmentBlocks;
    unsigned mCodeUnitWidth;
};

}

#endif // SOURCE_KERNEL_H
