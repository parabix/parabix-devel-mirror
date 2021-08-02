/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include <re/alphabet/alphabet.h>
#include <pablo/pablo_kernel.h>
#include <kernel/pipeline/driver/driver.h>
#include <string>

namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

    
class S2PKernel final : public MultiBlockKernel {
public:
    S2PKernel(BuilderRef b,
              StreamSet * const codeUnitStream,
              StreamSet * const BasisBits,
              StreamSet * zeroMask = nullptr);
protected:
    Bindings makeInputBindings(StreamSet * codeUnitStream, StreamSet * zeroMask);
    Bindings makeOutputBindings(StreamSet * const BasisBits);
    void generateMultiBlockLogic(BuilderRef b, llvm::Value * const numOfStrides) override;
private:
    bool mZeroMask;
    unsigned mNumOfStreams;
};

// Equivalent to S2P, but split into stages for better balancing
// with multiple cores.
void Staged_S2P(const std::unique_ptr<ProgramBuilder> & P,
                StreamSet * codeUnitStream, StreamSet * BasisBits,
                bool completionFromQuads = false);


class S2P_21Kernel final : public MultiBlockKernel {
public:
    S2P_21Kernel(BuilderRef b, StreamSet * const codeUnitStream, StreamSet * const BasisBits);
protected:
    void generateMultiBlockLogic(BuilderRef kb, llvm::Value * const numOfStrides) override;
};

class S2P_PabloKernel final : public pablo::PabloKernel {
public:
    S2P_PabloKernel(BuilderRef b, StreamSet * const codeUnitStream, StreamSet * const BasisBits);
protected:
    void generatePabloMethod() override;
private:
    const unsigned          mCodeUnitWidth;
};


}
#endif
