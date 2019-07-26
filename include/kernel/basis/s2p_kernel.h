/*
 *  Copyright (c) 2018 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef S2P_KERNEL_H
#define S2P_KERNEL_H

#include <re/alphabet/alphabet.h>
#include <pablo/pablo_kernel.h>
#include <string>

namespace IDISA { class IDISA_Builder; }  // lines 14-14
namespace llvm { class Value; }

namespace kernel {

class S2PKernel final : public MultiBlockKernel {
public:

    S2PKernel(const std::unique_ptr<kernel::KernelBuilder> &b,
              StreamSet * const codeUnitStream,
              StreamSet * const BasisBits,
              Scalar * signalNullObject = nullptr);


    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    Bindings makeOutputBindings(StreamSet * const BasisBits, bool abortOnNull);
    Bindings makeInputScalarBindings(Scalar * signalNullObject);
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    bool mAbortOnNull;
    unsigned mNumOfStreams;
};

class S2PMultipleStreamsKernel final : public MultiBlockKernel {
public:
    S2PMultipleStreamsKernel(const std::unique_ptr<kernel::KernelBuilder> & b,
                             StreamSet * codeUnitStream,
                             const StreamSets & outputStreams,
                             const bool aligned = true);
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
private:
    const bool mAligned;
};


class S2P_21Kernel final : public MultiBlockKernel {
public:
    S2P_21Kernel(const std::unique_ptr<kernel::KernelBuilder> &b, StreamSet * const codeUnitStream, StreamSet * const BasisBits);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & kb, llvm::Value * const numOfStrides) override;
};

class S2P_PabloKernel final : public pablo::PabloKernel {
public:
    S2P_PabloKernel(const std::unique_ptr<KernelBuilder> & b, StreamSet * const codeUnitStream, StreamSet * const BasisBits);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    const unsigned          mCodeUnitWidth;
};


}
#endif