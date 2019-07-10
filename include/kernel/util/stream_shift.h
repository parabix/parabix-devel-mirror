/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */
#ifndef STREAM_SHIFT_H
#define STREAM_SHIFT_H

#include <pablo/pablo_kernel.h>  // for PabloKernel
#include <vector>

namespace IDISA { class IDISA_Builder; }

namespace kernel {

class ShiftForward final : public pablo::PabloKernel {
public:
    ShiftForward(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * inputs, StreamSet * outputs, unsigned shiftAmount = 1);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mShiftAmount;
};

class ShiftBack final : public pablo::PabloKernel {
public:
    ShiftBack(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * inputs, StreamSet * outputs, unsigned shiftAmount = 1);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    unsigned mShiftAmount;
};

}

#endif

