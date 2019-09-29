/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef JSON_KERNEL_H
#define JSON_KERNEL_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>

namespace kernel {

class JSONMarker : public pablo::PabloKernel {
public:
    JSONMarker(const std::unique_ptr<KernelBuilder> & b, StreamSet * const lex, StreamSet * marker, StreamSet * callouts, StreamSet * err)
    : pablo::PabloKernel(b,
                         "jsonMarker",
                         {Binding{"lex", lex}},
                         {Binding{"marker", marker}, Binding{"callouts", callouts}, Binding{"err", err}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class ValidateJSONString : public pablo::PabloKernel {
public:
    ValidateJSONString(const std::unique_ptr<KernelBuilder> & b, StreamSet * const lex, StreamSet * strCallouts, StreamSet * err)
    : pablo::PabloKernel(b,
                         "validateJSONString",
                         {Binding{"lex", lex}},
                         {Binding{"strCallouts", strCallouts}, Binding{"err", err}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

}

#endif