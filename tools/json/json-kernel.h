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

/*
    Given the JSON lex for characters backslash and double quotes,
    this kernel returns the span of a JSON string, based on paper
    Parsing Gigabytes of JSON per Second (Daniel Lemire and Geoff Langdale)
*/
class JSONStringSpan : public pablo::PabloKernel {
public:
    JSONStringSpan(const std::unique_ptr<KernelBuilder> & b, StreamSet * const lex, StreamSet * span)
    : pablo::PabloKernel(b,
                         "jsonMarker",
                         {Binding{"lex", lex}},
                         {Binding{"span", span}}) {}
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