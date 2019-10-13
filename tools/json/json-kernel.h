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
    this kernel returns the marker of a JSON string, based on paper
    Parsing Gigabytes of JSON per Second (Daniel Lemire and Geoff Langdale)
    E.g.: "a string \\\", oops"
    In:   1............1......1
    Out:  1...................1
*/
class JSONStringMarker : public pablo::PabloKernel {
public:
    JSONStringMarker(const std::unique_ptr<KernelBuilder> & b, StreamSet * const lex, StreamSet * strMarker)
    : pablo::PabloKernel(b,
                         "jsonStrMarker",
                         {Binding{"lex", lex}},
                         {Binding{"marker", strMarker}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class JSONKeywordSpan : public pablo::PabloKernel {
public:
    JSONKeywordSpan(const std::unique_ptr<KernelBuilder> & b, StreamSet * const basis, StreamSet * const lex, StreamSet * const strSpan, StreamSet * kwSpan)
    : pablo::PabloKernel(b,
                         "jsonKeywordSpan",
                         {Binding{"basis", basis}, Binding{"lex", lex}, Binding{"strSpan", strSpan}},
                         {Binding{"kwSpan", kwSpan}}) {}
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