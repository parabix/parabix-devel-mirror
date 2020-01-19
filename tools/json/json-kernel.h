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

/*
   Marks keywords letters such as l', 'a', 's', 'r', 'u', 'e',
   joining it at the end with 'n', 't' and 'f'
*/
class JSONKeywordMarker : public pablo::PabloKernel {
public:
    JSONKeywordMarker(const std::unique_ptr<KernelBuilder> & b,
                      StreamSet * const basis, StreamSet * const lex, StreamSet * const strSpan,
                      StreamSet * kwMarker, StreamSet * kwLex)
    : pablo::PabloKernel(b,
                         "jsonKeywordMarker",
                         {Binding{"basis", basis}, Binding{"lex", lex}, Binding{"strSpan", strSpan}},
                         {Binding{"kwMarker", kwMarker}, Binding{"kwLex", kwLex}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class JSONKeywordSpan : public pablo::PabloKernel {
public:
    JSONKeywordSpan(const std::unique_ptr<KernelBuilder> & b,
                    StreamSet * const kwMarker, StreamSet * const kwLex,
                    StreamSet * kwSpan, StreamSet * kwErr)
    : pablo::PabloKernel(b,
                         "jsonKeywordSpan",
                         {Binding{"kwLex", kwLex}, Binding{"kwMarker", kwMarker, FixedRate(1), LookAhead(4)}},
                         {Binding{"kwSpan", kwSpan}, Binding{"kwErr", kwErr}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

/*
   Finds symbols used in numbers such as 'e', 'E', '.'
   and join them at the end if they match the expression:
   \-?(0|[1-9][0-9]*)(.[0-9]+)?([Ee][+-]?[0-9]+)?
*/
class JSONNumberSpan : public pablo::PabloKernel {
public:
    JSONNumberSpan(const std::unique_ptr<KernelBuilder> & b,
                   StreamSet * const basis, StreamSet * const lex, StreamSet * const strSpan,
                   StreamSet * nbrLex, StreamSet * nbrSpan, StreamSet * nbrErr)
    : pablo::PabloKernel(b,
                         "jsonNumberMarker",
                         {Binding{"basis", basis, FixedRate(1), LookAhead(1)}, Binding{"lex", lex}, Binding{"strSpan", strSpan}},
                         {Binding{"nbrLex", nbrLex}, Binding{"nbrSpan", nbrSpan}, Binding{"nbrErr", nbrErr}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

}

#endif