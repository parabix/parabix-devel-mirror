/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef ZTF_LOGIC_H
#define ZTF_LOGIC_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>

namespace kernel {

struct LengthGroup {unsigned lo; unsigned hi;};
    
class WordMarkKernel : public pablo::PabloKernel {
public:
    WordMarkKernel(const std::unique_ptr<KernelBuilder> & kb, StreamSet * BasisBits, StreamSet * WordMarks);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class ByteRun final: public pablo::PabloKernel {
public:
    ByteRun(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const basis, StreamSet * excluded, StreamSet * runMask)
    : pablo::PabloKernel(b, "byteRun", {Binding{"basis", basis}, Binding{"excluded", excluded}}, {Binding{"runMask", runMask}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class ZTF_DecodeLengths : public pablo::PabloKernel {
public:
    ZTF_DecodeLengths(const std::unique_ptr<KernelBuilder> & b, StreamSet * basisBits,
                      std::vector<LengthGroup> lengthGroups,
                      std::vector<StreamSet *> & groupStreams);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    std::vector<LengthGroup> mLengthGroups;
};

class ZTF_Symbols : public pablo::PabloKernel {
public:
    ZTF_Symbols(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basisBits, StreamSet * wordChar, StreamSet * symbolRuns)
    : pablo::PabloKernel(kb, "ZTF_Symbols",
                         {Binding{"basisBits", basisBits, FixedRate(1), LookAhead(1)},
                          Binding{"wordChar", wordChar, FixedRate(1), LookAhead(3)}},
                         {Binding{"symbolRuns", symbolRuns}}) { }
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class ZTF_SymbolEncoder final: public pablo::PabloKernel {
public:
    ZTF_SymbolEncoder(const std::unique_ptr<kernel::KernelBuilder> & b,
                      StreamSet * const basis, StreamSet * bixHash, StreamSet * extractionMask, StreamSet * runIdx, StreamSet * encoded)
    : pablo::PabloKernel(b, "ZTF_SymbolEncoder",
                         {Binding{"basis", basis},
                          Binding{"bixHash", bixHash, FixedRate(), LookAhead(1)},
                          Binding{"extractionMask", extractionMask},
                          Binding{"runIdx", runIdx}},
                         {Binding{"encoded", encoded}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

/*
 This kernel decodes the insertion length for two-byte ZTF code symbols
 in the range 0xC2-0xDF.   The insertion length is the number of zero
 bytes to insert so that, after insertion the zeroes together with the
 encoded symbol can be replaced by the dictionary symbol of the appropriate
 length.
 
 The following table shows the pattern of the insertion lengths.
 0xC2, 0xC3   final length 3, insertion length 1
 0xC4, 0xC5   final length 4, insertion length 2
 0xC6, 0xC7   final length 5, insertion length 3
 ...
 0xDE, 0xDF   final length 17, insertion length 15
 
 As it turns out, the insertion length calculation is very simple for
 the given symbols: simply using bits 1 through 4 of the basis stream.
 */

class ZTF_ExpansionDecoder final: public pablo::PabloKernel {
public:
    ZTF_ExpansionDecoder(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * const basis, StreamSet * insertBixNum)
    : pablo::PabloKernel(b, "ZTF_ExpansionDecoder",
                       {Binding{"basis", basis, FixedRate(), LookAhead(1)}},
                       {Binding{"insertBixNum", insertBixNum}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class LengthSorter final: public pablo::PabloKernel {
public:
    LengthSorter(const std::unique_ptr<kernel::KernelBuilder> & b,
                 StreamSet * symbolRun, StreamSet * const lengthBixNum,
                 StreamSet * overflow,
                 std::vector<LengthGroup> lengthGroups,
                 std::vector<StreamSet *> & groupStreams);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    std::vector<LengthGroup> mLengthGroups;
};
}
#endif

