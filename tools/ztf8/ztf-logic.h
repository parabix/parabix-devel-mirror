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

struct LengthGroup {unsigned lo; unsigned hi; unsigned hashBits;};
    
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

/*
 This kernel decodes the insertion length for two-byte ZTF code symbols.
 The insertion length is the number of zero bytes to insert so that,
 after insertion the zeroes together with the encoded symbol can be
 replaced by the dictionary symbol of the appropriate length.
*/

class ZTF_ExpansionDecoder final: public pablo::PabloKernel {
public:
    ZTF_ExpansionDecoder(const std::unique_ptr<kernel::KernelBuilder> & b,
                         StreamSet * const basis,
                         std::vector<LengthGroup> lengthGroups,
                         StreamSet * insertBixNum);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    std::vector<LengthGroup> mLengthGroups;
};

class ZTF_DecodeLengths : public pablo::PabloKernel {
public:
    ZTF_DecodeLengths(const std::unique_ptr<KernelBuilder> & b,
                      StreamSet * basisBits,
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
                      std::vector<LengthGroup> & lenGroups,
                      StreamSet * const basis,
                      StreamSet * bixHash,
                      StreamSet * extractionMask,
                      StreamSet * runIdx,
                      StreamSet * encoded);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
    std::vector<LengthGroup> & mLenGroups;
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

