/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#ifndef ZTF_KERNEL_H
#define ZTF_KERNEL_H

#include <pablo/pablo_kernel.h>
#include <kernel/core/kernel_builder.h>

namespace kernel {

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

class ZTF_Codes final: public pablo::PabloKernel {
public:
    ZTF_Codes(const std::unique_ptr<kernel::KernelBuilder> & b, StreamSet * basis, StreamSet * byteRunCodes, StreamSet * dictSymCodes)
    : pablo::PabloKernel(b, "ZTF_Codes", {Binding{"basis", basis, FixedRate(), LookAhead(1)}},
                  {Binding{"byteRunCodes", byteRunCodes}, Binding{"dictSymCodes", dictSymCodes}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

/*
 This kernel decodes the insertion length for two-byte ZTF code symbols
 in the range 0xC2-0xDF as well as byte run codes 0xF9-0xFF.
 The insertion length is the number of zero
 bytes to insert so that, after insertion the zeroes together with the
 encoded symbol can be replaced by the dictionary symbol of the appropriate
 length.

 The following table shows the pattern of the insertion lengths.
 0xC2, 0xC3   final length 3, insertion length 1
 0xC4, 0xC5   final length 4, insertion length 2
 0xC6, 0xC7   final length 5, insertion length 3
 ...
 0xDC, 0xDD   final length 16, insertion length 14
 0xF9         final length 3, insertion length 1
 ...
 0xFF         final length 9, insertion length 7

 As it turns out, the insertion length calculation is very simple for
 the given symbols: simply using bits 1 through 4 of the basis stream for 0xC2-0xDD,
 bits 0 through 2 for 0xF9-0xFF.
 */

class ZTF_ExpansionDecoder final: public pablo::PabloKernel {
public:
    ZTF_ExpansionDecoder(const std::unique_ptr<kernel::KernelBuilder> & b,
                         StreamSet * basis, StreamSet * byteRunCodes, StreamSet * dictSymCodes,
                         StreamSet * insertBixNum, StreamSet * byteRunStart)
    : pablo::PabloKernel(b, "ZTF_ExpansionDecoder",
                  {Binding{"basis", basis},
                   Binding{"byteRunCodes", byteRunCodes, FixedRate(1), LookAhead(1)},
                   Binding{"dictSymCodes", dictSymCodes}},
                  {Binding{"insertBixNum", insertBixNum},
                   Binding{"byteRunStart", byteRunStart}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

class ZTF_Byte_Run_Decompression final: public pablo::PabloKernel {
public:
    ZTF_Byte_Run_Decompression(const std::unique_ptr<kernel::KernelBuilder> & b,
                          StreamSet * byteRunStarts, StreamSet * runMask, StreamSet * ztf_u8_indexed, StreamSet * u8output)
    : pablo::PabloKernel(b, "ZTF_Byte_Run_Decompression",
                  {Binding{"byteRunStarts", byteRunStarts},
                      Binding{"runSpreadMask", runMask},
                      Binding{"ztf_u8_indexed", ztf_u8_indexed}},
                  {Binding{"u8output", u8output}}) {}
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};
}
#endif
