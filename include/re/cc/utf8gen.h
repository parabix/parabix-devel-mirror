/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef UTF8_GEN_H
#define UTF8_GEN_H

#include <pablo/pablo_kernel.h>
#include <kernels/kernel_builder.h>

//
// UTF-8 encoding requires one to four bytes per Unicode character.
// To generate UTF-8 encoded output from sets of basis bit streams
// representing Unicode characters (that is, codepoint-indexed streams
// having one bit position per codepoint), deposit masks are needed
// to identify the positions at which bits for each character are
// to be deposited.   A UTF-8 deposit mask will have one to four bit
// positions per character depending on the character being encoded, that is,
// depending on the number of bytes needed to encode the character.   Within
// each group of one to four positions for a single character, a deposit mask
// must have exactly one 1 bit set.  Different deposit masks are used for
// depositing bits, depending on the destination byte position within the
// ultimate byte sequence.
//
// The following deposit masks (shown in little-endian representation) are
// used for depositing bits.
//
//  UTF-8 sequence length:          1     2     3       4
//  Unicode bit position:
//  Unicode codepoint bits 0-5      1    10   100    1000    u8final
//  Bits 6-11                       1    01   010    0100    u8mask6_11
//  Bits 12-17                      1    01   001    0010    u8mask12_17
//  Bits 18-20                      1    01   001    0001    u8initial
//
//  To compute UTF-8 deposit masks, we begin by constructing an extraction
//  mask having 4 bit positions per character, but with the number of
//  1 bits to be kept dependent on the sequence length.  When this extraction
//  mask is applied to the repeating constant 4-bit mask 1000, u8final above
//  is produced.
//
//  UTF-8 sequence length:             1     2     3       4
//  extraction mask                 1000  1100  1110    1111
//  constant mask                   1000  1000  1000    1000
//  final position mask             1     10    100     1000
//  From this mask, other masks may subsequently computed by
//  bitwise logic and shifting.
//
//  The UTF8fieldDepositMask kernel produces this deposit mask
//  within 64-bit fields.

using namespace kernel;

class UTF8fieldDepositMask final : public BlockOrientedKernel {
public:
    UTF8fieldDepositMask(const std::unique_ptr<KernelBuilder> & b, StreamSet * u32basis, StreamSet * u8fieldMask, StreamSet * u8unitCounts, unsigned depositFieldWidth = sizeof(size_t) * 8);
private:
    void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) override;
    void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, llvm::Value * const remainingBytes) override;
    const unsigned mDepositFieldWidth;
};

//
// Given a u8-indexed bit stream marking the final code unit position
// of each UTF-8 sequence, this kernel computes the deposit masks
// u8initial, u8mask12_17, and u8mask6_11.
//
class UTF8_DepositMasks : public pablo::PabloKernel {
public:
    UTF8_DepositMasks(const std::unique_ptr<KernelBuilder> & kb, StreamSet * u8final, StreamSet * u8initial, StreamSet * u8mask12_17, StreamSet * u8mask6_11);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

// This kernel assembles the UTF-8 basis bit data, given four sets of deposited
// bits bits 18-20, 11-17, 6-11 and 0-5, as weil as the marker streams u8initial,
// u8final, u8prefix3 and u8prefix4.
//
class UTF8assembly : public pablo::PabloKernel {
public:
    UTF8assembly(const std::unique_ptr<KernelBuilder> & kb,
                 StreamSet * deposit18_20, StreamSet * deposit12_17, StreamSet * deposit6_11, StreamSet * deposit0_5,
                 StreamSet * u8initial, StreamSet * u8final, StreamSet * u8mask6_11, StreamSet * u8mask12_17,
                 StreamSet * u8basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

#endif
