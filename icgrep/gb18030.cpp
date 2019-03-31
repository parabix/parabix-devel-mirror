/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <IR_Gen/idisa_target.h>

#include <cc/alphabet.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <cc/cc_kernel.h>
#include <UCD/legacy/GB_18030_data.h>
#include <kernels/deletion.h>
#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/swizzle.h>
#include <kernels/zeroextend.h>
#include <pablo/builder.hpp>
#include <pablo/boolean.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <toolchain/cpudriver.h>
#include <toolchain/toolchain.h>

#include <sys/stat.h>
#include <fcntl.h>
#include <iostream>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;
using namespace re;

// Analyze GB 10830 encoded data to classify bytes as singletons (ASCII or error),
// prefixes of 2- or 4-byte sequences, or second bytes of 4-byte sequences.
//
class GB_18030_ClassifyBytes : public pablo::PabloKernel {
public:
    GB_18030_ClassifyBytes(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basis, StreamSet * GB_bytes);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_ClassifyBytes::GB_18030_ClassifyBytes (const std::unique_ptr<KernelBuilder> & b, StreamSet * basis, StreamSet * GB_bytes)
: PabloKernel(b, "GB_18030_ClassifyBytes",
{Binding{"basis", basis}},
              {Binding{"GB_bytes", GB_bytes}}) {}

void GB_18030_ClassifyBytes::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
    PabloAST * x81_FE = ccc.compileCC(makeByte(0x081, 0xFE));
    PabloAST * x30_39 = ccc.compileCC(makeByte(0x030, 0x39));
    //
    // GB 10830 sequences may be decoded by analysis of sequences of bytes
    // in the range 0x81-FE.   Such a byte occurring as the first byte of
    // a 2-byte sequence or the first or third bytes of a 4-byte sequence
    // is said to be a key byte.
    // It is possible that there is a sequence of bytes in the range 0x81-FE.
    // The first such byte is always a key byte.
    PabloAST * GB_key1 = pb.createAnd(x81_FE, pb.createNot(pb.createAdvance(x81_FE, 1)), "GB_key1");
    // In a run of consecutive x81_FE bytes, every second one must be the
    // second byte of a 2-byte sequence and hence is not a key byte.
    // Therefore, if the first key byte in a run of x81_FE bytes starts at an odd
    // position, then only the bytes of this run at odd positions are key bytes.
    // Similarly, the key bytes in a run whose first key byte is at an even
    // position are only those at even positions in the run.
    
    PabloAST * odd_positions = pb.createRepeat(1, pb.getInteger(0x55, 8));
    PabloAST * even_positions = pb.createRepeat(1, pb.getInteger(0xAA, 8));
    PabloAST * GB_key1_odd = pb.createAnd(GB_key1, odd_positions);
    PabloAST * GB_key1_even = pb.createAnd(GB_key1, even_positions);
    
    PabloAST * x81_FE_run_odd = pb.createMatchStar(GB_key1_odd, x81_FE);
    PabloAST * x81_FE_run_even = pb.createMatchStar(GB_key1_even, x81_FE);
    PabloAST * GB_keys_odd = pb.createAnd(x81_FE_run_odd, odd_positions);
    PabloAST * GB_keys_even = pb.createAnd(x81_FE_run_even, even_positions);
    PabloAST * GB_keys = pb.createOr(GB_keys_odd, GB_keys_even, "GB_keys");
    //
    // All bytes after a key byte are data bytes of 2 or 4 byte sequences.
    PabloAST * GB_data = pb.createAdvance(GB_keys, 1);
    //
    // A byte in the range 0x30-0x39 after a key byte is always a data
    // byte in the second or fourth position of a 4-byte sequence.
    PabloAST * GB_4_data = pb.createAnd(GB_data, x30_39);
    //
    // A run of 4-byte sequences consists of alternating 0x81-0xFE and 0x30-0x39
    // bytes.   Let k2 identify the second byte of such a 4-byte seqquence.   The
    // first k2 byte is always the first GB_4_data byte such that the
    // byte 2 positions prior is not a GB_4_data byte.
    PabloAST * GB_k2_1 = pb.createAnd(GB_4_data, pb.createNot(pb.createAdvance(GB_4_data, 2)));
    
    PabloAST * odd_pairs = pb.createRepeat(1, pb.getInteger(0x33, 8));
    PabloAST * even_pairs = pb.createRepeat(1, pb.getInteger(0xCC, 8));
    PabloAST * GB_k2_1_odd = pb.createAnd(GB_k2_1, odd_pairs);
    PabloAST * GB_k2_1_even = pb.createAnd(GB_k2_1, even_pairs);
    
    PabloAST * key_or_data_4 = pb.createOr(GB_keys, GB_4_data);
    
    PabloAST * data4_run_odd = pb.createMatchStar(GB_k2_1_odd, key_or_data_4);
    PabloAST * data4_run_even = pb.createMatchStar(GB_k2_1_even, key_or_data_4);
    
    PabloAST * GB_k2_odd = pb.createAnd(data4_run_odd, odd_pairs);
    PabloAST * GB_k2_even = pb.createAnd(data4_run_even, even_pairs);
    PabloAST * GB_k2 = pb.createOr(GB_k2_odd, GB_k2_even);
    //
    //  All bytes that are neither key nor data of 2 or 4 byte sequences
    //  are ASCII or error.
    //
    PabloAST * GB_single = pb.createNot(pb.createOr(GB_keys, GB_data));
    //
    // GB prefix bytes are the keys that are first in 2 or 4 byte sequences.
    PabloAST * GB_prefix = pb.createAnd(GB_keys, pb.createNot(pb.createAdvance(GB_k2, 1)));
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_bytes"), pb.getInteger(0)), GB_single);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_bytes"), pb.getInteger(1)), GB_prefix);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_bytes"), pb.getInteger(2)), GB_k2);
}

const unsigned BitsPerInputByte = 8;

std::pair<std::vector<CC *>, std::vector<CC *>> byteTranscoderClasses(std::vector<codepoint_t> table) {
    if (table.size() != 256) llvm::report_fatal_error("Need 256 entries for byte transcoding.");
    codepoint_t allBits = 0;
    for (unsigned ch_code = 0; ch_code < 1 << BitsPerInputByte; ch_code++) {
        allBits |= table[ch_code];
    }
    const unsigned BitsPerOutputUnit = std::log2(allBits);
    std::vector<CC *> bitXfrmClasses;
    bitXfrmClasses.reserve(BitsPerInputByte);
    for (unsigned i = 0; i < BitsPerInputByte; i++) {
        bitXfrmClasses.push_back(makeCC(&cc::Byte));
    }
    std::vector<CC *> outputBitClasses(BitsPerOutputUnit-BitsPerInputByte);
    for (unsigned ch_code = 0; ch_code < 256; ch_code++) {
        codepoint_t transcodedCh = table[ch_code];
        codepoint_t changedBits = transcodedCh ^ ch_code;
        for (unsigned i = 0; i < BitsPerInputByte; i++) {
            unsigned bit = 1 << i;
            if ((changedBits & bit) == bit) {
                bitXfrmClasses[i]->insert(ch_code);
            }
        }
        for (unsigned i = BitsPerInputByte; i < BitsPerOutputUnit; i++) {
            unsigned bit = 1 << i;
            if ((transcodedCh & bit) == bit) {
                outputBitClasses[i-BitsPerInputByte]->insert(ch_code);
            }
        }
    }
    return std::make_pair(bitXfrmClasses, outputBitClasses);
}

//
// The second byte of a 2-byte GB 18030 sequence must be in the
// range 0x40 to 0xFE, excluding 0x7F.   Given a table of output
// codepoint entries for the 190 valid bytes, produce a full table
// of 256 entries for all possible 2nd byte values, using
// replacementChar for invalid entries.
std::vector<codepoint_t> fullByteTable_GB10830_byte2(std::vector<codepoint_t> validByteTable,
                                                     codepoint_t replacementChar) {
    std::vector<codepoint_t> fullTable(256);
    for (unsigned i = 0; i < 0x40; i++) {
        fullTable[i] = replacementChar;
    }
    for (unsigned i = 0x40; i < 0x7F; i++) {
        fullTable[i] = validByteTable[i - 0x40];
    }
    fullTable[0x7F] = replacementChar;
    for (unsigned i = 0x80; i < 0xFF; i++) {
        fullTable[i] = validByteTable[i - 0x41];
    }
    return fullTable;
}

class GB_18030_DoubleByteLogic : public pablo::PabloKernel {
public:
    GB_18030_DoubleByteLogic(const std::unique_ptr<KernelBuilder> & kb, StreamSet * byte1_basis, StreamSet * byte2_basis, StreamSet * u16_basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_DoubleByteLogic::GB_18030_DoubleByteLogic
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * byte1_basis, StreamSet * byte2_basis, StreamSet * u16_basis)
: PabloKernel(kb, "DoubleByteLogic_GB18030",
              // input
{Binding{"byte1_basis", byte1_basis}, Binding{"byte2_basis", byte2_basis}},
              // output
{Binding{"u16_basis", u16_basis}}) {
}

void GB_18030_DoubleByteLogic::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> byte1_basis = getInputStreamSet("byte1_basis");
    cc::Parabix_CC_Compiler_Builder Byte1_compiler(getEntryScope(), byte1_basis);
    PabloAST * zeroes = pb.createZeroes();
    Var * u16[16];
    for (int i = 0; i < 16; ++i) {
        u16[i] = pb.createVar("u16" + std::to_string(i), zeroes);
    }
    for (unsigned char_code = 0x81; char_code < 0xFF; char_code++) {
        PabloAST * byte1 = Byte1_compiler.compileCC(makeCC(char_code));
        PabloBlock * nested = getEntryScope()->createScope();
        std::vector<PabloAST *> byte2_basis = getInputStreamSet("byte2_basis");
        cc::Parabix_CC_Compiler_Builder Byte2_compiler(nested, byte2_basis);
        std::vector<CC *> bitXfrmClasses;
        std::vector<CC *> outputBitClasses;
        auto xClasses = byteTranscoderClasses(fullByteTable_GB10830_byte2(GB_DoubleByteTable[char_code - 0x81], 0xFFFD));
        bitXfrmClasses = xClasses.first;
        outputBitClasses = xClasses.second;
        for (unsigned i = 0; i < BitsPerInputByte; i++) {
            PabloAST * xfrmStrm = Byte2_compiler.compileCC(bitXfrmClasses[i]);
            PabloAST * outStrm = pb.createXor(xfrmStrm, byte2_basis[i]);
            nested->createAssign(u16[i], nested->createOr(u16[i], outStrm));
        }
        for (unsigned i = BitsPerInputByte; i < BitsPerInputByte + outputBitClasses.size(); i++) {
            PabloAST * outStrm = Byte2_compiler.compileCC(outputBitClasses[i + BitsPerInputByte]);
            nested->createAssign(u16[i], nested->createOr(u16[i], outStrm));
        }
        pb.createIf(byte1, nested);
    }
    Var * const u16_output = getOutputStreamVar("u16_basis");
    for (unsigned i = 0; i < 16; i++) {
        pb.createAssign(pb.createExtract(u16_output, pb.getInteger(i)), u16[i]);
    }
}
