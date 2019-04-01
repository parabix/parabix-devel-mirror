/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <IR_Gen/idisa_target.h>

#include <cc/alphabet.h>
#include <cc/cc_compiler.h>
#include <cc/cc_compiler_target.h>
#include <cc/cc_kernel.h>
#include <cc/encodings/GB_18030_data.h>
#include <kernels/deletion.h>
#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/deletion.h>
#include <kernels/pdep_kernel.h>
#include <pablo/builder.hpp>
#include <pablo/pablo_kernel.h>
#include <pablo/boolean.h>
#include <pablo/pablo_kernel.h>
#include <pablo/pablo_toolchain.h>
#include <pablo/bixnum.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
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

class GB_18030_ExtractionMasks : public pablo::PabloKernel {
public:
    GB_18030_ExtractionMasks(const std::unique_ptr<KernelBuilder> & kb, StreamSet * GB_bytes, StreamSet * GB_1, StreamSet * GB_2, StreamSet * GB_3, StreamSet * GB_4);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};


GB_18030_ExtractionMasks::GB_18030_ExtractionMasks
    (const std::unique_ptr<KernelBuilder> & b,
     StreamSet * GB_bytes, StreamSet * GB_1, StreamSet * GB_2, StreamSet * GB_3, StreamSet * GB_4)
: PabloKernel(b, "GB_18030_ExtractionMasks",
{Binding{"GB_bytes", GB_bytes}},
{Binding{"GB_1", GB_1}, Binding{"GB_2", GB_2}, Binding{"GB_3", GB_3}, Binding{"GB_4", GB_4}}) {}

void GB_18030_ExtractionMasks::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> GB_bytes = getInputStreamSet("GB_bytes");
    PabloAST * GB_single = pb.createExtract(getInputStreamVar("GB_bytes"), pb.getInteger(0));
    PabloAST * GB_prefix = pb.createExtract(getInputStreamVar("GB_bytes"), pb.getInteger(1));
    PabloAST * GB_k2 = pb.createExtract(getInputStreamVar("GB_bytes"), pb.getInteger(2));
    //
    // mask1 is for the extraction of the first byte of data for every GB 1, 2 or 4 byte sequence.
    PabloAST * mask1 = pb.createOr(GB_single, GB_prefix);
    //
    // mask2 is for selecting the low 4 bits of the 2nd byte of a sequence (or first of a singleton).
    PabloAST * GB_second = pb.createAdvance(GB_prefix, 1);
    PabloAST * mask2 = pb.createOr(GB_single, GB_second);
    // mask3 is for the last byte of data for every GB 1 or 2 byte sequence, or the 3rd byte
    // of every 4 byte sequence.
    PabloAST * GB_2of2 = pb.createAnd(GB_second, pb.createNot(GB_k2));
    PabloAST * GB_3of4 = pb.createAdvance(pb.createAnd(GB_second, GB_k2), 1);
    PabloAST * mask3 = pb.createOr(GB_single, pb.createOr(GB_2of2, GB_3of4));
    // mask4 is for selecting the low 4 bits of the last byte of a sequence.
    PabloAST * mask4 = pb.createOr(GB_single, pb.createOr(GB_2of2, pb.createAdvance(GB_3of4, 1)));
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_1"), pb.getInteger(0)), mask1);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_2"), pb.getInteger(0)), mask2);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_3"), pb.getInteger(0)), mask3);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_4"), pb.getInteger(0)), mask4);
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

class GB_18030_CoreLogic : public pablo::PabloKernel {
public:
    GB_18030_CoreLogic(const std::unique_ptr<KernelBuilder> & kb,
                       StreamSet * ASCII, StreamSet * GB_4byte,
                       StreamSet * byte1_basis, StreamSet * byte2_basis, StreamSet * u16_basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_CoreLogic::GB_18030_CoreLogic
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * ASCII, StreamSet * GB_4byte,
 StreamSet * byte1_basis, StreamSet * byte2_basis, StreamSet * u16_basis)
: PabloKernel(kb, "GB_18030_CoreLogic",
              // input
{Binding{"ASCII", ASCII}, Binding{"GB_4byte", GB_4byte}, Binding{"byte1_basis", byte1_basis}, Binding{"byte2_basis", byte2_basis}},
              // output
{Binding{"u16_basis", u16_basis}}) {
}

void GB_18030_CoreLogic::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * ASCII = pb.createExtract(getInputStreamVar("ASCII"), pb.getInteger(0));
    PabloAST * GB_4byte = pb.createExtract(getInputStreamVar("GB_4byte"), pb.getInteger(0));
    std::vector<PabloAST *> byte1_basis = getInputStreamSet("byte1_basis");
    cc::Parabix_CC_Compiler_Builder Byte1_compiler(getEntryScope(), byte1_basis);
    PabloAST * zeroes = pb.createZeroes();
    Var * u16[16];
    std::vector<std::vector<UCD::codepoint_t>> GB_tbl = get_GB_DoubleByteTable();
    for (int i = 0; i < 7; ++i) {
        u16[i] = pb.createVar("u16" + std::to_string(i), pb.createAnd(ASCII, byte1_basis[i]));
    }
    for (int i = 8; i < 16; ++i) {
        u16[i] = pb.createVar("u16" + std::to_string(i), zeroes);
    }
    for (unsigned char_code = 0x81; char_code < 0xFF; char_code++) {
        PabloAST * byte1 = Byte1_compiler.compileCC(makeCC(char_code));
        PabloBlock * nested = getEntryScope()->createScope();
        std::vector<PabloAST *> byte2_basis = getInputStreamSet("byte2_basis");
        cc::Parabix_CC_Compiler_Builder Byte2_compiler(nested, byte2_basis);
        std::vector<CC *> bitXfrmClasses;
        std::vector<CC *> outputBitClasses;
        auto xClasses = byteTranscoderClasses(fullByteTable_GB10830_byte2(GB_tbl[char_code - 0x81], 0xFFFD));
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

class GB_18030_FourByteLogic : public pablo::PabloKernel {
public:
    GB_18030_FourByteLogic(const std::unique_ptr<KernelBuilder> & kb,
                           StreamSet * GB_4byte,
                           StreamSet * byte1, StreamSet * nybble1, StreamSet * byte2, StreamSet * nybble2,
                           StreamSet * u21_basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_FourByteLogic::GB_18030_FourByteLogic
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * GB_4byte,
 StreamSet * byte1, StreamSet * nybble1, StreamSet * byte2, StreamSet * nybble2,
 StreamSet * u21_basis)
: PabloKernel(kb, "GB_18030_FourByteLogic",
              // input
{Binding{"GB_4byte", GB_4byte},
    Binding{"byte1", byte1},
    Binding{"nybble1", nybble1},
    Binding{"byte2", byte2},
    Binding{"nybble2", nybble2}
},
              // output
{Binding{"u21_basis", u21_basis}}) {
}



void GB_18030_FourByteLogic::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * GB_4byte = pb.createExtract(getInputStreamVar("GB_4byte"), pb.getInteger(0));
    
    BixNum byte1_lo7 = getInputStreamSet("byte1");
    BixNum nybble1 = getInputStreamSet("nybble1");
    BixNum byte2 = getInputStreamSet("byte2");
    BixNum nybble2 = getInputStreamSet("nybble2");
    
    BixNum byte3_lo7(7);
    for (unsigned i = 0; i < 7; i++) {
        byte3_lo7[i] = pb.createAnd(GB_4byte, byte2[i]);
    }
    BixNum byte2_lo4(4);
    BixNum byte4_lo4(4);
    for (unsigned i = 0; i < 4; i++) {
        byte2_lo4[i] = pb.createAnd(GB_4byte, nybble1[i]);
        byte4_lo4[i] = pb.createAnd(GB_4byte, nybble2[i]);
    }

    BixNum lo11 = BixNumFullArithmetic(pb).Mul(BixNumModularArithmetic(pb).Sub(byte3_lo7, 1), 10);
    lo11 = BixNumModularArithmetic(pb).Add(lo11, byte4_lo4);
    BixNum lo15 = BixNumModularArithmetic(pb).Add(BixNumFullArithmetic(pb).Mul(byte2_lo4, 1260), lo11);
    BixNum byte1_X_12600 = BixNumFullArithmetic(pb).Mul(BixNumModularArithmetic(pb).Sub(byte1_lo7, 1), 12600);
    BixNum GB_val = BixNumModularArithmetic(pb).Add(byte1_X_12600, lo15);
    
    const unsigned GB_SupplementaryPlaneValue = 189000;
    PabloAST * aboveBMP = pb.createAnd(GB_4byte, BixNumArithmetic(pb).UGE(byte1_lo7, 0x10));
    
    Zeroes * zeroes = pb.createZeroes();
    std::vector<PabloAST *> u21(21, zeroes);
    BixNum SMP_offset = BixNumModularArithmetic(pb).Sub(GB_val, GB_SupplementaryPlaneValue);
    for (unsigned i = 0; i < 21; i++) {
        u21[i] = pb.createAnd(SMP_offset[i], aboveBMP);
    }
    
    std::vector<std::pair<unsigned, unsigned>> range_tbl = get_GB_RangeTable();
    PabloAST * GE_lo_bound = pb.createOnes();
    for (unsigned i = 0; i < range_tbl.size(); i++) {
        unsigned base = range_tbl[i].first;
        codepoint_t cp = range_tbl[i].second;
        codepoint_t offset = cp - base;
        PabloAST * GE_hi_bound;
        if (i < range_tbl.size() - 1) {
            GE_hi_bound = BixNumArithmetic(pb).UGE(GB_val, range_tbl[i+1].first);
        } else {
            GE_hi_bound = pb.createOnes();
        }
        PabloAST * in_range = pb.createAnd(GE_lo_bound, pb.createNot(GE_hi_bound));
        BixNum mapped = BixNumModularArithmetic(pb).Add(GB_val, offset);
        for (unsigned i = 0; i < 16; i++) {
            u21[i] = pb.createOr(u21[i], pb.createAnd(in_range, mapped[i]));
        }
        GE_lo_bound = GE_hi_bound;
    }
    
    Var * const u21_basis = getOutputStreamVar("u21_basis");
    for (unsigned i = 0; i < 21; i++) {
        pb.createAssign(pb.createExtract(u21_basis, pb.getInteger(i)), u21[i]);
    }

}

void extract(const std::unique_ptr<ProgramBuilder> & P, StreamSet * inputSet, unsigned inputBase, StreamSet * mask, StreamSet * outputs) {
    unsigned fw = 64;  // Best for PEXT extraction.
    StreamSet * const compressed = P->CreateStreamSet(outputs->getNumElements());
    P->CreateKernelCall<FieldCompressKernel>(fw, inputSet, mask, compressed, inputBase);
    P->CreateKernelCall<StreamCompressKernel>(fw, compressed, mask, outputs);
}


typedef void (*gb18030FunctionType)(uint32_t fd, const char *);

gb18030FunctionType generatePipeline(CPUDriver & pxDriver) {
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);
    
    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);
    
    StreamSet * const GB_bytes = P->CreateStreamSet(3);
    P->CreateKernelCall<GB_18030_ClassifyBytes>(BasisBits, GB_bytes);

    // Masks for extraction of 1 bit per GB code unit sequence.
    StreamSet * const GB_mask1 = P->CreateStreamSet(1);
    StreamSet * const GB_mask2 = P->CreateStreamSet(1);
    StreamSet * const GB_mask3 = P->CreateStreamSet(1);
    StreamSet * const GB_mask4 = P->CreateStreamSet(1);
    P->CreateKernelCall<GB_18030_ClassifyBytes>(GB_bytes, GB_mask1, GB_mask2, GB_mask3, GB_mask4);
    
    StreamSet * const ASCII = P->CreateStreamSet(1);    // markers for ASCII data
    StreamSet * const GB_4byte = P->CreateStreamSet(1); // markers for 4-byte sequences
    StreamSet * const byte1 = P->CreateStreamSet(7);    // only need the seven bits
    StreamSet * const nybble1 = P->CreateStreamSet(4);
    StreamSet * const byte2 = P->CreateStreamSet(8);
    StreamSet * const nybble2 = P->CreateStreamSet(4);
    
    extract(P, GB_bytes, 0, GB_mask1, ASCII);
    extract(P, GB_bytes, 2, GB_mask1, GB_4byte);
    extract(P, BasisBits, 0, GB_mask1, byte1);
    extract(P, BasisBits, 0, GB_mask2, nybble1);
    extract(P, BasisBits, 0, GB_mask3, byte2);
    extract(P, BasisBits, 0, GB_mask4, nybble2);
    
    StreamSet * const u16basis = P->CreateStreamSet(16);
    P->CreateKernelCall<GB_18030_CoreLogic>(ASCII, GB_4byte, byte1, byte2, u16basis);

    StreamSet * const u32basis = P->CreateStreamSet(21);
    P->CreateKernelCall<GB_18030_FourByteLogic>(GB_4byte, byte1, nybble1, byte2, nybble2, u16basis, u32basis);

    // Buffers for calculated deposit masks.
    StreamSet * const u8fieldMask = P->CreateStreamSet();
    StreamSet * const u8final = P->CreateStreamSet();
    StreamSet * const u8initial = P->CreateStreamSet();
    StreamSet * const u8mask12_17 = P->CreateStreamSet();
    StreamSet * const u8mask6_11 = P->CreateStreamSet();
    
    // Intermediate buffers for deposited bits
    StreamSet * const deposit18_20 = P->CreateStreamSet(3);
    StreamSet * const deposit12_17 = P->CreateStreamSet(6);
    StreamSet * const deposit6_11 = P->CreateStreamSet(6);
    StreamSet * const deposit0_5 = P->CreateStreamSet(6);
    
    // Final buffers for computed UTF-8 basis bits and byte stream.
    StreamSet * const u8basis = P->CreateStreamSet(8);
    StreamSet * const u8bytes = P->CreateStreamSet(1, 8);
    
    // Calculate the u8final deposit mask.
    StreamSet * const extractionMask = P->CreateStreamSet();
    P->CreateKernelCall<UTF8fieldDepositMask>(u32basis, u8fieldMask, extractionMask);
    P->CreateKernelCall<StreamCompressKernel>(u8fieldMask, extractionMask, u8final);
    
    P->CreateKernelCall<UTF8_DepositMasks>(u8final, u8initial, u8mask12_17, u8mask6_11);
    
    deposit(P, 18, 3, u8initial, u32basis, deposit18_20);
    deposit(P, 12, 6, u8mask12_17, u32basis, deposit12_17);
    deposit(P, 6, 6, u8mask6_11, u32basis, deposit6_11);
    deposit(P, 0, 6, u8final, u32basis, deposit0_5);
    
    P->CreateKernelCall<UTF8assembly>(deposit18_20, deposit12_17, deposit6_11, deposit0_5,
                                      u8initial, u8final, u8mask6_11, u8mask12_17,
                                      u8basis);
    
    P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);
    
    P->CreateKernelCall<StdOutKernel>(u8bytes);
    
    return reinterpret_cast<gb18030FunctionType>(P->compile());
}


size_t file_size(const int fd) {
    struct stat st;
    if (LLVM_UNLIKELY(fstat(fd, &st) != 0)) {
        st.st_size = 0;
    }
    return st.st_size;
}

int main(int argc, char *argv[]) {
    codegen::ParseCommandLineOptions(argc, argv, {&x8u16Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("gb18030");
    gb18030FunctionType gb18030Function = generatePipeline(pxDriver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        std::cerr << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        x8u16Function(fd, outputFile.c_str());
        close(fd);
    }
    return 0;
}
