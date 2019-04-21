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
#include <cc/encodings/utf8gen.h>
#include <kernels/deletion.h>
#include <kernels/kernel_builder.h>
#include <kernels/p2s_kernel.h>
#include <kernels/pipeline_builder.h>
#include <kernels/s2p_kernel.h>
#include <kernels/source_kernel.h>
#include <kernels/stdout_kernel.h>
#include <kernels/streams_merge.h>
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
#include <sstream>
#include <llvm/Support/raw_ostream.h>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace codegen;
using namespace re;

static cl::OptionCategory gb18030Options("gb18030 Options", "Transcoding control options.");

static cl::opt<std::string> inputFile(cl::Positional, cl::desc("<input file>"), cl::Required, cl::cat(gb18030Options));
static cl::opt<unsigned> ReplacementCharacter("replacement-character", cl::desc("Codepoint value of the character used to replace any unmapped or invalid input sequence."), cl::init(0xFFFD), cl::cat(gb18030Options));

// Analyze GB 10830 encoded data to classify bytes as singletons (ASCII or error),
// prefixes of 2- or 4-byte sequences, or second bytes of 4-byte sequences.
//
class GB_18030_ClassifyBytes : public pablo::PabloKernel {
public:
    GB_18030_ClassifyBytes(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basis, StreamSet * GB_ASCII, StreamSet * GB2_final, StreamSet * GB4_final, StreamSet * GB_final);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_ClassifyBytes::GB_18030_ClassifyBytes (const std::unique_ptr<KernelBuilder> & b, StreamSet * basis, StreamSet * GB_ASCII, StreamSet * GB2_final, StreamSet * GB4_final, StreamSet * GB_final)
: PabloKernel(b, "GB_18030_ClassifyBytes",
{Binding{"basis", basis}},
              {Binding{"GB_ASCII", GB_ASCII}, Binding{"GB2_final", GB2_final}, Binding{"GB4_final", GB4_final}, Binding{"GB_final", GB_final}}) {}

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
    PabloAST * GB_key1 = pb.createAnd(x81_FE, pb.createNot(pb.createAdvance(x81_FE, 1)), "gb_key1");
    // In a run of consecutive x81_FE bytes, every second one must be the
    // second byte of a 2-byte sequence and hence is not a key byte.
    // Therefore, if the first key byte in a run of x81_FE bytes starts at an odd
    // position, then only the bytes of this run at odd positions are key bytes.
    // Similarly, the key bytes in a run whose first key byte is at an even
    // position are only those at even positions in the run.

    PabloAST * odd_positions = pb.createRepeat(1, pb.getInteger(0xAA, 8));
    PabloAST * even_positions = pb.createRepeat(1, pb.getInteger(0x55, 8));
    PabloAST * GB_key1_odd = pb.createAnd(GB_key1, odd_positions, "gb_key1_odd");
    PabloAST * GB_key1_even = pb.createAnd(GB_key1, even_positions, "gb_key1_even");

    PabloAST * x81_FE_run_odd = pb.createAnd(x81_FE, pb.createMatchStar(GB_key1_odd, x81_FE));
    PabloAST * x81_FE_run_even = pb.createAnd(x81_FE, pb.createMatchStar(GB_key1_even, x81_FE));
    PabloAST * GB_keys_odd = pb.createAnd(x81_FE_run_odd, odd_positions, "gb_keys_odd");
    PabloAST * GB_keys_even = pb.createAnd(x81_FE_run_even, even_positions, "gb_keys_even");
    PabloAST * GB_keys = pb.createOr(GB_keys_odd, GB_keys_even, "gb_keys");
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

    PabloAST * odd_pairs = pb.createRepeat(1, pb.getInteger(0xCC, 8));
    PabloAST * even_pairs = pb.createRepeat(1, pb.getInteger(0x33, 8));
    PabloAST * GB_k2_1_odd = pb.createAnd(GB_k2_1, odd_pairs);
    PabloAST * GB_k2_1_even = pb.createAnd(GB_k2_1, even_pairs);

    PabloAST * key_or_data_4 = pb.createOr(GB_keys, GB_4_data);

    PabloAST * data4_run_odd = pb.createMatchStar(GB_k2_1_odd, key_or_data_4);
    PabloAST * data4_run_even = pb.createMatchStar(GB_k2_1_even, key_or_data_4);

    PabloAST * GB_k2_odd = pb.createAnd(data4_run_odd, odd_pairs);
    PabloAST * GB_k2_even = pb.createAnd(data4_run_even, even_pairs);
    PabloAST * GB_k2 = pb.createAnd(GB_4_data, pb.createOr(GB_k2_odd, GB_k2_even), "gb_k2");
    //
    //  All bytes that are neither key nor data of 2 or 4 byte sequences
    //  are ASCII or error.
    //
    PabloAST * GB_single = pb.createInFile(pb.createNot(pb.createOr(GB_keys, GB_data), "gb_single"));
    //
    // GB prefix bytes are the keys that are first in 2 or 4 byte sequences.
    PabloAST * GB_prefix = pb.createAnd(GB_keys, pb.createNot(pb.createAdvance(GB_k2, 1)), "gb_prefix");
    PabloAST * GB2_final = pb.createAnd(pb.createAdvance(GB_prefix, 1), pb.createNot(GB_k2));
    PabloAST * GB4_final = pb.createAdvance(GB_k2, 2);
    PabloAST * GB_final = pb.createOr3(GB_single, GB2_final, GB4_final);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_ASCII"), 0), GB_single);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB2_final"), 0), GB2_final);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB4_final"), 0), GB4_final);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_final"), 0), GB_final);
}

class GB_18030_Derived : public pablo::PabloKernel {
public:
    GB_18030_Derived(const std::unique_ptr<KernelBuilder> & kb, StreamSet * GB2_final, StreamSet * GB4_final, StreamSet * GB2_prefix, StreamSet * GB4_prefix, StreamSet * GB4_scope2, StreamSet * GB4_scope3);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_Derived::GB_18030_Derived (const std::unique_ptr<KernelBuilder> & b, StreamSet * GB2_final, StreamSet * GB4_final, StreamSet * GB2_prefix, StreamSet * GB4_prefix, StreamSet * GB4_scope2, StreamSet * GB4_scope3)
: PabloKernel(b, "GB_18030_Derived",
{Binding{"GB2_final", GB2_final, FixedRate(), LookAhead(1)}, Binding{"GB4_final", GB4_final, FixedRate(), LookAhead(3)}},
              {Binding{"GB2_prefix", GB2_prefix}, Binding{"GB4_prefix", GB4_prefix}, Binding{"GB4_scope2", GB4_scope2}, Binding{"GB4_scope3", GB4_scope3}}) {}

void GB_18030_Derived::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * GB2_final = pb.createExtract(getInputStreamVar("GB2_final"), pb.getInteger(0));
    PabloAST * GB4_final = pb.createExtract(getInputStreamVar("GB4_final"), pb.getInteger(0));
    PabloAST * GB2_prefix = pb.createLookahead(GB2_final, 1);
    PabloAST * GB4_prefix = pb.createLookahead(GB4_final, 3);
    PabloAST * GB4_scope2 = pb.createLookahead(GB4_final, 2);
    PabloAST * GB4_scope3 = pb.createLookahead(GB4_final, 1);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB2_prefix"), 0), GB2_prefix);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB4_prefix"), 0), GB4_prefix);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB4_scope2"), 0), GB4_scope2);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB4_scope3"), 0), GB4_scope3);
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
    PabloAST * mask1 = pb.createOr(GB_single, GB_prefix, "gb_mask1");
    //
    // mask2 is for selecting the low 4 bits of the 2nd byte of a sequence (or first of a singleton).
    PabloAST * GB_second = pb.createAdvance(GB_prefix, 1);
    PabloAST * mask2 = pb.createOr(GB_single, GB_second, "gb_mask2");
    // mask3 is for the last byte of data for every GB 1 or 2 byte sequence, or the 3rd byte
    // of every 4 byte sequence.
    PabloAST * GB_2of2 = pb.createAnd(GB_second, pb.createNot(GB_k2));
    PabloAST * GB_3of4 = pb.createAdvance(pb.createAnd(GB_second, GB_k2), 1);
    PabloAST * mask3 = pb.createOr(GB_single, pb.createOr(GB_2of2, GB_3of4), "gb_mask3");
    // mask4 is for selecting the low 4 bits of the last byte of a sequence.
    PabloAST * mask4 = pb.createOr(GB_single, pb.createOr(GB_2of2, pb.createAdvance(GB_3of4, 1)), "gb_mask4");
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_1"), pb.getInteger(0)), mask1);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_2"), pb.getInteger(0)), mask2);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_3"), pb.getInteger(0)), mask3);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_4"), pb.getInteger(0)), mask4);
}

class GB_18030_DoubleByteIndex : public pablo::PabloKernel {
public:
    GB_18030_DoubleByteIndex(const std::unique_ptr<KernelBuilder> & kb,
                             StreamSet * GB2_basis14_8, StreamSet * GB2_basis7_0, StreamSet * gb15_index);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_DoubleByteIndex::GB_18030_DoubleByteIndex
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * GB2_basis14_8, StreamSet * GB2_basis7_0, StreamSet * gb15_index)
: PabloKernel(kb, "GB_18030_DoubleByteIndex",
              // input
{Binding{"GB2_basis14_8", GB2_basis14_8}, Binding{"GB2_basis7_0", GB2_basis7_0}},
              // output
{Binding{"gb15_index", gb15_index}}) {
}

void GB_18030_DoubleByteIndex::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);

    std::vector<PabloAST *> GB2_basis14_8 = getInputStreamSet("GB2_basis14_8");
    std::vector<PabloAST *> GB2_basis7_0 = getInputStreamSet("GB2_basis7_0");

    // The valid values for the second byte of a 2-byte GB sequence are 0x40-7F and 0x80-0xFE.
    // Normalize these values to the range 0 through 190.
    BixNum x80 = {GB2_basis7_0[7]};
    BixNum b2 = bnc.SubModular(bnc.SubModular(GB2_basis7_0, x80), 0x40);

    // The valid values for the first byte of a 2-byte GB sequence are 0x81-0xFE.  Normalize
    // to the range 0-125 as seven-bit value.
    BixNum b1 = bnc.SubModular(bnc.Truncate(GB2_basis14_8, 7), 0x1);
    // Now compute the GB 2-byte index value:  190 * b1 + b2, as a 15-bit quantity.
    BixNum GB2idx = bnc.AddModular(bnc.MulFull(b1, 190), b2);

    Var * const gb15_index = getOutputStreamVar("gb15_index");
    for (unsigned i = 0; i < 15; i++) {
        pb.createAssign(pb.createExtract(gb15_index, pb.getInteger(i)), GB2idx[i]);
    }
}


class GB_18030_InitializeASCII : public pablo::PabloKernel {
public:
    GB_18030_InitializeASCII(const std::unique_ptr<KernelBuilder> & kb,
                             StreamSet * ASCII, StreamSet * byte1_basis, StreamSet * u16_basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    const unsigned ASCII_Bits = 7;
protected:
    void generatePabloMethod() override;
};

GB_18030_InitializeASCII::GB_18030_InitializeASCII
(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * ASCII, StreamSet * byte1_basis, StreamSet * u16_basis)
: PabloKernel(kb, "GB_18030_InitializeASCII",
              // input
{Binding{"ASCII", ASCII}, Binding{"byte1_basis", byte1_basis}},
              // output
{Binding{"u16_basis", u16_basis}}) {
}

void GB_18030_InitializeASCII::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * ASCII = pb.createExtract(getInputStreamVar("ASCII"), pb.getInteger(0));
    std::vector<PabloAST *> byte1_basis = getInputStreamSet("byte1_basis");

    // Initialize 16 bit stream variables with ASCII values.
    PabloAST * zeroes = pb.createZeroes();
    Var * const u16_output = getOutputStreamVar("u16_basis");
    for (unsigned i = 0; i < ASCII_Bits; ++i) {
        pb.createAssign(pb.createExtract(u16_output, pb.getInteger(i)), pb.createAnd(ASCII, byte1_basis[i]));
    }
    for (unsigned i = ASCII_Bits; i < 16; ++i) {
        pb.createAssign(pb.createExtract(u16_output, pb.getInteger(i)), zeroes);
    }
}

class GB_18030_DoubleByteRangeKernel : public pablo::PabloKernel {
public:
    GB_18030_DoubleByteRangeKernel(const std::unique_ptr<KernelBuilder> & kb,
                                   StreamSet * gb15_index, StreamSet * u16_in,
                                   StreamSet * u16_out,
                                   unsigned rangeBase, unsigned rangeBits);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    unsigned mRangeBase;
    unsigned mRangeBits;
    bool mHasU16input;
};

Bindings doubleByteKernelInputBindings(StreamSet * gb15_index, StreamSet * u16_in) {
    if (u16_in == nullptr) {
        return {Binding{"gb15_index", gb15_index}};
    }
    else {
        return {Binding{"gb15_index", gb15_index}, Binding{"u16_in", u16_in}};
    }
}


GB_18030_DoubleByteRangeKernel::GB_18030_DoubleByteRangeKernel
(const std::unique_ptr<KernelBuilder> & kb,
 StreamSet * gb15_index, StreamSet * u16_in,
 StreamSet * u16_out,
 unsigned rangeBase, unsigned rangeBits)
: PabloKernel(kb, "GB_18030_DoubleByteRangeKernel" + std::to_string(rangeBase) + "-" + std::to_string(rangeBase + (1 << rangeBits) - 1),
              // input
doubleByteKernelInputBindings(gb15_index, u16_in),
              // output
{Binding{"u16_out", u16_out}}), mRangeBase(rangeBase), mRangeBits(rangeBits), mHasU16input(u16_in != nullptr) {
}

void GB_18030_DoubleByteRangeKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    BixNum GB2idx = getInputStreamSet("gb15_index");

    std::vector<Var *> u16(16);
    if (mHasU16input) {
        BixNum u16_in = getInputStreamSet("u16_in");
        for (unsigned i = 0; i < u16.size(); ++i) {
            u16[i] = pb.createVar("u16" + std::to_string(i), u16_in[i]);
        }
    } else {
        PabloAST * zeroes = pb.createZeroes();
        for (unsigned i = 0; i < u16.size(); ++i) {
            u16[i] = pb.createVar("u16" + std::to_string(i), zeroes);
        }
    }

    //  Double byte sequences use a lookup table, with codepoints determined
    //  according to a calculated index.

    std::vector<UCD::codepoint_t> GB_tbl = get_GB_DoubleByteTable();
    const unsigned maxGB2limit = GB_tbl.size();

    BixNum kernelSelectBasis = bnc.HighBits(GB2idx, GB2idx.size()-mRangeBits);
    unsigned rangeLimit = mRangeBase + (1 << mRangeBits);
    PabloAST * inRange = bnc.EQ(kernelSelectBasis, mRangeBase >> mRangeBits);
    if (rangeLimit > maxGB2limit) {
        unsigned aboveBaseLimit = maxGB2limit % (1 << mRangeBits);
        BixNum aboveBase = bnc.Truncate(GB2idx, mRangeBits);
        inRange = pb.createAnd(inRange, bnc.ULT(aboveBase, aboveBaseLimit));
        rangeLimit = maxGB2limit;
    }

    BixNumTableCompiler tableCompiler(GB_tbl, GB2idx, u16);
    std::vector<unsigned> partitionLevels = {mRangeBits, 9, 7, 5};
    tableCompiler.setRecursivePartitionLevels(partitionLevels);
    PabloBuilder nb = pb.createScope();
    tableCompiler.compileSubTable(nb, mRangeBase, inRange);
    pb.createIf(inRange, nb);


    Var * const u16_out = getOutputStreamVar("u16_out");
    for (unsigned i = 0; i < 16; i++) {
        pb.createAssign(pb.createExtract(u16_out, pb.getInteger(i)), u16[i]);
    }
}

class GB_18030_FourByteLogic : public pablo::PabloKernel {
public:
    GB_18030_FourByteLogic(const std::unique_ptr<KernelBuilder> & kb,
                           StreamSet * GB4_basis21_15, StreamSet * GB2_basis14_11, StreamSet * GB4_basis10_4, StreamSet * GB4_basis3_0,
                           StreamSet * GB4_results);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_FourByteLogic::GB_18030_FourByteLogic
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * GB4_basis21_15, StreamSet * GB2_basis14_11, StreamSet * GB4_basis10_4, StreamSet * GB4_basis3_0,
 StreamSet * GB4_results)
: PabloKernel(kb, "GB_18030_FourByteLogic",
              // input
{Binding{"GB4_basis21_15", GB4_basis21_15},
    Binding{"GB2_basis14_11", GB2_basis14_11},
    Binding{"GB4_basis10_4", GB4_basis10_4},
    Binding{"GB4_basis3_0", GB4_basis3_0}
},
              // output
{Binding{"GB4_results", GB4_results}}) {
}

void GB_18030_FourByteLogic::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNum GB4_basis21_15 = getInputStreamSet("GB4_basis21_15");
    BixNum GB2_basis14_11 = getInputStreamSet("GB2_basis14_11");
    BixNum GB4_basis10_4 = getInputStreamSet("GB4_basis10_4");
    BixNum GB4_basis3_0 = getInputStreamSet("GB4_basis3_0");

    const unsigned BMP_bits = 16;
    const unsigned total_bits = 21;

    BixVar u16;
    u16.resize(BMP_bits);
    BixVar high5;
    high5.resize(total_bits - BMP_bits);
    PabloAST * zeroes = pb.createZeroes();
    for (unsigned i = 0; i < BMP_bits; ++i) {
        u16[i] = pb.createVar("u21_" + std::to_string(i), zeroes);
    }
    for (unsigned i = BMP_bits; i < total_bits; i++) {
        high5[i - BMP_bits] = pb.createVar("u21_" + std::to_string(i), zeroes);
    }

    BixNumCompiler bnc(pb);
    BixNum lo11 = bnc.MulFull(bnc.SubModular(GB4_basis10_4, 1), 10);
    lo11 = bnc.AddModular(lo11, GB4_basis3_0);
    BixNum lo15 = bnc.AddModular(bnc.MulFull(GB2_basis14_11, 1260), lo11);

    BixNum SMP_base = bnc.MulFull(bnc.SubModular(GB4_basis21_15, 0x10), 12600);
    SMP_base = bnc.AddModular(SMP_base, 0x10000);
    BixNum byte1_X_12600 = bnc.MulFull(bnc.SubModular(GB4_basis21_15, 1), 12600);
    BixNum GB_val = bnc.AddModular(byte1_X_12600, lo15);
    BixNum SMP_offset = bnc.AddModular(SMP_base, lo15);

    PabloAST * aboveBMP = bnc.UGE(GB4_basis21_15, 0x10);

    BixNumRangeTableCompiler tableCompiler(get_GB_RangeTable(), 39419, GB_val, u16);
    // This controls the insertion of an if-hierarchy for table lookup.
    // Each level deals with partitions of size 1 << mPartitionBits[k];
    std::vector<unsigned> partitionLevels = {16, 13, 9, 5};
    tableCompiler.setRecursivePartitionLevels(partitionLevels);
    tableCompiler.compileTable(pb, pb.createNot(aboveBMP));

    for (unsigned i = 0; i < BMP_bits; i++) {
        pb.createAssign(u16[i], pb.createOr(u16[i], pb.createAnd(SMP_offset[i], aboveBMP)));
    }
    for (unsigned i = BMP_bits; i < total_bits; i++) {
        pb.createAssign(high5[i - BMP_bits], pb.createAnd(SMP_offset[i], aboveBMP));
    }
    Var * const GB4_results = getOutputStreamVar("GB4_results");
    for (unsigned i = 0; i < BMP_bits; i++) {
        pb.createAssign(pb.createExtract(GB4_results, pb.getInteger(i)), u16[i]);
    }
    for (unsigned i = BMP_bits; i < total_bits; i++) {
        pb.createAssign(pb.createExtract(GB4_results, pb.getInteger(i)), high5[i - BMP_bits]);
    }
}

void deposit(const std::unique_ptr<ProgramBuilder> & P, Scalar * const base, const unsigned count, StreamSet * mask, StreamSet * inputs, StreamSet * outputs) {
    StreamSet * const expanded = P->CreateStreamSet(count);
    P->CreateKernelCall<StreamExpandKernel>(base, inputs, mask, expanded);
    if (AVX2_available() && BMI2_available()) {
        P->CreateKernelCall<PDEPFieldDepositKernel>(mask, expanded, outputs);
    } else {
        P->CreateKernelCall<FieldDepositKernel>(mask, expanded, outputs);
    }
}

void extract(const std::unique_ptr<ProgramBuilder> & P, StreamSet * inputSet, Scalar * inputBase, StreamSet * mask, StreamSet * outputs) {
    unsigned fw = 64;  // Best for PEXT extraction.
    StreamSet * const compressed = P->CreateStreamSet(outputs->getNumElements());
    P->CreateKernelCall<FieldCompressKernel>(fw, inputBase, inputSet, mask, compressed);
    P->CreateKernelCall<StreamCompressKernel>(compressed, mask, outputs, fw);
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

    StreamSet * const GB_ASCII = P->CreateStreamSet(1);
    StreamSet * const GB2_final = P->CreateStreamSet(1);
    StreamSet * const GB4_final = P->CreateStreamSet(1);
    StreamSet * const GB_final = P->CreateStreamSet(1);
    P->CreateKernelCall<GB_18030_ClassifyBytes>(BasisBits, GB_ASCII, GB2_final, GB4_final, GB_final);

    StreamSet * const GB2_prefix = P->CreateStreamSet(1);
    StreamSet * const GB4_prefix = P->CreateStreamSet(1);
    StreamSet * const GB4_scope2 = P->CreateStreamSet(1);
    StreamSet * const GB4_scope3 = P->CreateStreamSet(1);
    P->CreateKernelCall<GB_18030_Derived>(GB2_final, GB4_final, GB2_prefix, GB4_prefix, GB4_scope2, GB4_scope3);

    StreamSet * const ASCII_deposit_mask = P->CreateStreamSet(1);
    StreamSet * const GB2_deposit_mask = P->CreateStreamSet(1);
    StreamSet * const GB4_deposit_mask = P->CreateStreamSet(1);
    Scalar * ZERO = P->CreateConstant(b->getSize(0));
    extract(P, GB_ASCII, ZERO, GB_final, ASCII_deposit_mask);
    extract(P, GB2_final, ZERO, GB_final, GB2_deposit_mask);
    extract(P, GB4_final, ZERO, GB_final, GB4_deposit_mask);

    StreamSet * const ASCII_basis = P->CreateStreamSet(7);
    extract(P, BasisBits, ZERO, GB_ASCII, ASCII_basis);
    StreamSet * const ASCII_deposit = P->CreateStreamSet(7);
    deposit(P, P->CreateConstant(b->getSize(0)), 7, ASCII_deposit_mask, ASCII_basis, ASCII_deposit);

    StreamSet * const GB2_basis14_8 = P->CreateStreamSet(7);
    StreamSet * const GB2_basis7_0 = P->CreateStreamSet(8);
    extract(P, BasisBits, ZERO, GB2_prefix, GB2_basis14_8);
    extract(P, BasisBits, ZERO, GB2_final, GB2_basis7_0);

    StreamSet * const gb15index = P->CreateStreamSet(15);
    P->CreateKernelCall<GB_18030_DoubleByteIndex>(GB2_basis14_8, GB2_basis7_0, gb15index);

    const unsigned KernelSubrangeBits = 11;
    const unsigned KernelSubrangeSize = 1 << KernelSubrangeBits;
    const unsigned GB2_tblSize = get_GB_DoubleByteTable().size();

    StreamSet * gb2_accum = nullptr;
    for (unsigned rangeBase = 0; rangeBase < GB2_tblSize; rangeBase += KernelSubrangeSize) {
        //llvm::errs() << "rangeBase = " << rangeBase << "\n";
        StreamSet * gb2_results = P->CreateStreamSet(16);
        P->CreateKernelCall<GB_18030_DoubleByteRangeKernel>(gb15index, gb2_accum, gb2_results, rangeBase, KernelSubrangeBits);
        gb2_accum = gb2_results;
    }

    StreamSet * const gb2_deposit = P->CreateStreamSet(16);
    deposit(P, P->CreateConstant(b->getSize(0)), 16, GB2_deposit_mask, gb2_accum, gb2_deposit);

    StreamSet * const GB4_basis21_15 = P->CreateStreamSet(7);
    StreamSet * const GB2_basis14_11 = P->CreateStreamSet(4);
    StreamSet * const GB4_basis10_4 = P->CreateStreamSet(7);
    StreamSet * const GB4_basis3_0 = P->CreateStreamSet(4);
    extract(P, BasisBits, ZERO, GB4_prefix, GB4_basis21_15);
    extract(P, BasisBits, ZERO, GB4_scope2, GB2_basis14_11);
    extract(P, BasisBits, ZERO, GB4_scope3, GB4_basis10_4);
    extract(P, BasisBits, ZERO, GB4_final, GB4_basis3_0);

    StreamSet * const GB4_results = P->CreateStreamSet(21);
    P->CreateKernelCall<GB_18030_FourByteLogic>(GB4_basis21_15, GB2_basis14_11, GB4_basis10_4, GB4_basis3_0, GB4_results);

    StreamSet * const gb4_deposit = P->CreateStreamSet(21);
    deposit(P, P->CreateConstant(b->getSize(0)), 21, GB4_deposit_mask, GB4_results, gb4_deposit);

    StreamSet * const u32basis = P->CreateStreamSet(21);
    std::vector<StreamSet *> deposited = {ASCII_deposit, gb2_deposit, gb4_deposit};
    P->CreateKernelCall<StreamsMerge>(deposited, u32basis);


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

    deposit(P, P->CreateConstant(b->getSize(18)), 3, u8initial, u32basis, deposit18_20);
    deposit(P, P->CreateConstant(b->getSize(12)), 6, u8mask12_17, u32basis, deposit12_17);
    deposit(P, P->CreateConstant(b->getSize(6)), 6, u8mask6_11, u32basis, deposit6_11);
    deposit(P, P->CreateConstant(b->getSize(0)), 6, u8final, u32basis, deposit0_5);

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
    codegen::ParseCommandLineOptions(argc, argv, {&gb18030Options, pablo::pablo_toolchain_flags(), codegen::codegen_flags()});
    CPUDriver pxDriver("gb18030");
    gb18030FunctionType gb18030Function = generatePipeline(pxDriver);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        gb18030Function(fd, inputFile.c_str());
        close(fd);
    }
    return 0;
}

