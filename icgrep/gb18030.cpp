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
#include <kernels/deletion.h>
#include <kernels/pdep_kernel.h>
#include <kernels/error_monitor_kernel.h>
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
enum class GBerrorMode {Abort, DropBadInput, UseReplacementChar};
static cl::opt<GBerrorMode> GBerrorOption(cl::desc("Treatment of erroneous GB 18030 input (default Abort)"),
    cl::values(
        clEnumValN(GBerrorMode::Abort, "abort-on-error", "generate output for all valid GB18030 input up to the first error"),
        clEnumValN(GBerrorMode::DropBadInput, "drop-bad-characters", "drop bad GB18030 characters from the input"),
        clEnumValN(GBerrorMode::UseReplacementChar, "use-replacement-char", "replace bad input characters with a replacement character")
        CL_ENUM_VAL_SENTINEL), cl::cat(gb18030Options), cl::init(GBerrorMode::Abort));

static cl::opt<unsigned> ReplacementCharacter("replacement-character", cl::desc("Codepoint value of the character used to replace any unmapped or invalid input sequence."), cl::init(0xFFFD), cl::cat(gb18030Options));
static cl::opt<std::string> OutputEncoding("encoding", cl::desc("Output encoding (default: UTF-8)"), cl::init("UTF-8"), cl::cat(gb18030Options));

// Parse GB 10830 encoded data to identify 1-, 2- and 4-byte sequences,
// as well as to check that sequences are well-formed.  The following
// output marks are produced.
// scope2: positions at which the end of a 2-byte sequence is expected.
// scope4: positions at which the end of a 4-byte sequence is expected.
// checked: positions that mark the end of a well-formed 1-, 2- or 4-byte sequence.
// Thus: checked & scope2 gives the positions of well-formed 2-byte sequences.
//       checked & scope4 gives the positions of well-formed 4-byte sequences.
//       checked &~ (scope2|scope4) gives the positions of ASCII single bytes.
// Positions not marked as checked are either prefixes, 2nd or 3rd bytes of
// a 4-byte sequence or errors.
//
class GB_18030_Parser : public pablo::PabloKernel {
public:
    GB_18030_Parser(const std::unique_ptr<KernelBuilder> & kb, StreamSet * basis, StreamSet * GB_marks);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_Parser::GB_18030_Parser (const std::unique_ptr<KernelBuilder> & b, StreamSet * basis, StreamSet * GB_marks)
: PabloKernel(b, "GB_18030_Parser",
{Binding{"basis", basis}},
              {Binding{"GB_marks", GB_marks}}) {}

void GB_18030_Parser::generatePabloMethod() {
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
    PabloAST * wf1 = pb.createAnd(GB_single, ccc.compileCC(makeByte(0, 0x7F)));
    PabloAST * scope3 = pb.createAdvance(GB_k2, 1);
    PabloAST * GB_prefix = pb.createAnd(GB_keys, pb.createNot(scope3), "gb_prefix");
    PabloAST * scope2 = pb.createAnd(pb.createAdvance(GB_prefix, 1), pb.createNot(GB_k2), "gb_scope2");
    // A well formed 2 byte sequence has a key 0x81-0xFE byte followed by a byte in 0x40-0xFE except 0x7F.
    PabloAST * wf2 = pb.createAnd(scope2, ccc.compileCC(makeCC(makeByte(0x040, 0x7E), makeByte(0x080, 0xFE))));
    PabloAST * scope4 = pb.createAdvance(scope3, 1);
    // A well formed 4 byte sequence has two consecutive pairs of 0x81-FE 0x30-39 bytes
    PabloAST * wf4 = pb.createAnd(scope4, GB_4_data, "gb_wf4");
    PabloAST * checked = pb.createOr3(wf1, wf2, wf4, "GB_checked");
    Var * GB_marks = getOutputStreamVar("GB_marks");
    pb.createAssign(pb.createExtract(GB_marks, 0), scope2);
    pb.createAssign(pb.createExtract(GB_marks, 1), scope4);
    pb.createAssign(pb.createExtract(GB_marks, 2), checked);
}

//
// Extraction masks gather bits from GB 18030 code sequences so
// that one bit position is allocated for each Unicode codepoint
// (codepoint-indexing) or one bit position is gather for each
// UTF-16 code unit (u16-indexing).
//
enum class GB_18030_IndexingKind {Codepoint, UTF16};

class GB_18030_ExtractionMasks : public pablo::PabloKernel {
public:
    GB_18030_ExtractionMasks(const std::unique_ptr<KernelBuilder> & kb, StreamSet * GB_marks, StreamSet * basis, StreamSet * GB_1, StreamSet * GB_2, StreamSet * GB_3, StreamSet * GB_4, StreamSet * GB_prefix4, StreamSet * error, GB_18030_IndexingKind k = GB_18030_IndexingKind::Codepoint);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    GB_18030_IndexingKind mExtractionKind;
};


GB_18030_ExtractionMasks::GB_18030_ExtractionMasks
    (const std::unique_ptr<KernelBuilder> & b,
     StreamSet * GB_marks, StreamSet * basis, StreamSet * GB_1, StreamSet * GB_2, StreamSet * GB_3, StreamSet * GB_4, StreamSet * GB_prefix4, StreamSet * error, GB_18030_IndexingKind k)
: PabloKernel(b, (k == GB_18030_IndexingKind::UTF16) ? "GB_18030_ExtractionMasks_u16" : "GB_18030_ExtractionMasks",
{Binding{"GB_marks", GB_marks, FixedRate(1), LookAhead(3)}, Binding{"basis", basis}},
{Binding{"GB_1", GB_1}, Binding{"GB_2", GB_2}, Binding{"GB_3", GB_3}, Binding{"GB_4", GB_4}, Binding{"GB_prefix4", GB_prefix4}, Binding{"error", error}}),
mExtractionKind(k)  {}

void GB_18030_ExtractionMasks::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> GB_marks = getInputStreamSet("GB_marks");
    PabloAST * GB_scope2 = pb.createExtract(getInputStreamVar("GB_marks"), pb.getInteger(0));
    PabloAST * GB_scope4 = pb.createExtract(getInputStreamVar("GB_marks"), pb.getInteger(1));
    PabloAST * GB_checked = pb.createExtract(getInputStreamVar("GB_marks"), pb.getInteger(2));
    PabloAST * GB_prefix2 = pb.createLookahead(GB_scope2, 1);
    PabloAST * GB_prefix4 = pb.createLookahead(GB_scope4, 3);
    PabloAST * GB_checkedPrefix2 = pb.createAnd(pb.createLookahead(GB_checked, 1), GB_prefix2);
    PabloAST * GB_checkedPrefix4 = pb.createAnd(pb.createLookahead(GB_checked, 3), GB_prefix4);
    PabloAST * GB_single = pb.createAnd(GB_checked, pb.createNot(pb.createOr(GB_scope2, GB_scope4)));
    PabloAST * GB_prefix = pb.createOr(GB_checkedPrefix2, GB_checkedPrefix4);
    //
    // mask1 is for the extraction of the first byte of data for every valid GB 1, 2 or 4 byte sequence.
    PabloAST * mask1 = pb.createOr(GB_single, GB_prefix, "gb_mask1");
    //
    // mask2 is for selecting the low 4 bits of the 2nd byte of a sequence (or first of a singleton).
    PabloAST * GB_second = pb.createAdvance(GB_prefix, 1);
    PabloAST * mask2 = pb.createOr(GB_single, GB_second, "gb_mask2");
    // mask3 is for the last byte of data for every GB 1 or 2 byte sequence, or the 3rd byte
    // of every 4 byte sequence.
    PabloAST * GB_2of2 = pb.createAnd(GB_scope2, GB_checked);
    PabloAST * GB_3of4 = pb.createAdvance(GB_checkedPrefix4, 2);
    PabloAST * mask3 = pb.createOr(GB_single, pb.createOr(GB_2of2, GB_3of4), "gb_mask3");
    // mask4 is for selecting the low 4 bits of the last byte of a sequence.
    PabloAST * mask4 = GB_checked;
    //  Any position not in one of the masks is an error.
    PabloAST * error1 = pb.createInFile(pb.createNot(pb.createOr(pb.createOr(mask1, mask2), pb.createOr(mask3, mask4))), "gb_error");
    if (mExtractionKind == GB_18030_IndexingKind::UTF16) {
        // In UTF16 mode, we update the masks to gather from 2 positions of every
        // GB 4 byte sequence that maps to Unicode codepoint above the basic
        // multilingual plane (i.e., > 0xFFFF).  These code points are each encoded
        // as a surrogate pair in UTF-16 (two code units).
        // mask1 includes data from bytes 1 and 2 of the sequence.
        // mask2 is for selecting the low 4 bits of bytes 2 and 3.
        // mask3 selects data from bytes 3 and 4.
        // mask4 selects data from bytes 1 and 4.
        cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), getInputStreamSet("basis"));
        PabloAST * aboveSMP = pb.createAnd(GB_checkedPrefix4, ccc.compileCC(makeByte(0x090, 0xFE)));
        mask1 = pb.createOr(mask1, pb.createAdvance(aboveSMP, 1), "gb_mask1_u16");
        mask2 = pb.createOr(mask2, pb.createAdvance(aboveSMP, 2), "gb_mask2_u16");
        mask3 = pb.createOr(mask3, pb.createAdvance(aboveSMP, 3), "gb_mask3_u16");
        mask4 = pb.createOr(mask4, aboveSMP, "gb_mask4_u16");
    }
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_1"), pb.getInteger(0)), mask1);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_2"), pb.getInteger(0)), mask2);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_3"), pb.getInteger(0)), mask3);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_4"), pb.getInteger(0)), mask4);
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_prefix4"), pb.getInteger(0)), GB_checkedPrefix4);
    pb.createAssign(pb.createExtract(getOutputStreamVar("error"), pb.getInteger(0)), error1);
}

class GB_18030_DoubleByteIndex : public pablo::PabloKernel {
public:
    GB_18030_DoubleByteIndex(const std::unique_ptr<KernelBuilder> & kb,
                             StreamSet * GB_4byte, StreamSet * byte1_basis, StreamSet * byte2_basis,
                             StreamSet * GB_2byte, StreamSet * gb15_index);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
};

GB_18030_DoubleByteIndex::GB_18030_DoubleByteIndex
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * GB_4byte, StreamSet * byte1_basis, StreamSet * byte2_basis, StreamSet * GB_2byte, StreamSet * gb15_index)
: PabloKernel(kb, "GB_18030_DoubleByteIndex",
              // input
{Binding{"GB_4byte", GB_4byte}, Binding{"byte1_basis", byte1_basis}, Binding{"byte2_basis", byte2_basis}},
              // output
{Binding{"GB_2byte", GB_2byte}, Binding{"gb15_index", gb15_index}}) {
}

void GB_18030_DoubleByteIndex::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * GB_4byte = pb.createExtract(getInputStreamVar("GB_4byte"), pb.getInteger(0));
    std::vector<PabloAST *> byte1_basis = getInputStreamSet("byte1_basis");
    std::vector<PabloAST *> byte2_basis = getInputStreamSet("byte2_basis");
    PabloAST * ASCII = bnc.ULT(byte1_basis, 0x80);
    PabloAST * GB_2byte = pb.createInFile(pb.createNot(pb.createOr(ASCII, GB_4byte), "gb_2byte"));
    pb.createAssign(pb.createExtract(getOutputStreamVar("GB_2byte"), pb.getInteger(0)), GB_2byte);

    // The valid values for the second byte of a 2-byte GB sequence are 0x40-7F and 0x80-0xFE.
    // Normalize these values to the range 0 through 190.
    BixNum x80 = {byte2_basis[7]};
    BixNum b2 = bnc.SubModular(bnc.SubModular(byte2_basis, x80), 0x40);

    // The valid values for the first byte of a 2-byte GB sequence are 0x81-0xFE.  Normalize
    // to the range 0-125 as seven-bit value.
    BixNum b1 = bnc.SubModular(bnc.Truncate(byte1_basis, 7), 0x1);
    // Now compute the GB 2-byte index value:  190 * b1 + b2, as a 15-bit quantity.
    BixNum GB2idx = bnc.AddModular(bnc.MulFull(b1, 190), b2);

    Var * const gb15_index = getOutputStreamVar("gb15_index");
    for (unsigned i = 0; i < 15; i++) {
        pb.createAssign(pb.createExtract(gb15_index, pb.getInteger(i)), pb.createAnd(GB2idx[i], GB_2byte, "gb2idx[" + std::to_string(i) + "]"));
    }
}


class GB_18030_InitializeASCII : public pablo::PabloKernel {
public:
    GB_18030_InitializeASCII(const std::unique_ptr<KernelBuilder> & kb,
                             StreamSet * byte1_basis, StreamSet * u16_basis);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
    const unsigned ASCII_Bits = 7;
protected:
    void generatePabloMethod() override;
};

GB_18030_InitializeASCII::GB_18030_InitializeASCII
(const std::unique_ptr<kernel::KernelBuilder> & kb, StreamSet * byte1_basis, StreamSet * u16_basis)
: PabloKernel(kb, "GB_18030_InitializeASCII",
              // input
{Binding{"byte1_basis", byte1_basis}},
              // output
{Binding{"u16_basis", u16_basis}}) {
}

void GB_18030_InitializeASCII::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> byte1_basis = getInputStreamSet("byte1_basis");
    PabloAST * ASCII = pb.createNot(byte1_basis[ASCII_Bits]);

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
                                   StreamSet * GB_2byte, StreamSet * gb15_index, StreamSet * u16_in,
                                   StreamSet * u16_out,
                                   unsigned rangeBase, unsigned rangeBits);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    unsigned mRangeBase;
    unsigned mRangeBits;
};

GB_18030_DoubleByteRangeKernel::GB_18030_DoubleByteRangeKernel
(const std::unique_ptr<KernelBuilder> & kb,
 StreamSet * GB_2byte, StreamSet * gb15_index, StreamSet * u16_in,
 StreamSet * u16_out,
 unsigned rangeBase, unsigned rangeBits)
: PabloKernel(kb, "GB_18030_DoubleByteRangeKernel" + std::to_string(rangeBase) + "-" + std::to_string(rangeBase + (1 << rangeBits) - 1),
              // input
{Binding{"GB_2byte", GB_2byte}, Binding{"gb15_index", gb15_index}, Binding{"u16_in", u16_in}},
              // output
{Binding{"u16_out", u16_out}}), mRangeBase(rangeBase), mRangeBits(rangeBits) {
}

void GB_18030_DoubleByteRangeKernel::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    PabloAST * GB_2byte = pb.createExtract(getInputStreamVar("GB_2byte"), pb.getInteger(0));
    BixNum GB2idx = getInputStreamSet("gb15_index");
    BixNum u16_in = getInputStreamSet("u16_in");

    std::vector<Var *> u16(u16_in.size());
    for (unsigned i = 0; i < u16.size(); ++i) {
        u16[i] = pb.createVar("u16" + std::to_string(i), u16_in[i]);
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
    inRange = pb.createAnd(GB_2byte, inRange, "gb_inRange_" + std::to_string(mRangeBase) + "-" + std::to_string(rangeLimit));

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
                           StreamSet * GB_4byte, StreamSet * byte1, StreamSet * nybble1, StreamSet * byte2, StreamSet * nybble2, StreamSet * u16_basis,
                           StreamSet * UTF_out,
                           GB_18030_IndexingKind k = GB_18030_IndexingKind::Codepoint);
    bool isCachable() const override { return true; }
    bool hasSignature() const override { return false; }
protected:
    void generatePabloMethod() override;
private:
    GB_18030_IndexingKind mIndexingKind;
};

GB_18030_FourByteLogic::GB_18030_FourByteLogic
(const std::unique_ptr<kernel::KernelBuilder> & kb,
 StreamSet * GB_4byte, StreamSet * byte1, StreamSet * nybble1, StreamSet * byte2, StreamSet * nybble2, StreamSet * u16_basis,
 StreamSet * UTF_out, GB_18030_IndexingKind k)
: PabloKernel(kb, (k == GB_18030_IndexingKind::UTF16) ? "GB_18030_FourByteLogic_u16" : "GB_18030_FourByteLogic",
              // input
{Binding{"GB_4byte", GB_4byte},
    Binding{"byte1", byte1},
    Binding{"nybble1", nybble1},
    Binding{"byte2", byte2},
    Binding{"nybble2", nybble2, FixedRate(1), LookAhead(3)},
    Binding{"u16_basis", u16_basis}
},
              // output
{Binding{"UTF_out", UTF_out}}), mIndexingKind(k) {
}

void GB_18030_FourByteLogic::generatePabloMethod() {
    std::string UTF_prefix = (mIndexingKind == GB_18030_IndexingKind::UTF16) ? "UTF16_" : "UTF32_";
    PabloBuilder pb(getEntryScope());
    
    PabloAST * GB_4byte = pb.createExtract(getInputStreamVar("GB_4byte"), pb.getInteger(0));
    BixNum byte1 = getInputStreamSet("byte1");
    BixNum nybble1 = getInputStreamSet("nybble1");
    BixNum byte2 = getInputStreamSet("byte2");
    BixNum nybble2 = getInputStreamSet("nybble2");
    BixNum u16_in = getInputStreamSet("u16_basis");

    const unsigned BMP_bits = 16;
    const unsigned total_bits = 21;

    BixVar u16;
    u16.resize(BMP_bits);
    BixVar high5;
    high5.resize(total_bits - BMP_bits);
    PabloAST * zeroes = pb.createZeroes();
    for (unsigned i = 0; i < BMP_bits; ++i) {
        u16[i] = pb.createVar(UTF_prefix + std::to_string(i), u16_in[i]);
    }
    if (mIndexingKind == GB_18030_IndexingKind::Codepoint) {
        for (unsigned i = BMP_bits; i < total_bits; i++) {
            high5[i - BMP_bits] = pb.createVar(UTF_prefix + std::to_string(i), zeroes);
        }
    }

    PabloBuilder nb = pb.createScope();
    BixNumCompiler bnc(nb);
    
    BixNum GB4_basis21_15(7);
    BixNum GB4_basis14_11(4);
    BixNum GB4_basis10_4(7);
    BixNum GB4_basis3_0(4);
    
    for (unsigned i = 0; i < 7; i++) {
        GB4_basis21_15[i] = nb.createAnd(GB_4byte, byte1[i], "gb_byte1");
    }
    for (unsigned i = 0; i < 4; i++) {
        GB4_basis14_11[i] = nb.createAnd(GB_4byte, nybble1[i], "gb_nybble1");
    }
    for (unsigned i = 0; i < 7; i++) {
        GB4_basis10_4[i] = nb.createAnd(GB_4byte, byte2[i], "gb_byte2");
    }
    for (unsigned i = 0; i < 4; i++) {
        GB4_basis3_0[i] = nb.createAnd(GB_4byte, nybble2[i], "gb_nybble2");
    }
    PabloAST * aboveBMP = bnc.UGE(GB4_basis21_15, 0x10);

    if (mIndexingKind == GB_18030_IndexingKind::UTF16) {
        // In UTF16 mode, input stream sets have additional data for surrogate pairs.
        // Extracted mask1 data in byte1 has data at GB byte positions 1 and 2,
        // extracted mask2 data in nybble1 contains data selected GB byte positions 2 and 3
        // extracted mask3 data in byte2 contains data selected GB byte positions 3 and 4
        // extracted mask4 data in nybble2 contains data selected GB byte positions 2 and 4
        //
        // We assemble the correct GB data at each position.
        PabloAST * surrogate2 = nb.createAdvance(aboveBMP, 1, "gb_surrogate2");
        for (unsigned i = 0; i < 7; i++) {
            GB4_basis21_15[i] = nb.createSel(surrogate2, nb.createAdvance(GB4_basis21_15[i], 1), GB4_basis21_15[i], "gb_21_15");
        }
        for (unsigned i = 0; i < 4; i++) {
            GB4_basis14_11[i] = nb.createSel(surrogate2, nb.createAdvance(GB4_basis14_11[i], 1), GB4_basis14_11[i], "gb_14_11");
        }
        for (unsigned i = 0; i < 7; i++) {
            GB4_basis10_4[i] = nb.createSel(surrogate2, nb.createAdvance(GB4_basis10_4[i], 1), GB4_basis10_4[i], "gb_10_4");
        }
        for (unsigned i = 0; i < 4; i++) {
            GB4_basis3_0[i] = nb.createSel(aboveBMP, nb.createLookahead(nybble2[i], 1), GB4_basis3_0[i]);
            GB4_basis3_0[i] = nb.createSel(surrogate2, nybble2[i], GB4_basis3_0[i], "gb_3_0");
        }
    }

    // Now we calculate the 21 bit GB4 index value.
    BixNum lo11 = bnc.MulFull(bnc.SubModular(GB4_basis10_4, 1), 10);
    lo11 = bnc.AddModular(lo11, GB4_basis3_0);
    BixNum lo15 = bnc.AddModular(bnc.MulFull(GB4_basis14_11, 1260), lo11);
    BixNum byte1_X_12600 = bnc.MulFull(bnc.SubModular(GB4_basis21_15, 1), 12600);
    BixNum GB_val = bnc.AddModular(byte1_X_12600, lo15);

    // First deal with any GB 4 byte values above the basic multilingual plane.
    // The logic here differs for codepoint vs. UTF16 indexing.
    BixNum SMP_base = bnc.MulFull(bnc.SubModular(GB4_basis21_15, 0x10), 12600);
    BixNum SMP_offset = bnc.AddModular(SMP_base, lo15);
    //BixNum SMP_offset = bnc.SubModular(GB_val, 189000);

    if (mIndexingKind == GB_18030_IndexingKind::UTF16) {
        PabloAST * surrogate1 = aboveBMP;
        PabloAST * surrogate2 = nb.createAdvance(surrogate1, 1);
        PabloAST * surrogate = nb.createOr(surrogate1, surrogate2);
        // set the high six bits for surrogates: D8, DC
        nb.createAssign(u16[15], nb.createOr(u16[15], surrogate));
        nb.createAssign(u16[14], nb.createOr(u16[14], surrogate));
        // no setting needed for bit 13
        nb.createAssign(u16[12], nb.createOr(u16[12], surrogate));
        nb.createAssign(u16[11], nb.createOr(u16[11], surrogate));
        nb.createAssign(u16[10], nb.createOr(u16[10], surrogate2));
        
        for (unsigned i = 0; i < 10; i++) {
            PabloAST * surrogate_bit = nb.createSel(surrogate1, SMP_offset[i+10], SMP_offset[i]);
            nb.createAssign(u16[i], nb.createSel(surrogate, surrogate_bit, u16[i]));
        }
    } else {
        SMP_offset = bnc.AddModular(SMP_offset, 0x10000);

        for (unsigned i = 0; i < BMP_bits; i++) {
            nb.createAssign(u16[i], nb.createOr(u16[i], nb.createAnd(SMP_offset[i], aboveBMP)));
        }
        for (unsigned i = BMP_bits; i < total_bits; i++) {
            nb.createAssign(high5[i - BMP_bits], nb.createAnd(SMP_offset[i], aboveBMP));
        }
    }

    BixNumRangeTableCompiler tableCompiler(get_GB_RangeTable(), 39419, GB_val, u16);
    // This controls the insertion of an if-hierarchy for table lookup.
    // Each level deals with partitions of size 1 << mPartitionBits[k];
    std::vector<unsigned> partitionLevels = {16, 13, 9, 5};
    tableCompiler.setRecursivePartitionLevels(partitionLevels);
    tableCompiler.compileTable(nb, nb.createAnd(GB_4byte, nb.createNot(aboveBMP)));

    pb.createIf(GB_4byte, nb);
    Var * const UTF_out = getOutputStreamVar("UTF_out");
    for (unsigned i = 0; i < BMP_bits; i++) {
        pb.createAssign(pb.createExtract(UTF_out, pb.getInteger(i)), u16[i]);
    }
    if (mIndexingKind == GB_18030_IndexingKind::Codepoint) {
        for (unsigned i = BMP_bits; i < total_bits; i++) {
            pb.createAssign(pb.createExtract(UTF_out, pb.getInteger(i)), high5[i - BMP_bits]);
        }
    }
}

typedef void (*gb18030FunctionType)(uint32_t fd, const char *);

gb18030FunctionType generatePipeline(CPUDriver & pxDriver, unsigned encodingBits, cc::ByteNumbering byteNumbering) {
    auto & b = pxDriver.getBuilder();
    auto P = pxDriver.makePipeline({Binding{b->getInt32Ty(), "inputFileDecriptor"}, Binding{b->getInt8PtrTy(), "outputFileName"}}, {});
    Scalar * fileDescriptor = P->getInputScalar("inputFileDecriptor");
    // File data from mmap
    StreamSet * const ByteStream = P->CreateStreamSet(1, 8);
    P->CreateKernelCall<MMapSourceKernel>(fileDescriptor, ByteStream);

    // Transposed bits from s2p
    StreamSet * BasisBits = P->CreateStreamSet(8);
    P->CreateKernelCall<S2PKernel>(ByteStream, BasisBits);

    GB_18030_IndexingKind indexing = (encodingBits == 16) ? GB_18030_IndexingKind::UTF16 : GB_18030_IndexingKind::Codepoint;

    StreamSet * GB_marks = P->CreateStreamSet(4);
    P->CreateKernelCall<GB_18030_Parser>(BasisBits, GB_marks);

    // Masks for extraction of 1 bit per GB code unit sequence.
    StreamSet * GB_mask1 = P->CreateStreamSet(1);
    StreamSet * GB_mask2 = P->CreateStreamSet(1);
    StreamSet * GB_mask3 = P->CreateStreamSet(1);
    StreamSet * GB_mask4 = P->CreateStreamSet(1);
    StreamSet * GB_prefix4 = P->CreateStreamSet(1);
    StreamSet * const error = P->CreateStreamSet();
    P->CreateKernelCall<GB_18030_ExtractionMasks>(GB_marks, BasisBits, GB_mask1, GB_mask2, GB_mask3, GB_mask4, GB_prefix4, error, indexing);

    if (GBerrorOption == GBerrorMode::Abort) {
        StreamSet * BasisBitsOut = P->CreateStreamSet(8);
        StreamSet * GB_mask1Out = P->CreateStreamSet(1);
        StreamSet * GB_mask2Out = P->CreateStreamSet(1);
        StreamSet * GB_mask3Out = P->CreateStreamSet(1);
        StreamSet * GB_mask4Out = P->CreateStreamSet(1);
        StreamSet * GB_prefix4Out = P->CreateStreamSet(1);
        P->CreateKernelCall<ErrorMonitorKernel>(
            // error stream set
            error,
            // monitored stream bindings
            ErrorMonitorKernel::IOStreamBindings{
                {BasisBits, BasisBitsOut},
                {GB_mask1, GB_mask1Out},
                {GB_mask2, GB_mask2Out},
                {GB_mask3, GB_mask3Out},
                {GB_mask4, GB_mask4Out},
                {GB_prefix4, GB_prefix4Out}
            }
        );
        BasisBits = BasisBitsOut;
        GB_mask1 = GB_mask1Out;
        GB_mask2 = GB_mask2Out;
        GB_mask3 = GB_mask3Out;
        GB_mask4 = GB_mask4Out;
        GB_prefix4 = GB_prefix4Out;
    }
    
    StreamSet * GB_4byte = P->CreateStreamSet(1); // markers for 4-byte sequences
    StreamSet * const byte1 = P->CreateStreamSet(8);
    StreamSet * const nybble1 = P->CreateStreamSet(4);
    StreamSet * const byte2 = P->CreateStreamSet(8);
    StreamSet * const nybble2 = P->CreateStreamSet(4);

    FilterByMask(P, GB_mask1, GB_prefix4, GB_4byte);
    FilterByMask(P, GB_mask1, BasisBits, byte1);
    FilterByMask(P, GB_mask2, BasisBits, nybble1);
    FilterByMask(P, GB_mask3, BasisBits, byte2);
    FilterByMask(P, GB_mask4, BasisBits, nybble2);

    StreamSet * const GB_2byte = P->CreateStreamSet(1); // markers for 2-byte sequences
    StreamSet * const gb15index = P->CreateStreamSet(15);
    P->CreateKernelCall<GB_18030_DoubleByteIndex>(GB_4byte, byte1, byte2, GB_2byte, gb15index);

    StreamSet * u16basis = P->CreateStreamSet(16);

    P->CreateKernelCall<GB_18030_InitializeASCII>(byte1, u16basis);

    const unsigned KernelSubrangeBits = 11;
    const unsigned KernelSubrangeSize = 1 << KernelSubrangeBits;
    const unsigned GB2_tblSize = get_GB_DoubleByteTable().size();

    for (unsigned rangeBase = 0; rangeBase < GB2_tblSize; rangeBase += KernelSubrangeSize) {
        //llvm::errs() << "rangeBase = " << rangeBase << "\n";
        StreamSet * u16out = P->CreateStreamSet(16);
        P->CreateKernelCall<GB_18030_DoubleByteRangeKernel>(GB_2byte, gb15index, u16basis, u16out, rangeBase, KernelSubrangeBits);
        u16basis = u16out;
    }

    StreamSet * const utfBasis = P->CreateStreamSet((encodingBits == 16) ? 16 : 21);
    P->CreateKernelCall<GB_18030_FourByteLogic>(GB_4byte, byte1, nybble1, byte2, nybble2, u16basis, utfBasis, indexing);

    if (encodingBits == 32) {
        StreamSet * const u32data = P->CreateStreamSet(1, 32);
        P->CreateKernelCall<P2S21Kernel>(utfBasis, u32data, byteNumbering);
        P->CreateKernelCall<StdOutKernel>(u32data);
    } else if (encodingBits == 16) {
            StreamSet * const u16data = P->CreateStreamSet(1, 16);
            P->CreateKernelCall<P2S16Kernel>(utfBasis, u16data, byteNumbering);
            P->CreateKernelCall<StdOutKernel>(u16data);
    } else if (encodingBits == 8) {
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
        P->CreateKernelCall<UTF8fieldDepositMask>(utfBasis, u8fieldMask, extractionMask);
        P->CreateKernelCall<StreamCompressKernel>(u8fieldMask, extractionMask, u8final);

        P->CreateKernelCall<UTF8_DepositMasks>(u8final, u8initial, u8mask12_17, u8mask6_11);

        SpreadByMask(P, u8initial, utfBasis, deposit18_20, /* inputOffset = */ 18);
        SpreadByMask(P, u8mask12_17, utfBasis, deposit12_17, /* inputOffset = */ 12);
        SpreadByMask(P, u8mask6_11, utfBasis, deposit6_11, /* inputOffset = */ 6);
        SpreadByMask(P, u8final, utfBasis, deposit0_5, /* inputOffset = */ 0);

        P->CreateKernelCall<UTF8assembly>(deposit18_20, deposit12_17, deposit6_11, deposit0_5,
                                        u8initial, u8final, u8mask6_11, u8mask12_17,
                                        u8basis);

        P->CreateKernelCall<P2SKernel>(u8basis, u8bytes);

        P->CreateKernelCall<StdOutKernel>(u8bytes);
    } else {
        llvm::report_fatal_error("Unsupported output encoding: " + OutputEncoding);
    }

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

    // Default output UTF-8
    unsigned encodingBits = 8;
    // Default byte-numbering for UTF-16, UTF-32
    cc::ByteNumbering byteNumbering = cc::ByteNumbering::BigEndian;

    if ((OutputEncoding == "UTF16") || (OutputEncoding == "UTF-16")) {
        encodingBits = 16;
    } else if ((OutputEncoding == "UTF16LE") || (OutputEncoding == "UTF-16LE")) {
        encodingBits = 16;
        byteNumbering = cc::ByteNumbering::LittleEndian;
    } else if ((OutputEncoding == "UTF16BE") || (OutputEncoding == "UTF-16BE")) {
        encodingBits = 16;
    } else if ((OutputEncoding == "UTF32") || (OutputEncoding == "UTF-32")) {
        encodingBits = 32;
    } else if ((OutputEncoding == "UTF32LE") || (OutputEncoding == "UTF-32LE")) {
        encodingBits = 32;
        byteNumbering = cc::ByteNumbering::LittleEndian;
    } else if ((OutputEncoding == "UTF32BE") || (OutputEncoding == "UTF-32BE")) {
        encodingBits = 32;
    } else if ((OutputEncoding != "UTF8") &&  (OutputEncoding != "UTF-8")) {
        llvm::report_fatal_error("Unrecognized encoding.");
    }

    CPUDriver pxDriver("gb18030");
    gb18030FunctionType gb18030Function = generatePipeline(pxDriver, encodingBits, byteNumbering);
    const int fd = open(inputFile.c_str(), O_RDONLY);
    if (LLVM_UNLIKELY(fd == -1)) {
        llvm::errs() << "Error: cannot open " << inputFile << " for processing. Skipped.\n";
    } else {
        gb18030Function(fd, inputFile.c_str());
        close(fd);
    }
    return 0;
}

