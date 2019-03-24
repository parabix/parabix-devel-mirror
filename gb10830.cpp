

//
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

