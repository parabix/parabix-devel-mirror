/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <pablo/bixnum/utf8gen.h>

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <pablo/pe_zeroes.h>
#include <re/cc/cc_compiler.h>                     // for CC_Compiler
#include <pablo/pe_debugprint.h>
#include <re/adt/re_cc.h>
#include <pablo/bixnum/bixnum.h>

using namespace pablo;
using namespace kernel;
using namespace llvm;
using namespace re;

UTF8fieldDepositMask::UTF8fieldDepositMask(BuilderRef b, StreamSet * u32basis, StreamSet * u8fieldMask, StreamSet * u8unitCounts, unsigned depositFieldWidth, bool u16u8)
: BlockOrientedKernel(b, "u8depositMask",
{Binding{"basis", u32basis}},
{Binding{"fieldDepositMask", u8fieldMask, FixedRate(4)},
Binding{"extractionMask", u8unitCounts, FixedRate(4)}},
{}, {},
{InternalScalar{ScalarType::NonPersistent, b->getBitBlockType(), "EOFmask"}})
, mDepositFieldWidth(depositFieldWidth)
, mU16U8(u16u8) {}

void UTF8fieldDepositMask::generateDoBlockMethod(BuilderRef b) {
    Value * fileExtentMask = b->CreateNot(b->getScalarField("EOFmask"));
    // If any of bits 16 through 20 are 1, a four-byte UTF-8 sequence is required.
    Value * u8len4 = b->loadInputStreamBlock("basis", b->getSize(16), b->getSize(0));
    u8len4 = b->CreateOr(u8len4, b->loadInputStreamBlock("basis", b->getSize(17), b->getSize(0)));
    u8len4 = b->CreateOr(u8len4, b->loadInputStreamBlock("basis", b->getSize(18), b->getSize(0)));
    u8len4 = b->CreateOr(u8len4, b->loadInputStreamBlock("basis", b->getSize(19), b->getSize(0)));
    u8len4 = b->CreateOr(u8len4, b->loadInputStreamBlock("basis", b->getSize(20), b->getSize(0)), "u8len4");
    u8len4 = b->CreateAnd(u8len4, fileExtentMask);
    Value * u8len34 = u8len4;
    // Otherwise, if any of bits 11 through 15 are 1, a three-byte UTF-8 sequence is required.
    u8len34 = b->CreateOr(u8len34, b->loadInputStreamBlock("basis", b->getSize(11), b->getSize(0)));
    u8len34 = b->CreateOr(u8len34, b->loadInputStreamBlock("basis", b->getSize(12), b->getSize(0)));
    u8len34 = b->CreateOr(u8len34, b->loadInputStreamBlock("basis", b->getSize(13), b->getSize(0)));
    u8len34 = b->CreateOr(u8len34, b->loadInputStreamBlock("basis", b->getSize(14), b->getSize(0)));
    u8len34 = b->CreateOr(u8len34, b->loadInputStreamBlock("basis", b->getSize(15), b->getSize(0)));
    u8len34 = b->CreateAnd(u8len34, fileExtentMask);
    Value * nonASCII = u8len34;
    // Otherwise, if any of bits 7 through 10 are 1, a two-byte UTF-8 sequence is required.
    nonASCII = b->CreateOr(nonASCII, b->loadInputStreamBlock("basis", b->getSize(7), b->getSize(0)));
    nonASCII = b->CreateOr(nonASCII, b->loadInputStreamBlock("basis", b->getSize(8), b->getSize(0)));
    nonASCII = b->CreateOr(nonASCII, b->loadInputStreamBlock("basis", b->getSize(9), b->getSize(0)));
    nonASCII = b->CreateOr(nonASCII, b->loadInputStreamBlock("basis", b->getSize(10), b->getSize(0)), "nonASCII");
    nonASCII = b->CreateAnd(nonASCII, fileExtentMask);

    //TOOD: Receive basisExtent as an input parameter for the kernel
    //as it is needed only during transcoding u16 to u8
    if (mU16U8) {
        Value * basisExtent = nonASCII;
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(6), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(5), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(4), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(3), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(2), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(1), b->getSize(0)));
        basisExtent = b->CreateOr(basisExtent, b->loadInputStreamBlock("basis", b->getSize(0), b->getSize(0)));
        fileExtentMask = b->CreateAnd(fileExtentMask, basisExtent);
    }
    //
    //  UTF-8 sequence length:    1     2     3       4
    //  extraction mask        1000  1100  1110    1111
    //  interleave u8len3|u8len4, allOnes() for bits 1, 3:  x..., ..x.
    //  interleave prefix4, u8len2|u8len3|u8len4 for bits 0, 2:  .x.., ...x

    Value * maskA_lo = b->esimd_mergel(1, u8len34, fileExtentMask);
    Value * maskA_hi = b->esimd_mergeh(1, u8len34, fileExtentMask);
    Value * maskB_lo = b->esimd_mergel(1, u8len4, nonASCII);
    Value * maskB_hi = b->esimd_mergeh(1, u8len4, nonASCII);
    Value * extraction_mask[4];
    extraction_mask[0] = b->esimd_mergel(1, maskB_lo, maskA_lo);
    extraction_mask[1] = b->esimd_mergeh(1, maskB_lo, maskA_lo);
    extraction_mask[2] = b->esimd_mergel(1, maskB_hi, maskA_hi);
    extraction_mask[3] = b->esimd_mergeh(1, maskB_hi, maskA_hi);
    const unsigned bw = b->getBitBlockWidth();
    Constant * mask1000 = Constant::getIntegerValue(b->getIntNTy(bw), APInt::getSplat(bw, APInt::getHighBitsSet(4, 1)));
    for (unsigned j = 0; j < 4; ++j) {
        Value * deposit_mask = b->simd_pext(mDepositFieldWidth, mask1000, extraction_mask[j]);
        b->storeOutputStreamBlock("fieldDepositMask", b->getSize(0), b->getSize(j), deposit_mask);
        b->storeOutputStreamBlock("extractionMask", b->getSize(0), b->getSize(j), extraction_mask[j]);
    }
}
void UTF8fieldDepositMask::generateFinalBlockMethod(BuilderRef b, Value * const remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    b->setScalarField("EOFmask", b->bitblock_mask_from(remainingBytes));
    RepeatDoBlockLogic(b);
}


//
// Given a u8-indexed bit stream marking the final code unit position
// of each UTF-8 sequence, this kernel computes the deposit masks
// u8initial, u8mask12_17, and u8mask6_11.
//
UTF8_DepositMasks::UTF8_DepositMasks (BuilderRef iBuilder, StreamSet * u8final, StreamSet * u8initial, StreamSet * u8mask12_17, StreamSet * u8mask6_11)
: PabloKernel(iBuilder, "UTF8_DepositMasks",
              {Binding{"u8final", u8final, FixedRate(1), LookAhead(2)}},
              {Binding{"u8initial", u8initial},
               Binding{"u8mask12_17", u8mask12_17},
               Binding{"u8mask6_11", u8mask6_11}}) {}

void UTF8_DepositMasks::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * u8final = pb.createExtract(getInputStreamVar("u8final"), pb.getInteger(0));
    PabloAST * nonFinal = pb.createNot(u8final, "nonFinal");
    PabloAST * initial = pb.createInFile(pb.createNot(pb.createAdvance(nonFinal, 1)), "u8initial");
    PabloAST * ASCII = pb.createAnd(u8final, initial);
    PabloAST * lookAheadFinal = pb.createLookahead(u8final, 1, "lookaheadFinal");
    // Eliminate lookahead positions that are the final position of the prior unit.
    PabloAST * secondLast = pb.createAnd(lookAheadFinal, nonFinal);
    PabloAST * u8mask6_11 = pb.createInFile(pb.createOr(secondLast, ASCII, "u8mask6_11"));
    PabloAST * prefix2 = pb.createAnd(secondLast, initial);
    PabloAST * lookAhead2 = pb.createLookahead(u8final, 2, "lookahead2");
    PabloAST * thirdLast = pb.createAnd(pb.createAnd(lookAhead2, nonFinal), pb.createNot(secondLast));
    PabloAST * u8mask12_17 = pb.createInFile(pb.createOr(thirdLast, pb.createOr(prefix2, ASCII), "u8mask12_17"));
    pb.createAssign(pb.createExtract(getOutputStreamVar("u8initial"), pb.getInteger(0)), initial);
    pb.createAssign(pb.createExtract(getOutputStreamVar("u8mask6_11"), pb.getInteger(0)), u8mask6_11);
    pb.createAssign(pb.createExtract(getOutputStreamVar("u8mask12_17"), pb.getInteger(0)), u8mask12_17);
}

// This kernel assembles the UTF-8 basis bit data, given four sets of deposited
// bits bits 18-20, 11-17, 6-11 and 0-5, as weil as the marker streams u8initial,
// u8final, u8prefix3 and u8prefix4.
//
UTF8assembly::UTF8assembly (BuilderRef b,
                            StreamSet * deposit18_20, StreamSet * deposit12_17, StreamSet * deposit6_11, StreamSet * deposit0_5,
                            StreamSet * u8initial, StreamSet * u8final, StreamSet * u8mask6_11, StreamSet * u8mask12_17,
                            StreamSet * u8basis)
: PabloKernel(b, "UTF8assembly",
{Binding{"dep0_5", deposit0_5, FixedRate(1)},
 Binding{"dep6_11", deposit6_11, FixedRate(1), ZeroExtended()},
 Binding{"dep12_17", deposit12_17, FixedRate(1), ZeroExtended()},
 Binding{"dep18_20", deposit18_20, FixedRate(1), ZeroExtended()},
 Binding{"u8initial", u8initial, FixedRate(), ZeroExtended()},
 Binding{"u8final", u8final},
 Binding{"u8mask6_11", u8mask6_11, FixedRate(), ZeroExtended()},
 Binding{"u8mask12_17", u8mask12_17, FixedRate(), ZeroExtended()}},
{Binding{"u8basis", u8basis}}) {

}

void UTF8assembly::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> dep18_20 = getInputStreamSet("dep18_20");
    std::vector<PabloAST *> dep12_17 = getInputStreamSet("dep12_17");
    std::vector<PabloAST *> dep6_11 = getInputStreamSet("dep6_11");
    std::vector<PabloAST *> dep0_5 = getInputStreamSet("dep0_5");
    PabloAST * u8initial = pb.createExtract(getInputStreamVar("u8initial"), pb.getInteger(0));
    PabloAST * u8final = pb.createExtract(getInputStreamVar("u8final"), pb.getInteger(0));
    PabloAST * u8mask6_11 = pb.createExtract(getInputStreamVar("u8mask6_11"), pb.getInteger(0));
    PabloAST * u8mask12_17 = pb.createExtract(getInputStreamVar("u8mask12_17"), pb.getInteger(0));
    PabloAST * ASCII = pb.createAnd(u8initial, u8final);
    PabloAST * nonASCII = pb.createNot(ASCII, "nonASCII");
    PabloAST * u8basis[8];
    //
    // Deposit bit 6 is either used for bit 6 of an ASCII code unit, or
    // bit 0 for nonASCII units.   Extract the ASCII case separately.
    PabloAST * ASCIIbit6 = pb.createAnd(dep6_11[0], ASCII);
    dep6_11[0] = pb.createAnd(dep6_11[0], nonASCII);
    for (unsigned i = 0; i < 6; i++) {
        u8basis[i] = pb.createOr(dep0_5[i], dep6_11[i]);
        u8basis[i] = pb.createOr(u8basis[i], dep12_17[i], "basis" + std::to_string(i));
        if (i < 3) u8basis[i] = pb.createOr(u8basis[i], dep18_20[i]);
    }
    // The high bit of UTF-8 prefix and suffix bytes (any nonASCII byte) is always 1.
    u8basis[7] = nonASCII;
    // The second highest bit of UTF-8 units is 1 for any prefix, or ASCII byte with
    // a 1 in bit 6 of the Unicode representation.
    u8basis[6] = pb.createOr(pb.createAnd(u8initial, nonASCII), ASCIIbit6, "basis6");
    //
    // For any prefix of a 3-byte or 4-byte sequence the third highest bit is set to 1.
    u8basis[5] = pb.createOr(u8basis[5], pb.createAnd(u8initial, pb.createNot(u8mask6_11)), "basis5");
    // For any prefix of a 4-byte sequence the fourth highest bit is set to 1.
    u8basis[4] = pb.createOr(u8basis[4], pb.createAnd(u8initial, pb.createNot(u8mask12_17)), "basis4");
    for (unsigned i = 0; i < 8; i++) {
        pb.createAssign(pb.createExtract(getOutputStreamVar("u8basis"), pb.getInteger(i)), u8basis[i]);
    }
}

U8U16Kernel::U8U16Kernel(BuilderRef b, StreamSet *BasisBits, StreamSet *u8bits, StreamSet *selectors)
: PabloKernel(b, "u8u16",
// input
{Binding{"u8bit", BasisBits}},
// outputs
{Binding{"u16bit", u8bits},
 Binding{"selectors", selectors}}) {

}

void U8U16Kernel::generatePabloMethod() {
    PabloBuilder main(getEntryScope());
    Zeroes * zeroes = main.createZeroes();

    //  input: 8 basis bit streams
    std::vector<PabloAST *> u8_bits = getInputStreamSet("u8bit");

    //  output: 16 u8-indexed streams, + delmask stream + error stream
    Var * u16_hi[8];
    for (int i = 0; i < 8; ++i) {
        u16_hi[i] = main.createVar("u16_hi" + std::to_string(i), zeroes);
    }
    Var * u16_lo[8];
    for (int i = 0; i < 8; ++i) {
        u16_lo[i] = main.createVar("u16_lo" + std::to_string(i), zeroes);
    }

    Var * delmask = main.createVar("delmask", zeroes);
    Var * error_mask = main.createVar("error_mask", zeroes);

    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), u8_bits);

    // The logic for processing non-ASCII bytes will be embedded within an if-hierarchy.
    PabloAST * nonASCII = ccc.compileCC(makeByte(0x80, 0xFF));

    // Builder for the if statement handling all non-ASCII logic
    auto nAb = main.createScope();
    // Bits 3 through LE0 of a 2-byte prefix are data bits, needed to
    // produce the UTF-16 code unit data ...,
    PabloAST * bit4a1 = nAb.createAdvance(u8_bits[4], 1);
    PabloAST * bit3a1 = nAb.createAdvance(u8_bits[3], 1);
    PabloAST * bit2a1 = nAb.createAdvance(u8_bits[2], 1);
    PabloAST * bit1a1 = nAb.createAdvance(u8_bits[1], 1);
    PabloAST * bit0a1 = nAb.createAdvance(u8_bits[0], 1);

    // Entry condition for 3 or 4 byte sequences: we have a prefix byte in the range 0xE0-0xFF.
    PabloAST * pfx34 = ccc.compileCC(makeByte(0xE0, 0xFF), nAb);
    // Builder for the if statement handling all logic for 3- and 4-byte sequences.
    auto p34b = nAb.createScope();
    // Bits LE3 through LE0 of a 3-byte prefix are data bits.  They must be moved
    // to the final position of the 3-byte sequence.
    PabloAST * bit5a1 = p34b.createAdvance(u8_bits[5], 1);
    PabloAST * bit3a2 = p34b.createAdvance(bit3a1, 1);
    PabloAST * bit2a2 = p34b.createAdvance(bit2a1, 1);
    PabloAST * bit1a2 = p34b.createAdvance(bit1a1, 1);
    PabloAST * bit0a2 = p34b.createAdvance(bit0a1, 1);

    Var * const u8scope32 = nAb.createVar("u8scope32", zeroes);
    Var * const u8scope33 = nAb.createVar("u8scope33", zeroes);
    Var * const u8scope44 = nAb.createVar("u8scope44", zeroes);

    //
    // Logic for 4-byte UTF-8 sequences
    //
    // Entry condition  or 4 byte sequences: we have a prefix byte in the range 0xF0-0xFF.
    PabloAST * pfx4 = ccc.compileCC(makeByte(0xF0, 0xFF), p34b);
    // Builder for the if statement handling all logic for 4-byte sequences only.
    auto p4b = p34b.createScope();
    // Illegal 4-byte sequences
    PabloAST * F0 = ccc.compileCC(makeByte(0xF0), p4b);
    PabloAST * F4 = ccc.compileCC(makeByte(0xF4), p4b);
    PabloAST * F0_err = p4b.createAnd(p4b.createAdvance(F0, 1), ccc.compileCC(makeByte(0x80, 0x8F), p4b));
    PabloAST * F4_err = p4b.createAnd(p4b.createAdvance(F4, 1), ccc.compileCC(makeByte(0x90, 0xBF), p4b));
    PabloAST * F5_FF = ccc.compileCC(makeByte(0xF5, 0xFF), p4b);

    Var * FX_err = p34b.createVar("FX_err", zeroes);
    p4b.createAssign(FX_err, p4b.createOr(F5_FF, p4b.createOr(F0_err, F4_err)));
    //
    // 4-byte prefixes have a scope that extends over the next 3 bytes.

    Var * u8scope42 = p34b.createVar("u8scope42", zeroes);
    Var * u8scope43 = p34b.createVar("u8scope43", zeroes);

    p4b.createAssign(u8scope42, p4b.createAdvance(pfx4, 1));
    p4b.createAssign(u8scope43, p4b.createAdvance(u8scope42, 1));
    p4b.createAssign(u8scope44, p4b.createAdvance(u8scope43, 1));
    //

    //  From the 4-byte sequence 11110abc 10defghi 10jklmno 10pqrstu,
    //  we must calculate the value abcde - 1 to produce the bit values
    //  for u16_hi1, hi0, lo7, lo6 at the scope43 position.
    Var * s43_lo7 = nAb.createVar("scope43_lo7", zeroes);
    Var * s43_lo6 = nAb.createVar("scope43_lo6", zeroes);
    Var * s43_hi1 = nAb.createVar("scope43_hi1", zeroes);
    Var * s43_hi0 = nAb.createVar("scope43_hi0", zeroes);

    Var * s43_lo5 = main.createVar("scope43_lo5", zeroes);
    Var * s43_lo4 = main.createVar("scope43_lo4", zeroes);
    Var * s43_lo3 = main.createVar("scope43_lo3", zeroes);
    Var * s43_lo2 = main.createVar("scope43_lo2", zeroes);
    Var * s43_lo1 = main.createVar("scope43_lo1", zeroes);
    Var * s43_lo0 = main.createVar("scope43_lo0", zeroes);

    BixNum plane = BixNumCompiler(p4b).SubModular({bit4a1, bit5a1, bit0a2, bit1a2}, 1);
    p4b.createAssign(s43_lo6, p4b.createAnd(u8scope43, plane[0]));
    p4b.createAssign(s43_lo7, p4b.createAnd(u8scope43, plane[1]));
    p4b.createAssign(s43_hi0, p4b.createAnd(u8scope43, plane[2]));
    p4b.createAssign(s43_hi1, p4b.createAnd(u8scope43, plane[3]));
    //
    p4b.createAssign(s43_lo5, p4b.createAnd(u8scope43, bit3a1));
    p4b.createAssign(s43_lo4, p4b.createAnd(u8scope43, bit2a1));
    p4b.createAssign(s43_lo3, p4b.createAnd(u8scope43, bit1a1));
    p4b.createAssign(s43_lo2, p4b.createAnd(u8scope43, bit0a1));
    p4b.createAssign(s43_lo1, p4b.createAnd(u8scope43, u8_bits[5]));
    p4b.createAssign(s43_lo0, p4b.createAnd(u8scope43, u8_bits[4]));
    //
    //
    p34b.createIf(pfx4, p4b);
    //
    // Combined logic for 3 and 4 byte sequences
    //
    PabloAST * pfx3 = ccc.compileCC(makeByte(0xE0, 0xEF), p34b);

    p34b.createAssign(u8scope32, p34b.createAdvance(pfx3, 1));
    p34b.createAssign(u8scope33, p34b.createAdvance(u8scope32, 1));

    // Illegal 3-byte sequences
    PabloAST * E0 = ccc.compileCC(makeByte(0xE0), p34b);
    PabloAST * ED = ccc.compileCC(makeByte(0xED), p34b);
    PabloAST * E0_err = p34b.createAnd(p34b.createAdvance(E0, 1), ccc.compileCC(makeByte(0x80, 0x9F), p34b));
    PabloAST * ED_err = p34b.createAnd(p34b.createAdvance(ED, 1), ccc.compileCC(makeByte(0xA0, 0xBF), p34b));
    Var * EX_FX_err = nAb.createVar("EX_FX_err", zeroes);

    p34b.createAssign(EX_FX_err, p34b.createOr(p34b.createOr(E0_err, ED_err), FX_err));
    // Two surrogate UTF-16 units are computed at the 3rd and 4th positions of 4-byte sequences.
    PabloAST * surrogate = p34b.createOr(u8scope43, u8scope44);

    Var * p34del = nAb.createVar("p34del", zeroes);
    p34b.createAssign(p34del, p34b.createOr(u8scope32, u8scope42));


    // The high 5 bits of the UTF-16 code unit are only nonzero for 3 and 4-byte
    // UTF-8 sequences.
    p34b.createAssign(u16_hi[7], p34b.createOr(p34b.createAnd(u8scope33, bit3a2), surrogate));
    p34b.createAssign(u16_hi[6], p34b.createOr(p34b.createAnd(u8scope33, bit2a2), surrogate));
    p34b.createAssign(u16_hi[5], p34b.createAnd(u8scope33, bit1a2));
    p34b.createAssign(u16_hi[4], p34b.createOr(p34b.createAnd(u8scope33, bit0a2), surrogate));
    p34b.createAssign(u16_hi[3], p34b.createOr(p34b.createAnd(u8scope33, bit5a1), surrogate));

    //
    nAb.createIf(pfx34, p34b);
    //
    // Combined logic for 2, 3 and 4 byte sequences
    //

    Var * u8lastscope = main.createVar("u8lastscope", zeroes);

    PabloAST * pfx2 = ccc.compileCC(makeByte(0xC0, 0xDF), nAb);
    PabloAST * u8scope22 = nAb.createAdvance(pfx2, 1);
    nAb.createAssign(u8lastscope, nAb.createOr(u8scope22, nAb.createOr(u8scope33, u8scope44)));
    PabloAST * u8anyscope = nAb.createOr(u8lastscope, p34del);

    PabloAST * C0_C1_err = ccc.compileCC(makeByte(0xC0, 0xC1), nAb);
    PabloAST * scope_suffix_mismatch = nAb.createXor(u8anyscope, ccc.compileCC(makeByte(0x80, 0xBF), nAb));
    nAb.createAssign(error_mask, nAb.createOr(scope_suffix_mismatch, nAb.createOr(C0_C1_err, EX_FX_err)));
    nAb.createAssign(delmask, nAb.createOr(p34del, ccc.compileCC(makeByte(0xC0, 0xFF), nAb)));

    // The low 3 bits of the high byte of the UTF-16 code unit as well as the high bit of the
    // low byte are only nonzero for 2, 3 and 4 byte sequences.
    nAb.createAssign(u16_hi[2], nAb.createOr(nAb.createAnd(u8lastscope, bit4a1), u8scope44));
    nAb.createAssign(u16_hi[1], nAb.createOr(nAb.createAnd(u8lastscope, bit3a1), s43_hi1));
    nAb.createAssign(u16_hi[0], nAb.createOr(nAb.createAnd(u8lastscope, bit2a1), s43_hi0));
    nAb.createAssign(u16_lo[7], nAb.createOr(nAb.createAnd(u8lastscope, bit1a1), s43_lo7));

    Var * p234_lo6 = main.createVar("p234_lo6", zeroes);

    nAb.createAssign(p234_lo6, nAb.createOr(nAb.createAnd(u8lastscope, bit0a1), s43_lo6));

    main.createIf(nonASCII, nAb);
    //
    //
    PabloAST * ASCII = ccc.compileCC(makeByte(0x0, 0x7F));
    PabloAST * last_byte = main.createOr(ASCII, u8lastscope);
    main.createAssign(u16_lo[6], main.createOr(main.createAnd(ASCII, u8_bits[6]), p234_lo6));
    main.createAssign(u16_lo[5], main.createOr(main.createAnd(last_byte, u8_bits[5]), s43_lo5));
    main.createAssign(u16_lo[4], main.createOr(main.createAnd(last_byte, u8_bits[4]), s43_lo4));
    main.createAssign(u16_lo[3], main.createOr(main.createAnd(last_byte, u8_bits[3]), s43_lo3));
    main.createAssign(u16_lo[2], main.createOr(main.createAnd(last_byte, u8_bits[2]), s43_lo2));
    main.createAssign(u16_lo[1], main.createOr(main.createAnd(last_byte, u8_bits[1]), s43_lo1));
    main.createAssign(u16_lo[0], main.createOr(main.createAnd(last_byte, u8_bits[0]), s43_lo0));

    Var * output = getOutputStreamVar("u16bit");
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i + 8), u16_hi[i]);
    }
    for (unsigned i = 0; i < 8; i++) {
        main.createAssign(main.createExtract(output, i), u16_lo[i]);
    }
    PabloAST * selectors = main.createInFile(main.createNot(delmask));
    main.createAssign(main.createExtract(getOutputStreamVar("selectors"), main.getInteger(0)), selectors);
}

U16U8index::U16U8index(BuilderRef b, StreamSet * u16basis, StreamSet * u8len4, StreamSet * u8len3, StreamSet * u8len2, StreamSet * selectors)
: PabloKernel(b, "u8indexMask",
{Binding{"basis", u16basis}},
{Binding{"len4", u8len4},
Binding{"len3", u8len3},
Binding{"len2", u8len2},
Binding{"selectors", selectors}}) {}

void U16U8index::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::unique_ptr<cc::CC_Compiler> ccc;

    std::vector<PabloAST *> u16bytes = getInputStreamSet("basis");
    ccc = make_unique<cc::Parabix_CC_Compiler_Builder>(getEntryScope(), u16bytes);

    PabloAST * prefix = ccc->compileCC(makeByte(0xD800, 0xDBFF));
    PabloAST * suffix = ccc->compileCC(makeByte(0xDC00, 0xDFFF));
    PabloAST * len4 = pb.createOr(prefix, suffix);
    PabloAST * BOM = ccc->compileCC(makeByte(0xFEFF));
    PabloAST * fileExtent = ccc->compileCC(makeByte(0, 0xFFFF));
    //Remove BOM from u32basis, if exists
    PabloAST * toSel = pb.createAnd(fileExtent, pb.createAnd(pb.createNot(prefix), pb.createNot(BOM)));
    pb.createAssign(pb.createExtract(getOutputStreamVar("len4"), pb.getInteger(0)), prefix);
    pb.createAssign(pb.createExtract(getOutputStreamVar("len3"), pb.getInteger(0)), suffix);
    pb.createAssign(pb.createExtract(getOutputStreamVar("len2"), pb.getInteger(0)), len4);
    pb.createAssign(pb.createExtract(getOutputStreamVar("selectors"), pb.getInteger(0)), toSel);
}

shuffle::shuffle(BuilderRef b, StreamSet * const codeUnitStream, StreamSet * const BasisBits, StreamSet * const prefix, StreamSet * const suffix, StreamSet * const len4)
: BlockOrientedKernel(b, "shuffle",
{Binding{"basisBits", codeUnitStream, FixedRate(), Principal()},
Binding{"prefix", prefix},
Binding{"suffix", suffix},
Binding{"len4", len4}},
    {Binding{"filteredBits", BasisBits}}, {}, {}, {})  {
    }

void shuffle::generateDoBlockMethod(BuilderRef kb) {

    Value * prefix = kb->loadInputStreamBlock("prefix", kb->getSize(0));
    Value * suffix = kb->loadInputStreamBlock("suffix", kb->getSize(0));

    Value * basisbits0[10];
    Value * basisbits1[13];

    for (unsigned j = 0; j < 16; j++) {
        Value * bitBlock = kb->loadInputStreamBlock("basisBits", kb->getSize(j));
        if (j < 8) {
            basisbits0[j] = bitBlock;
        } else {
            basisbits1[j-8] = bitBlock;
        }
    }

    //extend the bitstream to 21 bits
    for (unsigned i = 8; i < 13; ++i) {
        basisbits1[i] = kb->allZeroes();
    }
    for (unsigned i = 0; i < 8; ++i) {
        Value * bit = kb->simd_and(prefix, basisbits0[i]);
        bit = kb->simd_slli(16, bit, 1);
        Value * not_suffix = kb->simd_not(suffix);
        bit = kb->simd_or(not_suffix, bit);
        basisbits1[i + 2] = kb->simd_and(bit, basisbits1[i + 2]);
    }
    basisbits1[8] = kb->simd_or(basisbits1[8], suffix);

    for (unsigned i = 0; i < 8; ++i) {
            kb->storeOutputStreamBlock("filteredBits", kb->getInt32(i), basisbits0[i]);
            kb->storeOutputStreamBlock("filteredBits", kb->getInt32(i + 8), basisbits1[i]);
    }
    for (unsigned i = 8; i < 13; ++i) {
            kb->storeOutputStreamBlock("filteredBits", kb->getInt32(i + 8), basisbits1[i]);
    }
}
