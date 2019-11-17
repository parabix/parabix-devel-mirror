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

using namespace pablo;
using namespace kernel;
using namespace llvm;

UTF8fieldDepositMask::UTF8fieldDepositMask(BuilderRef b, StreamSet * u32basis, StreamSet * u8fieldMask, StreamSet * u8unitCounts, unsigned depositFieldWidth)
: BlockOrientedKernel(b, "u8depositMask",
{Binding{"basis", u32basis}},
{Binding{"fieldDepositMask", u8fieldMask, FixedRate(4)},
Binding{"extractionMask", u8unitCounts, FixedRate(4)}},
{}, {},
{InternalScalar{ScalarType::NonPersistent, b->getBitBlockType(), "EOFmask"}})
, mDepositFieldWidth(depositFieldWidth) {}

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
    CreateDoBlockMethodCall(b);
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

