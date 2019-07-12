/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include <re/cc/utf16gen.h>

#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Value.h>
#include <llvm/Support/raw_ostream.h>
#include <pablo/builder.hpp>
#include <pablo/bixnum.h>
#include <pablo/pablo_toolchain.h>                 // for pablo_function_passes
#include <pablo/pe_zeroes.h>
#include <re/cc/cc_compiler.h>                     // for CC_Compiler

using namespace pablo;
using namespace kernel;
using namespace llvm;

UTF16_SupplementaryBasis::UTF16_SupplementaryBasis (const std::unique_ptr<KernelBuilder> & b, StreamSet * u32basis, StreamSet * u16_SMP_basis)
: PabloKernel(b, "UTF16_SupplementaryBasis",
{Binding{"basis", u32basis}},
{Binding{"u16_SMP_basis", u16_SMP_basis}}) {}

void UTF16_SupplementaryBasis::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    BixNumCompiler bnc(pb);
    BixNum high5(5);
    for (unsigned i = 0; i < 5; i++) {
        high5[i] = pb.createExtract(getInputStreamVar("basis"), pb.getInteger(i+16));
    }
    PabloAST * aboveBMP = bnc.UGT(high5, 0);
    BixNum SMP_val = bnc.SubModular(high5, 1);
    for (unsigned i = 0; i < 4; i++) {
        pb.createAssign(pb.createExtract(getOutputStreamVar("u16_SMP_basis"), pb.getInteger(i)), pb.createAnd(SMP_val[i], aboveBMP, "u16_SMP_basis"));
    }
}

UTF16fieldDepositMask::UTF16fieldDepositMask(const std::unique_ptr<KernelBuilder> & b, StreamSet * u32basis, StreamSet * u16fieldMask, StreamSet * extractionMask, unsigned depositFieldWidth)
: BlockOrientedKernel(b, "u16depositMask",
{Binding{"basis", u32basis}},
{Binding{"fieldDepositMask", u16fieldMask, FixedRate(2)},
    Binding{"extractionMask", extractionMask, FixedRate(2)}},
{}, {},
{InternalScalar{ScalarType::NonPersistent, b->getBitBlockType(), "EOFmask"}})
, mDepositFieldWidth(depositFieldWidth) {}

void UTF16fieldDepositMask::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & b) {
    Value * fileExtentMask = b->CreateNot(b->getScalarField("EOFmask"));
    // If any of bits 16 through 20 are 1, a UTF-16 surrogate pair sequence is required.
    Value * aboveBMP = b->loadInputStreamBlock("basis", b->getSize(16), b->getSize(0));
    aboveBMP = b->CreateOr(aboveBMP, b->loadInputStreamBlock("basis", b->getSize(17), b->getSize(0)));
    aboveBMP = b->CreateOr(aboveBMP, b->loadInputStreamBlock("basis", b->getSize(18), b->getSize(0)));
    aboveBMP = b->CreateOr(aboveBMP, b->loadInputStreamBlock("basis", b->getSize(19), b->getSize(0)));
    aboveBMP = b->CreateOr(aboveBMP, b->loadInputStreamBlock("basis", b->getSize(20), b->getSize(0)), "aboveBMP");
    aboveBMP = b->CreateAnd(aboveBMP, fileExtentMask);

    //  UTF-16 sequence length:    1     2
    //  extraction mask           10    11

    Value * extraction_mask[2];
    extraction_mask[0] = b->esimd_mergel(1, aboveBMP, fileExtentMask);
    extraction_mask[1] = b->esimd_mergeh(1, aboveBMP, fileExtentMask);
    const unsigned bw = b->getBitBlockWidth();
    Constant * mask10 = Constant::getIntegerValue(b->getIntNTy(bw), APInt::getSplat(bw, APInt::getHighBitsSet(2, 1)));
    for (unsigned j = 0; j < 2; ++j) {
        Value * deposit_mask = b->simd_pext(mDepositFieldWidth, mask10, extraction_mask[j]);
        b->storeOutputStreamBlock("fieldDepositMask", b->getSize(0), b->getSize(j), deposit_mask);
        b->storeOutputStreamBlock("extractionMask", b->getSize(0), b->getSize(j), extraction_mask[j]);
    }
}

void UTF16fieldDepositMask::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * const remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    b->setScalarField("EOFmask", b->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall(b);
}

//
// Given a u16-indexed bit stream marking the final code unit position
// of each UTF-16 sequence, this kernel computes the stream marking initial
// positions of each UTF-16 sequence.
//
UTF16_InitialMask::UTF16_InitialMask (const std::unique_ptr<KernelBuilder> & iBuilder, StreamSet * u16final, StreamSet * u16initial)
: PabloKernel(iBuilder, "UTF16_DepositMasks",
              {Binding{"u16final", u16final}},
              {Binding{"u16initial", u16initial}}) {}

void UTF16_InitialMask::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    PabloAST * u16final = pb.createExtract(getInputStreamVar("u16final"), pb.getInteger(0));
    PabloAST * nonFinal = pb.createNot(u16final, "nonFinal");
    PabloAST * initial = pb.createInFile(pb.createNot(pb.createAdvance(nonFinal, 1)), "u16initial");
    pb.createAssign(pb.createExtract(getOutputStreamVar("u16initial"), pb.getInteger(0)), initial);
}

// This kernel assembles the UTF-16 basis bit data, given three sets of deposited
// bits: SMPbits4_0, u16bits15_10, u16bits9_0, as well as the mask_lo stream
// (having bits set at all but surrogate1 positions).
//
UTF16assembly::UTF16assembly (const std::unique_ptr<KernelBuilder> & b,
                            StreamSet * SMPbits4_0, StreamSet * u16bits15_10, StreamSet * u16bits9_0, StreamSet * u16final,
                            StreamSet * u16basis)
: PabloKernel(b, "UTF16assembly",
{Binding{"u16bits9_0", u16bits9_0, FixedRate(1)},
 Binding{"u16bits15_10", u16bits15_10, FixedRate(1), ZeroExtended()},
 Binding{"SMPbits4_0", SMPbits4_0, FixedRate(1), ZeroExtended()},
 Binding{"u16final", u16final}},
{Binding{"u16basis", u16basis}}) {

}

void UTF16assembly::generatePabloMethod() {
    PabloBuilder pb(getEntryScope());
    std::vector<PabloAST *> SMPbits4_0 = getInputStreamSet("SMPbits4_0");
    std::vector<PabloAST *> u16bits15_10 = getInputStreamSet("u16bits15_10");
    std::vector<PabloAST *> u16bits9_0 = getInputStreamSet("u16bits9_0");
    PabloAST * mask_lo = pb.createExtract(getInputStreamVar("u16final"), pb.getInteger(0));
    PabloAST * surrogate1 = pb.createInFile(pb.createNot(mask_lo, "u16_hi_surrogate"));
    PabloAST * surrogate2 = pb.createAdvance(surrogate1, 1);
    PabloAST * surrogate = pb.createOr(surrogate1, surrogate2, "u16_surrogate");

    PabloAST * u16[16];
    for (unsigned i = 0; i < 6; i++) {
        u16[i] = pb.createSel(mask_lo, u16bits9_0[i], u16bits15_10[i]);
    }
    for (unsigned i = 6; i < 10; i++) {
        u16[i] = pb.createSel(mask_lo, u16bits9_0[i], SMPbits4_0[i-6]);
    }
    //  For surrogates, the high six bits are D8/DC.
    u16[15] = pb.createOr(u16bits15_10[5], surrogate);
    u16[14] = pb.createOr(u16bits15_10[4], surrogate);
    u16[13] = pb.createAnd(u16bits15_10[3], pb.createNot(surrogate));
    u16[12] = pb.createOr(u16bits15_10[2], surrogate);
    u16[11] = pb.createOr(u16bits15_10[1], surrogate);
    u16[10] = pb.createSel(surrogate, surrogate2, u16bits15_10[0]);

    for (unsigned i = 0; i < 16; i++) {
        pb.createAssign(pb.createExtract(getOutputStreamVar("u16basis"), pb.getInteger(i)), u16[i]);
    }
}
