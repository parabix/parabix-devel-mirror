/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "delmask_kernel.h"
#include <re/re_cc.h>
#include <pablo/pe_ones.h>          // for Ones
#include <pablo/pe_var.h>           // for Var
#include <pablo/pe_zeroes.h>        // for Zeroes
#include <cc/cc_compiler.h>
#include <pablo/builder.hpp>
#include <llvm/IR/Module.h>
#include <pablo/pablo_toolchain.h> 

using namespace kernel;
using namespace pablo;
using namespace llvm;

void DelMaskKernelBuilder::generatePabloMethod() {
    //  input: 8 basis bit streams
    
    const auto u8bitSet = this->getInputStreamVar("u8bit");
    
    //  output: delmask stream + error stream
    
    cc::CC_Compiler ccc(this, u8bitSet);
    
    PabloBuilder & main = ccc.getBuilder();
    const auto u8_bits = ccc.getBasisBits();
    Zeroes * zeroes = main.createZeroes();

    // Outputs
    Var * delmask = main.createVar("delmask", zeroes);
    Var * neg_delmask = main.createVar("neg_delmask", zeroes);
    Var * error_mask = main.createVar("error_mask", zeroes);
    
    PabloAST * ASCII = ccc.compileCC("ASCII", re::makeCC(0x0, 0x7F), main);
    PabloBuilder ascii = PabloBuilder::Create(main);
    main.createIf(ASCII, ascii);
    PabloAST * u8pfx = ccc.compileCC("u8pfx", re::makeCC(0xC0, 0xFF), main);
    PabloAST * nonASCII = ccc.compileCC("u8pfx", re::makeCC(0x80, 0xFF), main);
    PabloBuilder it = PabloBuilder::Create(main);
    main.createIf(nonASCII, it);
    Var * u8invalid = it.createVar("u8invalid", zeroes);
    PabloAST * u8pfx2 = ccc.compileCC(re::makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(re::makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = ccc.compileCC(re::makeCC(0xF0, 0xF4), it);
    PabloAST * u8suffix = ccc.compileCC("u8suffix", re::makeCC(0x80, 0xBF), it);
    //
    // Two-byte sequences
    Var * u8scope22 = it.createVar("u8scope22", zeroes);
    PabloBuilder it2 = PabloBuilder::Create(it);
    it.createIf(u8pfx2, it2);
    it2.createAssign(u8scope22, it2.createAdvance(u8pfx2, 1));
    //
    // Three-byte sequences
    Var * u8scope3X = it.createVar("u8scope3X", zeroes);
    Var * EX_invalid = it.createVar("EX_invalid", zeroes);
    Var * del3 = it.createVar("del3", zeroes);
    PabloBuilder it3 = PabloBuilder::Create(it);
    it.createIf(u8pfx3, it3);
    PabloAST * u8scope32 = it3.createAdvance(u8pfx3, 1, "u8scope32");
    PabloAST * u8scope33 = it3.createAdvance(u8scope32, 1, "u8scope33");
    it3.createAssign(u8scope3X, it3.createOr(u8scope32, u8scope33));
    PabloAST * E0_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(re::makeCC(0xE0), it3), 1), ccc.compileCC(re::makeCC(0x80, 0x9F), it3));
    PabloAST * ED_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(re::makeCC(0xED), it3), 1), ccc.compileCC(re::makeCC(0xA0, 0xBF), it3));
    it3.createAssign(EX_invalid, it3.createOr(E0_invalid, ED_invalid));
    it3.createAssign(del3, u8scope32);
    //
    // Four-byte sequences
    Var * u8scope4nonfinal = it.createVar("u8scope4nonfinal", zeroes);
    Var * u8scope4X = it.createVar("u8scope4X", zeroes);
    Var * FX_invalid = it.createVar("FX_invalid", zeroes);
    Var * del4 = it.createVar("del4", zeroes);
    PabloBuilder it4 = PabloBuilder::Create(it);
    it.createIf(u8pfx4, it4);
    PabloAST * u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(u8scope4nonfinal, it4.createOr(u8scope42, u8scope43));
    it4.createAssign(u8scope4X, it4.createOr(u8scope4nonfinal, u8scope44));
    PabloAST * F0_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(re::makeCC(0xF0), it4), 1), ccc.compileCC(re::makeCC(0x80, 0x8F), it4));
    PabloAST * F4_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(re::makeCC(0xF4), it4), 1), ccc.compileCC(re::makeCC(0x90, 0xBF), it4));
    it4.createAssign(FX_invalid, it4.createOr(F0_invalid, F4_invalid));
    it4.createAssign(del4, it4.createOr(u8scope42, u8scope43));
    //
    // Invalid cases
    PabloAST * anyscope = it.createOr(u8scope22, it.createOr(u8scope3X, u8scope4X), "anyscope");
    PabloAST * legalpfx = it.createOr(it.createOr(u8pfx2, u8pfx3), u8pfx4);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * mismatch = it.createXor(anyscope, u8suffix);
    PabloAST * EF_invalid = it.createOr(EX_invalid, FX_invalid);
    PabloAST * pfx_invalid = it.createXor(u8pfx, legalpfx);
    it.createAssign(u8invalid, it.createOr(pfx_invalid, it.createOr(mismatch, EF_invalid)));
    //PabloAST * u8valid = it.createNot(u8invalid, "u8valid");
    it.createAssign(error_mask, u8invalid);
    
    it.createAssign(delmask, it.createInFile(it.createOr(it.createOr(del3, del4), ccc.compileCC(re::makeCC(0xC0, 0xFF), it))));
    it.createAssign(neg_delmask, it.createInFile(it.createNot(delmask)));
    
    Var * delmask_out = this->getOutputStreamVar("delMask");
    Var * neg_delmask_out = this->getOutputStreamVar("neg_delMask");
    Var * error_mask_out = this->getOutputStreamVar("errMask");
    
    main.createAssign(main.createExtract(delmask_out, main.getInteger(0)), delmask);
    main.createAssign(main.createExtract(neg_delmask_out, main.getInteger(0)), neg_delmask);
    main.createAssign(main.createExtract(error_mask_out,  main.getInteger(0)), error_mask);

}
