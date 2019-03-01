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
#include <cc/cc_compiler_target.h>
#include <pablo/builder.hpp>
#include <llvm/IR/Module.h>
#include <pablo/pablo_toolchain.h> 

using namespace kernel;
using namespace pablo;
using namespace llvm;

using ClassTypeId = pablo::PabloAST::ClassTypeId;
using op3_pair_t = cc::CC_Compiler::op3_pair_t;

void DelMaskKernelBuilder::generatePabloMethod() {
    PabloBuilder main(getEntryScope());
    //  input: 8 basis bit streams
    
    std::vector<PabloAST *> u8_bits = getInputStreamSet("u8bit");
    //  output: delmask stream + error stream
    
    cc::Parabix_CC_Compiler_Builder ccc(getEntryScope(), u8_bits);
    
    Zeroes * zeroes = main.createZeroes();
    const op3_pair_t opOrAnd3 = std::make_pair(ClassTypeId::Or, ClassTypeId::And);
    const op3_pair_t opOr3 = std::make_pair(ClassTypeId::Or, ClassTypeId::Or);

    // Outputs
    Var * delmask = main.createVar("delmask", zeroes);
    Var * neg_delmask = main.createVar("neg_delmask", zeroes);
    Var * error_mask = main.createVar("error_mask", zeroes);
    
    PabloAST * u8pfx = ccc.compileCC(re::makeCC(0xC0, 0xFF));
    PabloAST * nonASCII = ccc.compileCC(re::makeCC(0x80, 0xFF));
    auto it = main.createScope();
    main.createIf(nonASCII, it);
    Var * u8invalid = it.createVar("u8invalid", zeroes);
    PabloAST * u8pfx2 = ccc.compileCC(re::makeCC(0xC2, 0xDF), it);
    PabloAST * u8pfx3 = ccc.compileCC(re::makeCC(0xE0, 0xEF), it);
    PabloAST * u8pfx4 = ccc.compileCC(re::makeCC(0xF0, 0xF4), it);
    PabloAST * u8suffix = ccc.compileCC("u8suffix", re::makeCC(0x80, 0xBF), it);
    //
    // Two-byte sequences
    Var * u8scope22 = it.createVar("u8scope22", zeroes);
    auto it2 = it.createScope();
    it.createIf(u8pfx2, it2);
    it2.createAssign(u8scope22, it2.createAdvance(u8pfx2, 1));
    //
    // Three-byte sequences
    Var * u8scope3X = it.createVar("u8scope3X", zeroes);
    Var * EX_invalid = it.createVar("EX_invalid", zeroes);
    Var * del3 = it.createVar("del3", zeroes);
    auto it3 = it.createScope();
    it.createIf(u8pfx3, it3);
    PabloAST * u8scope32 = it3.createAdvance(u8pfx3, 1, "u8scope32");
    PabloAST * u8scope33 = it3.createAdvance(u8scope32, 1, "u8scope33");
    it3.createAssign(u8scope3X, it3.createOr(u8scope32, u8scope33));
    PabloAST * E0_invalid = it3.createAnd(it3.createAdvance(ccc.compileCC(re::makeCC(0xE0), it3), 1), ccc.compileCC(re::makeCC(0x80, 0x9F), it3));
    PabloAST * ED_invalid1 = it3.createAdvance(ccc.compileCC(re::makeCC(0xED), it3), 1);
    PabloAST * ED_invalid2 = ccc.compileCC(re::makeCC(0xA0, 0xBF), it3);
    it3.createAssign(EX_invalid, ccc.createCCOp3(opOrAnd3, E0_invalid, ED_invalid1, ED_invalid2, it3));
    it3.createAssign(del3, u8scope32);
    //
    // Four-byte sequences
    Var * u8scope4nonfinal = it.createVar("u8scope4nonfinal", zeroes);
    Var * u8scope4X = it.createVar("u8scope4X", zeroes);
    Var * FX_invalid = it.createVar("FX_invalid", zeroes);
    Var * del4 = it.createVar("del4", zeroes);
    auto it4 = it.createScope();
    it.createIf(u8pfx4, it4);
    PabloAST * u8scope42 = it4.createAdvance(u8pfx4, 1, "u8scope42");
    PabloAST * u8scope43 = it4.createAdvance(u8scope42, 1, "u8scope43");
    PabloAST * u8scope44 = it4.createAdvance(u8scope43, 1, "u8scope44");
    it4.createAssign(u8scope4nonfinal, it4.createOr(u8scope42, u8scope43));
    it4.createAssign(u8scope4X, it4.createOr(u8scope4nonfinal, u8scope44));
    PabloAST * F0_invalid = it4.createAnd(it4.createAdvance(ccc.compileCC(re::makeCC(0xF0), it4), 1), ccc.compileCC(re::makeCC(0x80, 0x8F), it4));
    PabloAST * F4_invalid1 = it4.createAdvance(ccc.compileCC(re::makeCC(0xF4), it4), 1);
    PabloAST * F4_invalid2 = ccc.compileCC(re::makeCC(0x90, 0xBF), it4);
    it4.createAssign(FX_invalid, ccc.createCCOp3(opOrAnd3, F0_invalid, F4_invalid1, F4_invalid2, it4));
    it4.createAssign(del4, it4.createOr(u8scope42, u8scope43));
    //
    // Invalid cases
    PabloAST * anyscope = ccc.createCCOp3(opOr3, u8scope22, u8scope3X, u8scope4X, it); // "anyscope"
    PabloAST * legalpfx = ccc.createCCOp3(opOr3, u8pfx2, u8pfx3, u8pfx4, it);
    //  Any scope that does not have a suffix byte, and any suffix byte that is not in
    //  a scope is a mismatch, i.e., invalid UTF-8.
    PabloAST * mismatch = it.createXor(anyscope, u8suffix);
    PabloAST * EF_invalid = it.createOr(EX_invalid, FX_invalid);
    PabloAST * pfx_invalid = it.createXor(u8pfx, legalpfx);
    it.createAssign(u8invalid, ccc.createCCOp3(opOr3, pfx_invalid, mismatch, EF_invalid, it));
    //PabloAST * u8valid = it.createNot(u8invalid, "u8valid");
    it.createAssign(error_mask, u8invalid);
    
    it.createAssign(delmask, it.createInFile(ccc.createCCOp3(opOr3, del3, del4, ccc.compileCC(re::makeCC(0xC0, 0xFF), it), it)));
    it.createAssign(neg_delmask, it.createInFile(it.createNot(delmask)));
    
    Var * delmask_out = getOutputStreamVar("delMask");
    Var * neg_delmask_out = getOutputStreamVar("neg_delMask");
    Var * error_mask_out = getOutputStreamVar("errMask");
    
    main.createAssign(main.createExtract(delmask_out, main.getInteger(0)), delmask);
    main.createAssign(main.createExtract(neg_delmask_out, main.getInteger(0)), neg_delmask);
    main.createAssign(main.createExtract(error_mask_out,  main.getInteger(0)), error_mask);

}
