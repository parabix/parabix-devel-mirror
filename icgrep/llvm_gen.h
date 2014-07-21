/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef LLVM_GENERATOR_H
#define LLVM_GENERATOR_H

//Regular Expressions
#include "re_re.h"
#include "re_cc.h"
#include "re_name.h"
#include "re_start.h"
#include "re_end.h"
#include "re_seq.h"
#include "re_alt.h"
#include "re_rep.h"

//Pablo Expressions
#include "pe_pabloe.h"
#include "pe_sel.h"
#include "pe_advance.h"
#include "pe_all.h"
#include "pe_and.h"
#include "pe_charclass.h"
#include "pe_matchstar.h"
#include "pe_not.h"
#include "pe_or.h"
#include "pe_var.h"
#include "pe_xor.h"

//Pablo Statements
#include "ps_pablos.h"
#include "ps_assign.h"
#include "ps_if.h"
#include "ps_while.h"

#include "cc_codegenobject.h"
#include "pbix_compiler.h"

#include "llvm_gen_helper.h"

#include <iostream>
#include <string>
#include <sstream>
#include <list>
#include <map>
#include <algorithm>

#include <llvm/Support/raw_ostream.h>

#include <llvm/Pass.h>
#include <llvm/PassManager.h>
#include <llvm/ADT/SmallVector.h>
#include <llvm/Analysis/Verifier.h>
#include "llvm/Analysis/Passes.h"
#include <llvm/Assembly/PrintModulePass.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/IR/CallingConv.h>
#include <llvm/IR/Constants.h>
#include "llvm/IR/DataLayout.h"
#include "llvm/IR/DerivedTypes.h"
#include <llvm/IR/Function.h>
#include <llvm/IR/GlobalVariable.h>
#include <llvm/IR/InlineAsm.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/FormattedStream.h>
#include <llvm/Support/MathExtras.h>
#include <llvm/Support/Casting.h>

#include "llvm/Support/TargetSelect.h"
#include "llvm/Transforms/Scalar.h"

#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/ExecutionEngine/MCJIT.h>

#include <llvm/Linker.h>
#include <llvm/IRReader/IRReader.h>
#include <llvm/Bitcode/ReaderWriter.h>
#include <llvm/Support/MemoryBuffer.h>

#include <llvm/IR/IRBuilder.h>

#include <simd-lib/bitblock.hpp>

using namespace llvm;

struct LLVM_Gen_RetVal
{
    int carry_q_size;
    void *process_block_fptr;
};

class LLVM_Generator
{
public:
    LLVM_Generator(std::string basis_pattern, std::string lf_ccname, int bits);
    ~LLVM_Generator();
    LLVM_Gen_RetVal Generate_LLVMIR(CodeGenState cg_state,
                                    std::list<PabloS*> cc_cgo);
    void Print_Register(char* name, BitBlock bit_block);
private:
    void MakeLLVMModule();
    void DefineTypes();
    void DeclareFunctions();
    void StoreBitBlockMarkerPtr(std::string name, int index);
    void SetReturnMarker(std::string marker, int output_idx);
    Value* GetMarker(std::string name);
    std::string Generate_PabloStatements(std::list<PabloS*> stmts);
    std::string Generate_PabloS(PabloS* stmt);
    Value* Generate_PabloE(PabloE* expr);

    int         mBits;
    std::string m_lf_ccname;
    std::string mBasis_Pattern;

    Module*          mMod;
    BasicBlock*      mBasicBlock;

    ExecutionEngine* mExecutionEngine;

    VectorType*  m64x2Vect;
    PointerType* m64x2Vect_Ptr1;

    PointerType* mStruct_Basis_Bits_Ptr1;
    PointerType* mStruct_Output_Ptr1;

    std::map<std::string, Value*> mMarkerMap;

    bool        mInWhile;

    int         mCarryQueueIdx;
    Value*      mptr_carry_q;

    int         mCarryQueueSize;

    ConstantInt*           mConst_int64_neg1;
    ConstantAggregateZero* mConst_Aggregate_64x2_0;
    Constant*              mConst_Aggregate_64x2_neg1;

    FunctionType* mFuncTy_0;
    Function*     mFunc_process_block;

    Constant*     mFunc_print_register;

    AllocaInst*  mPtr_basis_bits_addr;
    AllocaInst*  mPtr_carry_q_addr;
    AllocaInst*  mPtr_output_addr;
};


#endif // LLVM_GENERATOR_H
