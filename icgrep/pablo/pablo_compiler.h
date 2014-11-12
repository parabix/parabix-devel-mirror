/*
 *  Copyright (c) 2014 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef PABLO_COMPILER_H
#define PABLO_COMPILER_H

//indicates that we use llvm.uadd.with.overflow.carryin for genAddWithCarry
//#define USE_UADD_OVERFLOW
//#define USE_LONG_INTEGER_SHIFT
//#define USE_TWO_UADD_OVERFLOW

#if defined(USE_TWO_UADD_OVERFLOW) && !defined(USE_UADD_OVERFLOW)
static_assert(false, "Need to turn on them together.");
#endif

//Pablo Expressions
#include <pablo/codegenstate.h>
#include <pablo/pabloAST.h>
#include "unicode_categories.h"
#include <string>
#include <list>
#include <map>
#include <algorithm>
#include <llvm/ADT/Twine.h>

namespace llvm {
    class Value;
    class Module;
    class ExecutionEngine;
    class VectorType;
    class PointerType;
    class ConstantAggregateZero;
    class Constant;
    class FunctionType;
    class Function;
}

using namespace llvm;

namespace pablo {

struct LLVM_Gen_RetVal
{
    int carry_q_size;
    int advance_q_size;
    void *process_block_fptr;
};

class PabloCompiler {
    #ifdef USE_UADD_OVERFLOW
    struct SumWithOverflowPack {
        Value * sum;
        Value * obit;
    };
    #endif

    typedef std::unordered_map<const pablo::String *, Value*>   StringToValueMap;
    typedef std::vector<Value*>                                 CarryQueueVector;

public:
    PabloCompiler(const std::vector<Var *> & basisBitVars);
    ~PabloCompiler();
    LLVM_Gen_RetVal compile(PabloBlock & pb);
private:
    void DefineTypes();
    void DeclareFunctions();
    void Examine(StatementList & stmts);
    void Examine(PabloAST * expr);
    void DeclareCallFunctions();
    void SetOutputValue(Value * marker, const unsigned index);

    Value* compileStatements(const StatementList & stmts);
    Value* compileStatement(const PabloAST * stmt);
    Value* compileExpression(const PabloAST * expr);
    Value* genCarryInLoad(const unsigned index);
    void   genCarryOutStore(Value* carryOut, const unsigned index);
    Value* genAdvanceInLoad(const unsigned index);
    void   genAdvanceOutStore(Value* advanceOut, const unsigned index);
    Value* genAddWithCarry(Value* e1, Value* e2);
    Value* genAdvanceWithCarry(Value* e1, int shift_amount);
    Value* genBitBlockAny(Value* test);
    Value* genShiftHighbitToLow(Value* e, const Twine & namehint = "");
    Value* genShiftLeft64(Value* e, const Twine & namehint = "") ;
    Value* genNot(Value* expr);

    #ifdef USE_UADD_OVERFLOW
    #ifdef USE_TWO_UADD_OVERFLOW
    Function* mFunctionUaddOverflow;
    SumWithOverflowPack callUaddOverflow(Value *e1, Value *e2);
    #else
    Function* mFunctionUaddOverflowCarryin;
    SumWithOverflowPack callUaddOverflow(Value *e1, Value *e2, Value *cin);
    #endif
    #endif


    StringToValueMap                    mMarkerMap;
    CarryQueueVector                    mCarryQueueVector;
    CarryQueueVector                    mAdvanceQueueVector;

    const std::vector<Var *> &          mBasisBits;

    Module* const                       mMod;
    BasicBlock*                         mBasicBlock;
    ExecutionEngine*                    mExecutionEngine;

    VectorType* const                   mBitBlockType;
    PointerType*                        mBasisBitsInputPtr;

    unsigned                            mCarryQueueIdx;
    Value*                              mCarryQueuePtr;
    unsigned                            mNestingDepth;
    unsigned                            mCarryQueueSize;

    unsigned                            mAdvanceQueueIdx;
    Value*                              mAdvanceQueuePtr;
    unsigned                            mAdvanceQueueSize;

    ConstantAggregateZero* const        mZeroInitializer;
    Constant* const                     mOneInitializer;

    FunctionType*                       mFunctionType;
    Function*                           mFunction;


    Value*                              mBasisBitsAddr;
    Value*                              mOutputAddrPtr;

    unsigned                            mMaxNestingDepth;

    StringToValueMap                    mCalleeMap;
};

}

#endif // LLVM_GENERATOR_H
