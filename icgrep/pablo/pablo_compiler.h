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
#include <string>
#include <list>
#include <vector>
#include <map>
#include <algorithm>
#include <unordered_map>
#include <pablo/pe_string.h>
#include <llvm/ADT/Twine.h>
#include <llvm/IR/IRBuilder.h>

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
    class BasicBlock;
}

namespace pablo {

using namespace llvm;

class PabloAST;
class PabloBlock;
class String;
class Var;
class Statement;
class StatementList;
class If;
class While;

static IRBuilder<> LLVM_Builder(getGlobalContext());

struct CompiledPabloFunction {
    const size_t        CarryDataSize;
    void * const        FunctionPointer;
private:
    Function *          mFunction;
    ExecutionEngine *   mExecutionEngine;
public:
    CompiledPabloFunction(size_t carryDataSize, Function * function, ExecutionEngine * executionEngine);

    inline CompiledPabloFunction(CompiledPabloFunction && cpf)
    : CarryDataSize(cpf.CarryDataSize)
    , FunctionPointer(cpf.FunctionPointer)
    , mFunction(cpf.mFunction)
    , mExecutionEngine(cpf.mExecutionEngine)
    {
        cpf.mFunction = nullptr;
        cpf.mExecutionEngine = nullptr;
    }

    ~CompiledPabloFunction();

};
#if (BLOCK_SIZE==256)
#define USE_UADD_OVERFLOW
#define USE_TWO_UADD_OVERFLOW
#endif

class PabloCompiler {
    #ifdef USE_UADD_OVERFLOW
    struct SumWithOverflowPack {
        Value * sum;
        Value * obit;
    };
    #endif

    typedef std::unordered_map<const pablo::PabloAST *, Value*>    ASTToValueMap;
    typedef std::unordered_map<const pablo::String *, Value*>      StringToValueMap;
    typedef std::vector<Value*>                                    CarryQueueVector;

public:
    PabloCompiler(const std::vector<Var *> & basisBitVars);
    ~PabloCompiler();
    void InstallExternalFunction(std::string C_fn_name, void * fn_ptr);
    CompiledPabloFunction compile(PabloBlock & pb);
private:
    void DefineTypes();
    void DeclareFunctions();
    void Examine(PabloBlock & blk);
    void DeclareCallFunctions();
    void SetOutputValue(Value * marker, const unsigned index);

    void genPrintRegister(std::string regName, Value * bitblockValue);
    void compileBlock(PabloBlock & block);
    void compileStatement(const Statement * stmt);
    void compileIf(const If * ifStmt);
    void compileWhile(const While * whileStmt);
    Value* compileExpression(const PabloAST * expr);
    Value* genCarryDataLoad(const unsigned index);
    void   genCarryDataStore(Value* carryOut, const unsigned index);
    Value* genAddWithCarry(Value* e1, Value* e2, unsigned localIndex);
    Value* genAdvanceWithCarry(Value* e1, int shift_amount, unsigned localIndex);
    Value* genUnitAdvanceWithCarry(Value* e1, unsigned localIndex);
    Value* genLongAdvanceWithCarry(Value* e1, int shift_amount, unsigned localIndex);
    Value* genBitBlockAny(Value* test);
    Value* genShiftHighbitToLow(unsigned FieldWidth, Value * op);
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


    ASTToValueMap                       mMarkerMap;
    CarryQueueVector                    mCarryInVector;
    CarryQueueVector                    mCarryOutVector;
    std::vector<int>                    mCarryDataSummaryIdx;

    const std::vector<Var *> &          mBasisBits;
#ifdef USE_LLVM_3_5
    Module* const                       mMod;
#else
    std::unique_ptr<Module>             mModOwner;
    Module *                            mMod;
#endif
    IRBuilder <> *                      mBuilder;
    ExecutionEngine*                    mExecutionEngine;

    VectorType* const                   mBitBlockType;
    PointerType*                        mBasisBitsInputPtr;

    PabloBlock *                        mPabloBlock;
    
    Value*                              mCarryDataPtr;
    Value*                              mBlockNo;
    unsigned                            mWhileDepth;
    unsigned                            mIfDepth;

    ConstantAggregateZero* const        mZeroInitializer;
    Constant* const                     mOneInitializer;

    FunctionType*                       mFunctionType;
    Function*                           mFunction;


    Value*                              mBasisBitsAddr;
    Value*                              mOutputAddrPtr;

    unsigned                            mMaxWhileDepth;

    std::map<std::string, void *>       mExternalMap;
    StringToValueMap                    mCalleeMap;

    Constant *                          mPrintRegisterFunction;
};

}

#endif // LLVM_GENERATOR_H
