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
#include <pablo/carry_manager.h>
#include <llvm/ADT/Twine.h>
#include <llvm/IR/IRBuilder.h>
#include <IDISA/idisa_builder.h>

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
class PabloFunction;
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

    inline Function * getLLVMFunction() const {
        return mFunction;
    }

    inline ExecutionEngine * getExecutionEngine() const {
        return mExecutionEngine;
    }

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
    PabloCompiler();
    ~PabloCompiler();
    void InstallExternalFunction(std::string C_fn_name, void * fn_ptr);
    CompiledPabloFunction compile(pablo::PabloFunction & function);
    Module * getModule();
private:
    void GenerateFunction(PabloFunction & function);
    void DeclareFunctions(ExecutionEngine * ee);
    void Examine(PabloBlock & blk);
    void DeclareCallFunctions(ExecutionEngine * ee);
    void SetOutputValue(Value * marker, const unsigned index);

    void genPrintRegister(std::string regName, Value * bitblockValue);
    void compileBlock(PabloBlock & block);
    void compileStatement(const Statement * stmt);
    Value * genBitTest2(Value * e1, Value * e2);
    void compileIf(const If * ifStmt);
    void compileWhile(const While * whileStmt);
    Value* compileExpression(const PabloAST * expr);
    Value* genAddWithCarry(Value* e1, Value* e2, unsigned localIndex);
    Value* genUnitAdvanceWithCarry(Value* e1, unsigned localIndex);
    Value* genShortAdvanceWithCarry(Value* e1, unsigned localIndex, int shift_amount);
    Value* genLongAdvanceWithCarry(Value* e1, unsigned localIndex, int shift_amount);
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

#ifdef USE_LLVM_3_5
    Module* const                       mMod;
#else
    std::unique_ptr<Module>             mModOwner;
    Module *                            mMod;
#endif
    IRBuilder <> *                      mBuilder;

    CarryManager *                      mCarryManager;

    VectorType* const                   mBitBlockType;
    IDISA::IDISA_Builder                iBuilder;
    PointerType*                        mInputPtr;

    PabloBlock *                        mPabloBlock;
    
    Value*                              mCarryDataPtr;
    unsigned                            mWhileDepth;
    unsigned                            mIfDepth;

    ConstantAggregateZero* const        mZeroInitializer;
    Constant* const                     mOneInitializer;

    Function *                          mFunction;
    Value *                             mInputAddressPtr;
    Value *                             mOutputAddressPtr;

    unsigned                            mMaxWhileDepth;

    std::map<std::string, void *>       mExternalMap;
    StringToValueMap                    mCalleeMap;

    Constant *                          mPrintRegisterFunction;
};

inline Module * PabloCompiler::getModule() {
    return mMod;
}

}

#endif // LLVM_GENERATOR_H
