/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_MANAGER_H
#define CARRY_MANAGER_H
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <IDISA/idisa_builder.h>
#include <codegenstate.h>

/* 
 * Carry Data Manager.
 * 
 * Each PabloBlock (Main, If, While) has a contiguous data area for carry information.
 * The data area may be at a fixed or variable base offset from the base of the 
 * main function carry data area.
 * The data area for each block consists of contiguous space for the local carries and 
 * advances of the block plus the areas of any ifs/whiles nested within the block.

*/

using namespace llvm;

namespace pablo {

class PabloBlock;

class CarryManager {
public:
  
    CarryManager(IRBuilder <> * b, VectorType * bitBlockType, ConstantAggregateZero * zero, Constant * one, IDISA::IDISA_Builder * idb)
    : mBuilder(b)
    , mBitBlockType(bitBlockType)
    , mZeroInitializer(zero)
    , mOneInitializer(one)
    , iBuilder(idb)
    , mPabloRoot(nullptr)
    , mCurrentScope(nullptr)
    , mCarryInfo(nullptr)
    , mCurrentScopeIndex(0)
    , mCarryDataPtr(nullptr)
    , mBlockNoPtr(nullptr)
    , mBlockNo(nullptr)
    , mTotalCarryDataSize(0)
    {

    }
    
    unsigned initialize(PabloBlock * blk, Value * carryDataPtr);  
    
    void generateBlockNoIncrement();
    
    Value * getBlockNoPtr();
    
    /* Entering and leaving scopes. */
    
    void enterScope(PabloBlock * blk);

    void leaveScope();
    
    /* Methods for processing individual carry-generating operations. */
    
    Value * getCarryOpCarryIn(int localIndex);

    void setCarryOpCarryOut(unsigned idx, Value * carry_out);

    Value * advanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm);
 
    /* Methods for getting and setting carry summary values for If statements */
   
    bool blockHasCarries();
    
    Value * getCarrySummaryExpr();
    
    void generateCarryOutSummaryCode();
    
    bool summaryNeededInParentBlock();
    
    void addSummaryPhi(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock);
    
    /* Methods for load/store of carries for non-while blocks. */
    
    void ensureCarriesLoadedLocal();

    void ensureCarriesStoredLocal();
    
    /* Methods for handling while statements */
    
    void ensureCarriesLoadedRecursive();

    void initializeCarryDataPhisAtWhileEntry(BasicBlock * whileBodyFinalBlock);

    void extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock);

    void ensureCarriesStoredRecursive();

    
private:
    IRBuilder <> * mBuilder;
    VectorType * mBitBlockType;
    ConstantAggregateZero * mZeroInitializer;
    Constant * mOneInitializer;
    IDISA::IDISA_Builder * iBuilder;
    PabloBlock * mPabloRoot;
    PabloBlock * mCurrentScope;
    PabloBlockCarryData * mCarryInfo;
    unsigned mCurrentScopeIndex;
    Value * mCarryDataPtr;
    Value * mBlockNoPtr;
    Value * mBlockNo;
    unsigned mTotalCarryDataSize;

    std::vector<Value *> mCarryInVector;
    std::vector<PHINode *> mCarryInPhis;  
    std::vector<PHINode *> mCarryOutAccumPhis;  
    std::vector<Value *> mCarryOutVector;

    Value * unitAdvanceCarryInCarryOut(int localIndex, Value * strm);
    Value * shortAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm);
    Value * longAdvanceCarryInCarryOut(int localIndex, int shift_amount, Value * strm);
    
};

}

#endif // CARRY_MANAGER_H
