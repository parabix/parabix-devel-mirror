/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_MANAGER_H
#define CARRY_MANAGER_H
#include <llvm/IR/IRBuilder.h>

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
  
    CarryManager(IRBuilder <> * b, VectorType * bitBlockType, ConstantAggregateZero * zero, Constant * one) :
        mBuilder(b), mBitBlockType(bitBlockType), mZeroInitializer(zero), mOneInitializer(one) {}
    
    unsigned initialize(PabloBlock * blk, Value * carryDataPtr);  
    
    void generateBlockNoIncrement();
    
    Value * getBlockNoPtr();
    
    /* Methods for getting and setting individual carry values. */
    
    Value * getCarryOpCarryIn(PabloBlock * blk, int localIndex);

    Value * getUnitAdvanceCarryIn(PabloBlock * blk, int localIndex);

    Value * getShortAdvanceCarryIn(PabloBlock * blk, int localIndex, int shift_amount);
      
    void setCarryOpCarryOut(PabloBlock * blk, unsigned idx, Value * carry_out);

    void setUnitAdvanceCarryOut(PabloBlock * blk, unsigned idx, Value * carry_out);

    void setShortAdvanceCarryOut(PabloBlock * blk, unsigned idx, int shift_amount, Value * carry_out); 
    
    Value * longAdvanceCarryInCarryOut(PabloBlock * blk, int localIndex, int shift_amount, Value * carry_out);
 
    /* Methods for getting and setting carry summary values for If statements */
   
    bool blockHasCarries(PabloBlock & blk);
    
    Value * getCarrySummaryExpr(PabloBlock & blk);
    
    void generateCarryOutSummaryCode(PabloBlock & blk);
    
    bool summaryNeededInParentBlock(PabloBlock & blk);
    
    void addSummaryPhi(PabloBlock & blk, BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock);
    
    /* Methods for load/store of carries for non-while blocks. */
    
    void ensureCarriesLoadedLocal(PabloBlock & blk);

    void ensureCarriesStoredLocal(PabloBlock & blk);
    
    /* Methods for handling while statements */
    
    void ensureCarriesLoadedRecursive(PabloBlock & whileBlk);

    void initializeCarryDataPhisAtWhileEntry(PabloBlock & whileBlk, BasicBlock * whileBodyFinalBlock);

    void extendCarryDataPhisAtWhileBodyFinalBlock(PabloBlock & whileBlk, BasicBlock * whileBodyFinalBlock);

    void ensureCarriesStoredRecursive(PabloBlock & whileBlk);

    
private:
  IRBuilder <> * mBuilder;
  VectorType * mBitBlockType;
  ConstantAggregateZero * mZeroInitializer;
  Constant * mOneInitializer;
  PabloBlock * mPabloRoot;
  Value * mCarryDataPtr;
  Value * mBlockNoPtr;
  Value * mBlockNo;
  unsigned mTotalCarryDataSize;
  
  std::vector<Value *> mCarryInVector;
  std::vector<PHINode *> mCarryInPhis;  
  std::vector<PHINode *> mCarryOutAccumPhis;  
  std::vector<Value *> mCarryOutVector;
};

}

#endif // CARRY_MANAGER_H
