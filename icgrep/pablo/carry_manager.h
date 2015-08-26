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
#include <pablo/codegenstate.h>
#include <pablo/carry_data.h>

/* 
 * Carry Data Manager.
 * 
 * Each PabloBlock (Main, If, While) has a contiguous data area for carry information.
 * The data area may be at a fixed or variable base offset from the base of the 
 * main function carry data area.
 * The data area for each block consists of contiguous space for the local carries and 
 * advances of the block plus the areas of any ifs/whiles nested within the block.

*/

enum CarryManagerStrategy {BitBlockStrategy, SequentialFullyPackedStrategy};


using namespace llvm;

namespace pablo {

class PabloBlock;



class CarryManager {
public:
  
    CarryManager(IRBuilder <> * b, VectorType * bitBlockType, ConstantAggregateZero * zero, Constant * one, IDISA::IDISA_Builder * idb)
    : mPACK_SIZE(BLOCK_SIZE)
    , mITEMS_PER_PACK(1)
    , mBuilder(b)
    , mBitBlockType(bitBlockType)
    , mZeroInitializer(zero)
    , mOneInitializer(one)
    , iBuilder(idb)
    , mPabloRoot(nullptr)
    , mCurrentScope(nullptr)
    , mCarryInfo(nullptr)
    , mCurrentFrameIndex(0)
    , mCarryPackBasePtr(nullptr)
    , mCarryBitBlockPtr(nullptr)
    , mPopcountBasePtr(nullptr)
    , mBlockNoPtr(nullptr)
    , mBlockNo(nullptr)
    , mPabloCountCount(0)
    , mTotalCarryDataBitBlocks(0)
    , mCarryDataAllocationSize(0)
    {

    }

    ~CarryManager();
    
    void initialize(Module * m, PabloBlock * blk);
    
    unsigned enumerate(PabloBlock * blk, unsigned ifDepth, unsigned whileDepth);
    
    void generateBlockNoIncrement();    
    Value * getBlockNoPtr();
    
    /* Entering and leaving scopes. */
    
    void enterScope(PabloBlock * blk);
    void leaveScope();
    
    /* Methods for processing individual carry-generating operations. */
    
    Value * getCarryOpCarryIn(int localIndex);
    void setCarryOpCarryOut(unsigned idx, Value * carry_out);
    Value * addCarryInCarryOut(int localIndex, Value* e1, Value* e2);


    Value * advanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * strm);
 
    /* Methods for getting and setting carry summary values for If statements */
   
    bool blockHasCarries();
    
    void initializeCarryDataAtIfEntry();
    
    Value * getCarrySummaryExpr();
    
    void generateCarryOutSummaryCodeIfNeeded();
    
    void buildCarryDataPhisAfterIfBody(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock);
    
    void addSummaryPhiIfNeeded(BasicBlock * ifEntryBlock, BasicBlock * ifBodyFinalBlock);
    
    /* Methods for handling while statements */
    
    void ensureCarriesLoadedRecursive();

    void initializeCarryDataPhisAtWhileEntry(BasicBlock * whileBodyFinalBlock);

    void extendCarryDataPhisAtWhileBodyFinalBlock(BasicBlock * whileBodyFinalBlock);

    void ensureCarriesStoredRecursive();

    void ensureCarriesStoredLocal();
    
    Value * popCount(Value * to_count, unsigned globalIdx);
    
    Value * declareCarryDataArray(Module * m);

    
private:
    unsigned mPACK_SIZE;
    unsigned mITEMS_PER_PACK;
    IRBuilder <> * mBuilder;
    VectorType * mBitBlockType;
    Constant * mZeroInitializer;
    Constant * mOneInitializer;
    IDISA::IDISA_Builder * iBuilder;
    PabloBlock * mPabloRoot;
    PabloBlock * mCurrentScope;
    PabloBlockCarryData * mCarryInfo;
    unsigned mCurrentFrameIndex;
    Value * mCarryPackBasePtr;
    Type * mCarryPackType;
    Value * mCarryBitBlockPtr;
    Value * mPopcountBasePtr;
    Value * mBlockNoPtr;
    Value * mBlockNo;
    unsigned mPabloCountCount; // Number of Pablo "Count" operations
    unsigned mTotalCarryDataBitBlocks;
    unsigned mCarryDataAllocationSize;
    
    std::vector<PabloBlockCarryData *> mCarryInfoVector;


    std::vector<Value *> mCarryPackPtr;
    std::vector<Value *> mCarryInPack;
    std::vector<PHINode *> mCarryInPhis;  
    std::vector<PHINode *> mCarryOutAccumPhis;  
    std::vector<Value *> mCarryOutPack;

    Value * unitAdvanceCarryInCarryOut(int localIndex, Value * strm);
    Value * shortAdvanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * strm);
    Value * longAdvanceCarryInCarryOut(int localIndex, unsigned shift_amount, Value * strm);
    
    
    /* Helper routines */
    Value * getCarryPack(unsigned packIndex);
    void storeCarryPack(unsigned packIndex);
    
    Value * maskSelectBitRange(Value * pack, unsigned lo_bit, unsigned bitCount);     
    Value * getCarryInBits(unsigned carryBitPos, unsigned bits);
    void extractAndSaveCarryOutBits(Value * strm, unsigned carryBit_lo, unsigned carryBitCount);
    Value * pack2bitblock(Value * pack);
    Value* genShiftLeft64(Value* e);


    unsigned absPosition(unsigned frameOffsetinBits, unsigned relPos);
    unsigned carryOpPosition(unsigned localIndex) ;
    unsigned advance1Position(unsigned localIndex);
    unsigned shortAdvancePosition(unsigned localIndex);
    unsigned longAdvanceBitBlockPosition(unsigned localIndex);
    unsigned localBasePack();
    unsigned scopeBasePack();
    unsigned summaryPackIndex();
    unsigned summaryPosition();
    unsigned summaryBits();
};

}

#endif // CARRY_MANAGER_H
