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
#include <llvm/IR/Constants.h>
#include <kernels/kernel.h>

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
  
    CarryManager(IDISA::IDISA_Builder * idb)
    : iBuilder(idb)
    , mBitBlockType(idb->getBitBlockType())
    , mBitBlockWidth(idb->getBitBlockWidth())
    , mRootScope(nullptr)
    , mCurrentScope(nullptr)
    , mCarryInfo(nullptr)
    , mCurrentFrameIndex(0)
    , mCarryPackBasePtr(nullptr)
    , mCarryBitBlockPtr(nullptr)
    , mPopcountBasePtr(nullptr)
    , mKernelBuilder(nullptr)
    , mPabloCountCount(0)
    , mTotalCarryDataBitBlocks(0)
    , mCarryDataAllocationSize(0)
    , mFilePosIdx(2)
    {

    }

    ~CarryManager();
    
    void initialize(PabloFunction * const function, kernel::KernelBuilder * const kBuilder);

    void reset();

    unsigned enumerate(PabloBlock * blk, unsigned ifDepth, unsigned whileDepth);
          
    /* Entering and leaving scopes. */
    
    void enterScope(PabloBlock * const scope);
    void leaveScope();
    
    /* Methods for processing individual carry-generating operations. */
    
    Value * addCarryInCarryOut(const unsigned localIndex, Value * const e1, Value * const e2);

    Value * advanceCarryInCarryOut(const unsigned localIndex, const unsigned shiftAmount, Value * const strm);
 
    /* Methods for getting and setting carry summary values for If statements */
   
    bool hasCarries() const;
       
    Value * generateSummaryTest(Value * condition);
    
    void storeCarryOutSummary();

    void addOuterSummaryToNestedSummary();

    void buildCarryDataPhisAfterIfBody(BasicBlock * const entry, BasicBlock * const end);
       
    /* Methods for handling while statements */
    
    void ensureCarriesLoadedRecursive();

    void initializeWhileEntryCarryDataPhis(BasicBlock * const end);

    void finalizeWhileBlockCarryDataPhis(BasicBlock * const end);

    void ensureCarriesStoredRecursive();
    
    Value * popCount(Value * to_count, unsigned globalIdx);
    
    Value * declareCarryDataArray(Module * m);

protected:

    Value * shortAdvanceCarryInCarryOut(const unsigned index, const unsigned shiftAmount, Value * const value);
    Value * longAdvanceCarryInCarryOut(const unsigned index, const unsigned shiftAmount, Value * const value);

    /* Methods for processing individual carry-generating operations. */

    Value * getCarryIn(const unsigned localIndex);
    void setCarryOut(const unsigned idx, Value * carryOut);

    /* Helper routines */
    Value * getCarryPack(const unsigned packIndex);
    void storeCarryOut(const unsigned packIndex);
    
    Value * addToSummary(Value * const value);

    bool hasSummary() const;
    unsigned relativeFrameOffset(const unsigned frameOffset, const unsigned index) const;
    unsigned addPosition(const unsigned localIndex) const;
    unsigned unitAdvancePosition(const unsigned localIndex) const;
    unsigned shortAdvancePosition(const unsigned localIndex) const;
    unsigned longAdvancePosition(const unsigned localIndex) const;
    unsigned localBasePack() const;
    unsigned scopeBasePack() const;
    unsigned summaryPack() const;

private:
    IDISA::IDISA_Builder * const iBuilder;
    Type * const mBitBlockType;
    const unsigned mBitBlockWidth;
    PabloBlock * mRootScope;
    PabloBlock * mCurrentScope;
    CarryData * mCarryInfo;
    unsigned mCurrentFrameIndex;
    Value * mCarryPackBasePtr;
    Type * mCarryPackType;
    Value * mCarryBitBlockPtr;
    Value * mPopcountBasePtr;
    kernel::KernelBuilder * mKernelBuilder;
    unsigned mPabloCountCount; // Number of Pablo "Count" operations
    unsigned mTotalCarryDataBitBlocks;
    unsigned mCarryDataAllocationSize;
    std::vector<CarryData *> mCarryInfoVector;
    std::vector<Value *> mCarryPackPtr;
    std::vector<Value *> mCarryInPack;
    std::vector<PHINode *> mCarryInPhis;
    std::vector<PHINode *> mCarryOutAccumPhis;
    std::vector<Value *> mCarryOutPack;
    std::vector<Value *> mCarrySummary;
    int mCdArrayIdx;
    int mPcArrayIdx;
    int mFilePosIdx;
};

inline bool CarryManager::hasCarries() const {
    return mCarryInfo->hasCarries();
}

}

#endif // CARRY_MANAGER_H
