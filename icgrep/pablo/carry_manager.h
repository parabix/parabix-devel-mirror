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
#include <pablo/pablo_kernel.h>
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

    enum { LONG_ADVANCE_BASE = 64 };

public:
  
    explicit CarryManager(IDISA::IDISA_Builder * idb) noexcept
    : iBuilder(idb)
    , mKernel(nullptr)
    , mSelf(nullptr)
    , mFunction(nullptr)
    , mBitBlockType(idb->getBitBlockType())
    , mBitBlockWidth(idb->getBitBlockWidth())
    , mCurrentFrameIndex(0)
    , mCurrentScope(nullptr)
    , mCarryInfo(nullptr)
    , mCarryPackType(mBitBlockType)
    , mCarryPackPtr(nullptr)
    , mIfDepth(0)
    , mLoopDepth(0) {

    }

    ~CarryManager() {

    }

    void initializeCarryData(PabloKernel * const kernel);

    void initializeCodeGen(Value * const self, Function *function);

    /* Entering and leaving loops. */

    void enterLoopScope(PabloBlock * const scope);

    void enterLoopBody(BasicBlock * const entryBlock);

    void leaveLoopBody(BasicBlock * const exitBlock);

    void leaveLoopScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock);

    /* Entering and leaving ifs. */

    void enterIfScope(PabloBlock * const scope);

    void enterIfBody(BasicBlock * const entryBlock);

    void leaveIfBody(BasicBlock * const exitBlock);

    void leaveIfScope(BasicBlock * const entryBlock, BasicBlock * const exitBlock);

    /* Methods for processing individual carry-generating operations. */
    
    Value * addCarryInCarryOut(const Statement * operation, Value * const e1, Value * const e2);

    Value * advanceCarryInCarryOut(const Advance * advance, Value * const strm);
 
    /* Methods for getting and setting carry summary values for If statements */
         
    Value * generateSummaryTest(Value * condition);
    
protected:

    static unsigned enumerate(PabloBlock * const scope, unsigned index = 0);
    static bool requiresVariableLengthMode(const PabloBlock * const scope);
    StructType * analyse(PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0);

    /* Entering and leaving scopes. */
    void enterScope(PabloBlock * const scope);
    void leaveScope();

    /* Methods for processing individual carry-generating operations. */
    Value * getNextCarryIn();
    void setNextCarryOut(Value * const carryOut);
    Value * longAdvanceCarryInCarryOut(const unsigned shiftAmount, Value * const value);

    /* Summary handling routines */
    void addToSummary(Value * const value);

    bool inCollapsingCarryMode() const;

private:

    IDISA::IDISA_Builder * const                iBuilder;
    PabloKernel *                               mKernel;
    Value *                                     mSelf;
    Function *                                  mFunction;
    Type * const                                mBitBlockType;
    const unsigned                              mBitBlockWidth;

    Value *                                     mCurrentFrame;
    unsigned                                    mCurrentFrameIndex;

    PabloBlock *                                mCurrentScope;
    CarryData *                                 mCarryInfo;

    Type *                                      mCarryPackType;
    Value *                                     mCarryPackPtr;

    unsigned                                    mIfDepth;

    unsigned                                    mLoopDepth;    
    Value *                                     mLoopSelector;
    std::vector<PHINode *>                      mLoopIndicies;

    std::vector<CarryData>                      mCarryMetadata;
    std::vector<std::pair<Value *, unsigned>>   mCarryFrame;

    std::vector<Value *>                        mCarrySummary;
};

}

#endif // CARRY_MANAGER_H
