/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_MANAGER_H
#define CARRY_MANAGER_H

#include <pablo/carry_data.h>
#include <vector>
namespace IDISA { class IDISA_Builder; }
namespace llvm { class BasicBlock; }
namespace llvm { class ConstantInt; }
namespace llvm { class Function; }
namespace llvm { class PHINode; }
namespace llvm { class StructType; }
namespace llvm { class Type; }
namespace llvm { class Value; }
namespace pablo { class Advance; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloKernel; }
namespace pablo { class Statement; }

/* 
 * Carry Data Manager.
 * 
 * Each PabloBlock (Main, If, While) has a contiguous data area for carry information.
 * The data area may be at a fixed or variable base offset from the base of the 
 * main function carry data area.
 * The data area for each block consists of contiguous space for the local carries and 
 * advances of the block plus the areas of any ifs/whiles nested within the block.

*/

namespace pablo {

class CarryManager {

    enum { LONG_ADVANCE_BASE = 64 };

public:
  
    CarryManager(PabloKernel * const kernel) noexcept;

    void initializeCarryData();

    void initializeCodeGen();

    void finalizeCodeGen();

    /* Entering and leaving loops. */

    void enterLoopScope(const PabloBlock * const scope);

    void enterLoopBody(llvm::BasicBlock * const entryBlock);

    void leaveLoopBody(llvm::BasicBlock * const exitBlock);

    void leaveLoopScope(llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Entering and leaving ifs. */

    void enterIfScope(const PabloBlock * const scope);

    void enterIfBody(llvm::BasicBlock * const entryBlock);

    void leaveIfBody(llvm::BasicBlock * const exitBlock);

    void leaveIfScope(llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Methods for processing individual carry-generating operations. */
    
    llvm::Value * addCarryInCarryOut(const Statement * operation, llvm::Value * const e1, llvm::Value * const e2);

    llvm::Value * advanceCarryInCarryOut(const Advance * advance, llvm::Value * const strm);
 
    /* Methods for getting and setting carry summary values for If statements */
         
    llvm::Value * generateSummaryTest(llvm::Value * condition);

protected:

    static unsigned getScopeCount(PabloBlock * const scope, unsigned index = 0);

    static bool hasIterationSpecificAssignment(const PabloBlock * const scope);

    llvm::StructType * analyse(PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false);

    /* Entering and leaving scopes. */
    void enterScope(const PabloBlock * const scope);
    void leaveScope();

    /* Methods for processing individual carry-generating operations. */
    llvm::Value * getNextCarryIn();
    void setNextCarryOut(llvm::Value * const carryOut);
    llvm::Value * longAdvanceCarryInCarryOut(llvm::Value * const value, const unsigned shiftAmount);
    llvm::Value * readCarryInSummary(llvm::ConstantInt *index) const;
    void writeCarryOutSummary(llvm::Value * const summary, llvm::ConstantInt * index) const;

    /* Summary handling routines */
    void addToCarryOutSummary(llvm::Value * const value);

    llvm::Type * getBitBlockType() const;

    unsigned getBitBlockWidth() const;

    llvm::Type * getCarryPackType() const;

private:

    PabloKernel * const                             mKernel;
    IDISA::IDISA_Builder *                          iBuilder;

    llvm::Value *                                   mCurrentFrame;
    unsigned                                        mCurrentFrameIndex;

    const PabloBlock *                              mCurrentScope;
    CarryData *                                     mCarryInfo;

    llvm::Value *                                   mNextSummaryTest;

    unsigned                                        mIfDepth;

    bool                                            mHasLongAdvance;

    bool                                            mHasLoop;
    unsigned                                        mLoopDepth;
    llvm::Value *                                   mLoopSelector;    
    llvm::Value *                                   mNextLoopSelector;
    llvm::Value *                                   mCarryPackPtr;
    std::vector<llvm::PHINode *>                    mLoopIndicies;

    std::vector<CarryData>                          mCarryMetadata;

    std::vector<std::pair<llvm::Value *, unsigned>> mCarryFrameStack;

    unsigned                                        mCarryScopes;
    std::vector<unsigned>                           mCarryScopeIndex;

    std::vector<llvm::Value *>                      mCarrySummaryStack;
};

}

#endif // CARRY_MANAGER_H
