/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_MANAGER_H
#define CARRY_MANAGER_H

#include <pablo/carry_data.h>
#include <llvm/ADT/SmallVector.h>
#include <memory>

namespace IDISA { class IDISA_Builder; }
namespace llvm { class BasicBlock; }
namespace llvm { class ConstantInt; }
namespace llvm { class Function; }
namespace llvm { class PHINode; }
namespace llvm { class StructType; }
namespace llvm { class Type; }
namespace llvm { class Value; }
namespace pablo { class Advance; }
namespace pablo { class IndexedAdvance; }
namespace pablo { class PabloBlock; }
namespace pablo { class PabloKernel; }
namespace pablo { class Statement; }
namespace kernel { class KernelBuilder; }

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

    template <typename T>
    using Vec = llvm::SmallVector<T, 64>;

    using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

public:

    CarryManager() noexcept;

    virtual ~CarryManager() = default;

    virtual void initializeCarryData(BuilderRef b, PabloKernel * const kernel);

    virtual void releaseCarryData(BuilderRef idb);

    virtual void initializeCodeGen(BuilderRef b);

    virtual void finalizeCodeGen(BuilderRef b);

    /* Entering and leaving loops. */

    virtual void enterLoopScope(BuilderRef b, const PabloBlock * const scope);

    virtual void enterLoopBody(BuilderRef b, llvm::BasicBlock * const entryBlock);

    virtual void leaveLoopBody(BuilderRef b, llvm::BasicBlock * const exitBlock);

    virtual void leaveLoopScope(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Entering and leaving ifs. */

    virtual void enterIfScope(BuilderRef b, const PabloBlock * const scope);

    virtual void enterIfBody(BuilderRef b, llvm::BasicBlock * const entryBlock);

    virtual void leaveIfBody(BuilderRef b, llvm::BasicBlock * const exitBlock);

    virtual void leaveIfScope(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Methods for processing individual carry-generating operations. */

    virtual llvm::Value * addCarryInCarryOut(BuilderRef b, const Statement * operation, llvm::Value * const e1, llvm::Value * const e2);

    virtual llvm::Value * subBorrowInBorrowOut(BuilderRef b, const Statement * operation, llvm::Value * const e1, llvm::Value * const e2);

    virtual llvm::Value * advanceCarryInCarryOut(BuilderRef b, const Advance * advance, llvm::Value * const strm);

    virtual llvm::Value * indexedAdvanceCarryInCarryOut(BuilderRef b, const IndexedAdvance * advance, llvm::Value * const strm, llvm::Value * const index_strm);

    /* Methods for getting and setting carry summary values for If statements */

    virtual llvm::Value * generateSummaryTest(BuilderRef b, llvm::Value * condition);

    /* Clear carry state for conditional regions */

    virtual void clearCarryData(BuilderRef idb);

protected:

    static unsigned getScopeCount(const PabloBlock * const scope, unsigned index = 0);

    virtual llvm::StructType * analyse(BuilderRef b, const PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false);

    /* Entering and leaving scopes. */
    virtual void enterScope(BuilderRef b);
    virtual void leaveScope();

    /* Methods for processing individual carry-generating operations. */
    virtual llvm::Value * getNextCarryIn(BuilderRef b);
    virtual void setNextCarryOut(BuilderRef b, llvm::Value * const carryOut);
    virtual llvm::Value * longAdvanceCarryInCarryOut(BuilderRef b, llvm::Value * const value, const unsigned shiftAmount);
    virtual llvm::Value * readCarryInSummary(BuilderRef b) const;
    virtual void writeCarryOutSummary(BuilderRef b, llvm::Value * const summary) const;

    /* Summary handling routines */
    virtual void addToCarryOutSummary(BuilderRef b, llvm::Value * const value);

    virtual void phiCurrentCarryOutSummary(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);
    virtual void phiOuterCarryOutSummary(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);
    virtual void writeCurrentCarryOutSummary(BuilderRef b);
    virtual void combineCarryOutSummary(BuilderRef b, const unsigned offset);

protected:

    const PabloKernel *                             mKernel;

    llvm::Value *                                   mCurrentFrame;
    unsigned                                        mCurrentFrameIndex;

    const CarryData *                               mCarryInfo;

    llvm::Value *                                   mNextSummaryTest;

    unsigned                                        mIfDepth;

    bool                                            mHasLongAdvance;
    unsigned                                        mIndexedLongAdvanceTotal;
    unsigned                                        mIndexedLongAdvanceIndex;
    bool                                            mHasNonCarryCollapsingLoops;
    bool                                            mHasLoop;
    unsigned                                        mLoopDepth;
    llvm::PHINode *                                 mNestedLoopCarryInMaskPhi;
    llvm::Value *                                   mLoopSelector;
    llvm::Value *                                   mNextLoopSelector;
    llvm::Value *                                   mCarryPackPtr;
    Vec<llvm::PHINode *>                            mLoopIndicies;

    Vec<CarryData>                                  mCarryMetadata;

    Vec<std::pair<llvm::Value *, unsigned>>         mCarryFrameStack;

    unsigned                                        mCarryScopes;
    Vec<unsigned>                                   mCarryScopeIndex;

    Vec<llvm::Value *>                              mCarrySummaryStack;
};

}

#endif // CARRY_MANAGER_H
