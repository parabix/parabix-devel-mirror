/*
 *  Copyright (c) 2015 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef CARRY_MANAGER_H
#define CARRY_MANAGER_H

#include <pablo/carry_data.h>
#include <memory>
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

public:

    CarryManager() noexcept;

    virtual ~CarryManager() = default;

    virtual void initializeCarryData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, PabloKernel * const kernel);

    virtual void releaseCarryData(const std::unique_ptr<kernel::KernelBuilder> & idb);

    virtual void initializeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    virtual void finalizeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    /* Entering and leaving loops. */

    virtual void enterLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);

    virtual void enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock);

    virtual void leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock);

    virtual void leaveLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Entering and leaving ifs. */

    virtual void enterIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);

    virtual void enterIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock);

    virtual void leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock);

    virtual void leaveIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Methods for processing individual carry-generating operations. */

    virtual llvm::Value * addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Statement * operation, llvm::Value * const e1, llvm::Value * const e2);

    virtual llvm::Value * advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Advance * advance, llvm::Value * const strm);

    virtual llvm::Value * indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const IndexedAdvance * advance, llvm::Value * const strm, llvm::Value * const index_strm);

    /* Methods for getting and setting carry summary values for If statements */

    virtual llvm::Value * generateSummaryTest(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * condition);

    /* Clear carry state for conditional regions */

    virtual void clearCarryData(const std::unique_ptr<kernel::KernelBuilder> & idb);

protected:

    static unsigned getScopeCount(const PabloBlock * const scope, unsigned index = 0);

    virtual llvm::StructType * analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false);

    // Implementation of analyse with provided carry/summary types
    virtual llvm::StructType * analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope, const unsigned ifDepth, const unsigned whileDepth, const bool isNestedWithinNonCarryCollapsingLoop, llvm::Type * carryTy, llvm::Type * summaryTy);

    /* Entering and leaving scopes. */
    virtual void enterScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);
    virtual void leaveScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    /* Methods for processing individual carry-generating operations. */
    virtual llvm::Value * getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);
    virtual void setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const carryOut);
    virtual llvm::Value * longAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value, const unsigned shiftAmount);
    virtual llvm::Value * readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::ConstantInt *index) const;
    virtual void writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const summary, llvm::ConstantInt * index) const;

    /* Summary handling routines */
    virtual void addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value);

protected:

    const PabloKernel *                             mKernel;

    llvm::Value *                                   mCurrentFrame;
    unsigned                                        mCurrentFrameIndex;

    const PabloBlock *                              mCurrentScope;
    const CarryData *                               mCarryInfo;

    llvm::Value *                                   mNextSummaryTest;

    unsigned                                        mIfDepth;

    bool                                            mHasLongAdvance;
    unsigned                                        mIndexedLongAdvanceTotal;
    unsigned                                        mIndexedLongAdvanceIndex;
    bool                                            mHasNonCarryCollapsingLoops;
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
