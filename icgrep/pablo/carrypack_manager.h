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
namespace pablo { class CarryProducingStatement; }
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
      
    struct CarryGroup {
        llvm::Value * carryIn;
        llvm::Value * carryOut;
        unsigned packedSize;
        unsigned groupSize;
        unsigned frameIndex;
        CarryGroup() = default;
    };

    using CarryFrame = std::pair<llvm::Value *, std::vector<llvm::Value *>>;

    CarryManager() noexcept;

    void initializeCarryData(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, PabloKernel * const kernel);

    void releaseCarryData(const std::unique_ptr<kernel::KernelBuilder> & idb);

    void initializeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    void finalizeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    /* Entering and leaving loops. */

    void enterLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);

    void enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock);

    void leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock);

    void leaveLoopScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Entering and leaving ifs. */

    void enterIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);

    void enterIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock);

    void leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock);

    void leaveIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock);

    /* Methods for processing individual carry-generating operations. */
    
    llvm::Value * addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op, llvm::Value * const e1, llvm::Value * const e2);

    llvm::Value * advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Advance * advance, llvm::Value * const strm);
 
    llvm::Value * indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const IndexedAdvance * advance, llvm::Value * const strm, llvm::Value * const index_strm);

    /* Methods for getting and setting carry summary values for If statements */
         
    llvm::Value * generateSummaryTest(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * condition);

protected:

    static unsigned getScopeCount(const PabloBlock * const scope, unsigned index = 0);

    unsigned assignDefaultCarryGroups(PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned loopDepth = 0, unsigned carryGroups = 0);

    static bool hasIterationSpecificAssignment(const PabloBlock * const scope);

    llvm::StructType * analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope,
                               const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false);

    /* Entering and leaving scopes. */
    void enterScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope);
    void leaveScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder);

    /* Methods for processing individual carry-generating operations. */
    llvm::Value * getCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const pablo::CarryProducingStatement * const op);
    llvm::Value * getCarryInGroup(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const pablo::CarryProducingStatement * const op);
    llvm::Value * readCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op);

    void setCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const pablo::CarryProducingStatement * const op, llvm::Value * carryOut);
    llvm::Value * getCarryOutGroupPtr(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const pablo::CarryProducingStatement * const op);
    llvm::Value * writeCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const CarryProducingStatement * const op, llvm::Value * carryOut);

    llvm::Value * longAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value, const unsigned shiftAmount);

    /* Summary handling routines */
    llvm::Value * readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::ConstantInt *index);
    void addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value);
    void writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const summary, llvm::ConstantInt * index);

private:

    const PabloKernel *                             mKernel;

    unsigned                                        mVectorWidth;
    unsigned                                        mElementWidth;

    llvm::Value *                                   mCurrentFrame;
    std::vector<llvm::Value *>                      mCurrentFrameOffset;
    unsigned                                        mCurrentFrameIndex;
    std::vector<CarryGroup>                         mCarryGroup;

    const PabloBlock *                              mCurrentScope;
    CarryData *                                     mCarryInfo;

    llvm::Value *                                   mNextSummaryTest;

    unsigned                                        mIfDepth;

    bool                                            mHasLongAdvance;
    bool                                            mHasNonCarryCollapsingLoops;
    bool                                            mHasLoop;
    unsigned                                        mLoopDepth;
    std::array<llvm::Value *, 2>                    mLoopSelector;

    std::vector<llvm::PHINode *>                    mLoopIndicies;

    std::vector<CarryData>                          mCarryMetadata;

    std::vector<CarryFrame>                         mNonCarryCollapsingLoopCarryFrameStack;

    unsigned                                        mCarryScopes;
    std::vector<unsigned>                           mCarryScopeIndex;

    std::vector<llvm::Value *>                      mCarrySummaryStack;
};

}

#endif // CARRY_MANAGER_H
