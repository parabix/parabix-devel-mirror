/*
 *  Copyright (c) 2019 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#ifndef COMPRESSED_CARRY_MANAGER_H
#define COMPRESSED_CARRY_MANAGER_H

#include <pablo/carry_manager.h>

namespace pablo {

class CompressedCarryManager final : public CarryManager {

    using BuilderRef = const std::unique_ptr<kernel::KernelBuilder> &;

public:
    CompressedCarryManager() noexcept;

    void initializeCodeGen(BuilderRef b) override;

    /* Entering and leaving loops. */

    void enterLoopBody(BuilderRef b, llvm::BasicBlock * const entryBlock) override;

    void leaveLoopBody(BuilderRef b, llvm::BasicBlock * const exitBlock) override;

    /* Entering and leaving ifs. */

    void enterIfScope(BuilderRef b, const PabloBlock * const scope) override;

    /* Methods for processing individual carry-generating operations. */

    llvm::Value * addCarryInCarryOut(BuilderRef b, const Statement * operation, llvm::Value * const e1, llvm::Value * const e2) override;

    llvm::Value * advanceCarryInCarryOut(BuilderRef b, const Advance * advance, llvm::Value * const strm) override;

    llvm::Value * indexedAdvanceCarryInCarryOut(BuilderRef b, const IndexedAdvance * advance, llvm::Value * const strm, llvm::Value * const index_strm) override;

    /* Methods for getting and setting carry summary values for If statements */

    llvm::Value * generateSummaryTest(BuilderRef b, llvm::Value * condition) override;

    /* Clear carry state for conditional regions */

protected:

    llvm::StructType * analyse(BuilderRef b, const PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false) override;

    /* Entering and leaving scopes. */
    void leaveScope() override;

    /* Methods for processing individual carry-generating operations. */
    llvm::Value * getNextCarryIn(BuilderRef b) override;

    void setNextCarryOut(BuilderRef b, llvm::Value * const carryOut) override;
    // llvm::Value * longAdvanceCarryInCarryOut(BuilderRef b, llvm::Value * const value, const unsigned shiftAmount) override;
    llvm::Value * readCarryInSummary(BuilderRef b) const override;
    void writeCarryOutSummary(BuilderRef b, llvm::Value * const summary) const override;

    /* Summary handling routines */
    void addToCarryOutSummary(BuilderRef b, llvm::Value * const value) override;

    void phiCurrentCarryOutSummary(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock) override;
    void phiOuterCarryOutSummary(BuilderRef b, llvm::BasicBlock * const entryBlock, llvm::BasicBlock * const exitBlock) override;
    void combineCarryOutSummary(BuilderRef b, const unsigned offset) override;

private:

    llvm::Type * mBaseSummaryType = nullptr;

    llvm::Value * convertFrameToImplicitSummary(BuilderRef b) const;
    llvm::Value * convertFrameToImplicitSummaryPtr(BuilderRef b) const;
    llvm::Value * loadImplicitSummaryFromPtr(BuilderRef b, llvm::Value * ptr) const;
    llvm::Type * getSummaryTypeFromCurrentFrame() const;

};

}

#endif
