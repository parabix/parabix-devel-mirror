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
public:
    CompressedCarryManager() noexcept;

    void initializeCodeGen(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;

    /* Entering and leaving loops. */

    void enterLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const entryBlock) override;

    void leaveLoopBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock) override;

    /* Entering and leaving ifs. */

    void enterIfScope(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope) override;

    void leaveIfBody(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::BasicBlock * const exitBlock) override;

    /* Methods for processing individual carry-generating operations. */

    llvm::Value * addCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Statement * operation, llvm::Value * const e1, llvm::Value * const e2) override;

    llvm::Value * advanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const Advance * advance, llvm::Value * const strm) override;

    llvm::Value * indexedAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const IndexedAdvance * advance, llvm::Value * const strm, llvm::Value * const index_strm) override;

    /* Methods for getting and setting carry summary values for If statements */

    llvm::Value * generateSummaryTest(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * condition) override;

    /* Clear carry state for conditional regions */

protected:

    llvm::StructType * analyse(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, const PabloBlock * const scope, const unsigned ifDepth = 0, const unsigned whileDepth = 0, const bool isNestedWithinNonCarryCollapsingLoop = false) override;

    /* Entering and leaving scopes. */
    void leaveScope() override;

    /* Methods for processing individual carry-generating operations. */
    llvm::Value * getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) override;

    void setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const carryOut) override;
    // llvm::Value * longAdvanceCarryInCarryOut(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value, const unsigned shiftAmount) override;
    llvm::Value * readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) const override;
    void writeCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const summary) const override;

    /* Summary handling routines */
    void addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder, llvm::Value * const value) override;

private:

    llvm::Type * mBaseSummaryType = nullptr;

    llvm::Value * convertFrameToImplicitSummary(const std::unique_ptr<kernel::KernelBuilder> & b) const;

};

}

#endif
