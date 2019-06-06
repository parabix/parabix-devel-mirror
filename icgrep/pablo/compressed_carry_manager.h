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
protected:
    llvm::StructType * analyse(const std::unique_ptr<kernel::KernelBuilder> & b,
                               const PabloBlock * const scope,
                               const unsigned ifDepth = 0,
                               const unsigned whileDepth = 0,
                               const bool isNestedWithinNonCarryCollapsingLoop = false) override;

    llvm::Value * getNextCarryIn(const std::unique_ptr<kernel::KernelBuilder> & b) override;

    void setNextCarryOut(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * carryOut) override;

    void addToCarryOutSummary(const std::unique_ptr<kernel::KernelBuilder> & b, llvm::Value * const value) override;

    llvm::Value * readCarryInSummary(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) const override;
};

}

#endif
