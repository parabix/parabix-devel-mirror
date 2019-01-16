#ifndef OPTIMIZATIONBRANCH_COMPILER_HPP
#define OPTIMIZATIONBRANCH_COMPILER_HPP

#include <kernels/optimizationbranch.h>
#include <kernels/streamset.h>
#include <kernels/kernel_builder.h>

namespace kernel {

    struct OptimizationBranchCompiler {

        OptimizationBranchCompiler(OptimizationBranch * const branch);




    private:

        OptimizationBranch * const          mBranch;



        std::vector<llvm::Value *>          mProcessedInputItems;
        std::vector<llvm::PHINode *>        mAccessibleInputItemPhi;

        std::vector<llvm::Value *>          mProducedOutputItems;
        std::vector<llvm::PHINode *>        mWritableOrConsumedOutputItemPhi;

    };

}


#endif // OPTIMIZATIONBRANCH_COMPILER_HPP
