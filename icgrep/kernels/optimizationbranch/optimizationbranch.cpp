#include <kernels/optimizationbranch.h>
#include "optimizationbranch_compiler.hpp"
#include <kernels/kernel_builder.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCompiler = llvm::make_unique<OptimizationBranchCompiler>(this);
    mCompiler->addBranchProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    mNonZeroKernel->addKernelDeclarations(b);
    mAllZeroKernel->addKernelDeclarations(b);
    Kernel::addKernelDeclarations(b);
}

OptimizationBranch::OptimizationBranch(const std::unique_ptr<KernelBuilder> & b,
    std::string && signature,
    not_null<Relationship *> condition,
    Kernel * const nonZeroKernel,
    Kernel * const allZeroKernel,
    Bindings && stream_inputs,
    Bindings && stream_outputs,
    Bindings && scalar_inputs,
    Bindings && scalar_outputs)
: Kernel(b, TypeId::OptimizationBranch, std::move(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs),
{Binding{b->getSizeTy(), ALL_ZERO_ACTIVE_THREADS}, Binding{b->getSizeTy(), NON_ZERO_ACTIVE_THREADS}})
, mCondition(condition)
, mNonZeroKernel(nonZeroKernel)
, mAllZeroKernel(allZeroKernel) {
   // TODO: need to pass in initial logical segment number to the pipeline branches
   // addAttribute(SynchronizationFree());
}

}
