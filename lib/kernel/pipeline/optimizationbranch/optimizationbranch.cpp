#include <kernel/pipeline/optimizationbranch.h>
#include "optimizationbranch_compiler.hpp"
#include <kernel/core/kernel_builder.h>

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addInternalProperties(BuilderRef b) {
    mCompiler = llvm::make_unique<OptimizationBranchCompiler>(this);
    mCompiler->addBranchProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(BuilderRef b) {
    mCompiler->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeThreadLocalMethod(BuilderRef b) {
    mCompiler->generateInitializeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(BuilderRef b) {
    mCompiler->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeThreadLocalMethod(BuilderRef b) {
    mCompiler->generateFinalizeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(BuilderRef b) {
    mCompiler->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addKernelDeclarations(BuilderRef b) {
    mNonZeroKernel->addKernelDeclarations(b);
    mAllZeroKernel->addKernelDeclarations(b);
    Kernel::addKernelDeclarations(b);
}

OptimizationBranch::OptimizationBranch(BuilderRef b,
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
{InternalScalar{b->getSizeTy(), LOGICAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), ALL_ZERO_LOGICAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), ALL_ZERO_ACTIVE_THREADS},
 InternalScalar{b->getSizeTy(), NON_ZERO_LOGICAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), NON_ZERO_ACTIVE_THREADS},
 InternalScalar{ScalarType::ThreadLocal, b->getSizeTy()->getPointerTo(), SPAN_BUFFER},
 InternalScalar{ScalarType::ThreadLocal, b->getSizeTy(), SPAN_CAPACITY}})
, mCondition(condition)
, mNonZeroKernel(nonZeroKernel)
, mAllZeroKernel(allZeroKernel) {
    addAttribute(InternallySynchronized());
}

}
