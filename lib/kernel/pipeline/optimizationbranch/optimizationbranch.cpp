#include <kernel/pipeline/optimizationbranch.h>
#include "optimizationbranch_compiler.hpp"
#include <kernel/core/kernel_builder.h>

namespace kernel {

#define COMPILER (reinterpret_cast<OptimizationBranchCompiler *>(b->getCompiler()))

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief instantiateKernelCompiler
 ** ------------------------------------------------------------------------------------------------------------- */
std::unique_ptr<KernelCompiler> OptimizationBranch::instantiateKernelCompiler(BuilderRef b) const {
    return llvm::make_unique<OptimizationBranchCompiler>(b, const_cast<OptimizationBranch *>(this));
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addKernelDeclarations(BuilderRef b) {
    mAllZeroKernel->addKernelDeclarations(b);
    mNonZeroKernel->addKernelDeclarations(b);
    Kernel::addKernelDeclarations(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::addInternalProperties(BuilderRef b) {
    COMPILER->addBranchProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasInternalStreamSets
 ** ------------------------------------------------------------------------------------------------------------- */
bool OptimizationBranch::allocatesInternalStreamSets() const {
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateSharedInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateAllocateSharedInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    COMPILER->generateAllocateSharedInternalStreamSetsMethod(b, expectedNumOfStrides);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateAllocateThreadLocalInternalStreamSetsMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateAllocateThreadLocalInternalStreamSetsMethod(BuilderRef b, Value * const expectedNumOfStrides) {
    COMPILER->generateAllocateThreadLocalInternalStreamSetsMethod(b, expectedNumOfStrides);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeMethod(BuilderRef b) {
    COMPILER->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateInitializeThreadLocalMethod(BuilderRef b) {
    COMPILER->generateInitializeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateDoSegmentMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateKernelMethod(BuilderRef b) {
    COMPILER->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeThreadLocalMethod(BuilderRef b) {
    COMPILER->generateFinalizeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void OptimizationBranch::generateFinalizeMethod(BuilderRef b) {
    COMPILER->generateFinalizeMethod(b);
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
{InternalScalar{b->getSizeTy(), ALL_ZERO_EXTERNAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), NON_ZERO_EXTERNAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), ALL_ZERO_INTERNAL_SEGMENT_NUMBER},
 InternalScalar{b->getSizeTy(), NON_ZERO_INTERNAL_SEGMENT_NUMBER},
 InternalScalar{ScalarType::ThreadLocal, b->getSizeTy()->getPointerTo(), SPAN_BUFFER},
 InternalScalar{ScalarType::ThreadLocal, b->getSizeTy(), SPAN_CAPACITY}})
, mCondition(condition)
, mNonZeroKernel(nonZeroKernel)
, mAllZeroKernel(allZeroKernel) {

}

}
