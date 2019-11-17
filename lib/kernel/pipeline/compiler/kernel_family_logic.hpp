#ifndef KERNEL_FAMILY_LOGIC_HPP
#define KERNEL_FAMILY_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyInitializationArgTypes
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addFamilyInitializationArgTypes(BuilderRef b, const Kernel * const kernel, InitArgTypes & argTypes) {
    if (LLVM_LIKELY(kernel->isStateful())) {
        argTypes.push_back(kernel->getSharedStateType()->getPointerTo());
    }
    if (kernel->hasThreadLocal()) {
        argTypes.push_back(kernel->getInitializeThreadLocalFunction(b)->getType());
    }
    argTypes.push_back(kernel->getDoSegmentFunction(b)->getType());
    if (kernel->hasThreadLocal()) {
        argTypes.push_back(kernel->getFinalizeThreadLocalFunction(b)->getType());
    }
    argTypes.push_back(kernel->getFinalizeFunction(b)->getType());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addFamilyKernelProperties(BuilderRef b, const unsigned index) const {
    const Kernel * const kernel = getKernel(index);
    if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
        const auto prefix = makeKernelName(index);
        if (LLVM_LIKELY(kernel->isStateful())) {
            auto sharedTy = kernel->getSharedStateType()->getPointerTo(); assert (sharedTy);
            mPipelineKernel->addInternalScalar(sharedTy, prefix);
        }
        if (kernel->hasThreadLocal()) {
            auto initThreadFn = kernel->getInitializeThreadLocalFunction(b); assert(initThreadFn);
            auto initThreadTy = initThreadFn->getType(); assert (initThreadTy);
            mPipelineKernel->addInternalScalar(initThreadTy, prefix + INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
        }
        auto segmentFn = kernel->getDoSegmentFunction(b); assert(segmentFn);
        Type * const segmentTy = segmentFn->getType(); assert(segmentTy);
        mPipelineKernel->addInternalScalar(segmentTy, prefix + DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
        if (kernel->hasThreadLocal()) {
            auto finalThreadFn = kernel->getFinalizeThreadLocalFunction(b); assert(finalThreadFn);
            auto finalThreadTy = finalThreadFn->getType(); assert(finalThreadTy);
            mPipelineKernel->addInternalScalar(finalThreadTy, prefix + FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
        }
        auto finalFn = kernel->getFinalizeFunction(b); assert (finalFn);
        auto finalTy = finalFn->getType(); assert (finalTy);
        mPipelineKernel->addInternalScalar(finalTy, prefix + FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelineKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::bindFamilyInitializationArguments(BuilderRef b, ArgIterator & arg, const ArgIterator & arg_end) const {

    auto nextArg = [&]() {
        assert (arg != arg_end);
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };

    b->setKernel(mPipelineKernel);

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            const auto prefix = makeKernelName(i);
            if (LLVM_LIKELY(kernel->isStateful())) {
                b->setScalarField(prefix, nextArg());
            }
            if (kernel->hasThreadLocal()) {
                b->setScalarField(prefix + INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX, nextArg());
            }
            b->setScalarField(prefix + DO_SEGMENT_FUNCTION_POINTER_SUFFIX, nextArg());
            if (kernel->hasThreadLocal()) {
                b->setScalarField(prefix + FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX, nextArg());
            }
            b->setScalarField(prefix + FINALIZE_FUNCTION_POINTER_SUFFIX, nextArg());
        }
    }

}

}

#endif
