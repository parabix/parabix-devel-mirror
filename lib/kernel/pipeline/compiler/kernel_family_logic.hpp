#ifndef KERNEL_FAMILY_LOGIC_HPP
#define KERNEL_FAMILY_LOGIC_HPP

#include "pipeline_compiler.hpp"

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::addFamilyKernelProperties(BuilderRef b, const unsigned index) const {
    const Kernel * const kernel = getKernel(index);
    if (LLVM_LIKELY(kernel->hasFamilyName())) {
        PointerType * const voidPtrTy = b->getVoidPtrTy();
        const auto prefix = makeKernelName(index);
        const auto tl = kernel->hasThreadLocal();
        const auto ai = kernel->allocatesInternalStreamSets();
        if (ai) {
            mTarget->addInternalScalar(voidPtrTy, prefix + ALLOCATE_SHARED_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX, index);
        }
        if (tl) {
            mTarget->addInternalScalar(voidPtrTy, prefix + INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX, index);
            if (ai) {
                mTarget->addInternalScalar(voidPtrTy, prefix + ALLOCATE_THREAD_LOCAL_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX, index);
            }
        }
        mTarget->addInternalScalar(voidPtrTy, prefix + DO_SEGMENT_FUNCTION_POINTER_SUFFIX, index);
        if (tl) {
            mTarget->addInternalScalar(voidPtrTy, prefix + FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX, index);
        }
        mTarget->addInternalScalar(voidPtrTy, prefix + FINALIZE_FUNCTION_POINTER_SUFFIX, index);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addPipelineKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineCompiler::bindFamilyInitializationArguments(BuilderRef b, ArgIterator & arg, const ArgIterator & arg_end) const {

    PointerType * const voidPtrTy = b->getVoidPtrTy();

    auto nextArg = [&]() {
        assert (arg != arg_end);
        Value * const v = b->CreatePointerCast(&*arg, voidPtrTy);
        std::advance(arg, 1);
        return v;
    };

    for (unsigned i = FirstKernel; i <= LastKernel; ++i) {
        const Kernel * const kernel = getKernel(i);
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {
            const auto prefix = makeKernelName(i);

            auto readNextScalar = [&](const StringRef name) {
                auto ptr = getScalarFieldPtr(b.get(), name); assert (ptr);
                Value * value = nextArg();

                if (LLVM_UNLIKELY(CheckAssertions)) {
                    b->CreateAssert(value, "family parameter (%s) was given a null value", b->GetString(name));
                }
                b->CreateStore(value, ptr);
            };

            if (LLVM_LIKELY(kernel->isStateful())) {
                readNextScalar(prefix);
            }
            if (LLVM_LIKELY(kernel->hasFamilyName())) {
                const auto tl = kernel->hasThreadLocal();
                const auto ai = kernel->allocatesInternalStreamSets();
                if (ai) {
                    readNextScalar(prefix + ALLOCATE_SHARED_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
                }
                if (tl) {
                    readNextScalar(prefix + INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
                    if (ai) {
                        readNextScalar(prefix + ALLOCATE_THREAD_LOCAL_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
                    }
                }
                readNextScalar(prefix + DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
                if (tl) {
                    readNextScalar(prefix + FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
                }
                readNextScalar(prefix + FINALIZE_FUNCTION_POINTER_SUFFIX);
            }
        }
    }

}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFamilyFunctionFromKernelState
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getFamilyFunctionFromKernelState(BuilderRef b, Type * const type, const std::string & suffix) const {
    const auto prefix = makeKernelName(mKernelId);
    Value * const funcPtr = b->getScalarField(prefix + suffix);
    assert (funcPtr->getType() == b->getVoidPtrTy());
    if (LLVM_UNLIKELY(CheckAssertions)) {
        b->CreateAssert(funcPtr, prefix + suffix + " is null");
    }
    return b->CreatePointerCast(funcPtr, type);
}

}

#endif
