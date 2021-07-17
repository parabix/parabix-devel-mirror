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
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };

    // To avoid the expense of instantiating a pipeline compiler when generating the
    // "main" function for the pipeline on a subsequent run, family args are passed in
    // order of the original kernels in the pipeline; however, the order of these kernels
    // are actually executed may be shuffled around in the pipeline itself. So to map the
    // original order to the pipeline order, we just search for a matching kernel object.

    for (const Kernel * const kernel : cast<PipelineKernel>(mTarget)->getKernels()) {
        if (LLVM_UNLIKELY(kernel->externallyInitialized())) {

            auto kernelId = PipelineInput;

            for (auto i = FirstKernel; i <= LastKernel; ++i) {
                if (kernel == getKernel(i)) {
                    kernelId = i;
                    break;
                }
            }

            if (LLVM_UNLIKELY(kernelId < FirstKernel)) {

                #ifndef NDEBUG
                SmallVector<char, 256> tmp;
                raw_svector_ostream out(tmp);
                out << "Warning: family kernel ";
                out << kernel->getName();
                out << " was removed from the pipeline.\n";
                errs() << out.str();
                #endif

                if (LLVM_LIKELY(kernel->isStateful())) {
                    nextArg();
                }
                if (LLVM_LIKELY(kernel->hasFamilyName())) {
                    const auto tl = kernel->hasThreadLocal();
                    const auto ai = kernel->allocatesInternalStreamSets();
                    if (ai) {
                        nextArg();
                    }
                    if (tl) {
                        nextArg();
                        if (ai) {
                            nextArg();
                        }
                    }
                    nextArg();
                    if (tl) {
                        nextArg();
                    }
                    nextArg();
                }

            } else {

                // get the internal prefix for this kernel.
                const auto prefix = makeKernelName(kernelId);

                auto readNextScalar = [&](const StringRef name) {
                    auto ptr = getScalarFieldPtr(b.get(), name); assert (ptr);
                    Value * value = b->CreatePointerCast(nextArg(), voidPtrTy);
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

}


/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::callKernelInitializeFunction(BuilderRef b, const ArgVec & args) const {
    Function * const init = mKernel->getInitializeFunction(b);
    assert (!mKernel->hasFamilyName());
    return b->CreateCall(init->getFunctionType(), init, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::callKernelInitializeThreadLocalFunction(BuilderRef b, Value * handle) const {
    Function * const init = mKernel->getInitializeThreadLocalFunction(b);
    Value * func = init;
    if (mKernel->hasFamilyName()) {
        func = getFamilyFunctionFromKernelState(b, init->getType(), INITIALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    FixedArray<Value *, 1> args;
    args[0] = handle;
    return b->CreateCall(init->getFunctionType(), func, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelAllocateSharedInternalStreamSetsFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelAllocateSharedInternalStreamSetsFunction(BuilderRef b) const {
    Function * const term = mKernel->getAllocateSharedInternalStreamSetsFunction(b, false);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, term->getType(), ALLOCATE_SHARED_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getKernelAllocateThreadLocalInternalStreamSetsFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelAllocateThreadLocalInternalStreamSetsFunction(BuilderRef b) const {
    Function * const term = mKernel->getAllocateThreadLocalInternalStreamSetsFunction(b, false);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, term->getType(), ALLOCATE_THREAD_LOCAL_INTERNAL_STREAMSETS_FUNCTION_POINTER_SUFFIX);
    }
    return term;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getDoSegmentFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::getKernelDoSegmentFunction(BuilderRef b) const {
    Function * const doSegment = mKernel->getDoSegmentFunction(b);
    if (mKernel->hasFamilyName()) {
        return getFamilyFunctionFromKernelState(b, doSegment->getType(), DO_SEGMENT_FUNCTION_POINTER_SUFFIX);
    }
    return doSegment;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getInitializationThreadLocalFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::callKernelFinalizeThreadLocalFunction(BuilderRef b, const SmallVector<Value *, 2> & args) const {
    Function * const finalize = mKernel->getFinalizeThreadLocalFunction(b);
    Value * func = finalize;
    if (mKernel->hasFamilyName()) {
        func = getFamilyFunctionFromKernelState(b, finalize->getType(), FINALIZE_THREAD_LOCAL_FUNCTION_POINTER_SUFFIX);
    }
    return b->CreateCall(finalize->getFunctionType(), func, args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalizeFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineCompiler::callKernelFinalizeFunction(BuilderRef b, const SmallVector<Value *, 1> & args) const {
    Function * const term = mKernel->getFinalizeFunction(b);
    Value * func = term;
    if (mKernel->hasFamilyName()) {
        func = getFamilyFunctionFromKernelState(b, term->getType(), FINALIZE_FUNCTION_POINTER_SUFFIX);
    }
    return b->CreateCall(term->getFunctionType(), func, args);
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
