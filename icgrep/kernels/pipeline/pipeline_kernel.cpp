#include <kernels/pipeline_kernel.h>
#include "pipeline_compiler.hpp"
#include <llvm/IR/Function.h>

// NOTE: the pipeline kernel is primarily a proxy for the pipeline compiler. Ideally, by making some kernels
// a "family", the pipeline kernel will be compiled once for the lifetime of a program. Thus we can avoid even
// constructing any data structures for the pipeline in normal usage.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addInternalProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCompiler = llvm::make_unique<PipelineCompiler>(b, this);
    mCompiler->addPipelineKernelProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> & args) {
    assert (args[0] && "cannot initialize before creation");
    assert (args[0]->getType()->getPointerElementType() == mSharedStateType);
    b->setKernel(this);

    // append the kernel pointers for any kernel belonging to a family
    Module * const m = b->getModule();
    for (Kernel * kernel : mKernels) {
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            PointerType * const voidPtrTy = b->getVoidPtrTy();
            if (LLVM_LIKELY(kernel->isStateful())) {
                Value * const handle = kernel->createInstance(b);
                args.push_back(b->CreatePointerCast(handle, voidPtrTy));
            }
            args.push_back(b->CreatePointerCast(kernel->getInitFunction(m), voidPtrTy));
            args.push_back(b->CreatePointerCast(kernel->getDoSegmentFunction(m), voidPtrTy));
            args.push_back(b->CreatePointerCast(kernel->getTerminateFunction(m), voidPtrTy));
        }
    }

    b->CreateCall(getInitFunction(m), args);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addKernelDeclarations(const std::unique_ptr<KernelBuilder> & b) {
    for (Kernel * kernel : mKernels) {
        kernel->addKernelDeclarations(b);
    }
    Kernel::addKernelDeclarations(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    mCompiler->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineKernel::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    return mCompiler->getFinalOutputScalars(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkExternalMethods
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) {
    for (const auto & k : mKernels) {
        k->linkExternalMethods(b);
    }
    for (CallBinding & call : mCallBindings) {
        call.Callee = b->LinkFunction(call.Name, call.Type, call.FunctionPointer);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addAdditionalFunctions
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addAdditionalFunctions(const std::unique_ptr<KernelBuilder> & b) {
    if (hasStaticMain()) {
        addOrDeclareMainFunction(b, AddExternal);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief hasStaticMain
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineKernel::hasStaticMain() const {
    for (Kernel * k : mKernels) {
        if (k->hasFamilyName()) {
            return false;
        }
    }
    return true;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getName
 *
 * The name of the pipeline kernel the list of family names of each owned kernel followed by the description of the
 * relationships between each.
 ** ------------------------------------------------------------------------------------------------------------- */
const std::string PipelineKernel::getName() const {
    return mKernelName;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setInputStreamSetAt(const unsigned i, StreamSet * const value) {
    mInputStreamSets[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setOutputStreamSetAt(const unsigned i, StreamSet * const value) {
    mOutputStreamSets[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setInputScalarAt(const unsigned i, Scalar * const value) {
    mInputScalars[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setOutputScalarAt(const unsigned i, Scalar * const value) {
    mOutputScalars[i].setRelationship(value);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineKernel::PipelineKernel(const std::unique_ptr<KernelBuilder> & b,
                               std::string && signature, const unsigned numOfThreads,
                               Kernels && kernels, CallBindings && callBindings,
                               Bindings && stream_inputs, Bindings && stream_outputs,
                               Bindings && scalar_inputs, Bindings && scalar_outputs)
: Kernel(b, TypeId::Pipeline,
         "P" + std::to_string(numOfThreads) + "_" + getStringHash(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs), {})
, mNumOfThreads(numOfThreads)
, mKernels(std::move(kernels))
, mCallBindings(std::move(callBindings))
, mSignature(std::move(signature)) {
    addAttribute(SynchronizationFree());
}


PipelineKernel::~PipelineKernel() {

}

}
