#include <kernels/pipeline_kernel.h>
#include <kernels/relationship.h>
#include "pipeline_compiler.hpp"
#include <llvm/IR/Function.h>

// NOTE: the pipeline kernel is primarily a proxy for the pipeline compiler. Ideally, by making some kernels
// a "family", the pipeline kernel will be compiled once for the lifetime of a program. Thus we can avoid even
// constructing any data structures for the pipeline in normal usage.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mCompiler = llvm::make_unique<PipelineCompiler>(b, this);
    mCompiler->addPipelineKernelProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> & args) {
    assert (args[0] && "cannot initialize before creation");
    assert (args[0]->getType()->getPointerElementType() == mKernelStateType);
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
 * @brief makeMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * PipelineKernel::addOrDeclareMainFunction(const std::unique_ptr<kernel::KernelBuilder> & b, const MainMethodGenerationType method) {

    b->setKernel(this);

    addKernelDeclarations(b);

    Module * const m = b->getModule();
    Function * const doSegment = getDoSegmentFunction(m);
    assert (doSegment->arg_size() >= 2);
    const auto numOfDoSegArgs = doSegment->arg_size() - 2;
    Function * const terminate = getTerminateFunction(m);

    // maintain consistency with the Kernel interface by passing first the stream sets
    // and then the scalars.
    std::vector<Type *> params;
    params.reserve(numOfDoSegArgs + getNumOfScalarInputs());

    // the first two params of doSegmentare its handle and numOfStrides; the remaining
    // are the stream set params
    auto doSegParam = doSegment->arg_begin(); ++doSegParam;
    const auto doSegEnd = doSegment->arg_end();
    while (++doSegParam != doSegEnd) {
        params.push_back(doSegParam->getType());
    }

    for (const auto & input : getInputScalarBindings()) {
        if (!input.hasAttribute(AttrId::Family)) {
            params.push_back(input.getType());
        }
    }

    const auto numOfInitArgs = params.size() - numOfDoSegArgs;

    // get the finalize method output type and set its return type as this function's return type
    FunctionType * const mainFunctionType = FunctionType::get(terminate->getReturnType(), params, false);

    const auto linkageType = (method == AddInternal) ? Function::InternalLinkage : Function::ExternalLinkage;

    Function * const main = Function::Create(mainFunctionType, linkageType, getName() + "_main", m);
    main->setCallingConv(CallingConv::C);

    if (method != DeclareExternal) {

        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", main));
        auto arg = main->arg_begin();

        // TODO: Even if a kernel is in a family, it may have a different kernel struct. Make sure this works here.
        Value * const handle = createInstance(b);
        setHandle(b, handle);

        std::vector<Value *> segmentArgs(doSegment->arg_size());
        segmentArgs[0] = handle;
        segmentArgs[1] = b->getSize(0);
        for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
            assert (arg != main->arg_end());
            segmentArgs[i + 2] = &*arg++;
        }

        std::vector<Value *> initArgs(numOfInitArgs + 1);
        initArgs[0] = handle;
        for (unsigned i = 0; i < numOfInitArgs; ++i) {
            assert (arg != main->arg_end());
            initArgs[i + 1] = &*arg++;
        }
        assert (arg == main->arg_end());
        // initialize the kernel
        initializeInstance(b, initArgs);
        // call the pipeline kernel
        b->CreateCall(doSegment, segmentArgs);
        // call and return the final output value(s)
        b->CreateRet(finalizeInstance(b, handle));
    }

    return main;
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
                               std::string && signature, const unsigned numOfThreads, const unsigned segmentIncrement,
                               Kernels && kernels, CallBindings && callBindings,
                               Bindings && stream_inputs, Bindings && stream_outputs,
                               Bindings && scalar_inputs, Bindings && scalar_outputs)
: Kernel(b, TypeId::Pipeline,
         "P" + std::to_string(numOfThreads) + "_" + std::to_string(segmentIncrement) + "_" + getStringHash(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs), {})
, mNumOfThreads(numOfThreads)
, mSegmentIncrement(segmentIncrement)
, mKernels(std::move(kernels))
, mCallBindings(std::move(callBindings))
, mSignature(std::move(signature)) {
    addAttribute(SynchronizationFree());
}


PipelineKernel::~PipelineKernel() {

}

}
