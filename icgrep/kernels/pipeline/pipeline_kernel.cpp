#include <kernels/pipeline_kernel.h>
#include <kernels/relationship.h>
#include "pipeline_compiler.hpp"
#include "pipeline_analysis.hpp"
#include "buffer_management_logic.hpp"
#include "consumer_logic.hpp"
#include "core_logic.hpp"
#include "kernel_logic.hpp"
#include "cycle_counter_logic.hpp"
#include "popcount_logic.hpp"
#include "pipeline_logic.hpp"
#include <llvm/IR/Function.h>

// NOTE: the pipeline kernel is primarily a proxy for the pipeline compiler. Ideally, by making some kernels
// a "family", the pipeline kernel will be compiled once for the lifetime of a program. Thus we can avoid even
// constructing any data structures for the pipeline in normal usage.

namespace kernel {

#warning make sure all virtual methods are proxied for when only one kernel exists in the pipeline

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        mKernels[0]->addInternalKernelProperties(b);
    } else { // add handles for each of unique streams
        mCompiler = llvm::make_unique<PipelineCompiler>(b, this);
        mCompiler->addHandlesToPipelineKernel(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief initializeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::initializeInstance(const std::unique_ptr<KernelBuilder> & b, std::vector<Value *> & args) {

    if (LLVM_UNLIKELY(isProxy())) {
        mKernels[0]->initializeInstance(b, args);
    } else {
        assert (args[0] && "cannot initialize before creation");
        assert (args[0]->getType()->getPointerElementType() == mKernelStateType);
        b->setKernel(this);

        // append the kernel pointers for any kernel belonging to a family
        Module * const m = b->getModule();
        for (auto & kernel : mKernels) {
            if (kernel->hasFamilyName()) {
                kernel->addKernelDeclarations(b);
                Value * const handle = kernel->createInstance(b);
                PointerType * const voidPtrTy = b->getVoidPtrTy();
                args.push_back(b->CreatePointerCast(handle, voidPtrTy));
                args.push_back(b->CreatePointerCast(kernel->getInitFunction(m), voidPtrTy));
                args.push_back(b->CreatePointerCast(kernel->getDoSegmentFunction(m), voidPtrTy));
                args.push_back(b->CreatePointerCast(kernel->getTerminateFunction(m), voidPtrTy));
            }
        }

        Value * const init = getInitFunction(m);
        assert (cast<FunctionType>(init->getType()->getPointerElementType())->getNumParams() == args.size());
        b->CreateCall(init, args);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateInitializeMethod(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        mKernels[0]->generateInitializeMethod(b);
    } else {
        // TODO: this isn't sufficient for composable PipelineKernel objects since would want to
        // allocate memory once during initialization but have the buffer/kernel struct visible in
        // the main kernel logic. This can be solved by heap allocating all structs or somehow
        // passing the structs via the function call but only reentrant pipelines require this
        // to maintain state.
        mCompiler->generateInitializeMethod(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        mKernels[0]->generateKernelMethod(b);
    } else {
        if (mNumOfThreads == 1) {
            mCompiler->generateSingleThreadKernelMethod(b);
        } else {
            mCompiler->generateMultiThreadKernelMethod(b, mNumOfThreads);
        }        
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief finalizeInstance
 ** ------------------------------------------------------------------------------------------------------------- */
Value * PipelineKernel::finalizeInstance(const std::unique_ptr<KernelBuilder> & b) {
    assert (mHandle && "was not set");
    if (LLVM_UNLIKELY(isProxy())) {
        return mKernels[0]->finalizeInstance(b);
    } else {
        Value * result = b->CreateCall(getTerminateFunction(b->getModule()), { mHandle });
        mHandle = nullptr;
        if (mOutputScalars.empty()) {
            assert (!result || result->getType()->isVoidTy());
            result = nullptr;
        }
        return result;
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        mKernels[0]->generateFinalizeMethod(b);
    } else {
        mCompiler->generateFinalizeMethod(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineKernel::getFinalOutputScalars(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        return mKernels[0]->getFinalOutputScalars(b);
    } else {
        return mCompiler->getFinalOutputScalars(b);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkExternalMethods
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::linkExternalMethods(const std::unique_ptr<KernelBuilder> & b) {
    if (LLVM_UNLIKELY(isProxy())) {
        return mKernels[0]->linkExternalMethods(b);
    } else {
        for (const auto & k : mKernels) {
            k->linkExternalMethods(b);
        }
        for (CallBinding & call : mCallBindings) {
            call.Callee = b->LinkFunction(call.Name, call.Type, call.FunctionPointer);
        }
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

    // maintain consistency with the Kernel interface
    const auto numOfDoSegArgs = (getNumOfStreamInputs() + getNumOfStreamOutputs()) * 2;
    std::vector<Type *> paramTypes;
    paramTypes.reserve(numOfDoSegArgs + getNumOfScalarInputs());
    for (const auto & input : getInputStreamSetBuffers()) {
        paramTypes.push_back(input->getType()->getPointerTo());
        paramTypes.push_back(b->getSizeTy());
    }
    for (const auto & output : getOutputStreamSetBuffers()) {
        paramTypes.push_back(output->getType()->getPointerTo());
        paramTypes.push_back(b->getSizeTy());
    }
    for (const auto & input : getInputScalarBindings()) {
        if (LLVM_LIKELY(!input.hasAttribute(AttrId::Family))) {
            paramTypes.push_back(input.getType());
        }
    }
    const auto numOfInitArgs = (paramTypes.size() - numOfDoSegArgs);

    // get the finalize method output type and set its return type as this function's return type
    Module * const m = b->getModule();
    Function * const tf = getTerminateFunction(m);
    FunctionType * const mainFunctionType = FunctionType::get(tf->getReturnType(), paramTypes, false);

    auto linkageType = (method == AddInternal) ? Function::InternalLinkage : Function::ExternalLinkage;

    Function * const main = Function::Create(mainFunctionType, linkageType, getName() + "_main", m);
    main->setCallingConv(CallingConv::C);

    if (method != DeclareExternal) {

        b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", main));
        auto arg = main->arg_begin();

        // TODO: Even if a kernel is in a family, it may have a different kernel struct. Make sure this works here.
        Value * const handle = createInstance(b);
        setHandle(b, handle);

        std::vector<Value *> segmentArgs;
        segmentArgs.reserve(numOfDoSegArgs + 2);
        segmentArgs.push_back(handle);
        segmentArgs.push_back(b->getSize(0));
        for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
            segmentArgs.push_back(&*arg++);
        }

        std::vector<Value *> initArgs;
        initArgs.reserve(numOfInitArgs + 1);
        initArgs.push_back(handle);
        for (unsigned i = 0; i < numOfInitArgs; ++i) {
            initArgs.push_back(&*arg++);
        }
        assert (arg == main->arg_end());

        // initialize the kernel
        initializeInstance(b, initArgs);

        // call the pipeline kernel
        Function * const doSegment = getDoSegmentFunction(m);
        assert (doSegment->getFunctionType()->getNumParams() == segmentArgs.size());
        b->CreateCall(doSegment, segmentArgs);

        // call and return the final output value(s)
        b->CreateRet(finalizeInstance(b));
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
    if (LLVM_UNLIKELY(isProxy())) {
        return mKernels[0]->getName();
    }
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
 * @brief makeKernelName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string PipelineKernel::makeKernelName(const Kernel * const kernel, const unsigned kernelIndex) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << '@';
    out << kernel->getName();
    out << '.';
    out << kernelIndex;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief makeBufferName
 ** ------------------------------------------------------------------------------------------------------------- */
std::string PipelineKernel::makeBufferName(const Kernel * const kernel, const unsigned kernelIndex, const Binding & binding) {
    std::string tmp;
    raw_string_ostream out(tmp);
    out << '@';
    out << kernel->getName();
    out << '_';
    out << binding.getName();
    out << '.';
    out << kernelIndex;
    out.flush();
    return tmp;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineKernel::PipelineKernel(std::string && signature, const unsigned numOfThreads,
                               Kernels && kernels, CallBindings && callBindings,
                               Bindings && stream_inputs, Bindings && stream_outputs,
                               Bindings &&scalar_inputs, Bindings && scalar_outputs)
: Kernel("p" + std::to_string(numOfThreads) + "_" + getStringHash(signature),
         std::move(stream_inputs), std::move(stream_outputs),
         std::move(scalar_inputs), std::move(scalar_outputs), {})
, mNumOfThreads(numOfThreads)
, mKernels(std::move(kernels))
, mCallBindings(std::move(callBindings))
, mSignature(std::move(signature)) {

}


PipelineKernel::~PipelineKernel() {

}

}