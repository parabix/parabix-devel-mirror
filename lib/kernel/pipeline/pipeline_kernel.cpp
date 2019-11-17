#include <kernel/pipeline/pipeline_kernel.h>
#include "compiler/pipeline_compiler.hpp"
#include <llvm/IR/Function.h>


// NOTE: the pipeline kernel is primarily a proxy for the pipeline compiler. Ideally, by making some kernels
// a "family", the pipeline kernel will be compiled once for the lifetime of a program. Thus we can avoid even
// constructing any data structures for the pipeline in normal usage.

namespace kernel {

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addInternalKernelProperties
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addInternalProperties(BuilderRef b) {
    mCompiler = llvm::make_unique<PipelineCompiler>(b, this);
    mCompiler->generateImplicitKernels(b);
    mCompiler->addPipelineKernelProperties(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addKernelDeclarations
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addKernelDeclarations(BuilderRef b) {
    for (Kernel * kernel : mKernels) {
        kernel->addKernelDeclarations(b);
    }
    for (CallBinding & call : mCallBindings) {
        call.Callee = b->LinkFunction(call.Name, call.Type, call.FunctionPointer);
    }
    Kernel::addKernelDeclarations(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateInitializeMethod(BuilderRef b) {
    mCompiler->generateInitializeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateInitializeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateInitializeThreadLocalMethod(BuilderRef b) {
    mCompiler->generateInitializeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateKernelMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateKernelMethod(BuilderRef b) {
    mCompiler->generateKernelMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateFinalizeMethod(BuilderRef b) {
    mCompiler->generateFinalizeMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief generateFinalizeThreadLocalMethod
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::generateFinalizeThreadLocalMethod(BuilderRef b) {
    mCompiler->generateFinalizeThreadLocalMethod(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFinalOutputScalars
 ** ------------------------------------------------------------------------------------------------------------- */
std::vector<Value *> PipelineKernel::getFinalOutputScalars(BuilderRef b) {
    return mCompiler->getFinalOutputScalars(b);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief linkExternalMethods
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::linkExternalMethods(BuilderRef b) {
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
void PipelineKernel::addAdditionalFunctions(BuilderRef b) {
    if (hasStaticMain()) {
        addOrDeclareMainFunction(b, AddExternal);
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief containsKernelFamilies
 ** ------------------------------------------------------------------------------------------------------------- */
bool PipelineKernel::containsKernelFamilies() const {
    for (Kernel * k : mKernels) {
        if (k->hasFamilyName()) {
            return true;
        }
        assert (!k->containsKernelFamilies());
    }
    return false;
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

#define JOIN3(X,Y,Z) BOOST_JOIN(X,BOOST_JOIN(Y,Z))

#define REPLACE_INTERNAL_KERNEL_BINDINGS(BindingType) \
    const auto * const from = JOIN3(m, BindingType, s)[i].getRelationship(); \
    for (auto * K : mKernels) { \
        const auto & B = K->JOIN3(get, BindingType, Bindings)(); \
        for (unsigned j = 0; j < B.size(); ++j) { \
            if (LLVM_UNLIKELY(B[j].getRelationship() == from)) { \
                K->JOIN3(set, BindingType, At)(j, value); } } } \
    JOIN3(m, BindingType, s)[i].setRelationship(value);

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setInputStreamSetAt(const unsigned i, StreamSet * const value) {
    REPLACE_INTERNAL_KERNEL_BINDINGS(InputStreamSet);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputStreamSetAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setOutputStreamSetAt(const unsigned i, StreamSet * const value) {
    REPLACE_INTERNAL_KERNEL_BINDINGS(OutputStreamSet);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setInputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setInputScalarAt(const unsigned i, Scalar * const value) {
    REPLACE_INTERNAL_KERNEL_BINDINGS(InputScalar);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief setOutputScalarAt
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::setOutputScalarAt(const unsigned i, Scalar * const value) {
    REPLACE_INTERNAL_KERNEL_BINDINGS(OutputScalar);
}

#undef JOIN3
#undef REPLACE_INTERNAL_KERNEL_BINDINGS

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addOrDeclareMainFunction
 ** ------------------------------------------------------------------------------------------------------------- */
Function * PipelineKernel::addOrDeclareMainFunction(BuilderRef b, const MainMethodGenerationType method) {

    b->setKernel(this);

    addKernelDeclarations(b);

    unsigned suppliedArgs = 1;
    if (LLVM_LIKELY(isStateful())) {
        ++suppliedArgs;
    }
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        ++suppliedArgs;
    }

    Module * const m = b->getModule();
    Function * const doSegment = getDoSegmentFunction(b);
    assert (doSegment->arg_size() >= suppliedArgs);
    const auto numOfDoSegArgs = doSegment->arg_size() - suppliedArgs;
    Function * const terminate = getFinalizeFunction(b);

    // maintain consistency with the Kernel interface by passing first the stream sets
    // and then the scalars.
    SmallVector<Type *, 32> params;
    params.reserve(numOfDoSegArgs + getNumOfScalarInputs());

    // The initial params of doSegment are its shared handle, thread-local handle and numOfStrides.
    // (assuming the kernel has both handles). The remaining are the stream set params
    auto doSegParam = doSegment->arg_begin();
    std::advance(doSegParam, suppliedArgs);
    const auto doSegEnd = doSegment->arg_end();
    while (doSegParam != doSegEnd) {
        params.push_back(doSegParam->getType());
        std::advance(doSegParam, 1);
    }
    for (const auto & input : getInputScalarBindings()) {
        params.push_back(input.getType());
    }

    // get the finalize method output type and set its return type as this function's return type
    FunctionType * const mainFunctionType = FunctionType::get(terminate->getReturnType(), params, false);

    const auto linkageType = (method == AddInternal) ? Function::InternalLinkage : Function::ExternalLinkage;

    Function * const main = Function::Create(mainFunctionType, linkageType, getName() + "_main", m);
    main->setCallingConv(CallingConv::C);

    // declaration only; exit
    if (method == DeclareExternal) {
        return main;
    }

    if (LLVM_UNLIKELY(hasAttribute(AttrId::InternallySynchronized))) {
        report_fatal_error("main cannot be externally synchronized");
    }

    b->SetInsertPoint(BasicBlock::Create(b->getContext(), "entry", main));
    auto arg = main->arg_begin();
    auto nextArg = [&]() {
        assert (arg != main->arg_end());
        Value * const v = &*arg;
        std::advance(arg, 1);
        return v;
    };

    SmallVector<Value *, 16> segmentArgs(doSegment->arg_size());
    segmentArgs[suppliedArgs - 1] = b->getSize(0);  // numOfStrides -> isFinal = True
    for (unsigned i = 0; i < numOfDoSegArgs; ++i) {
        segmentArgs[suppliedArgs + i] = nextArg();
    }
    ParamMap paramMap;
    for (const auto & input : getInputScalarBindings()) {
        paramMap.insert(std::make_pair(cast<Scalar>(input.getRelationship()), nextArg()));
    }
    assert (arg == main->arg_end());

    Value * sharedHandle = nullptr;
    {
        InitArgs args;
        sharedHandle = constructFamilyKernels(b, args, paramMap);
    }
    Value * threadLocalHandle = nullptr;
    if (LLVM_UNLIKELY(hasThreadLocal())) {
        threadLocalHandle = initializeThreadLocalInstance(b, sharedHandle);
    }
    if (LLVM_LIKELY(isStateful())) {
        segmentArgs[0] = sharedHandle;
    }
    if (hasThreadLocal()) {
        segmentArgs[suppliedArgs - 2] = threadLocalHandle;
    }

    #ifdef NDEBUG
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
    #endif
        BasicBlock * const handleCatch = b->CreateBasicBlock("");
        BasicBlock * const handleDeallocation = b->CreateBasicBlock("");

        IntegerType * const int32Ty = b->getInt32Ty();
        PointerType * const int8PtrTy = b->getInt8PtrTy();
        LLVMContext & C = b->getContext();
        StructType * const caughtResultType = StructType::get(C, { int8PtrTy, int32Ty });
        Function * const personalityFn = b->getDefaultPersonalityFunction();
        main->setPersonalityFn(personalityFn);

        b->CreateInvoke(doSegment, handleDeallocation, handleCatch, segmentArgs);

        b->SetInsertPoint(handleCatch);
        LandingPadInst * const caughtResult = b->CreateLandingPad(caughtResultType, 0);
        caughtResult->addClause(ConstantPointerNull::get(int8PtrTy));
        b->CreateCall(b->getBeginCatch(), {b->CreateExtractValue(caughtResult, 0)});
        b->CreateCall(b->getEndCatch());
        b->CreateBr(handleDeallocation);

        b->SetInsertPoint(handleDeallocation);
    #ifdef NDEBUG
    } else {
        b->CreateCall(doSegment, segmentArgs);
    }
    #endif
    if (hasThreadLocal()) {
        SmallVector<Value *, 2> args;
        if (LLVM_LIKELY(isStateful())) {
            args.push_back(sharedHandle);
        }
        args.push_back(threadLocalHandle);
        finalizeThreadLocalInstance(b, args);
    }
    if (isStateful()) {
        // call and return the final output value(s)
        b->CreateRet(finalizeInstance(b, sharedHandle));
    }
    return main;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief addFamilyInitializationArgTypes
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::addFamilyInitializationArgTypes(BuilderRef b, InitArgTypes & argTypes) const {
    const auto n = mKernels.size();
    for (unsigned i = 0; i != n; ++i) {
        const Kernel * const kernel = mKernels[i];
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            PipelineCompiler::addFamilyInitializationArgTypes(b, kernel, argTypes);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief bindFamilyInitializationArguments
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::bindFamilyInitializationArguments(BuilderRef b, ArgIterator & arg, const ArgIterator & arg_end) const {
    mCompiler->bindFamilyInitializationArguments(b, arg, arg_end);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief recursivelyConstructFamilyKernels
 ** ------------------------------------------------------------------------------------------------------------- */
void PipelineKernel::recursivelyConstructFamilyKernels(BuilderRef b, InitArgs & args, const ParamMap & params) const {
    for (const Kernel * const kernel : mKernels) {
        if (LLVM_UNLIKELY(kernel->hasFamilyName())) {
            kernel->constructFamilyKernels(b, args, params);
        } else if (LLVM_UNLIKELY(kernel->containsKernelFamilies())) {
            InitArgs nestedArgs;
            kernel->constructFamilyKernels(b, nestedArgs, params);
        }
    }
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief constructor
 ** ------------------------------------------------------------------------------------------------------------- */
PipelineKernel::PipelineKernel(BaseDriver & driver,
                               std::string && signature, const unsigned numOfThreads,
                               Kernels && kernels, CallBindings && callBindings,
                               Bindings && stream_inputs, Bindings && stream_outputs,
                               Bindings && scalar_inputs, Bindings && scalar_outputs)
: Kernel(driver.getBuilder(), TypeId::Pipeline,
         [] (const std::string & signature, const unsigned numOfThreads) {
             std::string tmp;
             tmp.reserve(32);
             const auto bufferSegments = std::max(numOfThreads > 1 ? numOfThreads + 1 : 1, codegen::BufferSegments);
             llvm::raw_string_ostream name(tmp);
             name << 'P' << numOfThreads
                  << 'B' << bufferSegments
                  << '_' << Kernel::getStringHash(signature);
             name.flush();
             return tmp;
         } (signature, numOfThreads),
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
