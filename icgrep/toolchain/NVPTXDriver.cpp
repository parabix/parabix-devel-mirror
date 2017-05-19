/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */

#include "NVPTXDriver.h"
#include <IR_Gen/idisa_target.h>
#include <kernels/kernel_builder.h>
#include <toolchain/toolchain.h>
#include <IR_Gen/llvm2ptx.h>
#include <llvm/Transforms/Scalar.h>


using namespace llvm;
using namespace parabix;

using Kernel = kernel::Kernel;
using KernelBuilder = kernel::KernelBuilder;


NVPTXDriver::NVPTXDriver(std::string && moduleName)
: mContext(new llvm::LLVMContext())
, mMainModule(new Module(moduleName, *mContext))
, iBuilder(nullptr)
, mTarget(nullptr)
, mEngine(nullptr) {

    InitializeAllTargets();
    InitializeAllTargetMCs();
    InitializeAllAsmPrinters();
    InitializeAllAsmParsers();

    PassRegistry *Registry = PassRegistry::getPassRegistry();
    initializeCore(*Registry);
    initializeCodeGen(*Registry);
    initializeLoopStrengthReducePass(*Registry);
    initializeLowerIntrinsicsPass(*Registry);
    initializeUnreachableBlockElimPass(*Registry);

    mMainModule->setDataLayout("e-p:64:64:64-i1:8:8-i8:8:8-i16:16:16-i32:32:32-i64:64:64-f32:32:32-f64:64:64-v16:16:16-v32:32:32-v64:64:64-v128:128:128-n16:32:64");
    mMainModule->setTargetTriple("nvptx64-nvidia-cuda");
    codegen::BlockSize = 64;

    iBuilder.reset(IDISA::GetIDISA_GPU_Builder(*mContext));
    iBuilder->setModule(mMainModule);
    iBuilder->CreateBaseFunctions();
}

ExternalBuffer * NVPTXDriver::addExternalBuffer(std::unique_ptr<ExternalBuffer> b) {
    mOwnedBuffers.emplace_back(std::move(b));
    return cast<ExternalBuffer>(mOwnedBuffers.back().get());
}

StreamSetBuffer * NVPTXDriver::addBuffer(std::unique_ptr<StreamSetBuffer> b) {
    b->allocateBuffer(iBuilder);
    mOwnedBuffers.emplace_back(std::move(b));
    return mOwnedBuffers.back().get();
}

kernel::Kernel * NVPTXDriver::addKernelInstance(std::unique_ptr<kernel::Kernel> kb) {
    mOwnedKernels.emplace_back(std::move(kb));
    return mOwnedKernels.back().get();
}

void NVPTXDriver::addKernelCall(Kernel & kb, const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    assert ("addKernelCall or makeKernelCall was already run on this kernel." && (kb.getModule() == nullptr));
    mPipeline.emplace_back(&kb);
    kb.bindPorts(inputs, outputs);
    kb.setModule(iBuilder, mMainModule);
}

void NVPTXDriver::makeKernelCall(Kernel * kb, const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs) {
    assert ("addKernelCall or makeKernelCall was already run on this kernel." && (kb->getModule() == nullptr));
    mPipeline.emplace_back(kb);    
    kb->bindPorts(inputs, outputs);
    kb->setModule(iBuilder, mMainModule);
}

void NVPTXDriver::generatePipelineIR() {
    #ifndef NDEBUG
    if (LLVM_UNLIKELY(mPipeline.empty())) {
        report_fatal_error("Pipeline cannot be empty");
    } else {
        for (auto i = mPipeline.begin(); i != mPipeline.end(); ++i) {
            for (auto j = i; ++j != mPipeline.end(); ) {
                if (LLVM_UNLIKELY(*i == *j)) {
                    report_fatal_error("Kernel instances cannot occur twice in the pipeline");
                }
            }
        }
    }
    #endif

    // note: instantiation of all kernels must occur prior to initialization
    for (const auto & k : mPipeline) {
        k->addKernelDeclarations(iBuilder);
    }
    for (const auto & k : mPipeline) {
        k->createInstance(iBuilder);
    }
    for (const auto & k : mPipeline) {
        k->initializeInstance(iBuilder);
    }
    
    generatePipelineLoop(iBuilder, mPipeline);
    
    for (const auto & k : mPipeline) {
        k->finalizeInstance(iBuilder);
    }
}

void NVPTXDriver::finalizeAndCompile(Function * mainFunc, std::string PTXFilename) {

    legacy::PassManager PM;
    PM.add(createPromoteMemoryToRegisterPass()); //Force the use of mem2reg to promote stack variables.
    PM.add(createReassociatePass());             //Reassociate expressions.
    PM.add(createGVNPass());                     //Eliminate common subexpressions.
    PM.add(createInstructionCombiningPass());    //Simple peephole optimizations and bit-twiddling.
    PM.add(createCFGSimplificationPass());

    for (kernel::Kernel * const kb : mPipeline) {
        iBuilder->setKernel(kb);
        kb->generateKernel(iBuilder);
    }

    MDNode * Node = MDNode::get(mMainModule->getContext(),
                                {llvm::ValueAsMetadata::get(mainFunc),
                                 MDString::get(mMainModule->getContext(), "kernel"), 
                                 ConstantAsMetadata::get(ConstantInt::get(iBuilder->getInt32Ty(), 1))});
    NamedMDNode *NMD = mMainModule->getOrInsertNamedMetadata("nvvm.annotations");
    NMD->addOperand(Node);

    PM.run(*mMainModule);  

    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::ShowIR)))
            mMainModule->dump();

    llvm2ptx(mMainModule, PTXFilename);
}

const std::unique_ptr<kernel::KernelBuilder> & NVPTXDriver::getBuilder() {
    return iBuilder;
}

NVPTXDriver::~NVPTXDriver() {
}
