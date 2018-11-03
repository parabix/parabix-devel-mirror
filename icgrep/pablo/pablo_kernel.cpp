/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include "pablo_kernel.h"
#include <pablo/codegenstate.h>
#include <pablo/pablo_compiler.h>
#include <pablo/pe_var.h>
#include <pablo/pe_zeroes.h>
#include <pablo/pe_ones.h>
#include <pablo/pablo_toolchain.h>
#include <kernels/kernel_builder.h>
#include <kernels/streamset.h>
#include <llvm/IR/Module.h>

#include <pablo/branch.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace IDISA;
using namespace llvm;

namespace pablo {

Var * PabloKernel::getInputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Input);
    return mInputs[index];
}

std::vector<PabloAST *> PabloKernel::getInputStreamSet(const std::string & name) {
    unsigned index; Port port;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Input);
    const Binding & input = getInputStreamSetBinding(index);
    const auto numOfStreams = IDISA::getNumOfStreams(input.getType());
    std::vector<PabloAST *> inputSet(numOfStreams);
    for (unsigned i = 0; i < numOfStreams; i++) {
        inputSet[i] = mEntryScope->createExtract(mInputs[index], mEntryScope->getInteger(i));
    }
    return inputSet;
}


Var * PabloKernel::getOutputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Output);
    return mOutputs[index];
}

Var * PabloKernel::getOutputScalarVar(const std::string & name) {
    for (Var * out : mScalarOutputVars) {
        if (out->getName().equals(name)) {
            return out;
        }
    }
    report_fatal_error("Kernel does not contain scalar " + name);
}

Var * PabloKernel::makeVariable(const String * name, Type * const type) {
    Var * const var = new (mAllocator) Var(name, type, mAllocator);
    mVariables.push_back(var);
    return var;
}

Zeroes * PabloKernel::getNullValue(Type * type) {
    if (LLVM_LIKELY(type == nullptr)) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Zeroes>(constant) && constant->getType() == type) {
            return cast<Zeroes>(constant);
        }
    }
    Zeroes * value = new (mAllocator) Zeroes(type, mAllocator);
    mConstants.push_back(value);
    return value;
}

Ones * PabloKernel::getAllOnesValue(Type * type) {
    if (LLVM_LIKELY(type == nullptr)) {
        type = getStreamTy();
    }
    for (PabloAST * constant : mConstants) {
        if (isa<Ones>(constant) && constant->getType() == type) {
            return cast<Ones>(constant);
        }
    }
    Ones * value = new (mAllocator) Ones(type, mAllocator);
    mConstants.push_back(value);
    return value;
}

void PabloKernel::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & b) {
    mSizeTy = b->getSizeTy();
    mStreamTy = b->getStreamTy();
    mSymbolTable = new SymbolGenerator(b->getContext(), mAllocator);
    mEntryScope = new (mAllocator) PabloBlock(this, mAllocator);
    mContext = &b->getContext();
    for (const Binding & ss : mInputStreamSets) {
        Var * param = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelInputParameter);
        param->addUser(this);
        mInputs.push_back(param);
        mVariables.push_back(param);
    }
    for (const Binding & ss : mOutputStreamSets) {
        Var * result = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
    }
    for (const Binding & ss : mOutputScalars) {
        Var * result = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
        mScalarOutputVars.push_back(result);
        result->setScalar();
    }
    generatePabloMethod();    
    pablo_function_passes(this);
    mPabloCompiler = new PabloCompiler(this);
    mPabloCompiler->initializeKernelData(b);
    mSizeTy = nullptr;
    mStreamTy = nullptr;   
}

void PabloKernel::generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder) {
    mSizeTy = iBuilder->getSizeTy();
    mStreamTy = iBuilder->getStreamTy();
    mPabloCompiler->compile(iBuilder);
    mSizeTy = nullptr;
    mStreamTy = nullptr;
}

#if 0
void PabloKernel::beginConditionalRegion(const std::unique_ptr<KernelBuilder> & b) {
    mPabloCompiler->clearCarryData(b);
}
#endif

void PabloKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & b, Value * const remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    b->setScalarField("EOFbit", b->bitblock_set_bit(remainingBytes));
    b->setScalarField("EOFmask", b->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall(b);

//    // TODO: clean this up. what if we have a subclass of block oriented kernels that has hooks for inserting code
//    // before/after calling the do block logic? it would simplify the logic and hopefully allow easier use. Moreover
//    // we may be able to convert some of the final block only scalars to stack oriented ones

//    Value * const baseOutputItems = getRemainingItems(b);
//    // clear all of the outputs past the EOF mark
//    for (unsigned i = 0; i < mStreamSetOutputs.size(); ++i) {
//        const Binding & output = mStreamSetOutputs[i];
//        // TODO: test if this is a bitstream output
//        Value * outputItems = baseOutputItems;
//        for (const kernel::Attribute & attr : output.getAttributes()) {
//            if (attr.isAdd()) {
//                outputItems = b->CreateAdd(outputItems, b->getSize(attr.amount()));
//                break;
//            }
//        }
//        Value * const inFileMask = b->CreateNot(b->bitblock_mask_from(outputItems));
//        BasicBlock * const zeroOutEntry = b->GetInsertBlock();
//        BasicBlock * const zeroOutLoop = b->CreateBasicBlock();
//        BasicBlock * const zeroOutExit = b->CreateBasicBlock();
//        Value * const n = b->getOutputStreamSetCount(output.getName());
//        b->CreateBr(zeroOutLoop);

//        b->SetInsertPoint(zeroOutLoop);
//        PHINode * const streamIndex = b->CreatePHI(b->getSizeTy(), 2);
//        streamIndex->addIncoming(b->getSize(0), zeroOutEntry);
//        Value * const ptr = b->getOutputStreamBlockPtr(output.getName(), streamIndex);
//        Value * const value = b->CreateBlockAlignedLoad(ptr);
//        b->CreateBlockAlignedStore(b->CreateAnd(value, inFileMask), ptr);
//        Value * const nextStreamIndex = b->CreateAdd(streamIndex, b->getSize(1));
//        streamIndex->addIncoming(nextStreamIndex, zeroOutLoop);
//        b->CreateCondBr(b->CreateICmpNE(nextStreamIndex, n), zeroOutLoop, zeroOutExit);

//        b->SetInsertPoint(zeroOutExit);
//    }

}

void PabloKernel::generateFinalizeMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mPabloCompiler->releaseKernelData(iBuilder);

    if (CompileOptionIsSet(PabloCompilationFlags::EnableProfiling)) {


        Value * fd = iBuilder->CreateOpenCall(iBuilder->GetString("./" + getName() + ".profile"),
                                              iBuilder->getInt32(O_WRONLY | O_CREAT | O_TRUNC),
                                              iBuilder->getInt32(S_IRUSR | S_IWUSR | S_IRGRP | S_IWGRP | S_IROTH));

        Function * dprintf = iBuilder->GetDprintf();



        Value * profile = iBuilder->getScalarFieldPtr("profile");


        unsigned branchCount = 0;

        for (const auto bb : mPabloCompiler->mBasicBlock) {

            std::string tmp;
            raw_string_ostream str(tmp);
            str << "%lu\t";
            str << bb->getName();
            str << "\n";

            Value * taken = iBuilder->CreateLoad(iBuilder->CreateGEP(profile, {iBuilder->getInt32(0), iBuilder->getInt32(branchCount++)}));
            iBuilder->CreateCall(dprintf, {fd, iBuilder->GetString(str.str()), taken});

        }

        iBuilder->CreateCloseCall(fd);
    }

}

String * PabloKernel::makeName(const llvm::StringRef & prefix) const {
    return mSymbolTable->makeString(prefix);
}

Integer * PabloKernel::getInteger(const int64_t value) const {
    return mSymbolTable->getInteger(value);
}

llvm::IntegerType * PabloKernel::getInt1Ty() const {
    return IntegerType::getInt1Ty(getModule()->getContext());
}

static inline std::string && annotateKernelNameWithPabloDebugFlags(std::string && name) {
    if (DebugOptionIsSet(DumpTrace)) {
        name += "_DumpTrace";
    }
    if (CompileOptionIsSet(EnableProfiling)) {
        name += "_BranchProfiling";
    }
    return std::move(name);
}

PabloKernel::PabloKernel(const std::unique_ptr<KernelBuilder> & b,
                         std::string && kernelName,
                         std::vector<Binding> stream_inputs,
                         std::vector<Binding> stream_outputs,
                         std::vector<Binding> scalar_parameters,
                         std::vector<Binding> scalar_outputs)
: BlockOrientedKernel(annotateKernelNameWithPabloDebugFlags(std::move(kernelName)),
                      std::move(stream_inputs), std::move(stream_outputs), 
                      std::move(scalar_parameters), std::move(scalar_outputs),
                      {Binding{b->getBitBlockType(), "EOFbit"}, Binding{b->getBitBlockType(), "EOFmask"}})
, PabloAST(PabloAST::ClassTypeId::Kernel, nullptr, mAllocator)
, mPabloCompiler(nullptr)
, mSymbolTable(nullptr)
, mEntryScope(nullptr)
, mSizeTy(nullptr)
, mStreamTy(nullptr)
, mContext(nullptr) {

}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable; 
}

}
