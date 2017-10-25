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
#include <llvm/IR/Module.h>

#include <pablo/branch.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <llvm/Support/raw_ostream.h>

using namespace kernel;
using namespace parabix;
using namespace IDISA;
using namespace llvm;

inline bool isStreamType(const Type * ty) {
    if (ty->isArrayTy()) {
        ty = ty->getArrayElementType();
    }
    if (ty->isVectorTy()) {
        return (ty->getVectorNumElements() == 0);
    }
    return false;
}

namespace pablo {

Var * PabloKernel::getInputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Input);
    return mInputs[index];
}

Var * PabloKernel::getOutputStreamVar(const std::string & name) {
    Port port; unsigned index;
    std::tie(port, index) = getStreamPort(name);
    assert (port == Port::Output);
    return mOutputs[index];
}

Var * PabloKernel::getOutputScalarVar(const std::string & name) {
    const auto f = mScalarOutputNameMap.find(name);
    if (LLVM_UNLIKELY(f == mScalarOutputNameMap.end())) {
        report_fatal_error("Kernel does not contain scalar: " + name);
    }
    return f->second;
}

Var * PabloKernel::addInput(const std::string & name, Type * const type) {
    Var * param = new (mAllocator) Var(makeName(name), type, mAllocator, Var::KernelInputParameter);
    param->addUser(this);
    mInputs.push_back(param);
    mVariables.push_back(param);
    if (isStreamType(type)) {
        mStreamMap.emplace(name, std::make_pair(Port::Input, mStreamSetInputs.size()));
        mStreamSetInputs.emplace_back(type, name);        
    } else {
        mScalarInputs.emplace_back(type, name);
        param->setScalar();
    }
    assert (mStreamSetInputs.size() + mScalarInputs.size() == mInputs.size());
    return param;
}

Var * PabloKernel::addOutput(const std::string & name, Type * const type) {
    Var * result = new (mAllocator) Var(makeName(name), type, mAllocator, Var::KernelOutputParameter);
    result->addUser(this);
    mOutputs.push_back(result);
    mVariables.push_back(result);
    if (isStreamType(type)) {
        mStreamMap.emplace(name, std::make_pair(Port::Output, mStreamSetOutputs.size()));
        mStreamSetOutputs.emplace_back(type, name);
    } else {
        mScalarOutputs.emplace_back(type, name);
        mScalarOutputNameMap.emplace(name, result);
        result->setScalar();
    }
    assert (mStreamSetOutputs.size() + mScalarOutputs.size() == mOutputs.size());
    return result;
}

Var * PabloKernel::makeVariable(String * name, Type * const type) {
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

void PabloKernel::addInternalKernelProperties(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) {
    mSizeTy = iBuilder->getSizeTy();
    mStreamTy = iBuilder->getStreamTy();
    generatePabloMethod();    
    pablo_function_passes(this);
    mPabloCompiler->initializeKernelData(iBuilder);
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

void PabloKernel::generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & iBuilder, Value * const remainingBytes) {
    // Standard Pablo convention for final block processing: set a bit marking
    // the position just past EOF, as well as a mask marking all positions past EOF.
    iBuilder->setScalarField("EOFbit", iBuilder->bitblock_set_bit(remainingBytes));
    iBuilder->setScalarField("EOFmask", iBuilder->bitblock_mask_from(remainingBytes));
    CreateDoBlockMethodCall(iBuilder);
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

//        Value * base = iBuilder->CreateLoad(iBuilder->CreateGEP(profile, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
//        base = iBuilder->CreateUIToFP(base, iBuilder->getDoubleTy());

//        unsigned branchCount = 0;
//        std::function<void (const PabloBlock *)> writeProfile = [&](const PabloBlock * const scope) {
//            for (const Statement * stmt : *scope) {
//                if (isa<Branch>(stmt)) {

//                    ++branchCount;

//                    std::string tmp;
//                    raw_string_ostream str(tmp);
//                    str << "%3.3f\t";
//                    str << mPabloCompiler->getBranchEntry(branchCount)->getName();
//                    str << "\n";

//                    Value * branches = iBuilder->CreateLoad(iBuilder->CreateGEP(profile, {iBuilder->getInt32(0), iBuilder->getInt32(branchCount)}));
//                    branches = iBuilder->CreateUIToFP(branches, iBuilder->getDoubleTy());
//                    Value * prob = iBuilder->CreateFDiv(branches, base);
//                    iBuilder->CreateCall(dprintf, {fd, iBuilder->GetString(str.str()), prob});

//                    writeProfile(cast<Branch>(stmt)->getBody());

//                }
//            }
//        };

//        writeProfile(getEntryBlock());
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

static inline std::string && annotateKernelNameWithDebugFlags(std::string && name) {
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
: BlockOrientedKernel(annotateKernelNameWithDebugFlags(std::move(kernelName)),
                      std::move(stream_inputs), std::move(stream_outputs), 
                      std::move(scalar_parameters), std::move(scalar_outputs),
                      {Binding{b->getBitBlockType(), "EOFbit"}, Binding{b->getBitBlockType(), "EOFmask"}})
, PabloAST(PabloAST::ClassTypeId::Kernel, nullptr, mAllocator)
, mPabloCompiler(new PabloCompiler(this))
, mSymbolTable(new SymbolGenerator(b->getContext(), mAllocator))
, mEntryBlock(PabloBlock::Create(this))
, mSizeTy(nullptr)
, mStreamTy(nullptr) {
    prepareStreamSetNameMap();
    for (const Binding & ss : mStreamSetInputs) {
        Var * param = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelInputParameter);
        param->addUser(this);
        mInputs.push_back(param);
        mVariables.push_back(param);
    }
    for (const Binding & ss : mStreamSetOutputs) {
        Var * result = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
    }
    for (const Binding & ss : mScalarOutputs) {
        Var * result = new (mAllocator) Var(makeName(ss.getName()), ss.getType(), mAllocator, Var::KernelOutputParameter);
        result->addUser(this);
        mOutputs.push_back(result);
        mVariables.push_back(result);
        mScalarOutputNameMap.emplace(ss.getName(), result);
        result->setScalar();
    }
}

PabloKernel::~PabloKernel() {
    delete mPabloCompiler;
    delete mSymbolTable; 
}

}
