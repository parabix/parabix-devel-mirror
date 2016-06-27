/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H


#include <string>
#include <vector>
#include <llvm/IR/Type.h>
#include <IDISA/idisa_builder.h>
#include "streamset.h"

struct ScalarBinding {
    llvm::Type * scalarType;
    std::string scalarName;
};

struct StreamSetBinding {
    kernel::StreamSetType ssType;
    std::string ssName;
};
   
const std::string init_suffix = "_Init";
const std::string doBlock_suffix = "_DoBlock";
const std::string finalBlock_suffix = "_FinalBlock";
const std::string accumulator_infix = "_get_";

class KernelInterface {

public:
    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<StreamSetBinding> stream_inputs,
                    std::vector<StreamSetBinding> stream_outputs,
                    std::vector<ScalarBinding> scalar_parameters,
                    std::vector<ScalarBinding> scalar_outputs,
                    std::vector<ScalarBinding> internal_scalars) :
    iBuilder(builder),
    mKernelName(kernelName),
    mStreamSetInputs(stream_inputs),
    mStreamSetOutputs(stream_outputs),
    mScalarInputs(scalar_parameters),
    mScalarOutputs(scalar_outputs),
    mInternalScalars(internal_scalars),
    mDoBlockReturnType(nullptr),
    mKernelStateType(nullptr) {}
    
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(Module * client);
    
    llvm::Value * createInstance(std::vector<llvm::Value *> initialParameters);
    llvm::Value * createDoBlockCall(llvm::Value * kernelInstance, std::vector<Value *> streamSets);
    llvm::Value * createFinalBlockCall(llvm::Value * kernelInstance, llvm::Value * remainingBytes, std::vector<llvm::Value *> streamSets);
    llvm::Value * createGetAccumulatorCall(llvm::Value * kernelInstance, std::string accumName);
    
protected:
    
    IDISA::IDISA_Builder * iBuilder;
    std::string mKernelName;
    std::vector<StreamSetBinding> mStreamSetInputs;
    std::vector<StreamSetBinding> mStreamSetOutputs;
    std::vector<ScalarBinding> mScalarInputs;
    std::vector<ScalarBinding> mScalarOutputs;
    std::vector<ScalarBinding> mInternalScalars;
    llvm::Type * mDoBlockReturnType;
    llvm::Type * mKernelStateType;
};

#endif 
