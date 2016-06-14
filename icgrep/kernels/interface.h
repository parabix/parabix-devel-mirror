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
#include <boost/container/flat_map.hpp>
#include "streamset.h"

struct ScalarBinding {
    llvm::Type * scalarType;
    std::string scalarName;
};

struct StreamSetBinding {
    kernel::StreamSetType ssType;
    std::string ssName;
};
   
class KernelInterface {
    using NameMap = boost::container::flat_map<std::string, llvm::ConstantInt *>;

public:
    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<StreamSetBinding> stream_inputs,
                    std::vector<StreamSetBinding> stream_outputs,
                    std::vector<ScalarBinding> scalar_parameters,
                    std::vector<ScalarBinding> scalar_outputs,
                    std::vector<ScalarBinding> internal_scalars);
    
    void finalizeKernelStateType();
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(Module * client);
    
    // Create a module for the kernel, including the kernel state type and
    // all required methods.  The init and accumulator output methods will be
    // defined, while the doBlock and finalBlock methods will initially be empty.
    //
    virtual std::unique_ptr<llvm::Module> createKernelModule();
    
    llvm::Value * createInstance(std::vector<llvm::Value *> initialParameters);
    llvm::Value * createDoBlockCall(llvm::Value * kernelInstance, std::vector<Value *> streamSets);
    llvm::Value * createFinalBlockCall(llvm::Value * kernelInstance, llvm::Value * remainingBytes, std::vector<llvm::Value *> streamSets);
    llvm::Value * createGetAccumulatorCall(llvm::Value * kernelInstance, std::string accumName);
    
    
protected:
    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    void addScalar(llvm::Type * t, std::string scalarName);
    
    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.
    
    // Get the index of a named scalar field within the kernel state struct.
    llvm::Value * getScalarIndex(std::string);
    
    // Get the value of a scalar field for a given instance.
    llvm::Value * getScalarField(llvm::Value * self, std::string fieldName);
    
    // Set the value of a scalar field for a given instance.
    void setScalarField(llvm::Value * self, std::string fieldName, llvm::Value * newFieldVal);
    
    // Get a parameter by name.
    llvm::Value * getParameter(llvm::Function * f, std::string paramName);


    IDISA::IDISA_Builder * iBuilder;
    std::string mKernelName;
    std::vector<StreamSetBinding> mStreamSetInputs;
    std::vector<StreamSetBinding> mStreamSetOutputs;
    
    std::vector<ScalarBinding> mScalarInputs;
    std::vector<ScalarBinding> mScalarOutputs;
    std::vector<ScalarBinding> mInternalScalars;

    std::vector<llvm::Type *>  mKernelFields;
    llvm::Type *               mKernelStateType;
    NameMap                    mInternalStateNameMap;
};

#endif 
