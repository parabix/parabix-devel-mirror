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
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(Module * client);
    
    // Create a module for the kernel, including the kernel state type and
    // all required methods.  The init and accumulator output methods will be
    // defined, while the doBlock and finalBlock methods will initially be empty.
    std::unique_ptr<llvm::Module> createKernelModule();
    
protected:
    // Get the index of a named scalar within the kernel state struct.
    llvm::Value * getInputStreamSetIndex(std::string);
    llvm::Value * getOutputStreamSetIndex(std::string);
    llvm::Value * getScalarIndex(std::string);
    
    
private:
    IDISA::IDISA_Builder * iBuilder;
    std::string mKernelName;
    std::vector<StreamSetBinding> mStreamSetInputs;
    std::vector<StreamSetBinding> mStreamSetOutputs;
    
    std::vector<ScalarBinding> mScalarInputs;
    std::vector<ScalarBinding> mScalarOutputs;
    std::vector<ScalarBinding> mInternalScalars;
    llvm::Type *               mKernelStateType;
    NameMap                    mInternalStateNameMap;

};


#endif 
