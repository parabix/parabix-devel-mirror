/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H


#include "interface.h"
#include <vector>
#include <llvm/IR/Type.h>
#include <IDISA/idisa_builder.h>
#include <boost/container/flat_map.hpp>


namespace kernel {
    
class KernelBuilder : public KernelInterface {
    using NameMap = boost::container::flat_map<std::string, llvm::ConstantInt *>;

public:
    KernelBuilder(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<StreamSetBinding> stream_inputs,
                    std::vector<StreamSetBinding> stream_outputs,
                    std::vector<ScalarBinding> scalar_parameters,
                    std::vector<ScalarBinding> scalar_outputs,
                    std::vector<ScalarBinding> internal_scalars);
    
    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    void addScalar(llvm::Type * t, std::string scalarName);
    
    void finalizeKernelStateType();
    
    // Create a module for the kernel, including the kernel state type and
    // all required methods.  The init and accumulator output methods will be
    // defined, while the doBlock and finalBlock methods will initially be empty.
    //
    virtual std::unique_ptr<llvm::Module> createKernelModule();
    
    // Generate Kernel to the current module.
    virtual void generateKernel();
    
    // Add a FinalBlock method that simply calls DoBlock without additional
    // preparation.
    void addTrivialFinalBlockMethod(Module * m);
    
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

protected:

    std::vector<llvm::Type *>  mKernelFields;
    NameMap                    mInternalStateNameMap;
};
}
#endif 
