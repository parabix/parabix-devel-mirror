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
    
    // Create a module for the kernel, including the kernel state type declaration and
    // the full implementation of all required methods.     
    //
    std::unique_ptr<llvm::Module> createKernelModule();
    
    // Generate the Kernel to the current module (iBuilder->getModule()).
    void generateKernel();
    
protected:
    //
    // Kernel builder subtypes define their logic of kernel construction
    // in terms of 3 virtual methods for
    // (a) preparing the Kernel state data structure
    // (b) defining the logic of the doBlock function, and
    // (c) defining the logic of the finalBlock function.
    //
    // Note: the kernel state data structure must only be finalized after
    // all scalar fields have been added.   If there are no fields to
    // be added, the default method for preparing kernel state may be used.
    
    virtual void prepareKernelStateType();
    
    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod() = 0;
    
    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.
    
    virtual void generateFinalBlockMethod();
    
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

protected:

    std::vector<llvm::Type *>  mKernelFields;
    NameMap                    mInternalStateNameMap;
};
}
#endif 
