/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H


#include "streamset.h"
#include "interface.h"
#include <vector>
#include <llvm/IR/Type.h>
#include <IDISA/idisa_builder.h>
#include <boost/container/flat_map.hpp>

const std::string blockNoScalar = "blockNo";
const std::string logicalSegmentNoScalar = "logicalSegNo";
const std::string processedItemCount = "processedItemCount";
const std::string producedItemCount = "producedItemCount";
const std::string terminationSignal = "terminationSignal";
const std::string structPtrSuffix = "_structPtr";
const std::string blkMaskSuffix = "_blkMask";

using namespace parabix;
namespace kernel {
    
class KernelBuilder : public KernelInterface {
    using NameMap = boost::container::flat_map<std::string, unsigned>;

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
    std::unique_ptr<llvm::Module> createKernelModule(std::vector<StreamSetBuffer *> input_buffers, std::vector<StreamSetBuffer *> output_buffers);
    
    // Generate the Kernel to the current module (iBuilder->getModule()).
    void generateKernel(std::vector<StreamSetBuffer *> input_buffers, std::vector<StreamSetBuffer *> output_buffers);
    
    llvm::Value * createInstance(std::vector<Value *> args) override;

    Function * generateThreadFunction(std::string name);

    Value * getBlockNo(Value * self);
    virtual llvm::Value * getLogicalSegmentNo(llvm::Value * kernelInstance) override;
    virtual llvm::Value * getProcessedItemCount(llvm::Value * kernelInstance) override;
    virtual llvm::Value * getProducedItemCount(llvm::Value * kernelInstance) override;
    virtual llvm::Value * getTerminationSignal(llvm::Value * kernelInstance) override;
    
    
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
    
    virtual void prepareKernel();
    
    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod() = 0;
    virtual void generateDoBlockLogic(Value * self, Value * blockNo);

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.
    
    virtual void generateFinalBlockMethod();
    
    virtual void generateDoSegmentMethod();
    
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
    
    // Stream set helpers.
    unsigned getStreamSetIndex(std::string ssName);
    
    llvm::Value * getStreamSetStructPtr(Value * self, std::string ssName);
    size_t getStreamSetBufferSize(Value * self, std::string ssName);

    llvm::Value * getStreamSetBlockPtr(Value * self, std::string ssName, Value * blockNo);

    void setBlockNo(Value * self, Value * newFieldVal);
    virtual void setLogicalSegmentNo(llvm::Value * self, Value * newFieldVal);
    virtual void setProcessedItemCount(llvm::Value * self, Value * newFieldVal);
    virtual void setProducedItemCount(llvm::Value * self, Value * newFieldVal);
    virtual void setTerminationSignal(llvm::Value * self, Value * newFieldVal);
    
    
protected:

    std::vector<llvm::Type *>  mKernelFields;
    NameMap                    mInternalStateNameMap;
    NameMap                    mStreamSetNameMap;
    std::vector<StreamSetBuffer *> mStreamSetInputBuffers;
    std::vector<StreamSetBuffer *> mStreamSetOutputBuffers;

};
}
#endif 
