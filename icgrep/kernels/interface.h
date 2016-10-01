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
    parabix::StreamSetType ssType;
    std::string ssName;
};
   
const std::string init_suffix = "_Init";
const std::string doBlock_suffix = "_DoBlock";
const std::string doSegment_suffix = "_DoSegment";
const std::string finalBlock_suffix = "_FinalBlock";
const std::string accumulator_infix = "_get_";

class KernelInterface {

public:
    /*
     
     This class defines the methods to be used to generate the code  
     necessary for declaring, allocating, calling and synchronizing
     kernels.   The methods to be used for constructing kernels are defined
     within the KernelBuilder class of kernel.h
     
     */
    
    std::string & getName() { return mKernelName;}
    
    std::vector<StreamSetBinding> getStreamInputs() {return mStreamSetInputs;}
    std::vector<StreamSetBinding> getStreamOutputs() {return mStreamSetOutputs;}
    std::vector<ScalarBinding> getScalarInputs() { return mScalarInputs;}
    std::vector<ScalarBinding> getScalarOutputs() { return mScalarOutputs;}
    
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(Module * client);
    
    virtual llvm::Value * createInstance(std::vector<llvm::Value *> initialParameters);

    llvm::Value * createDoSegmentCall(llvm::Value * kernelInstance, llvm::Value * blkCount);
    llvm::Value * createFinalBlockCall(llvm::Value * kernelInstance, llvm::Value * remainingBytes);
    llvm::Value * createGetAccumulatorCall(llvm::Value * kernelInstance, std::string accumName);
    
    unsigned getLookAhead() { return mLookAheadPositions; }
    
    
    virtual llvm::Value * getLogicalSegmentNo(llvm::Value * kernelInstance) = 0;
    virtual llvm::Value * getProcessedItemCount(llvm::Value * kernelInstance) = 0;
    virtual llvm::Value * getProducedItemCount(llvm::Value * kernelInstance) = 0;
    virtual llvm::Value * getTerminationSignal(llvm::Value * kernelInstance) = 0;
    
protected:
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
    mKernelStateType(nullptr),
    mLookAheadPositions(0) {}
    
    
    
    IDISA::IDISA_Builder * iBuilder;
    std::string mKernelName;
    std::vector<StreamSetBinding> mStreamSetInputs;
    std::vector<StreamSetBinding> mStreamSetOutputs;
    std::vector<ScalarBinding> mScalarInputs;
    std::vector<ScalarBinding> mScalarOutputs;
    std::vector<ScalarBinding> mInternalScalars;
    llvm::Type * mKernelStateType;
    unsigned mLookAheadPositions;
    
    void setLookAhead(unsigned lookAheadPositions) {mLookAheadPositions = lookAheadPositions;}
    llvm::Value * createDoBlockCall(llvm::Value * kernelInstance);

};
#endif 
