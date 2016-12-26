/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <string>
#include <vector>
#include <llvm/IR/Type.h>
#include <IR_Gen/idisa_builder.h>
#include <kernels/streamset.h>

struct Binding {
    llvm::Type * type;
    std::string name;

    Binding(llvm::Type * type, std::string name)
    : type(type)
    , name(std::move(name)) {

    }
};

static const std::string init_suffix = "_Init";
static const std::string doBlock_suffix = "_DoBlock";
static const std::string doSegment_suffix = "_DoSegment";
static const std::string finalBlock_suffix = "_FinalBlock";
static const std::string accumulator_infix = "_get_";

class KernelInterface {

public:
    /*
     
     This class defines the methods to be used to generate the code  
     necessary for declaring, allocating, calling and synchronizing
     kernels.   The methods to be used for constructing kernels are defined
     within the KernelBuilder class of kernel.h
     
     */
    
    std::string & getName() { return mKernelName;}
    
    std::vector<Binding> getStreamInputs() {return mStreamSetInputs;}
    std::vector<Binding> getStreamOutputs() {return mStreamSetOutputs;}
    std::vector<Binding> getScalarInputs() { return mScalarInputs;}
    std::vector<Binding> getScalarOutputs() { return mScalarOutputs;}
    
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(Module * client);
    
    void setInitialArguments(std::vector<llvm::Value *> initialParameters);
    virtual void createInstance();
    llvm::Value * getInstance() {return mKernelInstance;};

    llvm::Value * createDoSegmentCall(llvm::Value * kernelInstance, llvm::Value * blkCount);
    llvm::Value * createFinalBlockCall(llvm::Value * kernelInstance, llvm::Value * remainingBytes);
    llvm::Value * createGetAccumulatorCall(llvm::Value * kernelInstance, std::string accumName);
    
    unsigned getLookAhead() const {
        return mLookAheadPositions;
    }
    
    IDISA::IDISA_Builder * getBuilder() const {
        return iBuilder;
    }

    virtual llvm::Value * getProcessedItemCount(llvm::Value * kernelInstance) = 0;
    virtual llvm::Value * getProducedItemCount(llvm::Value * kernelInstance) = 0;
    virtual llvm::Value * getTerminationSignal(llvm::Value * kernelInstance) = 0;
    
    void setLookAhead(unsigned lookAheadPositions) {
        mLookAheadPositions = lookAheadPositions;
    }

    llvm::Value * createDoBlockCall(llvm::Value * kernelInstance);

protected:

    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<Binding> stream_inputs,
                    std::vector<Binding> stream_outputs,
                    std::vector<Binding> scalar_inputs,
                    std::vector<Binding> scalar_outputs,
                    std::vector<Binding> internal_scalars) :
    iBuilder(builder),
    mKernelName(kernelName),
    mStreamSetInputs(stream_inputs),
    mStreamSetOutputs(stream_outputs),
    mScalarInputs(scalar_inputs),
    mScalarOutputs(scalar_outputs),
    mInternalScalars(internal_scalars),
    mKernelStateType(nullptr),
    mLookAheadPositions(0) {}
    
protected:
    
    IDISA::IDISA_Builder * const iBuilder;
    std::string mKernelName;
    std::vector<Value *> mInitialArguments;
    std::vector<Binding> mStreamSetInputs;
    std::vector<Binding> mStreamSetOutputs;
    std::vector<Binding> mScalarInputs;
    std::vector<Binding> mScalarOutputs;
    std::vector<Binding> mInternalScalars;
    llvm::StructType * mKernelStateType;
    llvm::Value * mKernelInstance;
    unsigned mLookAheadPositions;
    
};
#endif 
