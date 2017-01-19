/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <string>  // for string
#include <vector>  // for vector
namespace IDISA { class IDISA_Builder; }
namespace llvm { class Module; }
namespace llvm { class StructType; }
namespace llvm { class Type; }
namespace llvm { class Value; }

struct Binding {
    llvm::Type * type;
    std::string name;
    Binding(llvm::Type * type, const std::string & name) : type(type), name(name) {}
    Binding(llvm::Type * type, std::string && name) : type(type), name(name) {}
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
    
    const std::string & getName() const { return mKernelName;}
       
    const std::vector<Binding> & getStreamInputs() const {return mStreamSetInputs;}
    const std::vector<Binding> & getStreamOutputs() const {return mStreamSetOutputs;}
    const std::vector<Binding> & getScalarInputs() const { return mScalarInputs;}
    const std::vector<Binding> & getScalarOutputs() const { return mScalarOutputs;}
    
    
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(llvm::Module * client) const;
    virtual void createInstance() = 0;
    void setInitialArguments(std::vector<llvm::Value *> args);
    llvm::Value * getInstance() const { return mKernelInstance; }

    llvm::Value * createDoSegmentCall(std::vector<llvm::Value *> args) const;
    llvm::Value * createFinalBlockCall(llvm::Value * self, llvm::Value * remainingBytes) const;
    llvm::Value * createGetAccumulatorCall(llvm::Value * self, std::string accumName) const;
    
    unsigned getLookAhead() const {
        return mLookAheadPositions;
    }
    
    IDISA::IDISA_Builder * getBuilder() const {
        return iBuilder;
    }

    virtual llvm::Value * getProcessedItemCount(llvm::Value * self, const std::string & ssName) const = 0;
    virtual llvm::Value * getProducedItemCount(llvm::Value * self, const std::string & ssName) const = 0;
    virtual llvm::Value * getTerminationSignal(llvm::Value * self) const = 0;
    
    void setLookAhead(unsigned lookAheadPositions) {
        mLookAheadPositions = lookAheadPositions;
    }

    llvm::Value * createDoBlockCall(llvm::Value * self) const;

protected:

    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string && kernelName,
                    std::vector<Binding> stream_inputs,
                    std::vector<Binding> stream_outputs,
                    std::vector<Binding> scalar_inputs,
                    std::vector<Binding> scalar_outputs,
                    std::vector<Binding> internal_scalars)
    : iBuilder(builder)
    , mKernelName(kernelName)
    , mStreamSetInputs(stream_inputs)
    , mStreamSetOutputs(stream_outputs)
    , mScalarInputs(scalar_inputs)
    , mScalarOutputs(scalar_outputs)
    , mInternalScalars(internal_scalars)
    , mKernelStateType(nullptr)
    , mKernelInstance(nullptr)
    , mLookAheadPositions(0) {

    }
    
protected:
    
    IDISA::IDISA_Builder * const iBuilder;
    const std::string mKernelName;
    std::vector<llvm::Value *> mInitialArguments;
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
