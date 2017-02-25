/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <string>  // for string
#include <vector>  // for vector
namespace IDISA { class IDISA_Builder; }
namespace llvm { class ConstantInt; }
namespace llvm { class Function; }
namespace llvm { class Module; }
namespace llvm { class PointerType; }
namespace llvm { class StructType; }
namespace llvm { class Type; }
namespace llvm { class Value; }


// Processing rate attributes are required for all stream set bindings for a kernel.
// These attributes describe the number of items that are processed or produced as
// a ratio in comparison to the principal input stream set (or the principal output
// stream set if there is no input.
//
// The default ratio is FixedRatio(1) which means that there is one item processed or
// produced for every item of the principal input or output stream.
// FixedRatio(m, n) means that for every group of n items of the principal stream,
// there are m items in the output stream (rounding up).
// 
// Kernels which produce a variable number of items use MaxRatio(n), for a maximum
// of n items produced or consumed per principal input or output item.  MaxRatio(m, n)
// means there are at most m items for every n items of the principal stream.
//
// RoundUpToMultiple(n) means that number of items produced is the same as the
// number of input items, rounded up to an exact multiple of n.
// 

struct ProcessingRate  {
    enum ProcessingRateKind : uint8_t {Fixed, RoundUp, Max, Unknown};
    ProcessingRate() {}
    ProcessingRateKind getKind() const {return mKind;}
    bool isExact() const {return (mKind == Fixed)||(mKind == RoundUp) ;}
    llvm::Value * CreateRatioCalculation(IDISA::IDISA_Builder * b, llvm::Value * principalInputItems) const;
    friend ProcessingRate FixedRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems);
    friend ProcessingRate MaxRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems);
    friend ProcessingRate RoundUpToMultiple(unsigned itemMultiple);
    friend ProcessingRate UnknownRate();
    
protected:
    ProcessingRate(ProcessingRateKind k, unsigned numerator, unsigned denominator) 
    : mKind(k), ratio_numerator(numerator), ratio_denominator(denominator) {}
    ProcessingRateKind mKind;
    uint16_t ratio_numerator;
    uint16_t ratio_denominator;
    bool isVariableRate();
}; 

ProcessingRate FixedRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems = 1);
ProcessingRate MaxRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems = 1);
ProcessingRate RoundUpToMultiple(unsigned itemMultiple);
ProcessingRate UnknownRate();

struct Binding {
    Binding(llvm::Type * type, const std::string & name, ProcessingRate r = FixedRatio(1))
    : type(type), name(name), rate(r) { }
    llvm::Type *        type;
    std::string         name;
    ProcessingRate      rate;
};

class KernelInterface {

public:
    /*
     
     This class defines the methods to be used to generate the code  
     necessary for declaring, allocating, calling and synchronizing
     kernels.   The methods to be used for constructing kernels are defined
     within the KernelBuilder class of kernel.h
     
     */
    
    const std::string & getName() const { return mKernelName; }

    void setName(std::string newName) { mKernelName = newName; }
       
    const std::vector<Binding> & getStreamInputs() const { return mStreamSetInputs; }

    const std::vector<Binding> & getStreamOutputs() const { return mStreamSetOutputs; }

    const std::vector<Binding> & getScalarInputs() const { return mScalarInputs; }

    const std::vector<Binding> & getScalarOutputs() const { return mScalarOutputs; }
        
    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(llvm::Module * client);

    virtual void createInstance() = 0;

    void setInitialArguments(std::vector<llvm::Value *> args);

    llvm::Value * getInstance() const { return mKernelInstance; }

    unsigned getLookAhead() const {
        return mLookAheadPositions;
    }
    
    IDISA::IDISA_Builder * getBuilder() const {
        return iBuilder;
    }

    virtual llvm::Value * getProcessedItemCount(llvm::Value * instance, const std::string & name) const = 0;

    virtual void setProcessedItemCount(llvm::Value * instance, const std::string & name, llvm::Value * value) const = 0;

    virtual llvm::Value * getProducedItemCount(llvm::Value * instance, const std::string & name) const = 0;

    virtual void setProducedItemCount(llvm::Value * instance, const std::string & name, llvm::Value * value) const = 0;

    virtual llvm::Value * getTerminationSignal(llvm::Value * instance) const = 0;

    virtual void setTerminationSignal(llvm::Value * instance) const = 0;
    
    void setLookAhead(unsigned lookAheadPositions) {
        mLookAheadPositions = lookAheadPositions;
    }

    llvm::Function * getInitFunction() const;

    llvm::Function * getDoSegmentFunction() const;

    llvm::Function * getAccumulatorFunction(const std::string & accumName) const;

protected:

    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string && kernelName,
                    std::vector<Binding> && stream_inputs,
                    std::vector<Binding> && stream_outputs,
                    std::vector<Binding> && scalar_inputs,
                    std::vector<Binding> && scalar_outputs,
                    std::vector<Binding> && internal_scalars)
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
    
//    virtual void addAdditionalKernelDeclarations(llvm::Module * module, llvm::PointerType * selfType) {}

protected:
    
    IDISA::IDISA_Builder * const iBuilder;
    std::string mKernelName;
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
