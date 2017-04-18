/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <string>  // for string
#include <vector>  // for vector
namespace IDISA { class IDISA_Builder; }
//namespace llvm { class ConstantInt; }
#include <llvm/IR/Constants.h>
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
    enum class ProcessingRateKind : uint8_t { Fixed, RoundUp, Add1, Max, Unknown };
    ProcessingRateKind getKind() const {return mKind;}
    bool isExact() const {return (mKind == ProcessingRateKind::Fixed)||(mKind == ProcessingRateKind::RoundUp)||(mKind == ProcessingRateKind::Add1) ;}
    bool isUnknown() const { return !isExact(); }
    llvm::Value * CreateRatioCalculation(IDISA::IDISA_Builder * b, llvm::Value * principalInputItems, llvm::Value * doFinal = nullptr) const;
    friend ProcessingRate FixedRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems, std::string && referenceStreamSet);
    friend ProcessingRate MaxRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems, std::string && referenceStreamSet);
    friend ProcessingRate RoundUpToMultiple(unsigned itemMultiple, std::string && referenceStreamSet);
    friend ProcessingRate Add1(std::string && referenceStreamSet);
    friend ProcessingRate UnknownRate();
    std::string referenceStreamSet() const { return mReferenceStreamSet;}
protected:
    ProcessingRate(ProcessingRateKind k, unsigned numerator, unsigned denominator, std::string && referenceStreamSet)
    : mKind(k), mRatioNumerator(numerator), mRatioDenominator(denominator), mReferenceStreamSet(referenceStreamSet) {}
private:
    const ProcessingRateKind mKind;
    const uint16_t mRatioNumerator;
    const uint16_t mRatioDenominator;
    const std::string mReferenceStreamSet;
}; 

ProcessingRate FixedRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems = 1, std::string && referenceStreamSet = "");
ProcessingRate MaxRatio(unsigned strmItemsPer, unsigned perPrincipalInputItems = 1, std::string && referenceStreamSet = "");
ProcessingRate RoundUpToMultiple(unsigned itemMultiple, std::string &&referenceStreamSet = "");
ProcessingRate Add1(std::string && referenceStreamSet = "");
ProcessingRate UnknownRate();

struct Binding {
    Binding(llvm::Type * type, const std::string & name, ProcessingRate r = FixedRatio(1))
    : type(type), name(name), rate(r) { }
    llvm::Type * const        type;
    const std::string         name;
    const ProcessingRate      rate;
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

    virtual llvm::Value * createInstance() = 0;

    virtual void initializeInstance() = 0;

    virtual void terminateInstance() = 0;

    void setInitialArguments(std::vector<llvm::Value *> args);

    llvm::Value * getInstance() const {
        return mKernelInstance;
    }

    unsigned getLookAhead() const {
        return mLookAheadPositions;
    }
    
    IDISA::IDISA_Builder * getBuilder() const {
        return iBuilder;
    }

    virtual llvm::Value * getProducedItemCount(const std::string & name, llvm::Value * doFinal = nullptr) const = 0;

    virtual void setProducedItemCount(const std::string & name, llvm::Value * value) const = 0;

    virtual llvm::Value * getProcessedItemCount(const std::string & name) const = 0;

    virtual void setProcessedItemCount(const std::string & name, llvm::Value * value) const = 0;

    virtual llvm::Value * getTerminationSignal() const = 0;

    virtual void setTerminationSignal() const = 0;
    
    void setLookAhead(unsigned lookAheadPositions) {
        mLookAheadPositions = lookAheadPositions;
    }

    llvm::Function * getInitFunction() const;

    llvm::Function * getDoSegmentFunction() const;

    llvm::Function * getAccumulatorFunction(const std::string & accumName) const;

    llvm::Function * getTerminateFunction() const;

protected:

    KernelInterface(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<Binding> && stream_inputs,
                    std::vector<Binding> && stream_outputs,
                    std::vector<Binding> && scalar_inputs,
                    std::vector<Binding> && scalar_outputs,
                    std::vector<Binding> && internal_scalars)
    : iBuilder(builder)
    , mKernelInstance(nullptr)
    , mKernelStateType(nullptr)
    , mLookAheadPositions(0)
    , mKernelName(kernelName)
    , mStreamSetInputs(stream_inputs)
    , mStreamSetOutputs(stream_outputs)
    , mScalarInputs(scalar_inputs)
    , mScalarOutputs(scalar_outputs)
    , mInternalScalars(internal_scalars)
    {

    }
    
    void setInstance(llvm::Value * const instance) {
        assert ("kernel instance cannot be null!" && instance);
        assert ("kernel instance must point to a valid kernel state type!" && (instance->getType()->getPointerElementType() == mKernelStateType));
        mKernelInstance = instance;
    }

protected:
    
    IDISA::IDISA_Builder * const    iBuilder;
    llvm::Value *                   mKernelInstance;
    llvm::StructType *              mKernelStateType;
    unsigned                        mLookAheadPositions;
    std::string                     mKernelName;
    std::vector<llvm::Value *>      mInitialArguments;
    std::vector<Binding>            mStreamSetInputs;
    std::vector<Binding>            mStreamSetOutputs;
    std::vector<Binding>            mScalarInputs;
    std::vector<Binding>            mScalarOutputs;
    std::vector<Binding>            mInternalScalars;
};

#endif 
