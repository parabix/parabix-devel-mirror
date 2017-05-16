/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <llvm/IR/Constants.h>
#include <string>
#include <vector>

namespace IDISA { class IDISA_Builder; }
namespace kernel { class KernelBuilder; }

// Processing rate attributes are required for all stream set bindings for a kernel.
// These attributes describe the number of items that are processed or produced as
// a ratio in comparison to a reference stream set, normally the principal input stream set 
// by default (or the principal output stream set if there is no input).
//
// The default ratio is FixedRatio(1) which means that there is one item processed or
// produced for every item of the reference stream.
// FixedRatio(m, n) means that for every group of n items of the refrence stream,
// there are m items in the output stream (rounding up).
// 
// Kernels which produce a variable number of items use MaxRatio(n), for a maximum
// of n items produced or consumed per principal input or output item.  MaxRatio(m, n)
// means there are at most m items for every n items of the reference stream.
//
// RoundUpToMultiple(n) means that number of items produced is the same as the
// number of reference items, rounded up to an exact multiple of n.
// 

struct ProcessingRate  {
    enum class ProcessingRateKind : uint8_t { FixedRatio, RoundUp, Add1, MaxRatio, Unknown };
    ProcessingRateKind getKind() const {return mKind;}
    bool isFixedRatio() const {return mKind == ProcessingRateKind::FixedRatio;}
    bool isMaxRatio() const {return mKind == ProcessingRateKind::MaxRatio;}
    bool isExact() const {return (mKind == ProcessingRateKind::FixedRatio)||(mKind == ProcessingRateKind::RoundUp)||(mKind == ProcessingRateKind::Add1) ;}
    bool isUnknownRate() const { return mKind == ProcessingRateKind::Unknown; }
    unsigned calculateRatio(unsigned referenceItems, bool doFinal = false) const;
    unsigned calculateMaxReferenceItems(unsigned outputItems, bool doFinal = false) const;
    llvm::Value * CreateRatioCalculation(IDISA::IDISA_Builder * const b, llvm::Value * referenceItems, llvm::Value * doFinal = nullptr) const;
    llvm::Value * CreateMaxReferenceItemsCalculation(IDISA::IDISA_Builder * const b, llvm::Value * outputItems, llvm::Value * doFinal = nullptr) const;
    friend ProcessingRate FixedRatio(unsigned strmItems, unsigned referenceItems, std::string && referenceStreamSet);
    friend ProcessingRate MaxRatio(unsigned strmItems, unsigned referenceItems, std::string && referenceStreamSet);
    friend ProcessingRate RoundUpToMultiple(unsigned itemMultiple, std::string && referenceStreamSet);
    friend ProcessingRate Add1(std::string && referenceStreamSet);
    friend ProcessingRate UnknownRate();
    uint16_t getRatioNumerator() const { return mRatioNumerator;}
    uint16_t getRatioDenominator() const { return mRatioDenominator;}
    const std::string & referenceStreamSet() const { return mReferenceStreamSet;}
protected:
    ProcessingRate(ProcessingRateKind k, unsigned numerator, unsigned denominator, std::string && referenceStreamSet)
    : mKind(k), mRatioNumerator(numerator), mRatioDenominator(denominator), mReferenceStreamSet(referenceStreamSet) {}
private:
    const ProcessingRateKind mKind;
    const uint16_t mRatioNumerator;
    const uint16_t mRatioDenominator;
    const std::string mReferenceStreamSet;
}; 

ProcessingRate FixedRatio(unsigned strmItems, unsigned referenceItems = 1, std::string && referenceStreamSet = "");
ProcessingRate MaxRatio(unsigned strmItems, unsigned referenceItems = 1, std::string && referenceStreamSet = "");
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
    
    const std::string & getName() const {
        return mKernelName;
    }
       
    virtual bool isCachable() const = 0;

    virtual std::string makeSignature(const std::unique_ptr<kernel::KernelBuilder> & idb) = 0;

    const std::vector<Binding> & getStreamInputs() const {
        return mStreamSetInputs;
    }

    const Binding & getStreamInput(const unsigned i) const {
        return mStreamSetInputs[i];
    }

    const std::vector<Binding> & getStreamOutputs() const {
        return mStreamSetOutputs;
    }

    const Binding & getStreamOutput(const unsigned i) const {
        return mStreamSetOutputs[i];
    }

    const std::vector<Binding> & getScalarInputs() const {
        return mScalarInputs;
    }

    const Binding & getScalarInput(const unsigned i) const {
        return mScalarInputs[i];
    }

    const std::vector<Binding> & getScalarOutputs() const {
        return mScalarOutputs;
    }

    const Binding & getScalarOutput(const unsigned i) const {
        return mScalarOutputs[i];
    }

    // Add ExternalLinkage method declarations for the kernel to a given client module.
    void addKernelDeclarations(const std::unique_ptr<kernel::KernelBuilder> & idb);

    virtual void linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> & idb) = 0;

    virtual llvm::Value * createInstance(const std::unique_ptr<kernel::KernelBuilder> & idb) = 0;

    virtual void initializeInstance(const std::unique_ptr<kernel::KernelBuilder> & idb) = 0;

    virtual void finalizeInstance(const std::unique_ptr<kernel::KernelBuilder> & idb) = 0;

    void setInitialArguments(std::vector<llvm::Value *> && args) {
        mInitialArguments.swap(args);
    }

    llvm::Value * getInstance() const {
        return mKernelInstance;
    }

    void setInstance(llvm::Value * const instance) {
        assert ("kernel instance cannot be null!" && instance);
        assert ("kernel instance must point to a valid kernel state type!" && (instance->getType()->getPointerElementType() == mKernelStateType));
        mKernelInstance = instance;
    }

    unsigned getLookAhead() const {
        return mLookAheadPositions;
    }

    void setLookAhead(const unsigned lookAheadPositions) {
        mLookAheadPositions = lookAheadPositions;
    }

protected:

    llvm::Function * getInitFunction(llvm::Module * const module) const;

    llvm::Function * getDoSegmentFunction(llvm::Module * const module) const;

    llvm::Function * getTerminateFunction(llvm::Module * const module) const;

    KernelInterface(std::string kernelName,
                    std::vector<Binding> && stream_inputs,
                    std::vector<Binding> && stream_outputs,
                    std::vector<Binding> && scalar_inputs,
                    std::vector<Binding> && scalar_outputs,
                    std::vector<Binding> && internal_scalars)
    : mKernelInstance(nullptr)
    , mModule(nullptr)
    , mKernelStateType(nullptr)
    , mLookAheadPositions(0)
    , mKernelName(kernelName)
    , mStreamSetInputs(stream_inputs)
    , mStreamSetOutputs(stream_outputs)
    , mScalarInputs(scalar_inputs)
    , mScalarOutputs(scalar_outputs)
    , mInternalScalars(internal_scalars) {

    }
    
protected:

    llvm::Value *                           mKernelInstance;
    llvm::Module *                          mModule;
    llvm::StructType *                      mKernelStateType;
    unsigned                                mLookAheadPositions;
    const std::string                       mKernelName;
    std::vector<llvm::Value *>              mInitialArguments;
    std::vector<Binding>                    mStreamSetInputs;
    std::vector<Binding>                    mStreamSetOutputs;
    std::vector<Binding>                    mScalarInputs;
    std::vector<Binding>                    mScalarOutputs;
    std::vector<Binding>                    mInternalScalars;
};

#endif 
