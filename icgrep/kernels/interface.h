/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_INTERFACE_H
#define KERNEL_INTERFACE_H

#include <kernels/processing_rate.h>
#include <kernels/attributes.h>
#include <memory>
#include <string>
#include <vector>

namespace IDISA { class IDISA_Builder; }
namespace kernel { class Kernel; }
namespace kernel { class KernelBuilder; }
namespace llvm { class CallInst; }
namespace llvm { class Function; }
namespace llvm { class Value; }
namespace llvm { class Module; }
namespace llvm { class StructType; }
namespace llvm { class Type; }

namespace kernel {

struct Binding {

    friend class KernelInterface;

    Binding(llvm::Type * type, const std::string & name, ProcessingRate r = FixedRate(1))
    : type(type), name(name), rate(r), attributes() { }


    Binding(llvm::Type * type, const std::string & name, ProcessingRate r, Attribute && attribute)
    : type(type), name(name), rate(r), attributes({std::move(attribute)}) { }


    Binding(llvm::Type * type, const std::string & name, ProcessingRate r, std::initializer_list<Attribute> attributes)
    : type(type), name(name), rate(r), attributes(attributes) { }

    llvm::Type * getType() const {
        return type;
    }

    const std::string & getName() const {
        return name;
    }

    const ProcessingRate & getRate() const {
        return rate;
    }

    const Attribute & getAttribute(const unsigned i) const {
        return attributes[i];
    }

    const std::vector<Attribute> & getAttributes() const {
        return attributes;
    }

    void addAttribute(Attribute attribute);

    bool hasAttributes() const {
        return !attributes.empty();
    }

private:
    llvm::Type * const          type;
    const std::string           name;
    ProcessingRate              rate;
    std::vector<Attribute>      attributes;
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

    unsigned getNumOfStreamInputs() const {
        return mStreamSetInputs.size();
    }

    const std::vector<Binding> & getStreamOutputs() const {
        return mStreamSetOutputs;
    }

    unsigned getNumOfStreamOutputs() const {
        return mStreamSetOutputs.size();
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

    void setInstance(llvm::Value * const instance);

    bool hasPrincipleItemCount() const {
        return mHasPrincipleItemCount;
    }

    unsigned getLookAhead(const unsigned i) const {
        return 0;
    }

    void setLookAhead(const unsigned i, const unsigned lookAheadPositions) {

    }

protected:

    llvm::Function * getInitFunction(llvm::Module * const module) const;

    llvm::Function * getDoSegmentFunction(llvm::Module * const module) const;

    llvm::Function * getTerminateFunction(llvm::Module * const module) const;

    llvm::CallInst * makeDoSegmentCall(KernelBuilder & idb, const std::vector<llvm::Value *> & args) const;

    KernelInterface(const std::string && kernelName,
                    std::vector<Binding> && stream_inputs,
                    std::vector<Binding> && stream_outputs,
                    std::vector<Binding> && scalar_inputs,
                    std::vector<Binding> && scalar_outputs,
                    std::vector<Binding> && internal_scalars)
    : mKernelInstance(nullptr)
    , mModule(nullptr)
    , mKernelStateType(nullptr)
    , mHasPrincipleItemCount(false)
    , mKernelName(kernelName)
    , mStreamSetInputs(stream_inputs)
    , mStreamSetOutputs(stream_outputs)
    , mScalarInputs(scalar_inputs)
    , mScalarOutputs(scalar_outputs)
    , mInternalScalars(internal_scalars) {
        normalizeStreamProcessingRates();
    }
    
private:

    void normalizeStreamProcessingRates();

protected:

    llvm::Value *                           mKernelInstance;
    llvm::Module *                          mModule;
    llvm::StructType *                      mKernelStateType;
    bool                                    mHasPrincipleItemCount;
    const std::string                       mKernelName;
    std::vector<llvm::Value *>              mInitialArguments;
    std::vector<Binding>                    mStreamSetInputs;
    std::vector<Binding>                    mStreamSetOutputs;
    std::vector<Binding>                    mScalarInputs;
    std::vector<Binding>                    mScalarOutputs;
    std::vector<Binding>                    mInternalScalars;

};

}

#endif 
