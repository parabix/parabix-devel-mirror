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

struct Binding : public AttributeSet {

    Binding(llvm::Type * type, const std::string & name, ProcessingRate r = FixedRate(1))
    : AttributeSet()
    , mType(type), mName(name), mRate(std::move(r)) { }


    Binding(llvm::Type * type, const std::string & name, ProcessingRate r, Attribute && attribute)
    : AttributeSet(std::move(attribute))
    , mType(type), mName(name), mRate(std::move(r)) { }


    Binding(llvm::Type * type, const std::string & name, ProcessingRate r, std::initializer_list<Attribute> attributes)
    : AttributeSet(attributes)
    , mType(type), mName(name), mRate(std::move(r)) { }

    llvm::Type * getType() const {
        return mType;
    }

    const std::string & getName() const {
        return mName;
    }

    const ProcessingRate & getRate() const {
        return mRate;
    }

    ProcessingRate & getRate() {
        return mRate;
    }

    bool isPrincipal() const {
        return hasAttribute(AttributeId::Principal);
    }

    bool hasLookahead() const {
        return hasAttribute(AttributeId::LookAhead);
    }

    unsigned const getLookahead() const {
        return findAttribute(AttributeId::LookAhead).amount();
    }

    bool nonDeferred() const {
        return !hasAttribute(AttributeId::Deferred);
    }

private:
    llvm::Type * const          mType;
    const std::string           mName;
    ProcessingRate              mRate;
};

using Bindings = std::vector<Binding>;

class KernelInterface : public AttributeSet {
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
        assert (i < getNumOfStreamInputs());
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
        assert (i < getNumOfStreamOutputs());
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

    bool hasPrincipalItemCount() const {
        return mHasPrincipalItemCount;
    }

protected:

    llvm::Function * getInitFunction(llvm::Module * const module) const;

    llvm::Function * getDoSegmentFunction(llvm::Module * const module) const;

    llvm::Function * getTerminateFunction(llvm::Module * const module) const;

    llvm::CallInst * makeDoSegmentCall(KernelBuilder & idb, const std::vector<llvm::Value *> & args) const;

    KernelInterface(const std::string && kernelName,
                    Bindings && stream_inputs,
                    Bindings && stream_outputs,
                    Bindings && scalar_inputs,
                    Bindings && scalar_outputs,
                    Bindings && internal_scalars)
    : mKernelInstance(nullptr)
    , mModule(nullptr)
    , mKernelStateType(nullptr)
    , mHasPrincipalItemCount(false)
    , mKernelName(kernelName)
    , mStreamSetInputs(stream_inputs)
    , mStreamSetOutputs(stream_outputs)
    , mScalarInputs(scalar_inputs)
    , mScalarOutputs(scalar_outputs)
    , mInternalScalars(internal_scalars) {

    }
    
protected:

    llvm::Value *                   mKernelInstance;
    llvm::Module *                  mModule;
    llvm::StructType *              mKernelStateType;
    bool                            mHasPrincipalItemCount;
    const std::string               mKernelName;
    std::vector<llvm::Value *>      mInitialArguments;
    Bindings                        mStreamSetInputs;
    Bindings                        mStreamSetOutputs;
    Bindings                        mScalarInputs;
    Bindings                        mScalarOutputs;
    Bindings                        mInternalScalars;
};

}

#endif 
