/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <string>           // for string
#include <memory>           // for unique_ptr
#include "interface.h"      // for KernelInterface
#include <boost/container/flat_map.hpp>
#include <IR_Gen/idisa_builder.h>
namespace llvm { class ConstantInt; }
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class LoadInst; }
namespace llvm { class Type; }
namespace llvm { class Value; }
namespace parabix { class StreamSetBuffer; }

namespace kernel {
    
class KernelBuilder : public KernelInterface {
    using NameMap = boost::container::flat_map<std::string, unsigned>;
public:
    
    // Create a module for the kernel, including the kernel state type declaration and
    // the full implementation of all required methods.     
    //
    std::unique_ptr<llvm::Module> createKernelModule(const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);
    
    // Generate the Kernel to the current module (iBuilder->getModule()).
    void generateKernel(const std::vector<parabix::StreamSetBuffer *> & inputs, const std::vector<parabix::StreamSetBuffer *> & outputs);
    
    void createInstance() override;

    virtual llvm::Value * getProcessedItemCount(llvm::Value * instance, const std::string & name) const override;

    virtual void setProcessedItemCount(llvm::Value * instance, const std::string & name, llvm::Value * value) const;

    virtual llvm::Value * getProducedItemCount(llvm::Value * instance, const std::string & name) const override;

    virtual void setProducedItemCount(llvm::Value * instance, const std::string & name, llvm::Value * value) const;

    bool hasNoTerminateAttribute() { return mNoTerminateAttribute;}
    
    llvm::Value * getTerminationSignal(llvm::Value * instance) const override final;

    void setTerminationSignal(llvm::Value * instance) const override final;

    llvm::Value * getScalarFieldPtr(llvm::Value * instance, const std::string & name) const;

    llvm::Value * getScalarFieldPtr(llvm::Value *instance, llvm::Value * index) const;

    llvm::Value * getStreamSetBufferPtr(llvm::Value * instance, const std::string & name) const;

    llvm::Value * getStreamSetBufferPtr(llvm::Value * instance, llvm::Value * index) const;
    
    // Get the value of a scalar field for a given instance.
    llvm::Value * getScalarField(llvm::Value * instance, const std::string & fieldName) const;

    llvm::Value * getScalarField(llvm::Value * instance, llvm::Value * index) const;

    // Set the value of a scalar field for a given instance.
    void setScalarField(llvm::Value *instance, const std::string & fieldName, llvm::Value * value) const;

    void setScalarField(llvm::Value * instance, llvm::Value * index, llvm::Value * value) const;

    // Synchronization actions for executing a kernel for a particular logical segment.
    //
    // Before the segment is processed, acquireLogicalSegmentNo must be used to load
    // the segment number of the kernel state to ensure that the previous segment is
    // complete (by checking that the acquired segment number is equal to the desired segment
    // number).
    // After all segment processing actions for the kernel are complete, and any necessary
    // data has been extracted from the kernel for further pipeline processing, the
    // segment number must be incremented and stored using releaseLogicalSegmentNo.
    llvm::LoadInst * acquireLogicalSegmentNo(llvm::Value * instance) const;

    void releaseLogicalSegmentNo(llvm::Value * instance, llvm::Value * newFieldVal) const;

    // Get a parameter by name.
    llvm::Argument * getParameter(llvm::Function * f, const std::string & name) const;

    inline llvm::IntegerType * getSizeTy() const {
        return getBuilder()->getSizeTy();
    }

    inline llvm::Type * getStreamTy(const unsigned FieldWidth = 1) {
        return getBuilder()->getStreamTy(FieldWidth);
    }
    
    inline llvm::Type * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return getBuilder()->getStreamSetTy(NumElements, FieldWidth);
    }
    
    virtual ~KernelBuilder() = 0;
    
    const std::vector<const parabix::StreamSetBuffer *> & getStreamSetInputBuffers() const { return mStreamSetInputBuffers; }

    const std::vector<const parabix::StreamSetBuffer *> & getStreamSetOutputBuffers() const { return mStreamSetOutputBuffers; }

    llvm::Value * createDoSegmentCall(const std::vector<llvm::Value *> & args) const;

    llvm::Value * createGetAccumulatorCall(llvm::Value * self, const std::string & accumName) const;

protected:

    // Constructor
    KernelBuilder(IDISA::IDISA_Builder * builder,
                    std::string && kernelName,
                    std::vector<Binding> && stream_inputs,
                    std::vector<Binding> && stream_outputs,
                    std::vector<Binding> && scalar_parameters,
                    std::vector<Binding> && scalar_outputs,
                    std::vector<Binding> && internal_scalars);

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
    
    void setNoTerminateAttribute(const bool noTerminate = true) {
        mNoTerminateAttribute = noTerminate;
    }

    void setDoBlockUpdatesProducedItemCountsAttribute(const bool doesUpdate = true) {
        mDoBlockUpdatesProducedItemCountsAttribute = doesUpdate;
    }
    
    virtual void prepareKernel();
       
    virtual void generateInitMethod() { }
    
    virtual void generateDoSegmentMethod(llvm::Value * doFinal, const std::vector<llvm::Value *> & producerPos) = 0;

    virtual void generateInternalMethods() { }

    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    unsigned addScalar(llvm::Type * type, const std::string & name);

    unsigned addUnnamedScalar(llvm::Type * type);

    unsigned getScalarCount() const;

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.
    
    // Get the index of a named scalar field within the kernel state struct.
    llvm::ConstantInt * getScalarIndex(const std::string & name) const;

    llvm::Value * getBlockNo() const;

    void setBlockNo(llvm::Value * value) const;
    
    // Get the value of a scalar field for a given instance.
    llvm::Value * getScalarField(const std::string & fieldName) const {
        return getScalarField(getSelf(), fieldName);
    }

    llvm::Value * getScalarField(llvm::Value * index) const {
        return getScalarField(getSelf(), index);
    }

    // Set the value of a scalar field for a given instance.
    void setScalarField(const std::string & fieldName, llvm::Value * value) const {
        return setScalarField(getSelf(), fieldName, value);
    }

    void setScalarField(llvm::Value * index, llvm::Value * value) const {
        return setScalarField(getSelf(), index, value);
    }

    llvm::Value * getStream(const std::string & name, llvm::Value * blockNo, llvm::Value * index) const;

    llvm::Value * getStream(const std::string & name, llvm::Value * blockNo, llvm::Value * index1, llvm::Value * index2) const;

    llvm::Value * getStreamView(const std::string & name, llvm::Value * blockNo, llvm::Value * index) const;

    llvm::Value * getStreamView(llvm::Type * type, const std::string & name, llvm::Value * blockNo, llvm::Value * index) const;

    // Stream set helpers.
    llvm::Value * getScalarFieldPtr(const std::string & name) const {
        return getScalarFieldPtr(getSelf(), name);
    }

    llvm::Value * getScalarFieldPtr(llvm::Value * index) const {
        return getScalarFieldPtr(getSelf(), index);
    }

    llvm::Value * getStreamSetBufferPtr(const std::string & name) const {
        return getStreamSetBufferPtr(getSelf(), name);
    }

    llvm::Value * getStreamSetBufferPtr(llvm::Value * index) const {
        return getStreamSetBufferPtr(getSelf(), index);
    }

    llvm::Value * getStreamSetPtr(const std::string & name, llvm::Value * blockNo) const;

    inline llvm::Value * getProcessedItemCount(const std::string & name) const {
        return getProcessedItemCount(getSelf(), name);
    }

    inline void setProcessedItemCount(const std::string & name, llvm::Value * value) const {
        setProcessedItemCount(getSelf(), name, value);
    }

    inline llvm::Value * getProducedItemCount(const std::string & name) const {
        return getProducedItemCount(getSelf(), name);
    }

    inline void setProducedItemCount(const std::string & name, llvm::Value * value) const {
        setProducedItemCount(getSelf(), name, value);
    }

    llvm::Value * getTerminationSignal() const {
        return getTerminationSignal(getSelf());
    }

    void setTerminationSignal() const {
        return setTerminationSignal(getSelf());
    }


    llvm::Value * getSelf() const {
        return mSelf;
    }

    llvm::BasicBlock * CreateBasicBlock(std::string && name) const;

private:

    unsigned getStreamSetIndex(const std::string & name) const;

    const parabix::StreamSetBuffer * getStreamSetBuffer(const std::string & name) const;

    void callGenerateInitMethod();

    void callGenerateDoSegmentMethod();

protected:

    llvm::Value *                                   mSelf;
    llvm::Function *                                mCurrentFunction;

    std::vector<llvm::Type *>                       mKernelFields;
    NameMap                                         mKernelMap;
    NameMap                                         mStreamSetNameMap;
    std::vector<const parabix::StreamSetBuffer *>   mStreamSetInputBuffers;
    std::vector<const parabix::StreamSetBuffer *>   mStreamSetOutputBuffers;
    bool                                            mNoTerminateAttribute;
    bool                                            mDoBlockUpdatesProducedItemCountsAttribute;

};

class SegmentOrientedKernel : public KernelBuilder {
protected:

    SegmentOrientedKernel(IDISA::IDISA_Builder * builder,
                          std::string && kernelName,
                          std::vector<Binding> && stream_inputs,
                          std::vector<Binding> && stream_outputs,
                          std::vector<Binding> && scalar_parameters,
                          std::vector<Binding> && scalar_outputs,
                          std::vector<Binding> && internal_scalars);

    virtual ~SegmentOrientedKernel() { }

};

class BlockOrientedKernel : public KernelBuilder {
public:

    llvm::CallInst * CreateDoBlockMethodCall() const;

    llvm::CallInst * CreateDoFinalBlockMethodCall(llvm::Value * remainingItems) const;

protected:

    virtual void addAdditionalKernelDeclarations(llvm::Module * module, llvm::PointerType * selfType);

    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod(llvm::Value * blockNo) = 0;

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.

    virtual void generateFinalBlockMethod(llvm::Value * remainingBytes, llvm::Value * blockNo);

    virtual void generateDoSegmentMethod(llvm::Value * doFinal, const std::vector<llvm::Value *> & producerPos) final;

    void generateInternalMethods() override final;

    BlockOrientedKernel(IDISA::IDISA_Builder * builder,
                        std::string && kernelName,
                        std::vector<Binding> && stream_inputs,
                        std::vector<Binding> && stream_outputs,
                        std::vector<Binding> && scalar_parameters,
                        std::vector<Binding> && scalar_outputs,
                        std::vector<Binding> && internal_scalars);

    virtual ~BlockOrientedKernel() { }

    llvm::Function * getDoBlockFunction() const;

    llvm::Function * getDoFinalBlockFunction() const;

private:

    void callGenerateDoBlockMethod();

    void callGenerateDoFinalBlockMethod();

};


}
#endif 
