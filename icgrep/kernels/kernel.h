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
#include <IR_Gen/idisa_builder.h>
#include <boost/container/flat_map.hpp>

const std::string blockNoScalar = "blockNo";
const std::string logicalSegmentNoScalar = "logicalSegNo";
const std::string processedItemCountSuffix = "_processedItemCount";
const std::string producedItemCountSuffix = "_producedItemCount";
const std::string terminationSignal = "terminationSignal";
const std::string structPtrSuffix = "_structPtr";
const std::string blkMaskSuffix = "_blkMask";

using namespace parabix;
namespace kernel {
    
class KernelBuilder : public KernelInterface {
    using NameMap = boost::container::flat_map<std::string, unsigned>;
public:
    
    // Create a module for the kernel, including the kernel state type declaration and
    // the full implementation of all required methods.     
    //
    std::unique_ptr<Module> createKernelModule(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs);
    
    // Generate the Kernel to the current module (iBuilder->getModule()).
    void generateKernel(const std::vector<StreamSetBuffer *> & inputs, const std::vector<StreamSetBuffer *> & outputs);
    
    void createInstance() override;

    Function * generateThreadFunction(const std::string & name) const;

    Value * getBlockNo(Value * self) const;
    virtual Value * getProcessedItemCount(Value * self, const std::string & ssName) const override;
    virtual Value * getProducedItemCount(Value * self, const std::string & ssName) const override;
    virtual void initializeKernelState(Value * self) const;
    Value * getTerminationSignal(Value * self) const override;
    
    inline IntegerType * getSizeTy() const {
        return getBuilder()->getSizeTy();
    }

    inline Type * getStreamTy(const unsigned FieldWidth = 1) {
        return getBuilder()->getStreamTy(FieldWidth);
    }
    
    inline Type * getStreamSetTy(const unsigned NumElements = 1, const unsigned FieldWidth = 1) {
        return getBuilder()->getStreamSetTy(NumElements, FieldWidth);
    }
    
    // Synchronization actions for executing a kernel for a particular logical segment.
    // 
    // Before the segment is processed, acquireLogicalSegmentNo must be used to load
    // the segment number of the kernel state to ensure that the previous segment is
    // complete (by checking that the acquired segment number is equal to the desired segment
    // number).
    // After all segment processing actions for the kernel are complete, and any necessary 
    // data has been extracted from the kernel for further pipeline processing, the 
    // segment number must be incremented and stored using releaseLogicalSegmentNo.
    LoadInst * acquireLogicalSegmentNo(Value * self) const;

    void releaseLogicalSegmentNo(Value * self, Value * newFieldVal) const;

    virtual ~KernelBuilder() = 0;

protected:

    // Constructor
    KernelBuilder(IDISA::IDISA_Builder * builder,
                    std::string kernelName,
                    std::vector<Binding> stream_inputs,
                    std::vector<Binding> stream_outputs,
                    std::vector<Binding> scalar_parameters,
                    std::vector<Binding> scalar_outputs,
                    std::vector<Binding> internal_scalars);

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
    virtual void generateDoBlockMethod() const = 0;

    virtual void generateDoBlockLogic(Value * self, Value * blockNo) const;

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.
    
    virtual void generateFinalBlockMethod() const;

    virtual void generateDoSegmentMethod() const;
    
    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    unsigned addScalar(Type * type, const std::string & name);

    unsigned getScalarCount() const;

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.
    
    // Get the index of a named scalar field within the kernel state struct.
    ConstantInt * getScalarIndex(const std::string & name) const;
    
    // Get the value of a scalar field for a given instance.
    Value * getScalarField(Value * self, const std::string & fieldName) const;

    // Set the value of a scalar field for a given instance.
    void setScalarField(Value * self, const std::string & fieldName, Value * newFieldVal) const;
    
    // Get a parameter by name.
    Value * getParameter(Function * f, const std::string & paramName) const;

    Value * getStream(Value * self, const std::string & name, Value * blockNo, const unsigned index) {
        return getStream(self, name, blockNo, iBuilder->getInt32(index));
    }

    Value * getStream(Value * self, const std::string & name, Value * blockNo, Value * index);
    
    // Stream set helpers.
    unsigned getStreamSetIndex(const std::string & name) const;
    
    Value * getScalarFieldPtr(Value * self, const std::string & name) const;

    Value * getStreamSetStructPtr(Value * self, const std::string & name) const;

    size_t getStreamSetBufferSize(Value * self, const std::string & name) const;

    Value * getStreamSetBlockPtr(Value * self, const std::string & name, Value * blockNo) const;
    
    void setBlockNo(Value * self, Value * newFieldVal) const;

    virtual void setProcessedItemCount(Value * self, const std::string & ssName, Value * newFieldVal) const;

    virtual void setProducedItemCount(Value * self, const std::string & ssName, Value * newFieldVal) const;

    void setTerminationSignal(Value * self) const;

protected:

    std::vector<Type *>             mKernelFields;
    NameMap                         mKernelMap;
    NameMap                         mStreamSetNameMap;
    std::vector<StreamSetBuffer *>  mStreamSetInputBuffers;
    std::vector<StreamSetBuffer *>  mStreamSetOutputBuffers;

};
}
#endif 
