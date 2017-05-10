/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_H
#define KERNEL_H

#include "interface.h"
#include <boost/container/flat_map.hpp>
#include <IR_Gen/idisa_builder.h>
#include <toolchain/pipeline.h>
#include <llvm/IR/Constants.h>

namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class LoadInst; }
namespace llvm { class Type; }
namespace llvm { class Value; }
namespace parabix { class StreamSetBuffer; }

namespace kernel {
    
class KernelBuilder;

class Kernel : public KernelInterface {
protected:
    using KernelMap = boost::container::flat_map<std::string, unsigned>;
    enum class Port { Input, Output };
    using StreamPort = std::pair<Port, unsigned>;
    using StreamMap = boost::container::flat_map<std::string, StreamPort>;
    using StreamSetBuffers = std::vector<parabix::StreamSetBuffer *>;
    using Kernels = std::vector<Kernel *>;

    friend class KernelBuilder;
    friend void ::generateSegmentParallelPipeline(const std::unique_ptr<kernel::KernelBuilder> &, const Kernels &);
    friend void ::generatePipelineLoop(const std::unique_ptr<kernel::KernelBuilder> &, const Kernels &);
    friend void ::generateParallelPipeline(const std::unique_ptr<kernel::KernelBuilder> &, const Kernels &);

    static const std::string DO_BLOCK_SUFFIX;
    static const std::string FINAL_BLOCK_SUFFIX;
    static const std::string MULTI_BLOCK_SUFFIX;
    static const std::string LOGICAL_SEGMENT_NO_SCALAR;
    static const std::string PROCESSED_ITEM_COUNT_SUFFIX;
    static const std::string CONSUMED_ITEM_COUNT_SUFFIX;
    static const std::string PRODUCED_ITEM_COUNT_SUFFIX;
    static const std::string TERMINATION_SIGNAL;
    static const std::string BUFFER_PTR_SUFFIX;
    static const std::string CONSUMER_SUFFIX;

public:
    
    // Kernel Signatures and Module IDs
    //
    // A kernel signature uniquely identifies a kernel and its full functionality.
    // In the event that a particular kernel instance is to be generated and compiled
    // to produce object code, and we have a cached kernel object code instance with 
    // the same signature and targetting the same IDISA architecture, then the cached 
    // object code may safely be used to avoid recompilation.
    //
    // A kernel signature is a byte string of arbitrary length.
    //
    // Kernel developers should take responsibility for designing appropriate signature
    // mechanisms that are short, inexpensive to compute and guarantee uniqueness
    // based on the semantics of the kernel.  
    //
    // If no other mechanism is available, the default generateKernelSignature() method
    // uses the full LLVM IR (before optimization) of the kernel instance.
    //
    // A kernel Module ID is short string that is used as a name for a particular kernel
    // instance.  Kernel Module IDs are used to look up and retrieve cached kernel instances
    // and so should be highly likely to uniquely identify a kernel instance.
    //
    // The ideal case is that a kernel Module ID serves as a full kernel signature thus
    // guaranteeing uniqueness.  In this case, the moduleIDisUnique() method 
    // should return true.
    //
       
    bool isCachable() const override { return false; }

    std::string makeSignature() override;

    // Can the module ID itself serve as the unique signature?
    virtual bool moduleIDisSignature() const { return false; }

    // Create a module stub for the kernel, populated only with its Module ID.     
    //

    void createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs);

    void createKernelStub(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs, llvm::Module * const kernelModule);

    llvm::Module * getModule() const {
        return mModule;
    }

    // Generate the Kernel to the current module (iBuilder->getModule()).
    void generateKernel();
    
    llvm::Value * createInstance() final;

    void initializeInstance() final;

    void finalizeInstance() final;

    llvm::Value * getProducedItemCount(const std::string & name, llvm::Value * doFinal = nullptr) const final;

    void setProducedItemCount(const std::string & name, llvm::Value * value) const final;

    llvm::Value * getProcessedItemCount(const std::string & name) const final;

    void setProcessedItemCount(const std::string & name, llvm::Value * value) const final;

    llvm::Value * getConsumedItemCount(const std::string & name) const final;

    void setConsumedItemCount(const std::string & name, llvm::Value * value) const final;

    llvm::Value * getTerminationSignal() const final;

    void setTerminationSignal() const final;

    // Get the value of a scalar field for the current instance.
    llvm::Value * getScalarFieldPtr(llvm::Value * index) const;

    llvm::Value * getScalarFieldPtr(const std::string & fieldName) const;

    llvm::Value * getScalarField(const std::string & fieldName) const;

    // Set the value of a scalar field for the current instance.
    void setScalarField(const std::string & fieldName, llvm::Value * value) const;

    // Synchronization actions for executing a kernel for a particular logical segment.
    //
    // Before the segment is processed, acquireLogicalSegmentNo must be used to load
    // the segment number of the kernel state to ensure that the previous segment is
    // complete (by checking that the acquired segment number is equal to the desired segment
    // number).
    // After all segment processing actions for the kernel are complete, and any necessary
    // data has been extracted from the kernel for further pipeline processing, the
    // segment number must be incremented and stored using releaseLogicalSegmentNo.
    llvm::LoadInst * acquireLogicalSegmentNo() const;

    void releaseLogicalSegmentNo(llvm::Value * nextSegNo) const;

    bool hasNoTerminateAttribute() const {
        return mNoTerminateAttribute;
    }

    const StreamSetBuffers & getStreamSetInputBuffers() const {
        return mStreamSetInputBuffers;
    }

    const parabix::StreamSetBuffer * getStreamSetInputBuffer(const unsigned i) const {
        return mStreamSetInputBuffers[i];
    }

    const StreamSetBuffers & getStreamSetOutputBuffers() const {
        return mStreamSetOutputBuffers;
    }

    const parabix::StreamSetBuffer * getStreamSetOutputBuffer(const unsigned i) const {
        return mStreamSetOutputBuffers[i];
    }

    llvm::CallInst * createDoSegmentCall(const std::vector<llvm::Value *> & args) const;

    llvm::Value * getAccumulator(const std::string & accumName) const;

    virtual ~Kernel() = 0;

protected:

    // Constructor
    Kernel(std::string && kernelName,
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

    void prepareStreamSetNameMap();

    void linkExternalMethods() override { }

    virtual void prepareKernel();

    virtual void generateInitializeMethod() { }
    
    virtual void generateDoSegmentMethod() = 0;

    virtual void generateFinalizeMethod() { }

    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    unsigned addScalar(llvm::Type * type, const std::string & name);

    unsigned addUnnamedScalar(llvm::Type * type);

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.
    
    // Get the index of a named scalar field within the kernel state struct.
    unsigned getScalarIndex(const std::string & name) const;

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) const;

    llvm::Value * loadInputStreamBlock(const std::string & name, llvm::Value * streamIndex) const;
    
    llvm::Value * getInputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) const;
    
    llvm::Value * loadInputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) const;
    
    llvm::Value * getInputStreamSetCount(const std::string & name) const;

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) const;
    
    void storeOutputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * toStore) const;
    
    llvm::Value * getOutputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) const;
    
    void storeOutputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * toStore) const;

    llvm::Value * getOutputStreamSetCount(const std::string & name) const;

    llvm::Value * getAdjustedInputStreamBlockPtr(llvm::Value * blockAdjustment, const std::string & name, llvm::Value * streamIndex) const;

    llvm::Value * getRawInputPointer(const std::string & name, llvm::Value * streamIndex, llvm::Value * absolutePosition) const;

    llvm::Value * getRawOutputPointer(const std::string & name, llvm::Value * streamIndex, llvm::Value * absolutePosition) const;

    llvm::Value * getBaseAddress(const std::string & name) const;

    void setBaseAddress(const std::string & name, llvm::Value * addr) const;

    llvm::Value * getBufferedSize(const std::string & name) const;

    void setBufferedSize(const std::string & name, llvm::Value * size) const;

    void reserveBytes(const std::string & name, llvm::Value * requested) const;

    llvm::Value * getAvailableItemCount(const std::string & name) const;

    llvm::Value * getLinearlyAccessibleItems(const std::string & name, llvm::Value * fromPosition) const;

    llvm::BasicBlock * CreateWaitForConsumers() const;

    llvm::BasicBlock * CreateBasicBlock(std::string && name) const;

    llvm::Value * getStreamSetBufferPtr(const std::string & name) const;

    llvm::Value * getIsFinal() const {
        return mIsFinal;
    }

    void callGenerateInitializeMethod();

    void callGenerateDoSegmentMethod();

    void callGenerateFinalizeMethod();

    StreamPort getStreamPort(const std::string & name) const;

    const parabix::StreamSetBuffer * getInputStreamSetBuffer(const std::string & name) const {
        const auto port = getStreamPort(name);
        assert (port.first == Port::Input);
        assert (port.second < mStreamSetInputBuffers.size());
        return mStreamSetInputBuffers[port.second];
    }

    const parabix::StreamSetBuffer * getOutputStreamSetBuffer(const std::string & name) const {
        const auto port = getStreamPort(name);
        assert (port.first == Port::Output);
        assert (port.second < mStreamSetOutputBuffers.size());
        return mStreamSetOutputBuffers[port.second];
    }

    const parabix::StreamSetBuffer * getAnyStreamSetBuffer(const std::string & name) const {
        unsigned index; Port port;
        std::tie(port, index) = getStreamPort(name);
        if (port == Port::Input) {
            assert (index < mStreamSetInputBuffers.size());
            return mStreamSetInputBuffers[index];
        } else {
            assert (index < mStreamSetOutputBuffers.size());
            return mStreamSetOutputBuffers[index];
        }
    }

private:

    llvm::Value * getConsumerLock(const std::string & name) const;

    void setConsumerLock(const std::string & name, llvm::Value * value) const;

    llvm::Value * computeBlockIndex(const std::vector<Binding> & binding, const std::string & name, llvm::Value * itemCount) const;

protected:

    llvm::Function *                    mCurrentMethod;
    bool                                mNoTerminateAttribute;
    bool                                mIsGenerated;

    llvm::Value *                       mIsFinal;
    std::vector<llvm::Value *>          mAvailableItemCount;
    llvm::Value *                       mOutputScalarResult;


    std::vector<llvm::Type *>           mKernelFields;
    KernelMap                           mKernelMap;
    StreamMap                           mStreamMap;
    StreamSetBuffers                    mStreamSetInputBuffers;
    StreamSetBuffers                    mStreamSetOutputBuffers;

};

class SegmentOrientedKernel : public Kernel {
protected:

    SegmentOrientedKernel(std::string && kernelName,
                          std::vector<Binding> && stream_inputs,
                          std::vector<Binding> && stream_outputs,
                          std::vector<Binding> && scalar_parameters,
                          std::vector<Binding> && scalar_outputs,
                          std::vector<Binding> && internal_scalars);

};

class BlockOrientedKernel : public Kernel {
protected:

    void CreateDoBlockMethodCall();

    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod() = 0;

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.

    virtual void generateFinalBlockMethod(llvm::Value * remainingItems);

    void generateDoSegmentMethod() override final;

    BlockOrientedKernel(std::string && kernelName,
                        std::vector<Binding> && stream_inputs,
                        std::vector<Binding> && stream_outputs,
                        std::vector<Binding> && scalar_parameters,
                        std::vector<Binding> && scalar_outputs,
                        std::vector<Binding> && internal_scalars);

private:

    virtual bool useIndirectBr() const;

    void writeDoBlockMethod();

    void writeFinalBlockMethod(llvm::Value * remainingItems);

private:

    llvm::Function *        mDoBlockMethod;
    llvm::BasicBlock *      mStrideLoopBody;
    llvm::IndirectBrInst *  mStrideLoopBranch;
    llvm::PHINode *         mStrideLoopTarget;
};

/*    
The Multi-Block Kernel Builder
------------------------------

The Multi-Block Kernel Builder is designed to simplify the programming of
efficient kernels with possibly variable and/or nonaligned output, subject to
exact or MaxRatio processing constraints.   The following restrictions apply.
    
#.  The input consists of one or more stream sets, the first of which is
    known as the principal input stream set.  
    
#.  If there is more than one input stream set, the additional stream sets must
    have a processing rate defined with respect to the input stream set of one
    of the following types:  FixedRate, Add1 or RoundUp.    Note that stream sets
    declared without a processing rate attribute have the FixedRate(1) attribute
    by default and therefore satisfy this constraint.
    
#.  All output stream sets must be declared with processing rate attributes
    of one of the following types:
    *  FixedRate, Add1, Roundup, or MaxRatio with respect to the principal input stream set.
    *  FixedRate with respect to some other output stream set.
    
    When using the Multi-Block Kernel Builder to program a new type of kernel,
    the programmer must implement the generateDoMultiBlockMethod for normal
    multi-block processing according to the requirements below, as well as
    providing for special final block processing, if necessary.
            
#.  The doMultiBlockMethod will be called with the following parameters:
    * the number of items of the principal input stream to process (itemsToDo),
    * pointers to linear contiguous buffer areas for each of the input stream sets, and
    * pointers to linear contiguous output buffer areas for each of the output stream sets.
    * pointers are to the address of the first item of the first stream of the stream set.

#.  The Multi-Block Kernel Builder will arrange that these input parameters may be
    processed under the following simplifying assumptions.
    * the number of itemsToDo will either be an exact multiple of the BlockSize,
      or, for processing the final block, a value less than BlockSize
    * all input buffers will be safe to access and have data available in
      accord with their processing rates based on the given number of itemsToDo
      of the principal input stream set; no further bounds checking is needed.
    * all output buffers will be safe to access and have space available
      for the given maximum output generation rates based on the given number
      of blocksToDo of the principal input stream set; no further bounds checking
      is needed.
    * for final block processing, all input buffers will be extended to be safely
      treated as containing data corresponding to a full block of the principal
      input stream set, with the actual data in each buffer padded with null values
      beyond the end of input.  Similarly, all output buffers will contain space
      sufficient for the maximum output that can be generated for a full block of
      input processing.
    * input and output pointers will be typed to allow convenient and logical access
      to corresponding streams based on their declared stream set type and processing rate.
    * for any input pointer p, a GEP instruction with a single int32 index i
      will produce a pointer to the buffer position corresponding to the ith block of the
      principal input stream set.  
    * for any output stream set declared with a Fixed or Add1 processing rate with respect
      to the principal input stream set, a GEP instruction with a single int32 index i
      will produce a pointer to the buffer position corresponding to the ith block of the
      principal input stream set.
                    
#.  Upon completion of multi-block processing, the Multi-Block Kernel Builder will arrange that
    processed and produced item counts are updated for all stream sets that have exact
    processing rate attributes.   Programmers are responsible for updating the producedItemCount
    of any stream set declared with a variable attribute (MaxRatio).
                            
#.  An important caveat is that buffer areas may change arbitrarily between
    calls to the doMultiBlockMethod.   In no case should a kernel store a
    buffer pointer in its internal state.   Furthermore a kernel must not make
    any assumptions about the accessibility of stream set data outside of the
    processing range outside of the block boundaries associated with the given itemsToDo.
*/

class MultiBlockKernel : public Kernel {
protected:

    MultiBlockKernel(std::string && kernelName,
                     std::vector<Binding> && stream_inputs,
                     std::vector<Binding> && stream_outputs,
                     std::vector<Binding> && scalar_parameters,
                     std::vector<Binding> && scalar_outputs,
                     std::vector<Binding> && internal_scalars);

    // Each multi-block kernel subtype must provide its own logic for handling
    // doMultiBlock calls, subject to the requirements laid out above. 
    // The generateMultiBlockLogic must be written to generate this logic, given
    // a created but empty function.  Upon entry to generateMultiBlockLogic,
    // the builder insertion point will be set to the entry block; upone
    // exit the RetVoid instruction will be added to complete the method.
    // 
    virtual void generateMultiBlockLogic () = 0;

    // Given a kernel subtype with an appropriate interface, the generateDoSegment
    // method of the multi-block kernel builder makes all the necessary arrangements
    // to translate doSegment calls into a minimal sequence of doMultiBlock calls.
    void generateDoSegmentMethod() override final;
};
    
    
}
#endif 
