/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#ifndef KERNEL_H
#define KERNEL_H

#include "interface.h"
#include <boost/container/flat_map.hpp>

namespace llvm { class BasicBlock; }
namespace llvm { class Function; }
namespace llvm { class IntegerType; }
namespace llvm { class IndirectBrInst; }
namespace llvm { class PHINode; }
namespace llvm { class LoadInst; }
namespace llvm { class Type; }
namespace llvm { class Value; }
namespace parabix { class StreamSetBuffer; }

namespace kernel {
    
class KernelBuilder;

class Kernel : public KernelInterface {
    friend class KernelBuilder;
public:
    enum class Port { Input, Output };

    using StreamPort = std::pair<Port, unsigned>;

protected:

    using KernelMap = boost::container::flat_map<std::string, unsigned>;
    using StreamMap = boost::container::flat_map<std::string, StreamPort>;
    using StreamSetBuffers = std::vector<parabix::StreamSetBuffer *>;
    using Kernels = std::vector<Kernel *>;

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
    static const std::string CYCLECOUNT_SCALAR;

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
    // If no other mechanism is available, the default makeSignature() method uses the
    // full LLVM IR (before optimization) of the kernel instance.
    //
    // A kernel Module ID is short string that is used as a name for a particular kernel
    // instance.  Kernel Module IDs are used to look up and retrieve cached kernel
    // instances and so should be highly likely to uniquely identify a kernel instance.
    //
    // The ideal case is that a kernel Module ID serves as a full kernel signature thus
    // guaranteeing uniqueness.  In this case, hasSignature() should return false.
    //

    //
    // Kernel builder subtypes define their logic of kernel construction
    // in terms of 3 virtual methods for
    // (a) preparing the Kernel state data structure
    // (c) defining the logic of the finalBlock function.
    //
    // Note: the kernel state data structure must only be finalized after
    // all scalar fields have been added.   If there are no fields to
    // be added, the default method for preparing kernel state may be used.

       
    bool isCachable() const override { return false; }

    std::string makeSignature(const std::unique_ptr<KernelBuilder> & idb) override;

    // Can the module ID itself serve as the unique signature?
    virtual bool hasSignature() const { return true; }

    // Create a module stub for the kernel, populated only with its Module ID.     
    //

    void bindPorts(const StreamSetBuffers & inputs, const StreamSetBuffers & outputs);

    StreamPort getStreamPort(const std::string & name) const;

    llvm::Module * setModule(llvm::Module * const module);

    llvm::Module * makeModule(const std::unique_ptr<kernel::KernelBuilder> & idb);

    llvm::Module * getModule() const {
        return mModule;
    }

    void generateKernel(const std::unique_ptr<kernel::KernelBuilder> & idb);
    
    llvm::Value * createInstance(const std::unique_ptr<kernel::KernelBuilder> & idb) final;

    void initializeInstance(const std::unique_ptr<KernelBuilder> & idb) final;

    void finalizeInstance(const std::unique_ptr<kernel::KernelBuilder> & idb) final;

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
    
    // Kernels typically perform block-at-a-time processing, but some kernels may require
    // a different stride.   In the case of multiblock kernels, the stride attribute 
    // determines the number of minimum number of items that will be provided to the kernel
    // on each doMultiBlock call.
    // 
    
    unsigned getKernelStride() const { return mStride; }
    
    virtual ~Kernel() = 0;

    void prepareKernel(const std::unique_ptr<KernelBuilder> & idb);

    void prepareCachedKernel(const std::unique_ptr<KernelBuilder> & idb);

    std::string getCacheName(const std::unique_ptr<KernelBuilder> & idb) const;

protected:

    void setKernelStride(unsigned stride) { mStride = stride; }

    virtual void addInternalKernelProperties(const std::unique_ptr<KernelBuilder> & idb) { }

    void getDoSegmentFunctionArguments(const std::vector<llvm::Value *> & availItems) const;

    // Constructor
    Kernel(std::string && kernelName,
                  std::vector<Binding> && stream_inputs,
                  std::vector<Binding> && stream_outputs,
                  std::vector<Binding> && scalar_parameters,
                  std::vector<Binding> && scalar_outputs,
                  std::vector<Binding> && internal_scalars);

    void setNoTerminateAttribute(const bool noTerminate = true) {
        mNoTerminateAttribute = noTerminate;
    }

    llvm::Value * getPrincipleItemCount() const {
        return mAvailablePrincipleItemCount;
    }

    unsigned getScalarIndex(const std::string & name) const;

    void prepareStreamSetNameMap();
    
    void linkExternalMethods(const std::unique_ptr<kernel::KernelBuilder> &) override { }

    virtual void generateInitializeMethod(const std::unique_ptr<kernel::KernelBuilder> & iBuilder) { }
    
    virtual void generateKernelMethod(const std::unique_ptr<KernelBuilder> & iBuilder) = 0;

    virtual void generateFinalizeMethod(const std::unique_ptr<KernelBuilder> & iBuilder) { }

    // Add an additional scalar field to the KernelState struct.
    // Must occur before any call to addKernelDeclarations or createKernelModule.
    unsigned addScalar(llvm::Type * type, const std::string & name);

    unsigned addUnnamedScalar(llvm::Type * type);

    void callGenerateInitializeMethod(const std::unique_ptr<KernelBuilder> & idb);

    void callGenerateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & idb);

    void callGenerateFinalizeMethod(const std::unique_ptr<KernelBuilder> & idb);


    std::pair<unsigned, unsigned> getStreamRate(const Port p, const unsigned i) const;

    const parabix::StreamSetBuffer * getInputStreamSetBuffer(const std::string & name) const {
        const auto port = getStreamPort(name);
        assert (port.first == Port::Input);
        assert (port.second < mStreamSetInputBuffers.size());
        assert (mStreamSetInputBuffers[port.second]);
        return mStreamSetInputBuffers[port.second];
    }

    const parabix::StreamSetBuffer * getOutputStreamSetBuffer(const std::string & name) const {
        const auto port = getStreamPort(name);
        assert (port.first == Port::Output);
        assert (port.second < mStreamSetOutputBuffers.size());
        assert (mStreamSetOutputBuffers[port.second]);
        return mStreamSetOutputBuffers[port.second];
    }

    const parabix::StreamSetBuffer * getAnyStreamSetBuffer(const std::string & name) const {
        unsigned index; Port port;
        std::tie(port, index) = getStreamPort(name);
        if (port == Port::Input) {
            assert (index < mStreamSetInputBuffers.size());
            assert (mStreamSetInputBuffers[index]);
            return mStreamSetInputBuffers[index];
        } else {
            assert (index < mStreamSetOutputBuffers.size());
            assert (mStreamSetOutputBuffers[index]);
            return mStreamSetOutputBuffers[index];
        }
    }

    llvm::Value * getStreamSetInputBufferPtr(const unsigned i) const {
        return mStreamSetInputBufferPtr[i];
    }

    llvm::Value * getStreamSetOutputBufferPtr(const unsigned i) const {
        return mStreamSetOutputBufferPtr[i];
    }

private:

    void addBaseKernelProperties(const std::unique_ptr<KernelBuilder> & idb);

    llvm::Value * getAvailableItemCount(const unsigned i) const {
        return mAvailableItemCount[i];
    }

protected:

    llvm::Function *                    mCurrentMethod;
    llvm::Value *                       mAvailablePrincipleItemCount;
    bool                                mNoTerminateAttribute;
    bool                                mIsGenerated;
    unsigned                            mStride;
    llvm::Value *                       mIsFinal;
    llvm::Value *                       mOutputScalarResult;


    std::vector<llvm::Value *>          mAvailableItemCount;

    std::vector<llvm::Type *>           mKernelFields;
    KernelMap                           mKernelMap;
    StreamMap                           mStreamMap;
    StreamSetBuffers                    mStreamSetInputBuffers;
    std::vector<llvm::Value *>          mStreamSetInputBufferPtr;
    StreamSetBuffers                    mStreamSetOutputBuffers;
    std::vector<llvm::Value *>          mStreamSetOutputBufferPtr;

};

class SegmentOrientedKernel : public Kernel {
protected:

    SegmentOrientedKernel(std::string && kernelName,
                          std::vector<Binding> && stream_inputs,
                          std::vector<Binding> && stream_outputs,
                          std::vector<Binding> && scalar_parameters,
                          std::vector<Binding> && scalar_outputs,
                          std::vector<Binding> && internal_scalars);
protected:

    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & b) final;

    virtual void generateDoSegmentMethod(const std::unique_ptr<KernelBuilder> & kb) = 0;

};

/*
The Multi-Block Kernel Builder
------------------------------

The Multi-Block Kernel Builder is designed to simplify the programming of
efficient kernels with possibly variable and/or nonaligned output, subject to
exact or MaxRatio processing constraints.   The following restrictions apply.

#.  The input consists of one or more stream sets, the first of which is
    known as the principal input stream set.

#.  If there is more than one input stream set, the additional stream sets
    are first classified as having either a derived processing rate or 
    a variable processing rate.   Stream sets with a derived processing rate
    have a processing rate defined with respect to the input stream set of one
    of the following types:  FixedRate, Add1 or RoundUp.    Note that stream sets
    declared without a processing rate attribute have the FixedRate(1) attribute
    by default and therefore satisfy this constraint.  All other processing rate
    types are classified as variable rate.

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
    * additional items available parameters for each additional input stream set
      that is classified as a variable rate stream set
    * pointers to linear contiguous buffer areas for each of the input stream sets, and
    * pointers to linear contiguous output buffer areas for each of the output stream sets.

    Notes:
    * if the kernel has a Lookahead dependency declared on any input stream set, then
      there will be two buffer pointers for that stream set, one for accessing stream set
      items without lookahead and one for accessing the items with lookahead.   
    * pointers are to the beginning of the block corresponding to the
      processedItemCount or producedItemCount of the given stream set.
    * the base type of each pointer is the StreamSetBlockType of that streamset

#.  The Multi-Block Kernel Builder will arrange that these input parameters may be
    processed under the following simplifying assumptions.
    * the number of itemsToDo will either be an exact multiple of the kernel stride,
      or, for processing the final block, a value less than the kernel stride
    * the input buffer of the principal stream set and all input buffers of stream sets
      with derived processing rates will be safe to access and have data available in
      accord with their processing rates based on the given number of itemsToDo
      of the principal input stream set; no further bounds checking is needed.  
    * input buffers of stream sets with MaxRatio attributes will be safe to access,
      but will only have valid data as specified by the available items parameter for
      that stream set.
    * the kernel programmer is responsible for safe access and bounds checking for any
      input stream set classified as Unknown rate.   No temporary buffers are used
      for such stream sets.
    * all output buffers will be safe to access and have space available
      for the given maximum output generation rates based on the given number
      of itemsToDo of the principal input stream set; no further bounds checking
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
      input stream set, relative to the initial block based on the processedItemCount.
    * for any output stream set declared with a Fixed or Add1 processing rate with respect
      to the principal input stream set, a GEP instruction with a single int32 index i
      will produce a pointer to the buffer position corresponding to the ith block of the
      stream set, relative to the initial block based on the producedItemCount.

#.  Upon completion of multi-block processing, the Multi-Block Kernel Builder will arrange that
    processed and produced item counts are updated for all stream sets that have exact
    processing rate attributes.   Programmers are responsible for updating the counts
    of any stream set declared with a variable attribute (MaxRatio or Unknown).

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
    virtual void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & idb, llvm::Value * const numOfStrides) = 0;

private:

    // Given a kernel subtype with an appropriate interface, the generateDoSegment
    // method of the multi-block kernel builder makes all the necessary arrangements
    // to translate doSegment calls into a minimal sequence of doMultiBlock calls.
    void generateKernelMethod(const std::unique_ptr<KernelBuilder> & kb) final;

    bool requiresCopyBack(const ProcessingRate & rate) const;

};


class BlockOrientedKernel : public MultiBlockKernel {
protected:

    void CreateDoBlockMethodCall(const std::unique_ptr<KernelBuilder> & idb);

    // Each kernel builder subtype must provide its own logic for generating
    // doBlock calls.
    virtual void generateDoBlockMethod(const std::unique_ptr<KernelBuilder> & idb) = 0;

    // Each kernel builder subtypre must also specify the logic for processing the
    // final block of stream data, if there is any special processing required
    // beyond simply calling the doBlock function.   In the case that the final block
    // processing may be trivially implemented by dispatching to the doBlock method
    // without additional preparation, the default generateFinalBlockMethod need
    // not be overridden.

    virtual void generateFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, llvm::Value * remainingItems);

    void generateMultiBlockLogic(const std::unique_ptr<KernelBuilder> & idb, llvm::Value * const numOfStrides) final;

    BlockOrientedKernel(std::string && kernelName,
                        std::vector<Binding> && stream_inputs,
                        std::vector<Binding> && stream_outputs,
                        std::vector<Binding> && scalar_parameters,
                        std::vector<Binding> && scalar_outputs,
                        std::vector<Binding> && internal_scalars);

private:

    void writeDoBlockMethod(const std::unique_ptr<KernelBuilder> & idb);

    void writeFinalBlockMethod(const std::unique_ptr<KernelBuilder> & idb, llvm::Value * remainingItems);

private:

    llvm::Function *        mDoBlockMethod;
    llvm::BasicBlock *      mStrideLoopBody;
    llvm::IndirectBrInst *  mStrideLoopBranch;
    llvm::PHINode *         mStrideLoopTarget;
};

}

#endif 
