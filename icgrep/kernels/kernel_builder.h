#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <IR_Gen/idisa_builder.h>
#include <kernels/kernel.h>

namespace kernel {

class Kernel;

class KernelBuilder : public virtual IDISA::IDISA_Builder {
    friend class Kernel;
public:

    // Get the value of a scalar field for the current instance.
    llvm::Value * getScalarFieldPtr(llvm::Value * index);

    llvm::Value * getScalarFieldPtr(const std::string & fieldName);

    llvm::Value * getScalarField(const std::string & fieldName);

    // Set the value of a scalar field for the current instance.
    void setScalarField(const std::string & fieldName, llvm::Value * value);

    // Synchronization actions for executing a kernel for a particular logical segment.
    //
    // Before the segment is processed, acquireLogicalSegmentNo must be used to load
    // the segment number of the kernel state to ensure that the previous segment is
    // complete (by checking that the acquired segment number is equal to the desired segment
    // number).
    // After all segment processing actions for the kernel are complete, and any necessary
    // data has been extracted from the kernel for further pipeline processing, the
    // segment number must be incremented and stored using releaseLogicalSegmentNo.
    llvm::LoadInst * acquireLogicalSegmentNo();

    void releaseLogicalSegmentNo(llvm::Value * nextSegNo);

    llvm::Value * getProducedItemCount(const std::string & name) {
        return getInternalItemCount(name, Kernel::PRODUCED_ITEM_COUNT_SUFFIX);
    }

    void setProducedItemCount(const std::string & name, llvm::Value * value) {
        setInternalItemCount(name, Kernel::PRODUCED_ITEM_COUNT_SUFFIX, value);
    }

    llvm::Value * getProcessedItemCount(const std::string & name) {        
        return getInternalItemCount(name, Kernel::PROCESSED_ITEM_COUNT_SUFFIX);
    }

    void setProcessedItemCount(const std::string & name, llvm::Value * value) {
        setInternalItemCount(name, Kernel::PROCESSED_ITEM_COUNT_SUFFIX, value);
    }

    llvm::Value * getConsumedItemCount(const std::string & name) {
        return getInternalItemCount(name, Kernel::CONSUMED_ITEM_COUNT_SUFFIX);
    }

    void setConsumedItemCount(const std::string & name, llvm::Value * value) {
        setInternalItemCount(name, Kernel::CONSUMED_ITEM_COUNT_SUFFIX, value);
    }

    llvm::Value * getTerminationSignal();

    void setTerminationSignal() { setTerminationSignal(getTrue()); }

    void setTerminationSignal(llvm::Value * const value);

    llvm::Value * getCycleCountPtr();

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) {
        return getInputStreamBlockPtr(name, streamIndex, getInt32(0));
    }

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * loadInputStreamBlock(const std::string & name, llvm::Value * streamIndex);

    llvm::Value * getInputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    llvm::Value * loadInputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    llvm::Value * getInputStreamSetCount(const std::string & name);

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) {
        return getOutputStreamBlockPtr(name, streamIndex, getInt32(0));
    }

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::StoreInst * storeOutputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * toStore);

    llvm::Value * getOutputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    llvm::StoreInst * storeOutputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * toStore);

    llvm::Value * getOutputStreamSetCount(const std::string & name);

    llvm::Value * getRawInputPointer(const std::string & name, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const std::string & name, llvm::Value * absolutePosition);

    llvm::Value * getBaseAddress(const std::string & name);

    llvm::Value * getBlockAddress(const std::string & name, llvm::Value * const blockIndex);

    void setBaseAddress(const std::string & name, llvm::Value * addr);

    llvm::Value * getBufferedSize(const std::string & name);
    
    void setBufferedSize(const std::string & name, llvm::Value * size);
    
    llvm::Value * getCapacity(const std::string & name);
    
    void setCapacity(const std::string & name, llvm::Value * c);
    
    llvm::Value * getAvailableItemCount(const std::string & name);

    llvm::Value * getLinearlyAccessibleItems(const std::string & name, llvm::Value * fromPos, llvm::Value * avail, bool reverse = false);
    
    llvm::Value * getLinearlyWritableItems(const std::string & name, llvm::Value * fromPos, bool reverse = false);
    
    void CreateStreamCpy(const std::string & name, llvm::Value * const target, llvm::Value * const targetOffset, llvm::Value * const source, llvm::Value * const sourceOffset, llvm::Value * const itemsToCopy, const unsigned itemAlignment);    

    llvm::BasicBlock * CreateConsumerWait();

    llvm::Value * getStreamHandle(const std::string & name);

    llvm::CallInst * createDoSegmentCall(const std::vector<llvm::Value *> & args);

    llvm::Value * getAccumulator(const std::string & accumName);

    llvm::Value * getConsumerLock(const std::string & name);

    void setConsumerLock(const std::string & name, llvm::Value * value);

    const Kernel * getKernel() const {
        return mKernel;
    }

    void setKernel(const Kernel * const kernel) {
        mKernel = kernel;
    }

    void protectOutputStream(const std::string & name, const bool readOnly);

    void doubleCapacity(const std::string & name);

protected:

    KernelBuilder(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
    : IDISA::IDISA_Builder(C, vectorWidth, stride)
    , mKernel(nullptr) {

    }

    llvm::Value * getScalarFieldPtr(llvm::Value * instance, llvm::Value * index);

    llvm::Value * getScalarFieldPtr(llvm::Value * instance, const std::string & fieldName);

    llvm::Value * getInternalItemCount(const std::string & name, const std::string & suffix);

    void setInternalItemCount(const std::string & name, const std::string & suffix, llvm::Value * const value);

protected:
    const Kernel * mKernel;
};

template <class SpecifiedArchitectureBuilder>
class KernelBuilderImpl final : public KernelBuilder, public SpecifiedArchitectureBuilder {
public:
    KernelBuilderImpl(llvm::LLVMContext & C, unsigned vectorWidth, unsigned stride)
    : IDISA::IDISA_Builder(C, vectorWidth, stride)
    , KernelBuilder(C, vectorWidth, stride)
    , SpecifiedArchitectureBuilder(C, vectorWidth, stride) {

    }
};

}

#endif // KERNEL_BUILDER_H
