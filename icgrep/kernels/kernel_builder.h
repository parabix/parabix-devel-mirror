#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <kernels/interface.h>
#include <IR_Gen/idisa_builder.h>

namespace kernel {

class Kernel;

class KernelBuilder : public virtual IDISA::IDISA_Builder {
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

    llvm::Value * getProducedItemCount(const std::string & name, llvm::Value * doFinal = nullptr);

    void setProducedItemCount(const std::string & name, llvm::Value * value);

    llvm::Value * getProcessedItemCount(const std::string & name);

    void setProcessedItemCount(const std::string & name, llvm::Value * value);

    llvm::Value * getConsumedItemCount(const std::string & name);

    void setConsumedItemCount(const std::string & name, llvm::Value * value);

    llvm::Value * getTerminationSignal();

    void setTerminationSignal();

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.

    // Get the index of a named scalar field within the kernel state struct.
    llvm::ConstantInt * getScalarIndex(const std::string & name);

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex);

    llvm::Value * loadInputStreamBlock(const std::string & name, llvm::Value * streamIndex);

    llvm::Value * getInputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    llvm::Value * loadInputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    llvm::Value * getInputStreamSetCount(const std::string & name);

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex);

    void storeOutputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * toStore);

    llvm::Value * getOutputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex);

    void storeOutputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * toStore);

    llvm::Value * getOutputStreamSetCount(const std::string & name);

    llvm::Value * getAdjustedInputStreamBlockPtr(llvm::Value * blockAdjustment, const std::string & name, llvm::Value * streamIndex);

    llvm::Value * getRawInputPointer(const std::string & name, llvm::Value * streamIndex, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const std::string & name, llvm::Value * streamIndex, llvm::Value * absolutePosition);

    llvm::Value * getBaseAddress(const std::string & name);

    void setBaseAddress(const std::string & name, llvm::Value * addr);

    llvm::Value * getBufferedSize(const std::string & name);

    void setBufferedSize(const std::string & name, llvm::Value * size);

    llvm::Value * getAvailableItemCount(const std::string & name);

    llvm::Value * getLinearlyAccessibleItems(const std::string & name, llvm::Value * fromPosition);

    llvm::BasicBlock * CreateWaitForConsumers();

    llvm::Value * getStreamSetBufferPtr(const std::string & name);

    Kernel * getKernel() const {
        return mKernel;
    }

    void setKernel(Kernel * const kernel) {
        mKernel = kernel;
    }

protected:

    KernelBuilder(llvm::Module * const module, unsigned registerWidth, unsigned blockWidth, unsigned stride)
    : IDISA::IDISA_Builder(module, registerWidth, blockWidth, stride) {

    }

private:

    llvm::Value * getConsumerLock(const std::string & name);

    void setConsumerLock(const std::string & name, llvm::Value * value);

    llvm::Value * computeBlockIndex(llvm::Value * itemCount);

protected:
    Kernel * mKernel;
};

template <class IBuilder>
class KernelBuilderImpl final : public KernelBuilder, public IBuilder {
public:
    KernelBuilderImpl(llvm::Module * const module, unsigned registerWidth, unsigned blockWidth, unsigned stride)
    : IDISA::IDISA_Builder(module, registerWidth, blockWidth, stride)
    , KernelBuilder(module, registerWidth, blockWidth, stride)
    , IBuilder(module, registerWidth, blockWidth, stride) {

    }
};

}

#endif // KERNEL_BUILDER_H
