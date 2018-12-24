#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <IR_Gen/idisa_builder.h>
#include <kernels/kernel.h>

namespace kernel {

class Kernel;

class KernelBuilder : public virtual IDISA::IDISA_Builder {
    friend class Kernel;
    friend class MultiBlockKernel;
    friend class PipelineGenerator;
public:

    // Get the value of a scalar field for the current instance.
    llvm::Value * getScalarFieldPtr(llvm::Value * index);

    llvm::Value * getScalarFieldPtr(const llvm::StringRef fieldName);

    llvm::Value * getScalarField(const llvm::StringRef fieldName);

    // Set the value of a scalar field for the current instance.
    void setScalarField(const llvm::StringRef fieldName, llvm::Value * value);

    llvm::Value * getAvailableItemCount(const llvm::StringRef name) {
        return mKernel->getAvailableInputItems(name);
    }

    llvm::Value * getAccessibleItemCount(const llvm::StringRef name) {
        return mKernel->getAccessibleInputItems(name);
    }

    llvm::Value * getProcessedItemCount(const llvm::StringRef name);

    void setProcessedItemCount(const llvm::StringRef name, llvm::Value * value);

    llvm::Value * getProducedItemCount(const llvm::StringRef name);

    void setProducedItemCount(const llvm::StringRef name, llvm::Value * value);

    llvm::Value * getConsumedItemCount(const llvm::StringRef name) const;

    llvm::Value * getTerminationSignal();

    void setTerminationSignal() { setTerminationSignal(getTrue()); }

    void setTerminationSignal(llvm::Value * const value);

    llvm::Value * getCycleCountPtr();

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) {
        return getInputStreamBlockPtr(name, streamIndex, nullptr);
    }

    llvm::Value * getInputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamLogicalBasePtr(const Binding & input);

    llvm::Value * loadInputStreamBlock(const std::string & name, llvm::Value * streamIndex) {
        return loadInputStreamBlock(name, streamIndex, nullptr);
    }

    llvm::Value * loadInputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return getInputStreamPackPtr(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * getInputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::Value * loadInputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return loadInputStreamPack(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * loadInputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamSetCount(const std::string & name);

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex) {
        return getOutputStreamBlockPtr(name, streamIndex, nullptr);
    }

    llvm::Value * getOutputStreamBlockPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getOutputStreamLogicalBasePtr(const Binding & output);

    llvm::StoreInst * storeOutputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * toStore) {
        return storeOutputStreamBlock(name, streamIndex, nullptr, toStore);
    }

    llvm::StoreInst * storeOutputStreamBlock(const std::string & name, llvm::Value * streamIndex, llvm::Value * blockOffset, llvm::Value * toStore);

    llvm::Value * getOutputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return getOutputStreamPackPtr(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * getOutputStreamPackPtr(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::StoreInst * storeOutputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * toStore) {
        return storeOutputStreamPack(name, streamIndex, packIndex, nullptr, toStore);
    }

    llvm::StoreInst * storeOutputStreamPack(const std::string & name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset, llvm::Value * toStore);

    llvm::Value * getOutputStreamSetCount(const std::string & name);

    llvm::Value * getRawInputPointer(const std::string & name, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const std::string & name, llvm::Value * absolutePosition);

    llvm::Value * getBaseAddress(const std::string & name);

    void setBaseAddress(const std::string & name, llvm::Value * addr);

    llvm::Value * getCapacity(const std::string & name);

    void setCapacity(const std::string & name, llvm::Value * capacity);

    llvm::CallInst * createDoSegmentCall(const std::vector<llvm::Value *> & args);

    const Kernel * getKernel() const {
        return mKernel;
    }

    void setKernel(const Kernel * const kernel) {
        mKernel = kernel;
    }

    // overloading wrongly subsitutes this for CBuilder function. renamed for now until I can investigate why.
    llvm::Value * CreateUDiv2(llvm::Value * const number, const ProcessingRate::RateValue & divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUDiv2(llvm::Value * const number, const ProcessingRate::RateValue & divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateMul2(llvm::Value * const number, const ProcessingRate::RateValue & factor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUMul2(llvm::Value * const number, const ProcessingRate::RateValue & factor, const llvm::Twine & Name = "");

    llvm::Type * resolveStreamSetType(llvm::Type * streamSetType);

    unsigned getStride() const {
        return mStride;
    }

protected:

    KernelBuilder(llvm::LLVMContext & C, unsigned nativeVectorWidth, unsigned vectorWidth, unsigned laneWidth)
    : IDISA::IDISA_Builder(C, nativeVectorWidth, vectorWidth, laneWidth)
    , mStride(vectorWidth)
    , mKernel(nullptr) {

    }

    const unsigned mStride;

    llvm::Value * getScalarFieldPtr(llvm::Value * handle, llvm::Value * index);

    llvm::Value * getScalarFieldPtr(llvm::Value * instance, const std::string & fieldName);

    std::string getKernelName() const final;

protected:
    const Kernel * mKernel;

};

template <class SpecifiedArchitectureBuilder>
class KernelBuilderImpl final : public KernelBuilder, public SpecifiedArchitectureBuilder {
public:
    KernelBuilderImpl(llvm::LLVMContext & C, unsigned vectorWidth, unsigned laneWidth)
    : IDISA::IDISA_Builder(C, SpecifiedArchitectureBuilder::NativeBitBlockWidth, vectorWidth, laneWidth)
    , KernelBuilder(C, SpecifiedArchitectureBuilder::NativeBitBlockWidth, vectorWidth, laneWidth)
    , SpecifiedArchitectureBuilder(C, vectorWidth, laneWidth) {

    }
};

}

#endif // KERNEL_BUILDER_H
