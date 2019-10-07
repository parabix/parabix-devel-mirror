#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <kernel/core/kernel.h>
#include <idisa/idisa_builder.h>

namespace kernel {

class Kernel;

class KernelBuilder : public virtual IDISA::IDISA_Builder {
    friend class Kernel;
    friend class MultiBlockKernel;
    friend class PipelineGenerator;
public:

    using Rational = boost::rational<unsigned>;

    enum TerminationCode : unsigned {
        None = 0
        , Terminated = 1
        , Fatal = 2
    };

    // Get the value of a scalar field for the current instance.
    llvm::Value * getScalarFieldPtr(const llvm::StringRef fieldName);

    llvm::Value * getScalarField(const llvm::StringRef fieldName);

    llvm::LoadInst * CreateMonitoredScalarFieldLoad(const llvm::StringRef fieldName, llvm::Value * ptr);

    llvm::StoreInst * CreateMonitoredScalarFieldStore(const llvm::StringRef fieldName, llvm::Value * toStore, llvm::Value * ptr);

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

    void setTerminationSignal() { setTerminationSignal(getSize(TerminationCode::Terminated)); }

    void setFatalTerminationSignal() { setTerminationSignal(getSize(TerminationCode::Fatal)); }

    void setTerminationSignal(llvm::Value * const value);

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


    llvm::Value * getRawInputPointer(const std::string & name, llvm::Value * const streamIndex, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const std::string & name, llvm::Value * const streamIndex, llvm::Value * absolutePosition);

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

    llvm::Value * CreateUDivRate(llvm::Value * const number, const Rational divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUDivRate(llvm::Value * const number, const Rational divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateMulRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUMulRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateURemRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateRoundDownRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateRoundUpRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

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
