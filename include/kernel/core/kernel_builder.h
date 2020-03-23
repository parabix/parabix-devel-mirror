#ifndef KERNEL_BUILDER_H
#define KERNEL_BUILDER_H

#include <kernel/core/kernel.h>
#include <idisa/idisa_builder.h>

namespace kernel {

class KernelBuilder : public virtual IDISA::IDISA_Builder {
    friend class Kernel;
    friend class KernelCompiler;
public:

    using Rational = boost::rational<unsigned>;

    enum TerminationCode : unsigned {
        None = 0
        , Terminated = 1
        , Fatal = 2
    };

    llvm::Value * getHandle() const noexcept;

    llvm::Value * getThreadLocalHandle() const noexcept;

    // Get the value of a scalar field for the current instance.
    llvm::Value * getScalarFieldPtr(const llvm::StringRef fieldName);

    llvm::Value * getScalarField(const llvm::StringRef fieldName);

    llvm::LoadInst * CreateMonitoredScalarFieldLoad(const llvm::StringRef fieldName, llvm::Value * ptr);

    llvm::StoreInst * CreateMonitoredScalarFieldStore(const llvm::StringRef fieldName, llvm::Value * toStore, llvm::Value * ptr);

    // Set the value of a scalar field for the current instance.
    void setScalarField(const llvm::StringRef fieldName, llvm::Value * value);

    llvm::Value * getAvailableItemCount(const llvm::StringRef name) const noexcept;

    llvm::Value * getAccessibleItemCount(const llvm::StringRef name) const noexcept;

    llvm::Value * getProcessedItemCount(const llvm::StringRef name);

    void setProcessedItemCount(const llvm::StringRef name, llvm::Value * value);

    llvm::Value * getProducedItemCount(const llvm::StringRef name);

    void setProducedItemCount(const llvm::StringRef name, llvm::Value * value);

    llvm::Value * getWritableOutputItems(const llvm::StringRef name) const noexcept;

    llvm::Value * getConsumedItemCount(const llvm::StringRef name) const noexcept;

    llvm::Value * getTerminationSignal();

    void setTerminationSignal() { setTerminationSignal(getSize(TerminationCode::Terminated)); }

    void setFatalTerminationSignal() { setTerminationSignal(getSize(TerminationCode::Fatal)); }

    void setTerminationSignal(llvm::Value * const value);

    // Run-time access of Kernel State and parameters of methods for
    // use in implementing kernels.

    llvm::Value * getInputStreamBlockPtr(const llvm::StringRef name, llvm::Value * streamIndex) {
        return getInputStreamBlockPtr(name, streamIndex, nullptr);
    }

    llvm::Value * getInputStreamBlockPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamLogicalBasePtr(const Binding & input);

    llvm::Value * loadInputStreamBlock(const llvm::StringRef name, llvm::Value * streamIndex) {
        return loadInputStreamBlock(name, streamIndex, nullptr);
    }

    llvm::Value * loadInputStreamBlock(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamPackPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return getInputStreamPackPtr(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * getInputStreamPackPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::Value * loadInputStreamPack(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return loadInputStreamPack(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * loadInputStreamPack(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::Value * getInputStreamSetCount(const llvm::StringRef name);

    llvm::Value * getOutputStreamBlockPtr(const llvm::StringRef name, llvm::Value * streamIndex) {
        return getOutputStreamBlockPtr(name, streamIndex, nullptr);
    }

    llvm::Value * getOutputStreamBlockPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * blockOffset);

    llvm::Value * getOutputStreamLogicalBasePtr(const Binding & output);

    llvm::StoreInst * storeOutputStreamBlock(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * toStore) {
        return storeOutputStreamBlock(name, streamIndex, nullptr, toStore);
    }

    llvm::StoreInst * storeOutputStreamBlock(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * blockOffset, llvm::Value * toStore);

    llvm::Value * getOutputStreamPackPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex) {
        return getOutputStreamPackPtr(name, streamIndex, packIndex, nullptr);
    }

    llvm::Value * getOutputStreamPackPtr(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset);

    llvm::StoreInst * storeOutputStreamPack(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * toStore) {
        return storeOutputStreamPack(name, streamIndex, packIndex, nullptr, toStore);
    }

    llvm::StoreInst * storeOutputStreamPack(const llvm::StringRef name, llvm::Value * streamIndex, llvm::Value * packIndex, llvm::Value * blockOffset, llvm::Value * toStore);

    llvm::Value * getOutputStreamSetCount(const llvm::StringRef name);

    llvm::Value * getRawInputPointer(const llvm::StringRef name, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const llvm::StringRef name, llvm::Value * absolutePosition);


    llvm::Value * getRawInputPointer(const llvm::StringRef name, llvm::Value * const streamIndex, llvm::Value * absolutePosition);

    llvm::Value * getRawOutputPointer(const llvm::StringRef name, llvm::Value * const streamIndex, llvm::Value * absolutePosition);

    llvm::Value * getBaseAddress(const llvm::StringRef name);

    void setBaseAddress(const llvm::StringRef name, llvm::Value * addr);

    llvm::Value * getCapacity(const llvm::StringRef name);

    void setCapacity(const llvm::StringRef name, llvm::Value * capacity);

    // internal state

    llvm::Value * getNumOfStrides() const noexcept;

    llvm::Value * getExternalSegNo() const noexcept;

    llvm::Value * isFinal() const noexcept;

    // input streamset bindings

    const Bindings & getInputStreamSetBindings() const noexcept;

    const Binding & getInputStreamSetBinding(const unsigned i) const noexcept;

    const Binding & getInputStreamSetBinding(const llvm::StringRef name) const noexcept;

    StreamSet * getInputStreamSet(const unsigned i) const noexcept;

    StreamSet * getInputStreamSet(const llvm::StringRef name) const noexcept;

    void setInputStreamSet(const llvm::StringRef name, StreamSet * value) noexcept;

    unsigned getNumOfStreamInputs() const noexcept;

    // input streamsets

    StreamSetBuffer * getInputStreamSetBuffer(const unsigned i) const noexcept;

    StreamSetBuffer * getInputStreamSetBuffer(const llvm::StringRef name) const noexcept;

    // output streamset bindings

    const Bindings & getOutputStreamSetBindings() const noexcept;

    const Binding & getOutputStreamSetBinding(const unsigned i) const noexcept;

    const Binding & getOutputStreamSetBinding(const llvm::StringRef name) const noexcept;

    StreamSet * getOutputStreamSet(const unsigned i) const noexcept;

    StreamSet * getOutputStreamSet(const llvm::StringRef name) const noexcept;

    void setOutputStreamSet(const llvm::StringRef name, StreamSet * value) noexcept;

    unsigned getNumOfStreamOutputs() const noexcept;

    // output streamsets

    StreamSetBuffer * getOutputStreamSetBuffer(const unsigned i) const noexcept;

    StreamSetBuffer * getOutputStreamSetBuffer(const llvm::StringRef name) const noexcept;

    // input scalar bindings

    const Bindings & getInputScalarBindings() const noexcept;

    const Binding & getInputScalarBinding(const unsigned i) const noexcept;

    const Binding & getInputScalarBinding(const llvm::StringRef name) const noexcept;

    unsigned getNumOfScalarInputs() const noexcept;

    // input scalars

    Scalar * getInputScalar(const unsigned i) noexcept;

    Scalar * getInputScalar(const llvm::StringRef name) noexcept;

    // output scalar bindings

    const Bindings & getOutputScalarBindings() const noexcept;

    const Binding & getOutputScalarBinding(const unsigned i) const noexcept;

    const Binding & getOutputScalarBinding(const llvm::StringRef name) const noexcept;

    unsigned getNumOfScalarOutputs() const noexcept;

    // output scalars

    Scalar * getOutputScalar(const unsigned i) noexcept;

    Scalar * getOutputScalar(const llvm::StringRef name) noexcept;

    // rational math functions

    llvm::Value * CreateUDivRate(llvm::Value * const number, const Rational divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUDivRate(llvm::Value * const number, const Rational divisor, const llvm::Twine & Name = "");

    llvm::Value * CreateMulRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateCeilUMulRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateURemRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateRoundDownRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    llvm::Value * CreateRoundUpRate(llvm::Value * const number, const Rational factor, const llvm::Twine & Name = "");

    KernelCompiler * getCompiler() const noexcept {
        return mCompiler;
    }

    std::string getKernelName() const noexcept final;

protected:

    KernelBuilder(llvm::LLVMContext & C, unsigned nativeVectorWidth, unsigned vectorWidth, unsigned laneWidth)
    : IDISA::IDISA_Builder(C, nativeVectorWidth, vectorWidth, laneWidth) {

    }

    void setCompiler(KernelCompiler * const compiler) noexcept {
        mCompiler = compiler;
    }

protected:

    KernelCompiler * mCompiler = nullptr;

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
