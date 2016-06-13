#ifndef INSTANCE_H
#define INSTANCE_H

#include <llvm/IR/Instructions.h>
#include <kernels/kernel.h>
#include <util/slab_allocator.h>
#include <llvm/Support/raw_ostream.h>

namespace kernel {

class Instance {
    friend class KernelBuilder;
    using InputStreamMap = KernelBuilder::InputStreamMap;
    using Allocator = SlabAllocator<Instance>;
public:

    llvm::Value * CreateDoBlockCall();

    llvm::Value * getKernelState() {
        return mKernelState;
    }
    
    llvm::Value * getInternalState(const std::string & name) {
        return mDefinition->getInternalStateInternal(mKernelState, name);
    }
    
    void setInternalState(const std::string & name, llvm::Value * value) {
        mDefinition->setInternalStateInternal(mKernelState, name, value);
    }

    llvm::Value * getInternalState(const unsigned index) {
        return mDefinition->getInternalStateInternal(mKernelState, iBuilder->getInt32(index));
    }

    llvm::Value * getInternalState(llvm::Value * const index) {
        return mDefinition->getInternalStateInternal(mKernelState, index);
    }

    void setInternalState(const unsigned index, llvm::Value * value) {
        mDefinition->setInternalStateInternal(mKernelState, iBuilder->getInt32(index), value);
    }

    void setInternalState(llvm::Value * const index, llvm::Value * value) {
        mDefinition->setInternalStateInternal(mKernelState, index, value);
    }

    inline llvm::Value * getInputStreamSet(const unsigned streamOffset = 0) {
        return getStreamSet(mDefinition->getInputStreamType(), mInputStreamSet, streamOffset, mInputBufferSize);
    }

    llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0) {
        return mDefinition->getInputStreamInternal(getInputStreamSet(streamOffset), iBuilder->getInt32(index));
    }

    llvm::Value * getInputStream(disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset = 0) {
        return mDefinition->getInputStreamInternal(getInputStreamSet(streamOffset), index);
    }

    llvm::Type * getInputStreamType() const {
        return mDefinition->getInputStreamType();
    }


    inline llvm::Value * getOutputStreamSet(const unsigned streamOffset = 0) {
        // do not pass the result of this into an instantiate method; instead call getOutputStreamBuffer.
        return getStreamSet(mDefinition->getOutputStreamType(), mOutputStreamSet, streamOffset, mOutputBufferSize);
    }

    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0) {
        return mDefinition->getOutputStreamInternal(getOutputStreamSet(streamOffset), iBuilder->getInt32(index));
    }

    llvm::Value * getOutputStream(disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset = 0) {
        return mDefinition->getOutputStreamInternal(getOutputStreamSet(streamOffset), index);
    }

    void clearOutputStreamSet();

    llvm::Value * getBlockNo() {
        return mDefinition->getBlockNoInternal(mKernelState);
    }

    inline std::pair<llvm::Value *, unsigned> getOutputStreamBuffer() const {
        return std::make_pair(mOutputStreamSet, mOutputBufferSize);
    }

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }

    void operator delete (void * ptr) {
        mAllocator.deallocate(static_cast<Allocator::value_type *>(ptr));
    }

protected:

    Instance(KernelBuilder * const definition, llvm::Value * const kernelState,
             llvm::Value * const inputStreamSet, const unsigned inputBufferSize,
             llvm::Value * const outputStreamSet, const unsigned outputBufferSize)
    : mDefinition(definition)
    , iBuilder(definition->iBuilder)
    , mKernelState(kernelState)
    , mInputStreamSet(inputStreamSet)
    , mInputBufferSize(inputBufferSize)
    , mOutputStreamSet(outputStreamSet)
    , mOutputBufferSize(outputBufferSize) {

    }

    llvm::Value * getStreamSet(Type * const type, llvm::Value * const base, const unsigned index, const unsigned bufferSize);

private:
    KernelBuilder * const                           mDefinition;
    IDISA::IDISA_Builder * const                    iBuilder;
    llvm::Value * const                             mKernelState;
    llvm::Value * const                             mInputStreamSet;
    const unsigned                                  mInputBufferSize;
    llvm::Value * const                             mOutputStreamSet;
    const unsigned                                  mOutputBufferSize;
    static Allocator                                mAllocator;
};

}

#endif // INSTANCE_H
