#ifndef INSTANCE_H
#define INSTANCE_H

#include <llvm/IR/Instructions.h>
#include <kernels/kernel.h>
#include <slab_allocator.h>

namespace kernel {

class Instance {
    friend class KernelBuilder;
    using Allocator = SlabAllocator<Instance>;
public:

    void call(llvm::Value * inputStreams) {
        assert (inputStreams);
        return mDefinition->call(mMemory, inputStreams);
    }

    llvm::Value * getInternalState(const std::string & name) {
        return mDefinition->getInternalState(mMemory, name);
    }

    void setInternalState(const std::string & name, llvm::Value * value) {
        mDefinition->setInternalState(mMemory, name, value);
    }

    llvm::Value * getInternalState(const unsigned index) {
        return mDefinition->getInternalState(mMemory, index);
    }

    void setInternalState(const unsigned index, llvm::Value * value) {
        mDefinition->setInternalState(mMemory, index, value);
    }

    llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0) {
        return mDefinition->getInputStream(mMemory, index, streamOffset);
    }

    llvm::Type * getInputStreamType() const {
        return mDefinition->getInputStreamType();
    }

    llvm::Value * getInputScalar(const unsigned index) {
        return mDefinition->getInputScalar(mMemory, index);
    }

    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0) {
        return mDefinition->getOutputStream(mMemory, index, streamOffset);
    }

    llvm::Value * getOutputStreamSet(const unsigned streamOffset = 0) {
        return mDefinition->getOutputStreamSet(mMemory, streamOffset);
    }

    llvm::Value * getOutputScalar(const unsigned index) {
        return mDefinition->getOutputScalar(mMemory, index);
    }

    llvm::Value * getBlockNo() {
        return mDefinition->getBlockNo(mMemory);
    }

    void* operator new (std::size_t size) noexcept {
        return mAllocator.allocate(size);
    }

    void operator delete (void * ptr) {
        mAllocator.deallocate(static_cast<Allocator::value_type *>(ptr));
    }

protected:

    Instance(KernelBuilder * definition, llvm::AllocaInst * space)
    : mDefinition(definition)
    , mMemory(space) {

    }

private:
    KernelBuilder * const mDefinition;
    llvm::AllocaInst * const mMemory;
    static Allocator mAllocator;
};

}

#endif // INSTANCE_H
