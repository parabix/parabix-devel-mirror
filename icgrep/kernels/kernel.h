#ifndef KERNEL_H
#define KERNEL_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <string>
#include <vector>
#include <boost/container/flat_map.hpp>

namespace llvm {
    class Value;
    class Module;
    class ExecutionEngine;
    class VectorType;
    class PointerType;
    class Constant;
    class FunctionType;
    class Function;
    class BasicBlock;
    class Type;
}

namespace pablo {
    class PabloAST;
    class PabloFunction;
}

namespace IDISA {
    class IDISA_Builder;
}

namespace kernel {

class Instance;

class KernelBuilder {
    friend class Instance;
    friend llvm::Function * generateScanWordRoutine(llvm::Module *, IDISA::IDISA_Builder *, unsigned, KernelBuilder *, bool);
    using NameMap = boost::container::flat_map<std::string, unsigned>;
public:
    // sets name & sets internal state to the kernel superclass state
    KernelBuilder(std::string name, llvm::Module * m, IDISA::IDISA_Builder * b);

    unsigned addInternalState(llvm::Type * const type);
    unsigned addInternalState(llvm::Type * const type, std::string && name);

    void addInputStream(const unsigned fields);
    void addInputStream(const unsigned fields, std::string && name);

    void addInputScalar(llvm::Type * const type);
    void addInputScalar(llvm::Type * const type, std::string && name);

    unsigned addOutputStream(const unsigned fields);
    unsigned addOutputScalar(llvm::Type * const type);

    llvm::Function * prepareFunction();

    void increment();

    inline llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0) {
        return getInputStream(mInputParam, index, streamOffset);
    }

    inline llvm::Value * getInputScalar(const unsigned index) {
        return getInputScalar(mInputParam, index);
    }

    llvm::Value * getInternalState(const std::string & name) {
        return getInternalState(mKernelParam, name);
    }

    void setInternalState(const std::string & name, llvm::Value * value) {
        setInternalState(mKernelParam, name, value);
    }

    llvm::Value * getInternalState(const unsigned index) {
        return getInternalState(mKernelParam, index);
    }

    void setInternalState(const unsigned index, llvm::Value * value) {
        setInternalState(mKernelParam, index, value);
    }

    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0) {
        return getOutputStream(mKernelParam, index, streamOffset);
    }
    llvm::Value * getOutputStreamSet(const unsigned streamOffset = 0) {
        return getOutputStreamSet(mKernelParam, streamOffset);
    }

    llvm::Value * getOutputScalar(const unsigned index) {
        return getOutputScalar(mKernelParam, index);
    }

    llvm::Value * getBlockNo() {
        return getBlockNo(mKernelParam);
    }

    llvm::Type * getInputStreamType() const;

    void finalize();

    kernel::Instance * instantiate();

    unsigned getSegmentBlocks() const;

    llvm::Type * getKernelStateType() const;

    llvm::Value * getKernelState() const;

    llvm::Function * getDoBlockFunction() const;

    void setLongestLookaheadAmount(const unsigned bits);

    void setBlocksPerSegment(const unsigned blocks);

protected:

    llvm::Value * getInputStream(llvm::Value * const instance, const unsigned index, const unsigned streamOffset);

    llvm::Value * getInputScalar(llvm::Value * const instance, const unsigned index);

    llvm::Value * getInternalState(llvm::Value * const instance, const std::string & name);

    void setInternalState(llvm::Value * const instance, const std::string & name, llvm::Value * const value);

    llvm::Value * getInternalState(llvm::Value * const instance, const unsigned index);

    void setInternalState(llvm::Value * const instance, const unsigned index, llvm::Value * const value);

    llvm::Value * getOutputStream(llvm::Value * const instance, const unsigned index, const unsigned streamOffset);

    llvm::Value * getOutputStreamSet(llvm::Value * const instance, const unsigned streamOffset);

    llvm::Value * getOutputScalar(llvm::Value * const instance, const unsigned index);

    llvm::Value * getOffset(llvm::Value * const instance, const unsigned value);

    llvm::Value * getBlockNo(llvm::Value * const instance);

    void call(llvm::Value * const instance, llvm::Value * inputStreams);

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    std::string							mKernelName;
    llvm::Type *                        mBitBlockType;
    llvm::Function* 					mConstructor;
    llvm::Function*						mFunction;
    unsigned                            mBlockSize;
    unsigned                            mBlocksPerSegment;
    unsigned                            mCircularBufferModulo;
    llvm::Type *                        mKernelStructType;
    llvm::Type *                        mInputStreamType;
    llvm::Type *                        mInputScalarType;
    llvm::Value *                       mInputParam;
    llvm::Value *                       mKernelParam;
    unsigned                            mSegmentIndex;
    unsigned                            mBlockIndex;
    std::vector<llvm::Type *>           mInputStream;
    std::vector<std::string>            mInputStreamName;
    std::vector<llvm::Type *>           mInputScalar;
    std::vector<std::string>            mInputScalarName;
    std::vector<llvm::Type *>           mOutputStream;
    std::vector<llvm::Type *>           mOutputScalar;
    std::vector<llvm::Type *> 			mInternalState;
    NameMap                             mInternalStateNameMap;
};

inline unsigned KernelBuilder::getSegmentBlocks() const {
    return mBlocksPerSegment;
}

inline llvm::Function * KernelBuilder::getDoBlockFunction() const {
    return mFunction;
}

inline llvm::Type * KernelBuilder::getKernelStateType() const{
    return mKernelStructType;
}

inline llvm::Value * KernelBuilder::getKernelState() const {
    return mKernelParam;
}

inline void KernelBuilder::setBlocksPerSegment(const unsigned blocks) {
    mBlocksPerSegment = blocks;
}

inline llvm::Type * KernelBuilder::getInputStreamType() const {
    return mInputStreamType;
}

inline void KernelBuilder::increment() {
    ++mSegmentIndex;
}

inline llvm::Value * KernelBuilder::getBlockNo(llvm::Value * const instance) {
    return getInternalState(instance, mBlockIndex);
}

} // end of namespace kernel

#endif // KERNEL_H
