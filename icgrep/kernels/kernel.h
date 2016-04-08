#ifndef KERNEL_H
#define KERNEL_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <string>
#include <vector>
#include <boost/container/flat_map.hpp>
#include <IDISA/idisa_builder.h>

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

namespace kernel {

class Instance;

class KernelBuilder {
    friend class Instance;
    friend llvm::Function * generateScanWordRoutine(llvm::Module *, IDISA::IDISA_Builder *, unsigned, KernelBuilder *, bool);
    using InputStreamMap = boost::container::flat_map<unsigned, llvm::Value *>;
    using NameMap = boost::container::flat_map<std::string, llvm::ConstantInt *>;
public:

    KernelBuilder(IDISA::IDISA_Builder * builder, std::string && name, const unsigned defaultBufferSize);

    template<typename T>
    struct disable_implicit_conversion {
        inline disable_implicit_conversion(T const value) : _value(value) { assert(_value); }
        inline disable_implicit_conversion(std::nullptr_t) = delete;
        inline disable_implicit_conversion(unsigned) = delete;
        operator T() const { return _value; }
        T operator-> () const { return _value; }
        T get() const { return _value; }
    private:
        T const  _value;
    };

    unsigned addInternalState(llvm::Type * const type);
    unsigned addInternalState(llvm::Type * const type, std::string && name);

    void addInputStream(const unsigned fields);
    void addInputStream(const unsigned fields, std::string && name);

    void addInputScalar(llvm::Type * const type);
    void addInputScalar(llvm::Type * const type, std::string && name);

    unsigned addOutputStream(const unsigned fields);
    unsigned addOutputScalar(llvm::Type * const type);

    inline llvm::Function * prepareFunction() {
        return prepareFunction({0});
    }

    llvm::Function * prepareFunction(std::vector<unsigned> && inputStreamOffsets);

    inline llvm::Value * getInternalState(const std::string & name) {
        return getInternalStateInternal(mKernelStateParam, name);
    }

    inline void setInternalState(const std::string & name, llvm::Value * value) {
        setInternalStateInternal(mKernelStateParam, name, value);
    }

    inline llvm::Value * getInternalState(const unsigned index) {
        assert (index < mInternalState.size());
        return getInternalStateInternal(mKernelStateParam, iBuilder->getInt32(index));
    }

    inline llvm::Value * getInternalState(disable_implicit_conversion<llvm::Value *> const index) {
        return getInternalStateInternal(mKernelStateParam, index);
    }

    void setInternalState(const unsigned index, llvm::Value * value) {
        assert (index < mInternalState.size());
        setInternalStateInternal(mKernelStateParam, iBuilder->getInt32(index), value);
    }

    void setInternalState(disable_implicit_conversion<llvm::Value *> const index, llvm::Value * value) {
        setInternalStateInternal(mKernelStateParam, index, value);
    }

    inline llvm::Type * getKernelStateType() const{
        return mKernelStateType;
    }

    inline llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0) {
        return getInputStreamInternal(getInputStreamParam(streamOffset), iBuilder->getInt32(index));
    }

    inline llvm::Value * getInputStream(disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset = 0) {
        return getInputStreamInternal(getInputStreamParam(streamOffset), index);
    }

    inline unsigned getNumOfInputStreams() const {
        return mInputStream.size();
    }

    inline llvm::Type * getInputStreamType() const {
        return mInputStreamType;
    }

    inline llvm::Value * getInputScalar(const unsigned index) {
        assert (index < getNumOfInputScalars());
        return getInputScalarInternal(mInputScalarParam, iBuilder->getInt32(index));
    }

    inline llvm::Value * getInputScalar(disable_implicit_conversion<llvm::Value *> const index) {
        return getInputScalarInternal(mInputScalarParam, index);
    }

    inline unsigned getNumOfInputScalars() const {
        return mInputScalar.size();
    }

    inline llvm::Type * getInputScalarType() const {
        return mInputScalarType;
    }

    inline llvm::Value * getOutputStream(const unsigned index) {
        assert (index < getNumOfOutputStreams());
        return getOutputStreamInternal(mOutputStreamParam, iBuilder->getInt32(index));
    }

    inline llvm::Value * getOutputStream(disable_implicit_conversion<llvm::Value *> const index) {
        return getOutputStreamInternal(mOutputStreamParam, index);
    }

    inline unsigned getNumOfOutputStreams() const {
        return mOutputStream.size();
    }

    inline llvm::Type * getOutputStreamType() const {
        return mOutputStreamType;
    }

    inline llvm::Value * getOutputScalar(const unsigned index) {
        assert (index < getNumOfOutputScalars());
        return getOutputScalarInternal(mOutputScalarParam, iBuilder->getInt32(index));
    }

    inline llvm::Value * getOutputScalar(disable_implicit_conversion<llvm::Value *> const index) {
        return getOutputScalarInternal(mOutputScalarParam, index);
    }

    inline unsigned getNumOfOutputScalars() const {
        return mOutputScalar.size();
    }

    inline llvm::Type * getOutputScalarType() const {
        return mOutputStreamType;
    }

    inline llvm::Value * getBlockNo() {
        return getBlockNoInternal(mKernelStateParam);
    }

    unsigned getDefaultBufferSize() const;

    void finalize();

    kernel::Instance * instantiate(std::pair<llvm::Value *, unsigned> && inputStreamSet) {
        return instantiate(std::move(inputStreamSet), getDefaultBufferSize());
    }

    kernel::Instance * instantiate(std::pair<llvm::Value *, unsigned> && inputStreamSet, const unsigned outputBufferSize);

    kernel::Instance * instantiate(llvm::Value * const inputStream) {
        return instantiate(std::make_pair(inputStream, 0));
    }

    kernel::Instance * instantiate(std::initializer_list<llvm::Value *> inputStreams);

    llvm::Value * getKernelState() const;

    llvm::Function * getDoBlockFunction() const;

protected:

    Type * packDataTypes(const std::vector<llvm::Type *> & types);

    llvm::Value * getInputStreamInternal(llvm::Value * const inputStreamSet, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getInputScalarInternal(llvm::Value * const inputScalarSet, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getInternalStateInternal(llvm::Value * const kernelState, const std::string & name);

    void setInternalStateInternal(llvm::Value * const kernelState, const std::string & name, llvm::Value * const value);

    llvm::Value * getInternalStateInternal(llvm::Value * const kernelState, disable_implicit_conversion<llvm::Value *> index);

    void setInternalStateInternal(llvm::Value * const kernelState, const unsigned index, llvm::Value * const value);

    void setInternalStateInternal(llvm::Value * const kernelState, disable_implicit_conversion<llvm::Value *> index, llvm::Value * const value);

    llvm::Value * getOutputStreamInternal(llvm::Value * const outputStreamSet, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getOutputScalarInternal(llvm::Value * const outputScalarSet, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getBlockNoInternal(llvm::Value * const instance) {
        return getInternalStateInternal(instance, mBlockNoIndex);
    }

    llvm::Function * getOutputStreamSetFunction() const;

    llvm::Value * getInputStreamParam(const unsigned streamOffset) const;

    const std::vector<unsigned> & getInputStreamOffsets() const {
        return mInputStreamOffsets;
    }

private:

    IDISA::IDISA_Builder * const        iBuilder;
    const std::string                   mKernelName;
    unsigned                            mDefaultBufferSize;

    llvm::Type *                        mBitBlockType;
    llvm::ConstantInt *                 mBlockNoIndex;
    llvm::Function * 					mConstructor;
    llvm::Function *					mDoBlock;

    llvm::Type *                        mKernelStateType;
    llvm::Type *                        mInputScalarType;
    llvm::Type *                        mInputStreamType;
    llvm::Type *                        mOutputScalarType;
    llvm::Type *                        mOutputStreamType;

    llvm::Value *                       mKernelStateParam;
    llvm::Value *                       mInputScalarParam;
    InputStreamMap                      mInputStreamParam;
    llvm::Value *                       mOutputScalarParam;
    llvm::Value *                       mOutputStreamParam;

    std::vector<llvm::Type *>           mInputScalar;
    std::vector<std::string>            mInputScalarName;    
    std::vector<llvm::Type *>           mInputStream;
    std::vector<std::string>            mInputStreamName;
    std::vector<unsigned>               mInputStreamOffsets;
    std::vector<llvm::Type *>           mOutputScalar;
    std::vector<llvm::Type *>           mOutputStream;
    std::vector<llvm::Type *> 			mInternalState;
    NameMap                             mInternalStateNameMap;
};

inline llvm::Function * KernelBuilder::getDoBlockFunction() const {
    return mDoBlock;
}

inline llvm::Value * KernelBuilder::getKernelState() const {
    return mKernelStateParam;
}

inline unsigned KernelBuilder::getDefaultBufferSize() const {
    return mDefaultBufferSize;
}

} // end of namespace kernel

#endif // KERNEL_H
