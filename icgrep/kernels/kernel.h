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
    KernelBuilder(std::string name, llvm::Module * m, IDISA::IDISA_Builder * b, const unsigned bufferSize = 1);

    template<typename T>
    struct disable_implicit_conversion {
        inline disable_implicit_conversion(T const value) : _value(value) {}
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

    llvm::Function * prepareFunction();

    inline llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0) {
        return getInputStream(mKernelState, index, streamOffset);
    }

    inline llvm::Value * getInputStream(disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset = 0) {
        return getInputStream(mKernelState, index, streamOffset);
    }

    inline llvm::Value * getInputScalar(const unsigned index) {
        return getInputScalar(mKernelState, index);
    }

    inline llvm::Value * getInputScalar(disable_implicit_conversion<llvm::Value *> const index) {
        return getInputScalar(mKernelState, index);
    }

    llvm::Value * getInternalState(const std::string & name) {
        return getInternalState(mKernelState, name);
    }

    void setInternalState(const std::string & name, llvm::Value * value) {
        setInternalState(mKernelState, name, value);
    }

    llvm::Value * getInternalState(const unsigned index) {
        return getInternalState(mKernelState, index);
    }

    llvm::Value * getInternalState(disable_implicit_conversion<llvm::Value *> const index) {
        return getInternalState(mKernelState, index);
    }

    void setInternalState(const unsigned index, llvm::Value * value) {
        setInternalState(mKernelState, index, value);
    }

    void setInternalState(disable_implicit_conversion<llvm::Value *> const index, llvm::Value * value) {
        setInternalState(mKernelState, index, value);
    }

    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0) {
        return getOutputStream(mKernelState, index, streamOffset);
    }

    llvm::Value * getOutputStream(disable_implicit_conversion<llvm::Value *> const index, const unsigned streamOffset = 0) {
        return getOutputStream(mKernelState, index, streamOffset);
    }

    inline unsigned getNumOfOutputStreams() const {
        return mOutputStream.size();
    }

    llvm::Value * getOutputScalar(const unsigned index) {
        return getOutputScalar(mKernelState, index);
    }

    llvm::Value * getOutputScalar(disable_implicit_conversion<llvm::Value *> const index) {
        return getOutputScalar(mKernelState, index);
    }

    inline unsigned getNumOfOutputScalars() const {
        return mOutputScalar.size();
    }

    llvm::Value * getBlockNo() {
        return getBlockNo(mKernelState);
    }

    llvm::Type * getInputStreamType() const;

    void setInputBufferSize(const unsigned bufferSize);

    unsigned getInputBufferSize() const;

    unsigned getBufferSize() const;

    void finalize();

    kernel::Instance * instantiate(llvm::Value * const inputStream);

    kernel::Instance * instantiate(std::initializer_list<llvm::Value *> inputStreams);

    kernel::Instance * instantiate(std::pair<llvm::Value *, unsigned> && inputStream);

    llvm::Type * getKernelStateType() const;

    llvm::Value * getKernelState() const;

    llvm::Function * getDoBlockFunction() const;

    void clearOutputStreamSet(llvm::Value * const instance, const unsigned streamOffset = 0);

protected:

    llvm::Type * packDataTypes(const std::vector<llvm::Type *> & types);

    llvm::Value * getInputStream(llvm::Value * const instance, const unsigned index, const unsigned streamOffset);

    llvm::Value * getInputStream(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset);

    llvm::Value * getInputScalar(llvm::Value * const instance, const unsigned index);

    llvm::Value * getInputScalar(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getInternalState(llvm::Value * const instance, const std::string & name);

    void setInternalState(llvm::Value * const instance, const std::string & name, llvm::Value * const value);

    llvm::Value * getInternalState(llvm::Value * const instance, const unsigned index);

    llvm::Value * getInternalState(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index);

    void setInternalState(llvm::Value * const instance, const unsigned index, llvm::Value * const value);

    void setInternalState(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index, llvm::Value * const value);

    llvm::Value * getOutputStream(llvm::Value * const instance, const unsigned index, const unsigned streamOffset);

    llvm::Value * getOutputStream(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index, const unsigned streamOffset);

    llvm::Value * getOutputScalar(llvm::Value * const instance, const unsigned index);

    llvm::Value * getOutputScalar(llvm::Value * const instance, disable_implicit_conversion<llvm::Value *> index);

    llvm::Value * getStreamOffset(llvm::Value * const instance, const unsigned index);

    llvm::Value * getBlockNo(llvm::Value * const instance);

    llvm::Function * getOutputStreamSetFunction() const;

    void CreateDoBlockCall(llvm::Value * const instance);

    llvm::Function * CreateModFunction(const unsigned size);

    void eliminateRedundantMemoryOperations(llvm::Function * const function);

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    std::string							mKernelName;
    llvm::Type *                        mBitBlockType;
    llvm::Function * 					mConstructor;
    llvm::Function *					mDoBlock;

    unsigned                            mBufferSize;

    llvm::Type *                        mKernelStateType;
    llvm::Type *                        mInputStreamType;
    llvm::Type *                        mInputScalarType;
    llvm::Type *                        mOutputStreamType;

    llvm::Value *                       mKernelState;
    unsigned                            mBlockNoIndex;

    std::vector<llvm::Type *>           mInputStream;
    std::vector<std::string>            mInputStreamName;
    std::vector<llvm::Type *>           mInputScalar;
    std::vector<std::string>            mInputScalarName;    
    std::vector<llvm::Type *>           mOutputStream;
    std::vector<llvm::Type *>           mOutputScalar;
    std::vector<llvm::Type *> 			mInternalState;
    NameMap                             mInternalStateNameMap;
};

inline llvm::Function * KernelBuilder::getDoBlockFunction() const {
    return mDoBlock;
}

inline llvm::Type * KernelBuilder::getKernelStateType() const{
    return mKernelStateType;
}

inline llvm::Value * KernelBuilder::getKernelState() const {
    return mKernelState;
}

inline llvm::Type * KernelBuilder::getInputStreamType() const {
    return mInputStreamType;
}

inline llvm::Value * KernelBuilder::getBlockNo(llvm::Value * const instance) {
    return getInternalState(instance, mBlockNoIndex);
}

inline unsigned KernelBuilder::getBufferSize() const {
    return mBufferSize;
}

} // end of namespace kernel

#endif // KERNEL_H
