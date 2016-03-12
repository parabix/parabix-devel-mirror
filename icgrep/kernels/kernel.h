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

class KernelBuilder {
    using NameMap = boost::container::flat_map<std::string, unsigned>;
public:
    // sets name & sets internal state to the kernel superclass state
    KernelBuilder(std::string name, llvm::Module * m, IDISA::IDISA_Builder * b);

    unsigned addInternalState(llvm::Type * const type);
    unsigned addInternalState(llvm::Type * const type, std::string name);

    void addInputStream(const unsigned fields);
    void addInputStream(const unsigned fields, std::string name);

    void addInputScalar(llvm::Type * const type);
    void addInputScalar(llvm::Type * const type, std::string name);


    void addOutputStream(const unsigned fields);
    void addOutputScalar(llvm::Type * const type);

    llvm::Function * prepareFunction();

    void increment();

    llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0);

    llvm::Value * getInputScalar(const unsigned index);

    llvm::Value * getInternalState(const std::string & name, llvm::Value * const inputStruct = nullptr);

    llvm::Value * getInternalState(const unsigned index, llvm::Value * const inputStruct = nullptr);

    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0);

    llvm::Value * getOutputScalar(const unsigned index);

    void finalize();

    llvm::Value * generateKernelInstance();
	void generateInitCall();
    void generateDoBlockCall(llvm::Value * inputStreams);

    unsigned getSegmentBlocks() const;
    llvm::Function * getDoBlockFunction() const;
    llvm::Type * getKernelStructType() const;
    llvm::Value * getKernelStructParam() const;

    void setLongestLookaheadAmount(const unsigned bits);
    void setBlocksPerSegment(const unsigned blocks);

    void setInternalState(const unsigned index, llvm::Value * const value);

    llvm::Value * getBlockIndexScalar();

protected:

    llvm::Value * getOffset(const unsigned value);

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    std::string							mKernelName;
    llvm::Type *                        mBitBlockType;
    std::vector<llvm::Type *> 			mStates;
    std::vector<llvm::Type *>           mInputStreams;
    std::vector<llvm::Type *>           mOutputStreams;
    std::vector<llvm::Type *>           mInputScalars;
    std::vector<llvm::Type *>           mOutputScalar;
    std::vector<std::string>            mInputStreamNames;
    std::vector<std::string>            mInputScalarNames;
    llvm::Function* 					mConstructor;
    llvm::Function*						mInitFunction;
    llvm::Function*						mFunction;
    unsigned                            mBlockSize;
    unsigned                            mBlocksPerSegment;
    unsigned                            mCircularBufferModulo;
    llvm::Type *                        mKernelStructType;
    llvm::Type *                        mInputStreamType;
    llvm::Type *                        mInputScalarType;
    llvm::Value *                       mInputParam;
    llvm::Value *                       mKernelStruct;
    llvm::Value *                       mKernelParam;
    unsigned                            mSegmentIndex;
    unsigned                            mBlockIndex;


    NameMap                             mStateNameMap;

};

inline unsigned KernelBuilder::getSegmentBlocks() const {
    return mBlocksPerSegment;
}

inline llvm::Function * KernelBuilder::getDoBlockFunction() const {
    return mFunction;
}

inline llvm::Type * KernelBuilder::getKernelStructType() const{
    return mKernelStructType;
}

inline llvm::Value * KernelBuilder::getKernelStructParam() const {
    return mKernelParam;
}

inline void KernelBuilder::setBlocksPerSegment(const unsigned blocks) {
    mBlocksPerSegment = blocks;
}

inline void KernelBuilder::increment() {
    ++mSegmentIndex;
}

inline llvm::Value * KernelBuilder::getBlockIndexScalar() {
    return getInternalState(mBlockIndex);
}

#endif // KERNEL_H
