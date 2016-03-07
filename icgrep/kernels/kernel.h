#ifndef KERNEL_H
#define KERNEL_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */

#include <string>
#include <vector>

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

public:
    // sets name & sets internal state to the kernel superclass state
    KernelBuilder(std::string name, llvm::Module * m, IDISA::IDISA_Builder * b);

    unsigned addInternalStateType(llvm::Type * type);
    void addOutputStream(const unsigned fields);
    void addOutputAccum(llvm::Type * t);
    void addInputStream(const unsigned fields, std::string name);
    void addInputScalar(llvm::Type * t, std::string name);

    llvm::Function * prepareFunction();

    void increment();
    void incrementCircularBuffer();

    llvm::Value * getInputStream(const unsigned index, const unsigned streamOffset = 0);
    llvm::Value * getKernelState(const unsigned index, const unsigned streamOffset = 0);
    llvm::Value * getOutputStream(const unsigned index, const unsigned streamOffset = 0);
    llvm::Value * getOutputScalar(const unsigned index, const unsigned streamOffset = 0);

    void finalize();

    llvm::Value * generateKernelInstance();
	void generateInitCall();
    void generateDoBlockCall(llvm::Value * inputStreams);

    unsigned getSegmentBlocks() const;
    llvm::Function * getDoBlockFunction() const;
    llvm::Type * getKernelStructType() const;
    llvm::Value * getKernelStructParam() const;

    void setCircularBufferSize(const unsigned blocks);
    void setBlocksPerSegment(const unsigned blocks);

    void setInternalState(const unsigned index, llvm::Value * const value);
    llvm::Value * getInternalState(const unsigned index);

protected:

    llvm::Value * getOffset(const unsigned offset);

private:
    llvm::Module *                      mMod;
    IDISA::IDISA_Builder *              iBuilder;
    std::string							mKernelName;
    llvm::Type *                        mBitBlockType;
    std::vector<llvm::Type *> 			mStates;
    std::vector<llvm::Type *>           mInputStreams;
    std::vector<llvm::Type *>           mOutputStreams;
    std::vector<llvm::Type *>           mInputScalars;
    std::vector<llvm::Type *>           mOutputAccums;
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
    unsigned                            mStartIndex;
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

inline void KernelBuilder::setCircularBufferSize(const unsigned blocks) {
    mCircularBufferModulo = blocks;
}

inline void KernelBuilder::setBlocksPerSegment(const unsigned blocks) {
    mBlocksPerSegment = blocks;
}

inline void KernelBuilder::increment() {
    ++mSegmentIndex;
}

#endif // KERNEL_H
