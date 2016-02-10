#ifndef KERNEL_H
#define KERNEL_H
/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 */


#include <IDISA/idisa_builder.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/Module.h>

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

using namespace llvm;

typedef Value* valptr;

struct Inputs {
    std::vector<std::vector<valptr>> streams;
    std::vector<valptr> scalars;
};

struct Outputs {
    std::vector<valptr *> streams;
    std::vector<valptr> accums;
};

class KernelBuilder{
public:
    // sets name & sets internal state to the kernel superclass state
	KernelBuilder(std::string name, Module * m, IDISA::IDISA_Builder * b);
	~KernelBuilder();

	int extendKernelInternalStateType(Type * t);
	void addKernelOutputStream(int fw);
	void addKernelOutputAccum(Type * t);
	void addKernelInputStream(int fw, std::string name);
	void addKernelInputScalar(Type * t, std::string name);
    Function* CreateDoBlockFunction();
	struct Inputs openDoBlock();
	void closeDoBlock(struct Outputs);
	void finalizeMethods();
	void generateKernelInstance(int buffersize);
	void generateInitCall();
	Value * generateDoBlockCall(Value * input);
    int getSegmentBlocks();

    void changeKernelInternalState(int idx, Value * stateValue);
    Value * getKernelInternalState(int idx);
    Value * getKernelInternalStatePtr(int idx);

private:
	Module *                            mMod;
    IDISA::IDISA_Builder *              iBuilder;
    std::string							mKernelName;
    int                                 mPredifinedStates;
    Type*                               mBitBlockType;
    std::vector<Type *> 				mStates;
    std::vector<Type *>                 mInputStreams;
    std::vector<Type *>                 mOutputStreams;
    std::vector<Type *>                 mInputScalars;
    std::vector<Type *>                 mOutputAccums;
    std::vector<std::string>            mInputStreamNames;
    std::vector<std::string>            mInputScalarNames;
    Function* 							mConstructor;
    Function*							mInitFunction;
    Function*							mDoBlockFunction;
    int                                 mBufferSize;
    int                                 mBlockSize;
    int                                 mSegmentBlocks;
    Type *                              mKernelStructType;
    Value*                              mKernelStruct;
    Value*                              mKernelStructParam;
};

#endif // KERNEL_H
