#pragma once

#include <toolchain/toolchain.h>

#include <llvm/ADT/Twine.h>
#include <llvm/IR/LegacyPassManager.h>

namespace llvm {

struct Align;

class BasicBlock;
class CallInst;
class Constant;
class LLVMContext;
class InvokeInst;
class MDNode;
class Module;
class raw_fd_ostream;
class StringRef;
class StructType;
class TargetMachine;
class Type;
class Value;
class VectorType;

template <class T>
class ArrayRef;
}

namespace llvm_version {

llvm::Constant * getSplat(const unsigned fieldCount, llvm::Constant *Elt);
llvm::VectorType * getVectorType(llvm::Type *ElementType, unsigned NumElements);
llvm::StructType * getTypeByName(llvm::Module *M, llvm::StringRef Name);
llvm::CallInst * CreateCall(llvm::Value *callee, llvm::ArrayRef< llvm::Value * > args, const llvm::Twine & Name = "");
llvm::InvokeInst * CreateInvoke(llvm::Value * const doSegment, llvm::BasicBlock * const invokeOk, llvm::BasicBlock * mRethrowException,  llvm::ArrayRef< llvm::Value * > args);
llvm::CallInst * CreateMemMove(void * data, llvm::Value * Dst, llvm::Value * Src, llvm::Value *Size, unsigned Align, bool isVolatile, llvm::MDNode *TBAATag, llvm::MDNode *ScopeTag, llvm::MDNode *NoAliasTag);

} // namespace llvm_version
