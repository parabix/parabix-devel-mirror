#pragma once

#include <toolchain/toolchain.h>
#include <cbuilder_config.hpp>

#include <llvm/ADT/Twine.h>
#include <llvm/IR/LegacyPassManager.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 9, 0)
#include <llvm/Transforms/Scalar/GVN.h>
#endif
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(6, 0, 0)
#include <llvm/Transforms/Scalar/SROA.h>
#endif
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
#include <llvm/Transforms/InstCombine/InstCombine.h>
#include <llvm/Transforms/Utils.h>
#endif

#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(4, 0, 0)
#include <llvm/ExecutionEngine/MCJIT.h>
#endif

#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(8, 0, 0)
#include <llvm/IR/LegacyPassManager.h>
#else
#include <llvm/IR/PassTimingInfo.h>
#endif
#if LLVM_VERSION_MAJOR >= 10
#include <llvm/Support/Host.h>
#include <llvm/IR/IntrinsicsNVPTX.h>
#include <llvm/IR/IntrinsicsX86.h>
#endif

namespace llvm {

struct Align;

class BasicBlock;
class CallInst;
class Constant;
class LLVMContext;
class InvokeInst;
class IRBuilderBase;
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

#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(10, 0, 0)
    typedef unsigned            AlignType;
#else
    typedef llvm::Align         AlignType;
#endif

llvm::Constant * getSplat(const unsigned fieldCount, llvm::Constant *Elt);
llvm::VectorType * getVectorType(llvm::Type *ElementType, unsigned NumElements);
llvm::StructType * getTypeByName(llvm::Module *M, llvm::StringRef Name);
llvm::CallInst * CreateCall(CBuilderBase * b, llvm::Value *callee,
                            llvm::ArrayRef< llvm::Value * > args, const llvm::Twine Name = "");
llvm::InvokeInst * CreateInvoke(CBuilderBase * b, llvm::Value * const Callee,
                                llvm::BasicBlock * const NormalDest, llvm::BasicBlock * UnwindDest, llvm::ArrayRef< llvm::Value * > args,
                                const llvm::Twine Name = "");
llvm::CallInst * CreateMemMove(CBuilderBase * b, llvm::Value * Dst, llvm::Value * Src, llvm::Value *Size, unsigned Align, bool isVolatile,
                               llvm::MDNode *TBAATag, llvm::MDNode *ScopeTag, llvm::MDNode *NoAliasTag);
llvm::CallInst * CreateMemCpy(CBuilderBase * b, llvm::Value * Dst, llvm::Value * Src, llvm::Value * Size, unsigned Align,
                              bool isVolatile, llvm::MDNode * TBAATag, llvm::MDNode * TBAAStructTag, llvm::MDNode * ScopeTag, llvm::MDNode * NoAliasTag);
void checkAddPassesToEmitFile(llvm::TargetMachine * mTarget, std::unique_ptr<llvm::legacy::PassManager> const & mPassManager,
                              std::unique_ptr<llvm::raw_fd_ostream> & mASMOutputStream);

} // namespace llvm_version
