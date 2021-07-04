/* Some wrappers around the Clang/LLVM API used to preserve
*  compatibility with older API versions while reducing
*  the ifdef clutter in the rest of the subtree.
*/
#pragma once

#include <toolchain/toolchain.h>
#include <cbuilder_config.hpp>

#include <llvm/ADT/Twine.h>
#include <llvm/IR/LegacyPassManager.h>
#if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(10, 0, 0)
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
llvm::CallInst * CreateCall(CBuilderBase * b, llvm::Value *callee,
                            llvm::ArrayRef< llvm::Value * > args, llvm::ArrayRef< llvm::OperandBundleDef > OpBundles,
                            const llvm::Twine Name = "");
llvm::InvokeInst * CreateInvoke(CBuilderBase * b, llvm::Value * const Callee,
                                llvm::BasicBlock * const NormalDest, llvm::BasicBlock * UnwindDest, llvm::ArrayRef< llvm::Value * > args,
                                const llvm::Twine Name = "");
llvm::InvokeInst * CreateInvoke(CBuilderBase * b, llvm::Value * Callee, llvm::BasicBlock * NormalDest, llvm::BasicBlock * UnwindDest,
                                llvm::ArrayRef<llvm::Value *> args, llvm::ArrayRef<llvm::OperandBundleDef> OpBundles,
                                const llvm::Twine Name = "");
llvm::CallInst * CreateMemMove(CBuilderBase * b, llvm::Value * Dst, llvm::Value * Src, llvm::Value *Size, unsigned Align, bool isVolatile,
                               llvm::MDNode * TBAATag, llvm::MDNode * ScopeTag, llvm::MDNode * NoAliasTag);
llvm::CallInst * CreateMemCpy(CBuilderBase * b, llvm::Value * Dst, llvm::Value * Src, llvm::Value * Size, unsigned Align,
                              bool isVolatile, llvm::MDNode * TBAATag, llvm::MDNode * TBAAStructTag, llvm::MDNode * ScopeTag, llvm::MDNode * NoAliasTag);

} // namespace llvm_version
