#include "LLVMVersion.h"

#include <codegen/CBuilder.h>

#include <llvm/IR/Constants.h>
#include <llvm/Support/Alignment.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/TypeSize.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Metadata.h>
#include <llvm/IR/Module.h>
#include <llvm/IR/Instructions.h>
#include <llvm/IR/Value.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/BasicBlock.h>
#include <llvm/ADT/APInt.h>

using namespace llvm;

namespace llvm_version {

  Constant * getSplat(const unsigned fieldCount, Constant *Elt) {
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(12, 0, 0)
      return ConstantVector::getSplat(ElementCount::get(fieldCount, false), Elt);
    #else
      return ConstantVector::getSplat({fieldCount, false}, Elt);
    #endif
  }

  VectorType * getVectorType(Type *ElementType, unsigned NumElements) {
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(12, 0, 0)
      return VectorType::get(ElementType, ElementCount::get(NumElements, false));
    #else
      return VectorType::get(ElementType, {NumElements, false});
    #endif
  }

  StructType * getTypeByName(Module *M, StringRef Name) {
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(12, 0, 0)
      return StructType::getTypeByName(M->getContext(), Name);
    #else
      return M->getTypeByName(Name);
    #endif
  }

  CallInst * CreateCall(IRBuilderBase * b, Value *Callee, ArrayRef< Value * > args, const Twine Name) {
    #if LLVM_VERSION_MAJOR >= 11
        auto *calleePtrType = llvm::cast<llvm::PointerType>(Callee->getType());
        auto *calleeType = llvm::cast<llvm::FunctionType>(calleePtrType->getElementType());
        return b->CreateCall(calleeType, Callee, args, Name);
    #else
        return b->CreateCall(Callee, args, Name);
    #endif
  }

  InvokeInst * CreateInvoke(IRBuilderBase *b, Value * const Callee, BasicBlock * const NormalDest, BasicBlock * UnwindDest,
                      ArrayRef< Value * > args, const Twine Name) {
    #if LLVM_VERSION_MAJOR >= 11
        auto *calleePtrType = llvm::cast<llvm::PointerType>(Callee->getType());
        auto *calleeType = llvm::cast<llvm::FunctionType>(calleePtrType->getElementType());
        return b->CreateInvoke(calleeType, Callee, NormalDest, UnwindDest, args);
    #else
        return b->CreateInvoke(Callee, NormalDest, UnwindDest, args);
    #endif
}


  void checkAddPassesToEmitFile(TargetMachine * mTarget, std::unique_ptr<legacy::PassManager> const & mPassManager, std::unique_ptr<llvm::raw_fd_ostream> & mASMOutputStream) {
    #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(3, 7, 0)
      if (LLVM_UNLIKELY(codegen::ShowASMOption != codegen::OmittedOption)) {
        if (!codegen::ShowASMOption.empty()) {
            std::error_code error;
            mASMOutputStream = std::make_unique<raw_fd_ostream>(codegen::ShowASMOption, error, llvm::sys::fs::OpenFlags::F_None);
        } else {
            mASMOutputStream = std::make_unique<raw_fd_ostream>(STDERR_FILENO, false, true);
        }
        #if LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(10, 0, 0)
            if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, nullptr, CGFT_AssemblyFile))) {
        #elif LLVM_VERSION_INTEGER >= LLVM_VERSION_CODE(7, 0, 0)
            if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, nullptr, llvm::LLVMTargetMachine::CGFT_AssemblyFile))) {
        #else
            if (LLVM_UNLIKELY(mTarget->addPassesToEmitFile(*mPassManager, *mASMOutputStream, TargetMachine::CGFT_AssemblyFile))) {
        #endif
            report_fatal_error("LLVM error: could not add emit assembly pass");
      }
    }
    #endif
  }
}
