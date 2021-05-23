#include "LLVMVersion.h"

#include <llvm/IR/Constants.h>
#include <llvm/Support/TypeSize.h>
#include <llvm/IR/DerivedTypes.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace llvm_version {

Constant * getSplat(const unsigned fieldCount, Constant *Elt) {
  return ConstantVector::getSplat({fieldCount, false}, Elt);
}

VectorType * getVectorType(Type *ElementType, unsigned NumElements) {
  //use FixedVectorType by default
  return VectorType::get(ElementType, {NumElements, false});
}

StructType * getTypeByName(Module *M, StringRef Name) {
  return M->getTypeByName(Name);
}

}