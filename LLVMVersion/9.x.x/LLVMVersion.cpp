#include "LLVMVersion.h"

#include <llvm/IR/Constants.h>

using namespace llvm;

namespace llvm_version {

Constant * getSplat(const unsigned fieldCount, Constant *Elt) {
  return ConstantVector::getSplat(fieldCount, Elt);
}

}