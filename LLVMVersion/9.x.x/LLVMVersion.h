namespace llvm {

class Constant;

}

namespace llvm_version {

llvm::Constant * getSplat(const unsigned fieldCount, llvm::Constant *Elt);

} // namespace llvm_version