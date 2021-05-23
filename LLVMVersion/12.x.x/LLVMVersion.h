namespace llvm {

class Constant;
class Type;
class VectorType;
class Module;
class StructType;
class StringRef;

}

namespace llvm_version {

llvm::Constant * getSplat(const unsigned fieldCount, llvm::Constant *Elt);
llvm::VectorType * getVectorType(llvm::Type *ElementType, unsigned NumElements);
llvm::StructType * getTypeByName(llvm::Module *M, llvm::StringRef Name);

} // namespace llvm_version