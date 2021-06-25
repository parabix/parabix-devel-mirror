#include <kernel/core/relationship.h>

#include <llvm/IR/Constant.h>
#include <llvm/IR/DerivedTypes.h>

#include "LLVMVersion.h"

using namespace llvm;
using namespace llvm_version;

namespace kernel {

inline VectorType * LLVM_READNONE getStreamTy(LLVMContext & C, const unsigned FieldWidth) {
    return llvm_version::getVectorType(IntegerType::getIntNTy(C, FieldWidth), unsigned{0});
}

inline ArrayType * LLVM_READNONE getStreamSetTy(LLVMContext & C, const unsigned NumElements, const unsigned FieldWidth) {
    return ArrayType::get(getStreamTy(C, FieldWidth), NumElements);
}

unsigned StreamSet::getNumElements() const {
    return mType->getArrayNumElements();
}

unsigned StreamSet::getFieldWidth() const {
    return mType->getArrayElementType()->getContainedType(0)->getIntegerBitWidth();
}

std::string StreamSet::shapeString() {
    return std::to_string(getNumElements()) + "x" + std::to_string(getFieldWidth());
}

unsigned Scalar::getFieldWidth() const {
    return mType->getIntegerBitWidth();
}

StreamSet::StreamSet(LLVMContext & C, const unsigned NumElements, const unsigned FieldWidth) noexcept
: Relationship(Relationship::ClassTypeId::StreamSet, getStreamSetTy(C, NumElements, FieldWidth)) {

}

Scalar::Scalar(const ClassTypeId typeId, llvm::Type * type) noexcept
: Relationship(typeId, type){

}

Scalar::Scalar(not_null<Type *> type) noexcept
: Scalar(Relationship::ClassTypeId::Scalar, type.get()){

}

ScalarConstant::ScalarConstant(not_null<Constant *> constant) noexcept
: Scalar(Relationship::ClassTypeId::ScalarConstant, constant->getType())
, mConstant(constant.get()) {

}

}
