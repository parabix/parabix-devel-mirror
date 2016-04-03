#include <kernels/instance.h>
#include <IDISA/idisa_builder.h>
#include <llvm/Support/raw_ostream.h>

using namespace llvm;

namespace kernel {

SlabAllocator<Instance> Instance::mAllocator;

inline bool isPowerOfTwo(const unsigned x) {
    return (x != 0) && (x & (x - 1)) == 0;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getStreamSet
 *
 * Get the stream of the given offset value.
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Instance::getStreamSet(Type * const type, Value * const base, const unsigned index, const unsigned bufferSize) {
    assert ("Base stream set cannot be null!" && base);
    assert ("Illegal stream set index provided!" && ((index == 0 && bufferSize == 0) || (index < bufferSize)));
    Value * addr = base;
    if (bufferSize != 1) {
        Value * offset = iBuilder->CreateLoad(getBlockNo());
        assert (offset);
        if (index) {
            offset = iBuilder->CreateAdd(offset, ConstantInt::get(offset->getType(), index));
        }
        if (bufferSize != 0) {
            if (isPowerOfTwo(bufferSize)) {
                offset = iBuilder->CreateAnd(offset, ConstantInt::get(offset->getType(), bufferSize - 1));
            } else {
                offset = iBuilder->CreateURem(offset, ConstantInt::get(offset->getType(), bufferSize));
            }
        }
        addr = iBuilder->CreateGEP(base, offset);
    }
    return iBuilder->CreatePointerCast(addr, type->getPointerTo());
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateDoBlockCall
 ** ------------------------------------------------------------------------------------------------------------- */
Value * Instance::CreateDoBlockCall() {
    assert (mDefinition->getDoBlockFunction());
    std::vector<Value *> params;
    params.push_back(mKernelState);
    if (mInputScalarSet) {
        params.push_back(mKernelState);
    }
    if (mInputStreamSet) {
        for (unsigned offset : mDefinition->getInputStreamOffsets()) {
            params.push_back(getInputStreamSet(offset));
        }
    }
    if (mOutputScalarSet) {
        params.push_back(mOutputScalarSet);
    }
    if (mOutputStreamSet) {
        params.push_back(getOutputStreamSet());
    }
    return iBuilder->CreateCall(mDefinition->getDoBlockFunction(), params);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief clearOutputStreamSet
 ** ------------------------------------------------------------------------------------------------------------- */
void Instance::clearOutputStreamSet() {
    Value * ptr = getOutputStreamSet();
    Type * type = ptr->getType();
    assert(type->isPtrOrPtrVectorTy());
    type = type->getPointerElementType();
    assert(type->isStructTy() || type->isArrayTy());
    iBuilder->CreateStore(Constant::getNullValue(type), ptr);
}

}
