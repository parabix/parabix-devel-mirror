#include "kernel_builder.h"
#include <toolchain/toolchain.h>
#include <kernels/kernel.h>
#include <kernels/streamset.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/IR/Module.h>

using namespace llvm;
using namespace parabix;

inline static bool is_power_2(const uint64_t n) {
    return ((n & (n - 1)) == 0) && n;
}

namespace kernel {

using Port = Kernel::Port;

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const instance, Value * const index) {
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(instance, "getScalarFieldPtr: instance cannot be null!");
    }
    return CreateGEP(instance, {getInt32(0), index});
}

Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const handle, const std::string & fieldName) {
    return getScalarFieldPtr(handle, getInt32(mKernel->getScalarIndex(fieldName)));
}

llvm::Value * KernelBuilder::getScalarFieldPtr(llvm::Value * const index) {
    return getScalarFieldPtr(mKernel->getInstance(), index);
}

llvm::Value *KernelBuilder:: getScalarFieldPtr(const std::string & fieldName) {
    return getScalarFieldPtr(mKernel->getInstance(), fieldName);
}

Value * KernelBuilder::getScalarField(const std::string & fieldName) {
    return CreateLoad(getScalarFieldPtr(fieldName), fieldName);
}

void KernelBuilder::setScalarField(const std::string & fieldName, Value * value) {
    CreateStore(value, getScalarFieldPtr(fieldName));
}

Value * KernelBuilder::getStreamHandle(const std::string & name) {
    Value * const ptr = getScalarField(name + Kernel::BUFFER_PTR_SUFFIX);
    if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
        CreateAssert(ptr, name + " handle cannot be null!");
    }
    return ptr;
}

LoadInst * KernelBuilder::acquireLogicalSegmentNo() {
    return CreateAtomicLoadAcquire(getScalarFieldPtr(Kernel::LOGICAL_SEGMENT_NO_SCALAR));
}

void KernelBuilder::releaseLogicalSegmentNo(Value * nextSegNo) {
    CreateAtomicStoreRelease(nextSegNo, getScalarFieldPtr(Kernel::LOGICAL_SEGMENT_NO_SCALAR));
}

Value * KernelBuilder::getCycleCountPtr() {
    return getScalarFieldPtr(Kernel::CYCLECOUNT_SCALAR);
}

Value * KernelBuilder::getInternalItemCount(const std::string & name, const std::string & suffix) {
    const ProcessingRate & rate = mKernel->getBinding(name).getRate();
    Value * itemCount = nullptr;
    if (LLVM_UNLIKELY(rate.isRelative())) {
        Port port; unsigned index;
        std::tie(port, index) = mKernel->getStreamPort(rate.getReference());
        if (port == Port::Input) {
            itemCount = getProcessedItemCount(rate.getReference());
        } else {
            itemCount = getProducedItemCount(rate.getReference());
        }
        const auto & r = rate.getRate();
        if (r.numerator() != 1) {
            itemCount = CreateMul(itemCount, ConstantInt::get(itemCount->getType(), r.numerator()));
        }
        if (r.denominator() != 1) {
            itemCount = CreateExactUDiv(itemCount, ConstantInt::get(itemCount->getType(), r.denominator()));
        }
    } else if (LLVM_UNLIKELY(rate.isPopCount())) {
        Port port; unsigned index;
        std::tie(port, index) = mKernel->getStreamPort(rate.getReference());




    } else {
        itemCount = getScalarField(name + suffix);
    }
    return itemCount;
}

void KernelBuilder::setInternalItemCount(const std::string & name, const std::string & suffix, llvm::Value * const value) {
    const ProcessingRate & rate = mKernel->getBinding(name).getRate();
    if (LLVM_UNLIKELY(rate.isDerived())) {
        report_fatal_error("Cannot set item count: " + name + " is a Derived rate");
    }
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": " + name + suffix, value);
    }
    setScalarField(name + suffix, value);
}


Value * KernelBuilder::getAvailableItemCount(const std::string & name) {
    const auto & inputs = mKernel->getStreamInputs();
    for (unsigned i = 0; i < inputs.size(); ++i) {
        if (inputs[i].getName() == name) {
            return mKernel->getAvailableItemCount(i);
        }
    }
    return nullptr;
}

Value * KernelBuilder::getTerminationSignal() {
    return CreateICmpNE(getScalarField(Kernel::TERMINATION_SIGNAL), getSize(0));
}

void KernelBuilder::setTerminationSignal(llvm::Value * const value) {
    assert (value->getType() == getInt1Ty());
    if (codegen::DebugOptionIsSet(codegen::TraceCounts)) {
        CallPrintIntToStderr(mKernel->getName() + ": setTerminationSignal", value);
    }
    setScalarField(Kernel::TERMINATION_SIGNAL, CreateZExt(value, getSizeTy()));
}

Value * KernelBuilder::getLinearlyAccessibleItems(const std::string & name, Value * fromPosition, Value * avail, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getLinearlyAccessibleItems(this, getStreamHandle(name), fromPosition, avail, reverse);
}

Value * KernelBuilder::getLinearlyWritableItems(const std::string & name, Value * fromPosition, bool reverse) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getLinearlyWritableItems(this, getStreamHandle(name), fromPosition, getConsumedItemCount(name), reverse);
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isConstantZero
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isConstantZero(Value * const v) {
    return isa<ConstantInt>(v) && cast<ConstantInt>(v)->isNullValue();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief isConstantOne
 ** ------------------------------------------------------------------------------------------------------------- */
inline bool isConstantOne(Value * const v) {
    return isa<ConstantInt>(v) && cast<ConstantInt>(v)->isOne();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getItemWidth
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned getItemWidth(const Type * ty) {
    if (LLVM_LIKELY(isa<ArrayType>(ty))) {
        ty = ty->getArrayElementType();
    }
    return cast<IntegerType>(ty->getVectorElementType())->getBitWidth();
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief getFieldWidth
 ** ------------------------------------------------------------------------------------------------------------- */
inline unsigned getFieldWidth(const unsigned bitWidth, const unsigned blockWidth) {
    for (unsigned k = 16; k <= blockWidth; k *= 2) {
        if ((bitWidth & (k - 1)) != 0) {
            return k / 2;
        }
    }
    return blockWidth;
}

/** ------------------------------------------------------------------------------------------------------------- *
 * @brief CreateStreamCpy
 ** ------------------------------------------------------------------------------------------------------------- */
void KernelBuilder::CreateStreamCpy(const std::string & name, Value * target, Value * targetOffset, Value * source, Value * sourceOffset, Value * itemsToCopy, const unsigned itemAlignment) {

    assert (target && targetOffset);
    assert (source && sourceOffset);
    assert (target->getType() == source->getType());
    assert (target->getType()->isPointerTy());
    assert (isConstantZero(targetOffset) || isConstantZero(sourceOffset));

    const StreamSetBuffer * const buf = mKernel->getAnyStreamSetBuffer(name);

    const auto itemWidth = getItemWidth(buf->getBaseType());
    assert ("invalid item width" && is_power_2(itemWidth));
    const auto blockWidth = getBitBlockWidth();
    // Although our item width may be n bits, if we know we're always processing m items per block, our field width
    // (w.r.t the stream copy) would be n*m. By taking this into account we can optimize and simplify the copy code.
    const auto fieldWidth = getFieldWidth(itemWidth * itemAlignment, blockWidth);
    const auto alignment = (fieldWidth + 7) / 8;

    if (LLVM_LIKELY(itemWidth < fieldWidth)) {
        const auto factor = fieldWidth / itemWidth;
        Constant * const FACTOR = getSize(factor);
        if (LLVM_UNLIKELY(codegen::DebugOptionIsSet(codegen::EnableAsserts))) {
            ConstantInt * const ALIGNMENT = getSize(alignment);
            const auto kernelName = mKernel->getName()+ ": " + name;
            CreateAssertZero(CreateURem(CreatePtrToInt(target, getSizeTy()), ALIGNMENT), kernelName + " target is misaligned (" + std::to_string(alignment) + ")");
            CreateAssertZero(CreateURem(targetOffset, FACTOR), kernelName + " target offset is misaligned (" + std::to_string(factor) + ")");
            CreateAssertZero(CreateURem(CreatePtrToInt(source, getSizeTy()), ALIGNMENT), kernelName + " source is misaligned (" + std::to_string(alignment) + ")");
            CreateAssertZero(CreateURem(sourceOffset, FACTOR), kernelName + " source offset is misaligned (" + std::to_string(factor) + ")");
        }
        targetOffset = CreateUDiv(targetOffset, FACTOR);
        sourceOffset = CreateUDiv(sourceOffset, FACTOR);
    }

    /*
       Streams are conceptually modelled as:

                                            BLOCKS

                                      A     B     C     D
           STREAM SET ELEMENT   1  |aaaaa|bbbbb|ccccc|dddd |
                                2  |eeeee|fffff|ggggg|hhhh |
                                3  |iiiii|jjjjj|kkkkk|llll |

       But the memory layout is actually:

           A_1   A_2   A_3   B_1   B_2   B_3   C_1   C_2   C_3   D_1   D_2   D_3

         |aaaaa|eeeee|iiiii|bbbbb|fffff|jjjjj|ccccc|ggggg|kkkkk|dddd |hhhh |llll |


       So if we're copying the entire stream set block or our stream set has one element, we can use memcpy.

       One compilication here is when the BlockSize of a stream is not equal to the BitBlockWidth.


    */

    Type * const fieldWidthTy = getIntNTy(fieldWidth);

    Value * const n = buf->getStreamSetCount(this, getStreamHandle(name));

    if (isConstantOne(n) || fieldWidth == blockWidth || (isConstantZero(targetOffset) && isConstantZero(sourceOffset))) {
        if (isConstantOne(n)) {
            if (LLVM_LIKELY(itemWidth < 8)) {
                itemsToCopy = CreateUDivCeil(itemsToCopy, getSize(8 / itemWidth));
            } else if (LLVM_UNLIKELY(itemWidth > 8)) {
                itemsToCopy = CreateMul(itemsToCopy, getSize(itemWidth / 8));
            }
        } else {
            if (LLVM_LIKELY(blockWidth > (itemWidth * 8))) {
                itemsToCopy = CreateUDivCeil(itemsToCopy, getSize(blockWidth / (8 * itemWidth)));
            } else if (LLVM_LIKELY(blockWidth < (itemWidth * 8))) {
                itemsToCopy = CreateUDivCeil(CreateMul(itemsToCopy, getSize(8)), getSize(blockWidth / itemWidth));
            }
            itemsToCopy = CreateMul(itemsToCopy, n);
        }
        PointerType * const ptrTy = fieldWidthTy->getPointerTo();
        target = CreateGEP(CreatePointerCast(target, ptrTy), targetOffset);
        source = CreateGEP(CreatePointerCast(source, ptrTy), sourceOffset);
        CreateMemCpy(target, source, itemsToCopy, alignment);

    } else { // either the target offset or source offset is non-zero but not both

        VectorType * const blockTy = getBitBlockType();
        PointerType * const blockPtrTy = blockTy->getPointerTo();

        target = CreatePointerCast(target, blockPtrTy, "target");
        source = CreatePointerCast(source, blockPtrTy, "source");

        assert ((blockWidth % fieldWidth) == 0);

        VectorType * const shiftTy = VectorType::get(fieldWidthTy, blockWidth / fieldWidth);
        Constant * const width = getSize(blockWidth / itemWidth);
        Constant * const ZERO = getSize(0);
        Constant * const ONE = getSize(1);
        BasicBlock * const entry = GetInsertBlock();

        if (isConstantZero(targetOffset)) {

            /*
                                                BLOCKS

                                          A     B     C     D
               SOURCE STREAM        1  |aaa--|bbbBB|cccCC|  dDD|
                                    2  |eee--|fffFF|gggGG|  hHH|
                                    3  |iii--|jjjJJ|kkkKK|  lLL|


                                          A     B     C     D
               TARGET STREAM        1  |BBaaa|CCbbb|DDccc|    d|
                                    2  |FFeee|GGfff|HHggg|    h|
                                    3  |JJiii|KKjjj|LLkkk|    l|
             */

            Value * const blocksToCopy = CreateMul(CreateUDiv(itemsToCopy, width), n);
            Value * const offset = CreateURem(sourceOffset, width);
            Value * const offsetVector = simd_fill(fieldWidth, CreateTrunc(offset, fieldWidthTy));
            Value * const remaining = CreateSub(width, offset);
            Value * const remainingVector = simd_fill(fieldWidth, CreateTrunc(remaining, fieldWidthTy));

            BasicBlock * const streamCopy = CreateBasicBlock(name + "PullCopy");
            BasicBlock * const streamCopyRemaining = CreateBasicBlock(name + "PullCopyRemaining");
            BasicBlock * const streamCopyEnd = CreateBasicBlock(name + "PullCopyEnd");

            CreateCondBr(CreateICmpNE(blocksToCopy, ZERO), streamCopy, streamCopyRemaining);

            SetInsertPoint(streamCopy);
            PHINode * const i = CreatePHI(getSizeTy(), 2);
            i->addIncoming(n, entry);
            Value * prior = CreateAlignedLoad(CreateGEP(source, CreateSub(i, n)), alignment);
            prior = CreateBitCast(CreateLShr(CreateBitCast(prior, shiftTy), offsetVector), blockTy);
            Value * value = CreateAlignedLoad(CreateGEP(source, i), alignment);
            value = CreateBitCast(CreateShl(CreateBitCast(value, shiftTy), remainingVector), blockTy);
            CreateAlignedStore(CreateOr(value, prior), CreateGEP(target, i), alignment);
            Value * const next_i = CreateAdd(i, ONE);
            i->addIncoming(next_i, streamCopy);
            CreateCondBr(CreateICmpNE(next_i, blocksToCopy), streamCopy, streamCopyRemaining);

            SetInsertPoint(streamCopyRemaining);
            PHINode * const j = CreatePHI(getSizeTy(), 2);
            j->addIncoming(blocksToCopy, entry);
            j->addIncoming(blocksToCopy, streamCopy);
            Value * final = CreateAlignedLoad(CreateGEP(source, j), alignment);
            final = CreateBitCast(CreateLShr(CreateBitCast(final, shiftTy), offsetVector), blockTy);
            CreateAlignedStore(final, CreateGEP(target, j), alignment);
            Value * const next_j = CreateAdd(j, ONE);
            j->addIncoming(next_j, streamCopyRemaining);
            CreateCondBr(CreateICmpNE(next_j, CreateAdd(blocksToCopy, n)), streamCopyRemaining, streamCopyEnd);

            SetInsertPoint(streamCopyEnd);

        } else if (isConstantZero(sourceOffset)) {

            /*
                                                BLOCKS

                                          A     B     C     D
               SOURCE STREAM        1  |AAAaa|BBBaa|CCCcc|    d|
                                    2  |EEEee|FFFff|GGGgg|    h|
                                    3  |IIIii|JJJjj|KKKkk|    l|


                                          A     B     C     D
               TARGET STREAM        1  |aa---|bbAAA|ccBBB| dCCC|
                                    2  |ee---|ffEEE|ggFFF| hGGG|
                                    3  |ii---|jjIII|kkJJJ| lKKK|

            */

            BasicBlock * const streamCopy = CreateBasicBlock(name + "PushCopy");
            BasicBlock * const streamCopyRemainingCond = CreateBasicBlock(name + "PushCopyRemainingCond");
            BasicBlock * const streamCopyRemaining = CreateBasicBlock(name + "PushCopyRemaining");
            BasicBlock * const streamCopyEnd = CreateBasicBlock(name + "PushCopyEnd");

            Value * const pos = CreateURem(targetOffset, width);
            Value * const copied = CreateSub(width, pos);
            Value * const copiedVector = simd_fill(fieldWidth, CreateTrunc(copied, fieldWidthTy));
            Value * const mask = CreateLShr(Constant::getAllOnesValue(shiftTy), copiedVector);
            Value * const offsetVector = simd_fill(fieldWidth, CreateTrunc(pos, fieldWidthTy));

            CreateBr(streamCopy);

            SetInsertPoint(streamCopy);
            PHINode * const i = CreatePHI(getSizeTy(), 2);
            i->addIncoming(ZERO, entry);
            Value * priorTargetValue = CreateAlignedLoad(CreateGEP(target, i), alignment);
            priorTargetValue = CreateBitCast(CreateAnd(CreateBitCast(priorTargetValue, shiftTy), mask), blockTy);
            Value * sourceValue = CreateAlignedLoad(CreateGEP(source, i), alignment);
            sourceValue = CreateBitCast(CreateShl(CreateBitCast(sourceValue, shiftTy), offsetVector), blockTy);
            CreateAlignedStore(CreateOr(sourceValue, priorTargetValue), CreateGEP(target, i), alignment);
            Value * const next_i = CreateAdd(i, ONE);
            i->addIncoming(next_i, streamCopy);
            CreateCondBr(CreateICmpNE(next_i, n), streamCopy, streamCopyRemainingCond);

            SetInsertPoint(streamCopyRemainingCond);
            Value * const blocksToCopy = CreateMul(CreateUDiv(CreateSub(itemsToCopy, copied), width), n);
            CreateCondBr(CreateICmpULT(copied, itemsToCopy), streamCopyRemaining, streamCopyEnd);

            SetInsertPoint(streamCopyRemaining);
            PHINode * const j = CreatePHI(getSizeTy(), 2);
            j->addIncoming(n, streamCopyRemainingCond);
            Value * prior = CreateAlignedLoad(CreateGEP(source, CreateSub(j, n)), alignment);
            prior = CreateBitCast(CreateShl(CreateBitCast(prior, shiftTy), offsetVector), blockTy);
            Value * value = CreateAlignedLoad(CreateGEP(source, j), alignment);
            value = CreateBitCast(CreateLShr(CreateBitCast(value, shiftTy), copiedVector), blockTy);
            CreateAlignedStore(CreateOr(value, prior), CreateGEP(target, j), alignment);
            Value * const next_j = CreateAdd(j, ONE);
            j->addIncoming(next_j, streamCopyRemaining);
            CreateCondBr(CreateICmpNE(next_j, blocksToCopy), streamCopyRemaining, streamCopyEnd);

            SetInsertPoint(streamCopyEnd);
        }
    }
}

Value * KernelBuilder::getConsumerLock(const std::string & name) {
    return getScalarField(name + Kernel::CONSUMER_SUFFIX);
}

void KernelBuilder::setConsumerLock(const std::string & name, Value * value) {
    setScalarField(name + Kernel::CONSUMER_SUFFIX, value);
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * streamIndex) {
    Value * const addr = mKernel->getStreamSetInputAddress(name);
    if (addr) {
        return CreateGEP(addr, {getInt32(0), streamIndex});
    } else {
        const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
        Value * const blockIndex = CreateLShr(getProcessedItemCount(name), std::log2(getBitBlockWidth()));
        return buf->getStreamBlockPtr(this, getStreamHandle(name), getBaseAddress(name), streamIndex, blockIndex, true);
    }
}

Value * KernelBuilder::loadInputStreamBlock(const std::string & name, Value * streamIndex) {
    return CreateBlockAlignedLoad(getInputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getInputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) {
    Value * const addr = mKernel->getStreamSetInputAddress(name);
    if (addr) {
        return CreateGEP(addr, {getInt32(0), streamIndex, packIndex});
    } else {
        const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
        Value * const blockIndex = CreateLShr(getProcessedItemCount(name), std::log2(getBitBlockWidth()));
        return buf->getStreamPackPtr(this, getStreamHandle(name), getBaseAddress(name), streamIndex, blockIndex, packIndex, true);
    }
}

Value * KernelBuilder::loadInputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex) {
    return CreateBlockAlignedLoad(getInputStreamPackPtr(name, streamIndex, packIndex));
}

Value * KernelBuilder::getInputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getStreamSetCount(this, getStreamHandle(name));
}

Value * KernelBuilder::getInputStreamBlockPtr(const std::string & name, Value * const streamIndex, Value * const blockOffset) {
    Value * const addr = mKernel->getStreamSetInputAddress(name);
    if (addr) {
        return CreateGEP(addr, {blockOffset, streamIndex});
    } else {
        const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
        Value * blockIndex = CreateLShr(getProcessedItemCount(name), std::log2(getBitBlockWidth()));
        blockIndex = CreateAdd(blockIndex, blockOffset);
        return buf->getStreamBlockPtr(this, getStreamHandle(name), getBaseAddress(name), streamIndex, blockIndex, true);
    }
}

Value * KernelBuilder::getOutputStreamBlockPtr(const std::string & name, Value * streamIndex) {
    Value * const addr = mKernel->getStreamSetOutputAddress(name);
    if (addr) {
        return CreateGEP(addr, {getInt32(0), streamIndex});
    } else {
        const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
        Value * const blockIndex = CreateLShr(getProducedItemCount(name), std::log2(getBitBlockWidth()));
        return buf->getStreamBlockPtr(this, getStreamHandle(name), getBaseAddress(name), streamIndex, blockIndex, false);
    }
}

StoreInst * KernelBuilder::storeOutputStreamBlock(const std::string & name, Value * streamIndex, Value * toStore) {
    return CreateBlockAlignedStore(toStore, getOutputStreamBlockPtr(name, streamIndex));
}

Value * KernelBuilder::getOutputStreamPackPtr(const std::string & name, Value * streamIndex, Value * packIndex) {
    Value * const addr = mKernel->getStreamSetOutputAddress(name);
    if (addr) {
        return CreateGEP(addr, {getInt32(0), streamIndex, packIndex});
    } else {
        const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
        Value * const blockIndex = CreateLShr(getProducedItemCount(name), std::log2(getBitBlockWidth()));
        return buf->getStreamPackPtr(this, getStreamHandle(name), getBaseAddress(name), streamIndex, blockIndex, packIndex, false);
    }
}

StoreInst * KernelBuilder::storeOutputStreamPack(const std::string & name, Value * streamIndex, Value * packIndex, Value * toStore) {
    return CreateBlockAlignedStore(toStore, getOutputStreamPackPtr(name, streamIndex, packIndex));
}

Value * KernelBuilder::getOutputStreamSetCount(const std::string & name) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getStreamSetCount(this, getStreamHandle(name));
}

Value * KernelBuilder::getRawInputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getInputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, getStreamHandle(name), absolutePosition);
}

Value * KernelBuilder::getRawOutputPointer(const std::string & name, Value * absolutePosition) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    return buf->getRawItemPointer(this, getStreamHandle(name), absolutePosition);
}

Value * KernelBuilder::getBaseAddress(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getBaseAddress(this, getStreamHandle(name));
}

void KernelBuilder::setBaseAddress(const std::string & name, Value * const addr) {
    return mKernel->getAnyStreamSetBuffer(name)->setBaseAddress(this, getStreamHandle(name), addr);
}

Value * KernelBuilder::getBufferedSize(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getBufferedSize(this, getStreamHandle(name));
}

void KernelBuilder::setBufferedSize(const std::string & name, Value * size) {
    mKernel->getAnyStreamSetBuffer(name)->setBufferedSize(this, getStreamHandle(name), size);
}

Value * KernelBuilder::getCapacity(const std::string & name) {
    return mKernel->getAnyStreamSetBuffer(name)->getCapacity(this, getStreamHandle(name));
}

void KernelBuilder::setCapacity(const std::string & name, Value * c) {
    mKernel->getAnyStreamSetBuffer(name)->setCapacity(this, getStreamHandle(name), c);
}

Value * KernelBuilder::getBlockAddress(const std::string & name, Value * blockIndex) {
    const StreamSetBuffer * const buf = mKernel->getAnyStreamSetBuffer(name);
    return buf->getBlockAddress(this, getStreamHandle(name), blockIndex);
}

void KernelBuilder::protectOutputStream(const std::string & name, const bool readOnly) {
    const StreamSetBuffer * const buf = mKernel->getOutputStreamSetBuffer(name);
    Value * const handle = getStreamHandle(name);
    Value * const base = buf->getBaseAddress(this, handle);
    Value * sz = ConstantExpr::getSizeOf(buf->getType());
    sz = CreateMul(sz, getInt64(buf->getBufferBlocks()));
    sz = CreateMul(sz, CreateZExt(buf->getStreamSetCount(this, handle), getInt64Ty()));
    CreateMProtect(base, sz, readOnly ? CBuilder::READ : (CBuilder::READ | CBuilder::WRITE));
}
    
CallInst * KernelBuilder::createDoSegmentCall(const std::vector<Value *> & args) {
    return mKernel->makeDoSegmentCall(*this, args);
}

Value * KernelBuilder::getAccumulator(const std::string & accumName) {
    auto results = mKernel->mOutputScalarResult;
    if (LLVM_UNLIKELY(results == nullptr)) {
        report_fatal_error("Cannot get accumulator " + accumName + " until " + mKernel->getName() + " has terminated.");
    }
    const auto & outputs = mKernel->getScalarOutputs();
    const auto n = outputs.size();
    if (LLVM_UNLIKELY(n == 0)) {
        report_fatal_error(mKernel->getName() + " has no output scalars.");
    } else {
        for (unsigned i = 0; i < n; ++i) {
            const Binding & b = outputs[i];
            if (b.getName() == accumName) {
                if (n == 1) {
                    return results;
                } else {
                    return CreateExtractValue(results, {i});
                }
            }
        }
        report_fatal_error(mKernel->getName() + " has no output scalar named " + accumName);
    }
}

BasicBlock * KernelBuilder::CreateConsumerWait() {
    const auto consumers = mKernel->getStreamOutputs();
    BasicBlock * const entry = GetInsertBlock();
    if (consumers.empty()) {
        return entry;
    } else {
        Function * const parent = entry->getParent();
        IntegerType * const sizeTy = getSizeTy();
        ConstantInt * const zero = getInt32(0);
        ConstantInt * const one = getInt32(1);
        ConstantInt * const size0 = getSize(0);

        Value * const segNo = acquireLogicalSegmentNo();
        const auto n = consumers.size();
        BasicBlock * load[n + 1];
        BasicBlock * wait[n];
        for (unsigned i = 0; i < n; ++i) {
            load[i] = BasicBlock::Create(getContext(), consumers[i].getName() + "Load", parent);
            wait[i] = BasicBlock::Create(getContext(), consumers[i].getName() + "Wait", parent);
        }
        load[n] = BasicBlock::Create(getContext(), "Resume", parent);
        CreateBr(load[0]);
        for (unsigned i = 0; i < n; ++i) {

            SetInsertPoint(load[i]);
            Value * const outputConsumers = getConsumerLock(consumers[i].getName());

            Value * const consumerCount = CreateLoad(CreateGEP(outputConsumers, {zero, zero}));
            Value * const consumerPtr = CreateLoad(CreateGEP(outputConsumers, {zero, one}));
            Value * const noConsumers = CreateICmpEQ(consumerCount, size0);
            CreateUnlikelyCondBr(noConsumers, load[i + 1], wait[i]);

            SetInsertPoint(wait[i]);
            PHINode * const consumerPhi = CreatePHI(sizeTy, 2);
            consumerPhi->addIncoming(size0, load[i]);

            Value * const conSegPtr = CreateLoad(CreateGEP(consumerPtr, consumerPhi));
            Value * const processedSegmentCount = CreateAtomicLoadAcquire(conSegPtr);
            Value * const ready = CreateICmpEQ(segNo, processedSegmentCount);
            assert (ready->getType() == getInt1Ty());
            Value * const nextConsumerIdx = CreateAdd(consumerPhi, CreateZExt(ready, sizeTy));
            consumerPhi->addIncoming(nextConsumerIdx, wait[i]);
            Value * const next = CreateICmpEQ(nextConsumerIdx, consumerCount);
            CreateCondBr(next, load[i + 1], wait[i]);
        }

        BasicBlock * const exit = load[n];
        SetInsertPoint(exit);
        return exit;
    }
}

}
