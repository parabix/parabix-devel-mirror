/*
 *  Copyright (c) 2016 International Characters.
 *  This software is licensed to the public under the Open Software License 3.0.
 *  icgrep is a trademark of International Characters.
 */
#include <idisa/idisa_nvptx_builder.h>

#include <toolchain/toolchain.h>
#include <llvm/IR/InlineAsm.h>
#include <llvm/IR/Module.h>

using namespace llvm;

namespace IDISA {

std::string IDISA_NVPTX20_Builder::getBuilderUniqueName() { return "NVPTX20_" + std::to_string(groupThreads);}

unsigned IDISA_NVPTX20_Builder::getGroupThreads() const{
    return groupThreads;
}

Value * IDISA_NVPTX20_Builder::bitblock_any(Value * val) {
    Type * const int32ty = getInt32Ty();
    FunctionType * const barrierOrFuncTy = FunctionType::get(int32ty, {int32ty});
    Module * const m = getModule();
    Function * barrierOrFunc = Function::Create(barrierOrFuncTy, Function::ExternalLinkage, "llvm.nvvm.barrier0.or", m);
    Value * nonZero_i1 = CreateICmpUGT(val, ConstantInt::getNullValue(mBitBlockType));
    Value * nonZero_i32 = CreateZExt(CreateBitCast(nonZero_i1, getInt1Ty()), int32ty);
    Value * anyNonZero = CreateCall(barrierOrFunc, nonZero_i32);
    return CreateICmpNE(anyNonZero,  ConstantInt::getNullValue(int32ty));
}

Value * IDISA_NVPTX20_Builder::bitblock_mask_from(Value * pos, const bool safe){
    Type * const int64ty = getInt64Ty();
    Value * id = CreateCall(tidFunc);
    Value * id64 = CreateZExt(id, int64ty);
    Value * threadSize = getInt64(groupThreads);
    Value * fullBlocks = CreateUDiv(pos, threadSize);
    Value * finalBlockSelect = CreateSExt(CreateICmpEQ(id64, fullBlocks), int64ty);
    Value * finalBlockMask = CreateShl(getInt64(-1), CreateURem(pos, threadSize));
    Value * unusedBlockMask = CreateSExt(CreateICmpUGT(id64, fullBlocks), int64ty);
    return CreateBitCast(CreateOr(CreateAnd(finalBlockMask, finalBlockSelect), unusedBlockMask), mBitBlockType);
}

Value * IDISA_NVPTX20_Builder::bitblock_set_bit(Value * pos, const bool safe){
    Type * const int64ty = getInt64Ty();
    Value * id = CreateCall(tidFunc);
    Value * id64 = CreateZExt(id, int64ty);
    Value * threadSize = getInt64(groupThreads);
    Value * fullBlocks = CreateUDiv(pos, threadSize);
    Value * finalBlockSelect = CreateSExt(CreateICmpEQ(id64, fullBlocks), int64ty);
    Value * finalBlockMask = CreateShl(getInt64(1), CreateURem(pos, threadSize));
    return CreateBitCast(CreateAnd(finalBlockMask, finalBlockSelect), mBitBlockType);
}

std::pair<Value *, Value *> IDISA_NVPTX20_Builder::bitblock_advance(Value * a, Value * shiftin, unsigned shift) {
    Value * id = CreateCall(tidFunc);
    Value * retVal = CreateCall(mLongAdvanceFunc, {id, a, CreateBitCast(getInt64(shift), mBitBlockType), shiftin});
    Value * shifted = CreateExtractValue(retVal, {0});
    Value * shiftOut = CreateExtractValue(retVal, {1});
    return std::pair<Value *, Value *>(shiftOut, shifted);
}

std::pair<Value *, Value *> IDISA_NVPTX20_Builder::bitblock_add_with_carry(Value * a, Value * b, Value * carryIn) {
    Value * id = CreateCall(tidFunc);
    Value * retVal = CreateCall(mLongAddFunc, {id, a, b, carryIn});
    Value * sum = CreateExtractValue(retVal, {0});
    Value * carry_out_strm = CreateExtractValue(retVal, {1});
    return std::pair<Value *, Value *>(carry_out_strm, sum);
}

void IDISA_NVPTX20_Builder::CreateGlobals(){
    Module * const m = getModule();
    Type * const carryTy = ArrayType::get(mBitBlockType, groupThreads+1);
    carry = new GlobalVariable(*m,
        /*Type=*/carryTy,
        /*isConstant=*/false,
        /*Linkage=*/GlobalValue::InternalLinkage,
        /*Initializer=*/0,
        /*Name=*/"carry",
        /*InsertBefore*/nullptr,
        /*TLMode */GlobalValue::NotThreadLocal,
        /*AddressSpace*/ 3,
        /*isExternallyInitialized*/false);

    Type * const bubbleTy = ArrayType::get(mBitBlockType, groupThreads);

    bubble = new GlobalVariable(*m,
        /*Type=*/bubbleTy,
        /*isConstant=*/false,
        /*Linkage=*/GlobalValue::InternalLinkage,
        /*Initializer=*/0,
        /*Name=*/"bubble",
        /*InsertBefore*/nullptr,
        /*TLMode */GlobalValue::NotThreadLocal,
        /*AddressSpace*/ 3,
        /*isExternallyInitialized*/false);

    ConstantAggregateZero* carryConstArray = ConstantAggregateZero::get(carryTy);
    carry->setInitializer(carryConstArray);
    ConstantAggregateZero* bubbleConstAray = ConstantAggregateZero::get(bubbleTy);
    bubble->setInitializer(bubbleConstAray);

}

void IDISA_NVPTX20_Builder::CreateBuiltinFunctions(){
    Type * const voidTy = getVoidTy();
    Type * const int32ty = getInt32Ty();
    Module * const m = getModule();
    FunctionType * barrierTy = FunctionType::get(voidTy, {});
    FunctionType * tIdTy = FunctionType::get(int32ty, {});

    barrierFunc = Function::Create(barrierTy, Function::ExternalLinkage, "llvm.nvvm.barrier0", m);
    tidFunc = Function::Create(tIdTy, Function::ExternalLinkage, "llvm.nvvm.read.ptx.sreg.tid.x", m);
}

void IDISA_NVPTX20_Builder::CreateLongAdvanceFunc(){
    Type * const int32ty = getInt32Ty();
    Module * const m = getModule();
    Type * returnType = StructType::get(m->getContext(), {mBitBlockType, mBitBlockType});
    FunctionType * longAdvTy = FunctionType::get(returnType, {int32ty, mBitBlockType, mBitBlockType, mBitBlockType}, false);
    mLongAdvanceFunc = Function::Create(longAdvTy, Function::InternalLinkage, "LongAdvance", m);
    mLongAdvanceFunc->setCallingConv(CallingConv::C);
    auto args = mLongAdvanceFunc->arg_begin();

    Value * const id = &*(args++);
    id->setName("id");
    Value * const val = &*(args++);
    val->setName("val");
    Value * const shftAmount = &*(args++);
    shftAmount->setName("shftAmount");
    Value * const blockCarry = &*(args++);
    blockCarry->setName("blockCarry");

    SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", mLongAdvanceFunc,0));

    Value * firstCarryPtr = CreateGEP(carry, {getInt32(0), getInt32(0)});
    CreateStore(blockCarry, firstCarryPtr);

    Value * adv0 = CreateShl(val, shftAmount);
    Value * nextid = CreateAdd(id, getInt32(1));
    Value * carryNextPtr = CreateGEP(carry, {getInt32(0), nextid});
    Value * lshr0 = CreateLShr(val, CreateSub(CreateBitCast(getInt64(64), mBitBlockType), shftAmount));
    CreateStore(lshr0, carryNextPtr);

    CreateCall(barrierFunc);

    Value * lastCarryPtr = CreateGEP(carry, {getInt32(0), getInt32(groupThreads)});
    Value * blockCarryOut = CreateLoad(lastCarryPtr, "blockCarryOut");

    Value * carryPtr = CreateGEP(carry, {getInt32(0), id});
    Value * carryVal = CreateLoad(carryPtr, "carryVal");
    Value * adv1 = CreateOr(adv0, carryVal);


    Value * retVal = UndefValue::get(returnType);
    retVal = CreateInsertValue(retVal, adv1, 0);
    retVal = CreateInsertValue(retVal, blockCarryOut, 1);
    CreateRet(retVal);

}



void IDISA_NVPTX20_Builder::CreateLongAddFunc(){
  Type * const int64ty = getInt64Ty();
  Type * const int32ty = getInt32Ty();
  Module * const m = getModule();

  Type * returnType = StructType::get(m->getContext(), {mBitBlockType, mBitBlockType});
  FunctionType * longAddTy = FunctionType::get(returnType, {int32ty, mBitBlockType, mBitBlockType, mBitBlockType}, false);
  mLongAddFunc = Function::Create(longAddTy, Function::InternalLinkage, "LongAdd", m);
  mLongAddFunc->setCallingConv(CallingConv::C);
  Function::arg_iterator args = mLongAddFunc->arg_begin();

  Value * const id = &*(args++);
  id->setName("id");
  Value * const valA = &*(args++);
  valA->setName("valA");
  Value * const valB = &*(args++);
  valB->setName("valB");
  Value * const blockCarry = &*(args++);
  blockCarry->setName("blockCarry");

  BasicBlock * entryBlock = BasicBlock::Create(m->getContext(), "entry", mLongAddFunc, 0);
  BasicBlock * bubbleCalculateBlock = BasicBlock::Create(m->getContext(), "bubbleCalculate", mLongAddFunc, 0);
  BasicBlock * bubbleSetBlock = BasicBlock::Create(m->getContext(), "bubbleSet", mLongAddFunc, 0);

  SetInsertPoint(entryBlock);

  Value * id64 = CreateZExt(id, int64ty);

  Value * partial_sum = CreateAdd(valA, valB);
  Value * gen = CreateAnd(valA, valB);
  Value * prop = CreateXor(valA, valB);

  Value * carryPtr = CreateGEP(carry, {getInt32(0), id});
  Value * carryInitVal = CreateAnd(CreateOr(gen, CreateAnd(prop, CreateNot(partial_sum))), CreateBitCast(getInt64(0x8000000000000000), mBitBlockType));
  carryInitVal = CreateLShr(carryInitVal, CreateBitCast(CreateSub(getInt64(63), id64), mBitBlockType));
  CreateStore(carryInitVal, carryPtr);

  Value * bubbleCond = CreateICmpEQ(CreateAdd(CreateBitCast(partial_sum, int64ty), getInt64(1)), getInt64(0));
  CreateCondBr(bubbleCond, bubbleCalculateBlock, bubbleSetBlock);

  SetInsertPoint(bubbleCalculateBlock);
  Value * calcBubble = CreateBitCast(CreateShl(getInt64(1), id64), mBitBlockType);
  CreateBr(bubbleSetBlock);

  SetInsertPoint(bubbleSetBlock);
  PHINode * bubbleInitVal = CreatePHI(mBitBlockType, 2, "bubbleInitVal");
  bubbleInitVal->addIncoming(CreateBitCast(getInt64(0), mBitBlockType), entryBlock);
  bubbleInitVal->addIncoming(calcBubble, bubbleCalculateBlock);

  Value * bubblePtr = CreateGEP(bubble, {getInt32(0), id});
  CreateStore(bubbleInitVal, bubblePtr);

  CreateCall(barrierFunc);

  Value * carryVal = carryInitVal;
  Value * bubbleVal = bubbleInitVal;

  for (unsigned offset = groupThreads/2; offset>0; offset=offset>>1){
    Value * carryOffsetPtr = CreateGEP(carry, {getInt32(0), CreateXor(id, getInt32(offset))});
    carryVal = CreateOr(carryVal, CreateLoad(carryOffsetPtr));
    CreateStore(carryVal, carryPtr);
    Value * bubbleOffsetPtr = CreateGEP(bubble, {getInt32(0), CreateXor(id, getInt32(offset))});
    bubbleVal = CreateOr(bubbleVal, CreateLoad(bubbleOffsetPtr));
    CreateStore(bubbleVal, bubblePtr);
    CreateCall(barrierFunc);
  }

  Value * firstCarryPtr = CreateGEP(carry, {getInt32(0), getInt32(0)});
  Value * carryVal0 = CreateLoad(firstCarryPtr, "carry0");
  Value * carry_mask = CreateOr(CreateShl(carryVal0, 1), blockCarry);
  Value * firstBubblePtr = CreateGEP(bubble, {getInt32(0), getInt32(0)});
  Value * bubble_mask = CreateLoad(firstBubblePtr, "bubble_mask");

  Value * s = CreateAnd(CreateAdd(carry_mask, bubble_mask), CreateNot(bubble_mask));
  Value * inc = CreateOr(s, CreateSub(s, carry_mask));
  Value * rslt = CreateAdd(partial_sum, CreateAnd(CreateLShr(inc, CreateBitCast(id64, mBitBlockType)), CreateBitCast(getInt64(1), mBitBlockType)));

  Value * blockCarryOut = CreateLShr(CreateOr(carryVal0, CreateAnd(bubble_mask, inc)), 63);

  Value * retVal = UndefValue::get(returnType);
  retVal = CreateInsertValue(retVal, rslt, 0);
  retVal = CreateInsertValue(retVal, blockCarryOut, 1);
  CreateRet(retVal);

}

void IDISA_NVPTX20_Builder::CreateBallotFunc(){
    Type * const int32ty = getInt32Ty();
    Module * const m = getModule();
    FunctionType * ballotTy = FunctionType::get(int32ty, {int1Ty});
    Function * const ballotFn = Function::Create(ballotTy, Function::InternalLinkage, "ballot_nvptx", m);

    ballotFn->setCallingConv(CallingConv::C);
    auto args = ballotFn->arg_begin();

    Value * const input = &*(args++);
    input->setName("input");

    SetInsertPoint(BasicBlock::Create(m->getContext(), "entry", ballotFn, 0));

    Value * conv = CreateZExt(input, int32ty);

    const char * AsmStream = "{.reg .pred %p1;"
                             "setp.ne.u32 %p1, $1, 0;"
                             "vote.ballot.b32  $0, %p1;}";
    FunctionType * AsmFnTy = FunctionType::get(int32ty, int32ty, false);
    InlineAsm *IA = InlineAsm::get(AsmFnTy, AsmStream, "=r,r", true, false);
    CallInst * result = CreateCall(IA, conv);
#if LLVM_VERSION_INTEGER < LLVM_VERSION_CODE(5, 0, 0)
    result->addAttribute(AttributeSet::FunctionIndex, Attribute::NoUnwind);
#else
    result->addAttribute(AttributeList::FunctionIndex, Attribute::NoUnwind);
#endif
    CreateRet(result);
}

LoadInst * IDISA_NVPTX20_Builder::CreateAtomicLoadAcquire(Value * ptr) {
    return CreateLoad(ptr);
}

StoreInst * IDISA_NVPTX20_Builder::CreateAtomicStoreRelease(Value * val, Value * ptr) {
    return CreateStore(val, ptr);
}

void IDISA_NVPTX20_Builder::CreateBaseFunctions() {
    CreateGlobals();
    CreateBuiltinFunctions();
    CreateLongAdvanceFunc();
    CreateLongAddFunc();
    CreateBallotFunc();
}

#ifdef HAS_ADDRESS_SANITIZER
LoadInst * IDISA_NVPTX20_Builder::CreateLoad(Value * Ptr, const char * Name) {
    return IRBuilder<>::CreateLoad(Ptr, Name);
}

LoadInst * IDISA_NVPTX20_Builder::CreateLoad(Value * Ptr, const Twine & Name) {
    return IRBuilder<>::CreateLoad(Ptr, Name);
}

LoadInst * IDISA_NVPTX20_Builder::CreateLoad(Type * Ty, Value * Ptr, const Twine & Name) {
    return IRBuilder<>::CreateLoad(Ty, Ptr, Name);
}

LoadInst * IDISA_NVPTX20_Builder::CreateLoad(Value * Ptr, bool isVolatile, const Twine & Name) {
    return IRBuilder<>::CreateLoad(Ptr, isVolatile, Name);
}

StoreInst * IDISA_NVPTX20_Builder::CreateStore(Value * Val, Value * Ptr, bool isVolatile) {
    return IRBuilder<>::CreateStore(Val, Ptr, isVolatile);
}
#endif

}
