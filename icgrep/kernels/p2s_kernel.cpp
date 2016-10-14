#include "p2s_kernel.h"
#include "kernels/kernel.h"
#include "IDISA/idisa_builder.h"
#include <llvm/IR/TypeBuilder.h>
#include <llvm/IR/Type.h>
#include <iostream>
#include <stdint.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/raw_ostream.h>



namespace kernel{
	
void p2s_step(IDISA::IDISA_Builder * iBuilder, Value * p0, Value * p1, Value * hi_mask, unsigned shift, Value * &s1, Value * &s0) {
    Value * t0 = iBuilder->simd_if(1, hi_mask, p0, iBuilder->simd_srli(16, p1, shift));
    Value * t1 = iBuilder->simd_if(1, hi_mask, iBuilder->simd_slli(16, p0, shift), p1);
    s1 = iBuilder->esimd_mergeh(8, t1, t0);
    s0 = iBuilder->esimd_mergel(8, t1, t0);
}

inline void p2s(IDISA::IDISA_Builder * iBuilder, Value * p[], Value * s[]) {
    Value * bit00004444[2];
    Value * bit22226666[2];
    Value * bit11115555[2];
    Value * bit33337777[2];
    p2s_step(iBuilder, p[0], p[4], iBuilder->simd_himask(8), 4, bit00004444[1], bit00004444[0]);
    p2s_step(iBuilder, p[1], p[5], iBuilder->simd_himask(8), 4, bit11115555[1], bit11115555[0]);
    p2s_step(iBuilder, p[2], p[6], iBuilder->simd_himask(8), 4, bit22226666[1], bit22226666[0]);
    p2s_step(iBuilder, p[3], p[7], iBuilder->simd_himask(8), 4, bit33337777[1], bit33337777[0]);

    Value * bit00224466[4];
    Value * bit11335577[4];
    for (unsigned j = 0; j<2; j++) {
        p2s_step(iBuilder, bit00004444[j], bit22226666[j],iBuilder->simd_himask(4), 2, bit00224466[2*j+1], bit00224466[2*j]);
        p2s_step(iBuilder, bit11115555[j], bit33337777[j],iBuilder->simd_himask(4), 2, bit11335577[2*j+1], bit11335577[2*j]);
    }
    for (unsigned j = 0; j<4; j++) {
        p2s_step(iBuilder, bit00224466[j], bit11335577[j], iBuilder->simd_himask(2), 1, s[2*j+1], s[2*j]);
    }
}
    		
void p2sKernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    
    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * basisBitsBlock_ptr = getStreamSetBlockPtr(self, "basisBits", blockNo);
    Value * byteStreamBlock_ptr = getStreamSetBlockPtr(self, "byteStream", blockNo);

    Value * p_bitblock[8];
    for (unsigned i = 0; i < 8; i++) {
        p_bitblock[i] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }
    Value * s_bytepack[8];
    p2s(iBuilder, p_bitblock, s_bytepack);
    for (unsigned j = 0; j < 8; ++j) {
        iBuilder->CreateBlockAlignedStore(s_bytepack[j], byteStreamBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
	
    
void p2sKernel_withCompressedOutput::prepareKernel() {
    KernelBuilder::prepareKernel();
}

void p2sKernel_withCompressedOutput::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Type * i8PtrTy = iBuilder->getInt8PtrTy(); 
    Type * i32 = iBuilder->getIntNTy(32); 
    Type * bitBlockPtrTy = llvm::PointerType::get(iBuilder->getBitBlockType(), 0); 
    
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * basisBitsBlock_ptr = getStreamSetBlockPtr(self, "basisBits", blockNo);
    Value * delCountBlock_ptr = getStreamSetBlockPtr(self, "deletionCounts", blockNo);
    Value * byteStreamBlock_ptr = getStreamSetBlockPtr(self, "byteStream", blockNo);
    
    Value * p_bitblock[8];
    for (unsigned i = 0; i < 8; i++) {
        p_bitblock[i] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(i)});
    }
    Value * s_bytepack[8];
    p2s(iBuilder, p_bitblock, s_bytepack);
    
    unsigned units_per_register = iBuilder->getBitBlockWidth()/8;
    
    Value * unit_counts = iBuilder->fwCast(units_per_register, iBuilder->CreateBlockAlignedLoad(delCountBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    
    Value * output_ptr = iBuilder->CreateBitCast(byteStreamBlock_ptr, i8PtrTy);
    Value * offset = ConstantInt::get(i32, 0);
    
    for (unsigned j = 0; j < 8; ++j) {
        iBuilder->CreateAlignedStore(s_bytepack[j], iBuilder->CreateBitCast(iBuilder->CreateGEP(output_ptr, offset), bitBlockPtrTy), 1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(j)), i32);
    }
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
    
void p2s_16Kernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * basisBitsBlock_ptr = getStreamSetBlockPtr(self, "basisBits", blockNo);
    Value * i16StreamBlock_ptr = getStreamSetBlockPtr(self, "i16Stream", blockNo);
    
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);
    
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j+8)});
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);
    
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        iBuilder->CreateBlockAlignedStore(merge0, i16StreamBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(2*j)});
        iBuilder->CreateBlockAlignedStore(merge1, i16StreamBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0), iBuilder->getInt32(2*j+1)});
    }
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
        

void p2s_16Kernel_withCompressedOutput::prepareKernel() {
    KernelBuilder::prepareKernel();
}
    

void p2s_16Kernel_withCompressedOutput::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Type * i32 = iBuilder->getIntNTy(32); 
    Type * bitBlockPtrTy = llvm::PointerType::get(iBuilder->getBitBlockType(), 0); 

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    Constant * stride = ConstantInt::get(iBuilder->getSizeTy(), iBuilder->getStride());

    Value * self = getParameter(doBlockFunction, "self");
    Value * blockNo = getScalarField(self, blockNoScalar);
    Value * basisBitsBlock_ptr = getStreamSetBlockPtr(self, "basisBits", blockNo);
    Value * delCountBlock_ptr = getStreamSetBlockPtr(self, "deletionCounts", blockNo);
    Value * i16UnitsGenerated = getProducedItemCount(self); // units generated to buffer
    Value * i16BlockNo = iBuilder->CreateUDiv(i16UnitsGenerated, stride);
    
    Value * i16StreamBase_ptr = iBuilder->CreateBitCast(getStreamSetBlockPtr(self, "i16Stream", i16BlockNo), PointerType::get(iBuilder->getInt16Ty(), 0));
    
    Value * u16_output_ptr = iBuilder->CreateGEP(i16StreamBase_ptr, iBuilder->CreateURem(i16UnitsGenerated, stride));

    
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j)});
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);
    
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(j+8)});
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);
    
    unsigned UTF_16_units_per_register = iBuilder->getBitBlockWidth()/16;
    
    Value * unit_counts = iBuilder->fwCast(UTF_16_units_per_register, iBuilder->CreateBlockAlignedLoad(delCountBlock_ptr, {iBuilder->getInt32(0), iBuilder->getInt32(0)}));
    
    Value * offset = ConstantInt::get(i32, 0);
    
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        //iBuilder->CallPrintRegister("merge0", merge0);
        iBuilder->CreateAlignedStore(merge0, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2*j)), i32);
        //iBuilder->CallPrintInt("offset", offset);
        iBuilder->CreateAlignedStore(merge1, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        //iBuilder->CallPrintRegister("merge1", merge1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2*j+1)), i32);
        //iBuilder->CallPrintInt("offset", offset);
    }
    
    i16UnitsGenerated = iBuilder->CreateAdd(i16UnitsGenerated, iBuilder->CreateZExt(offset, iBuilder->getSizeTy()));
    setProducedItemCount(self, i16UnitsGenerated);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}

void p2s_16Kernel_withCompressedOutput::generateFinalBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    Function * finalBlockFunction = m->getFunction(mKernelName + finalBlock_suffix);
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "fb_entry", finalBlockFunction, 0));
    // Final Block arguments: self, remaining, then the standard DoBlock args.
    Function::arg_iterator args = finalBlockFunction->arg_begin();
    Value * self = &*(args++);
    /* Skip "remaining" arg */ args++;
    std::vector<Value *> doBlockArgs = {self};
    while (args != finalBlockFunction->arg_end()){
        doBlockArgs.push_back(&*args++);
    }
    Value * i16UnitsGenerated = getProducedItemCount(self); // units generated to buffer

    iBuilder->CreateCall(doBlockFunction, doBlockArgs);
    i16UnitsGenerated = getProducedItemCount(self); // units generated to buffer
    for (unsigned i = 0; i < mStreamSetOutputs.size(); i++) {
        Value * ssStructPtr = getStreamSetStructPtr(self, mStreamSetOutputs[i].ssName);
        Value * producerPosPtr = mStreamSetOutputBuffers[i]->getProducerPosPtr(ssStructPtr);
        iBuilder->CreateAtomicStoreRelease(i16UnitsGenerated, producerPosPtr);
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
    
    
}
