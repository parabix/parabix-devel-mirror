#include "p2s_kernel.h"
#include "kernels/kernel.h"
#include "IDISA/idisa_builder.h"
#include <llvm/IR/TypeBuilder.h>
#include <llvm/IR/Type.h>
#include <iostream>
#include <stdint.h>
#include <llvm/Support/FileSystem.h>
#include <llvm/Support/raw_ostream.h>



extern "C" {
    void buffered_write(const char * ptr, size_t bytes) {
        outs().write(ptr, bytes);
    }
};

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
    
    Value * basisBitsBlock_ptr = getParameter(doBlockFunction, "basisBits");  // input
    Value * byteStreamBlock_ptr = getParameter(doBlockFunction, "byteStream"); // output
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
	
void p2s_16Kernel::generateDoBlockMethod() {
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    
    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    
    Value * basisBitsBlock_ptr = getParameter(doBlockFunction, "basisBits");  // input
    Value * i16StreamBlock_ptr = getParameter(doBlockFunction, "i16Stream"); // output
    
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
        
    
Function * create_write(Module * const mod) {
    Function * write = mod->getFunction("write");
    if (write == nullptr) {
        FunctionType *write_type =
        TypeBuilder<long(int, char *, long), false>::get(mod->getContext());
        write = cast<Function>(mod->getOrInsertFunction("write", write_type,
                                                        AttributeSet().addAttribute(mod->getContext(), 2U, Attribute::NoAlias)));
    }
    return write;
}

const size_t OutputBufferSize=65536;

void p2s_16Kernel_withCompressedOutputKernel::generateDoBlockMethod() {
    outs().SetBufferSize(OutputBufferSize);
    IDISA::IDISA_Builder::InsertPoint savePoint = iBuilder->saveIP();
    Module * m = iBuilder->getModule();
    Type * i8PtrTy = iBuilder->getInt8PtrTy(); 
    Type * i64 = iBuilder->getIntNTy(64); 
    Type * bitBlockPtrTy = llvm::PointerType::get(iBuilder->getBitBlockType(), 0); 
    
    Function * writefn = cast<Function>(m->getOrInsertFunction("buffered_write", iBuilder->getVoidTy(), i8PtrTy, i64, nullptr));

    Function * doBlockFunction = m->getFunction(mKernelName + doBlock_suffix);
    
    iBuilder->SetInsertPoint(BasicBlock::Create(iBuilder->getContext(), "entry", doBlockFunction, 0));
    
    Value * basisBitsBlock_ptr = getParameter(doBlockFunction, "basisBits");  // input
    Value * delCountBlock_ptr = getParameter(doBlockFunction, "deletionCounts");
    Value * i16StreamBlock_ptr = getParameter(doBlockFunction, "i16Stream"); // output

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
    
    Value * u16_output_ptr = iBuilder->CreateBitCast(i16StreamBlock_ptr, PointerType::get(iBuilder->getInt16Ty(), 0));
    Value * offset = ConstantInt::get(i64, 0);
    
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        //iBuilder->CallPrintRegister("merge0", merge0);
        iBuilder->CreateAlignedStore(merge0, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2*j)), i64);
        //iBuilder->CallPrintInt("offset", offset);
        iBuilder->CreateAlignedStore(merge1, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        //iBuilder->CallPrintRegister("merge1", merge1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2*j+1)), i64);
        //iBuilder->CallPrintInt("offset", offset);
    }
    Value * byte_offset = iBuilder->CreateAdd(offset, offset);
    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->CreateBitCast(i16StreamBlock_ptr, i8PtrTy), byte_offset}));
    iBuilder->CreateRetVoid();
    iBuilder->restoreIP(savePoint);
}
        
}
