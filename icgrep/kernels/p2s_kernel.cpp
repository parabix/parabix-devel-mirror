#include "p2s_kernel.h"
#include "kernels/kernel.h"
#include "IDISA/idisa_builder.h"
#include <llvm/IR/TypeBuilder.h>
#include <llvm/IR/Type.h>
#include <iostream>

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
    		
void generateP2SKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder) {
    for (unsigned i = 0; i < 8; ++i) {
        kBuilder->addInputStream(1);
    }
    kBuilder->addOutputStream(8);
    kBuilder->prepareFunction();
    Value * input[8];
    for (unsigned j = 0; j < 8; ++j) {
        input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j));
    }
    Value * output[8];
    p2s(iBuilder, input, output);
    Value * output_ptr = kBuilder->getOutputStream(0);
    for (unsigned j = 0; j < 8; ++j) {

        iBuilder->CreateBlockAlignedStore(output[j], iBuilder->CreateGEP(output_ptr, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(j) })));
    }
    kBuilder->finalize();
}

void generateP2S_16Kernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder) {
    for (unsigned i = 0; i < 16; ++i) {
        kBuilder->addInputStream(1);
    }
    kBuilder->addOutputStream(16);
    kBuilder->prepareFunction();
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j));
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);
    
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j+8));
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);
    
    Value * output_ptr = kBuilder->getOutputStream(0);
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        iBuilder->CreateBlockAlignedStore(merge0, iBuilder->CreateGEP(output_ptr, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(2*j) })));
        iBuilder->CreateBlockAlignedStore(merge1, iBuilder->CreateGEP(output_ptr, std::vector<Value *>({ iBuilder->getInt32(0), iBuilder->getInt32(2*j+1) })));
    }
    kBuilder->finalize();
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

void generateP2S_16_withCompressedOutputKernel(Module * m, IDISA::IDISA_Builder * iBuilder, KernelBuilder * kBuilder) {
    for (unsigned i = 0; i < 16; ++i) {
        kBuilder->addInputStream(1);
    }        
    kBuilder->addInputStream(1);  // partial popcounts
    kBuilder->addOutputStream(16);

    kBuilder->prepareFunction();
    Function * writefn = create_write(m);
    
    Type * i8PtrTy = iBuilder->getInt8PtrTy(); 
    Type * i64 = iBuilder->getIntNTy(64); 
    Type * bitBlockPtrTy = llvm::PointerType::get(iBuilder->getBitBlockType(), 0); 
    
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j));
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);
    
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(j+8));
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);
    
    unsigned UTF_16_units_per_register = iBuilder->getBitBlockWidth()/16;
    
    Value * partial_counts = iBuilder->fwCast(UTF_16_units_per_register, iBuilder->CreateBlockAlignedLoad(kBuilder->getInputStream(16)));
    if (UTF_16_units_per_register < 16) {
        partial_counts = iBuilder->CreateZExt(partial_counts, VectorType::get(iBuilder->getIntNTy(16), iBuilder->getBitBlockWidth()/UTF_16_units_per_register));
    }
    Value * byte_counts = iBuilder->CreateAdd(partial_counts, partial_counts); // double the code unit count to get byte counts
    
    Value * output_ptr = iBuilder->CreateBitCast(kBuilder->getOutputStream(0), i8PtrTy);
    Value * byte_offset = ConstantInt::get(i64, 0);
    
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        //iBuilder->CallPrintRegister("merge0", merge0);
        iBuilder->CreateAlignedStore(merge0, iBuilder->CreateBitCast(iBuilder->CreateGEP(output_ptr, byte_offset), bitBlockPtrTy), 1);
        byte_offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(byte_counts, iBuilder->getInt32(2*j)), i64);
        //iBuilder->CallPrintInt("byte_offset", byte_offset);
        iBuilder->CreateAlignedStore(merge1, iBuilder->CreateBitCast(iBuilder->CreateGEP(output_ptr, byte_offset), bitBlockPtrTy), 1);
        //iBuilder->CallPrintRegister("merge1", merge1);
        byte_offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(byte_counts, iBuilder->getInt32(2*j+1)), i64);
        //iBuilder->CallPrintInt("byte_offset", byte_offset);
    }
    iBuilder->CreateCall(writefn, std::vector<Value *>({iBuilder->getInt32(1), output_ptr, byte_offset}));
    
    kBuilder->finalize();
}

}
