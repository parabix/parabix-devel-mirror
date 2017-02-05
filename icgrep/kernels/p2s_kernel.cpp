#include "p2s_kernel.h"
#include "IR_Gen/idisa_builder.h"  // for IDISA_Builder
#include "llvm/IR/Constant.h"      // for Constant
#include "llvm/IR/Constants.h"     // for ConstantInt
#include "llvm/IR/DerivedTypes.h"  // for PointerType, VectorType
#include "llvm/IR/Function.h"      // for Function, Function::arg_iterator
#include <llvm/IR/Module.h>
#include <kernels/streamset.h>
namespace llvm { class Value; }

using namespace llvm;
using namespace parabix;

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
    		
void P2SKernel::generateDoBlockMethod() {
    Value * p_bitblock[8];
    for (unsigned i = 0; i < 8; i++) {
        Value * ptr = getInputStream("basisBits", iBuilder->getInt32(i));
        p_bitblock[i] = iBuilder->CreateBlockAlignedLoad(ptr);
    }
    Value * s_bytepack[8];
    p2s(iBuilder, p_bitblock, s_bytepack);
    for (unsigned j = 0; j < 8; ++j) {
        Value * ptr = getOutputStream("byteStream", iBuilder->getInt32(0), iBuilder->getInt32(j));
        iBuilder->CreateBlockAlignedStore(s_bytepack[j], ptr);
    }
}

P2SKernel::P2SKernel(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "p2s",
              {Binding{iBuilder->getStreamSetTy(8, 1), "basisBits"}},
              {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
              {}, {}, {}) {

}
    

void P2SKernelWithCompressedOutput::generateDoBlockMethod() {
    PointerType * i8PtrTy = iBuilder->getInt8PtrTy();
    IntegerType * i32 = iBuilder->getInt32Ty();
    PointerType * bitBlockPtrTy = PointerType::get(iBuilder->getBitBlockType(), 0);

    Value * basisBits[8];
    for (unsigned i = 0; i < 8; i++) {
        Value * basisBitsBlock_ptr = getInputStream("basisBits", iBuilder->getInt32(i));
        basisBits[i] = iBuilder->CreateBlockAlignedLoad(basisBitsBlock_ptr);
    }
    Value * bytePack[8];
    p2s(iBuilder, basisBits, bytePack);

    unsigned units_per_register = iBuilder->getBitBlockWidth()/8;
    Value * delCountBlock_ptr = getInputStream("deletionCounts", iBuilder->getInt32(0));
    Value * unit_counts = iBuilder->fwCast(units_per_register, iBuilder->CreateBlockAlignedLoad(delCountBlock_ptr));

    Value * unitsGenerated = getProducedItemCount("byteStream"); // units generated to buffer
    Value * output_ptr = getStreamView(i8PtrTy, "byteStream", getBlockNo(), iBuilder->getInt32(0));
    Value * offset = iBuilder->getInt32(0);
    for (unsigned j = 0; j < 8; ++j) {
        iBuilder->CreateStore(bytePack[j], iBuilder->CreateBitCast(iBuilder->CreateGEP(output_ptr, offset), bitBlockPtrTy));
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(j)), i32);
    }
    unitsGenerated = iBuilder->CreateAdd(unitsGenerated, iBuilder->CreateZExt(offset, iBuilder->getSizeTy()));
    setProducedItemCount("byteStream", unitsGenerated);
}
    
P2SKernelWithCompressedOutput::P2SKernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "p2s_compress",
              {Binding{iBuilder->getStreamSetTy(8, 1), "basisBits"}, Binding{iBuilder->getStreamSetTy(1, 1), "deletionCounts"}},
              {Binding{iBuilder->getStreamSetTy(1, 8), "byteStream"}},
              {}, {}, {}) {
    setDoBlockUpdatesProducedItemCountsAttribute(true);
}
    
    

void P2S16Kernel::generateDoBlockMethod() {
    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        Value * ptr = getInputStream("basisBits", iBuilder->getInt32(0), iBuilder->getInt32(j));
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(ptr);
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);    
    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        Value * ptr = getInputStream("basisBits", iBuilder->getInt32(0), iBuilder->getInt32(j + 8));
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(ptr);
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);   
    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        Value * ptr0 = getOutputStream("i16Stream", iBuilder->getInt32(2 * j));
        iBuilder->CreateBlockAlignedStore(merge0, ptr0);
        Value * ptr1 = getOutputStream("i16Stream", iBuilder->getInt32(2 * j + 1));
        iBuilder->CreateBlockAlignedStore(merge1, ptr1);
    }
}
    

P2S16Kernel::P2S16Kernel(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "p2s_16",
              {Binding{iBuilder->getStreamSetTy(16, 1), "basisBits"}},
              {Binding{iBuilder->getStreamSetTy(1, 16), "i16Stream"}},
              {}, {}, {}) {

}

    
void P2S16KernelWithCompressedOutput::generateDoBlockMethod() {
    IntegerType * i32Ty = iBuilder->getInt32Ty();
    PointerType * bitBlockPtrTy = iBuilder->getBitBlockType()->getPointerTo();

    Value * hi_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        Value * ptr = getInputStream("basisBits", iBuilder->getInt32(j));
        hi_input[j] = iBuilder->CreateBlockAlignedLoad(ptr);
    }
    Value * hi_bytes[8];
    p2s(iBuilder, hi_input, hi_bytes);

    Value * lo_input[8];
    for (unsigned j = 0; j < 8; ++j) {
        Value * ptr = getInputStream("basisBits", iBuilder->getInt32(j + 8));
        lo_input[j] = iBuilder->CreateBlockAlignedLoad(ptr);
    }
    Value * lo_bytes[8];
    p2s(iBuilder, lo_input, lo_bytes);

    Value * delCountBlock_ptr = getInputStream("deletionCounts", iBuilder->getInt32(0));
    Value * unit_counts = iBuilder->fwCast(iBuilder->getBitBlockWidth() / 16, iBuilder->CreateBlockAlignedLoad(delCountBlock_ptr));
    PointerType * int16PtrTy = PointerType::get(iBuilder->getInt16Ty(), 0);

    ConstantInt * stride = iBuilder->getSize(iBuilder->getStride());
    Value * i16UnitsGenerated = getProducedItemCount("i16Stream"); // units generated to buffer
    Value * i16BlockNo = iBuilder->CreateUDiv(i16UnitsGenerated, stride);
    Value * u16_output_ptr = getStreamView(int16PtrTy, "i16Stream", i16BlockNo, iBuilder->CreateURem(i16UnitsGenerated, stride));

    Value * offset = ConstantInt::get(i32Ty, 0);

    for (unsigned j = 0; j < 8; ++j) {
        Value * merge0 = iBuilder->bitCast(iBuilder->esimd_mergel(8, hi_bytes[j], lo_bytes[j]));
        iBuilder->CreateAlignedStore(merge0, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2 * j)), i32Ty);

        Value * merge1 = iBuilder->bitCast(iBuilder->esimd_mergeh(8, hi_bytes[j], lo_bytes[j]));
        iBuilder->CreateAlignedStore(merge1, iBuilder->CreateBitCast(iBuilder->CreateGEP(u16_output_ptr, offset), bitBlockPtrTy), 1);
        offset = iBuilder->CreateZExt(iBuilder->CreateExtractElement(unit_counts, iBuilder->getInt32(2 * j + 1)), i32Ty);
    }
    Value * i16UnitsFinal = iBuilder->CreateAdd(i16UnitsGenerated, iBuilder->CreateZExt(offset, iBuilder->getSizeTy()));
    setProducedItemCount("i16Stream", i16UnitsFinal);
    auto const &b  = getStreamSetBuffer("i16Stream");

    if (auto cb = dyn_cast<CircularCopybackBuffer>(b)) {
        BasicBlock * copyBack = CreateBasicBlock("copyBack");
        BasicBlock * p2sCompressDone = CreateBasicBlock("p2sCompressDone");
        
        // Check for overflow into the buffer overflow area and copy data back if so.
        Value * accessible = cb->getLinearlyAccessibleItems(i16UnitsGenerated);
        offset = iBuilder->CreateZExt(offset, iBuilder->getSizeTy());
        Value * wraparound = iBuilder->CreateICmpULT(accessible, offset);
        iBuilder->CreateCondBr(wraparound, copyBack, p2sCompressDone);
        
        iBuilder->SetInsertPoint(copyBack);
        Value * copyItems = iBuilder->CreateSub(offset, accessible);
        cb->createCopyBack(getStreamSetBufferPtr("i16Stream"), copyItems);
        iBuilder->CreateBr(p2sCompressDone);
        iBuilder->SetInsertPoint(p2sCompressDone);
    }
}
    
P2S16KernelWithCompressedOutput::P2S16KernelWithCompressedOutput(IDISA::IDISA_Builder * iBuilder)
: BlockOrientedKernel(iBuilder, "p2s_16_compress",
              {Binding{iBuilder->getStreamSetTy(16, 1), "basisBits"}, Binding{iBuilder->getStreamSetTy(1, 1), "deletionCounts"}},
              {Binding{iBuilder->getStreamSetTy(1, 16), "i16Stream"}},
              {},
              {},
              {Binding{iBuilder->getSizeTy(), "unitsGenerated"}, Binding{iBuilder->getSizeTy(), "unitsWritten"}}) {
    setDoBlockUpdatesProducedItemCountsAttribute(true);
}
    
    
}
